#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"

#include <map>
#include <vector>
#include <cmath>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WiredSegmentedBroadcasting");

// 平均算出用のグローバル変数
std::vector<double> g_waitTimes;
uint32_t g_completedNodes = 0;
uint32_t g_totalNodes = 3;

// --- ClientApp ---
class ClientApp : public Application {
public:
  ClientApp ()
    : m_rawReceivedCount (0),
      m_totalSegments (0),
      m_receivedCount (0),
      m_allReceived (false) {}

  void Setup (uint32_t totalSegments) {
    m_totalSegments = totalSegments;
  }

  uint32_t GetReceivedCount () const {
    return m_receivedCount;
  }

  uint32_t GetRawReceivedCount () const {
    return m_rawReceivedCount;
  }

private:
  uint32_t m_rawReceivedCount;
  Time m_startTime;
  uint32_t m_totalSegments;
  uint32_t m_receivedCount;
  bool m_allReceived;
  std::map<uint32_t, bool> m_receivedMap;
  std::vector<Ptr<Socket>> m_sockets;

  void HandleRead (Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom (from))) {
      m_rawReceivedCount++;

      SeqTsHeader header;
      packet->RemoveHeader (header);
      uint32_t segmentId = header.GetSeq ();

      if (segmentId >= 1 && segmentId <= m_totalSegments && !m_receivedMap[segmentId]) {
        m_receivedMap[segmentId] = true;
        m_receivedCount++;

        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          Time finishTime = Simulator::Now ();
          double wait = (finishTime - m_startTime).GetSeconds ();

          g_waitTimes.push_back (wait);
          g_completedNodes++;

          NS_LOG_UNCOND (" Node " << GetNode ()->GetId ()
                         << " 完了! (待ち時間: " << wait << "s)");

          // 有線版では、全ノード完了後すぐ止める。
          // 余計な送信が続くと送信数だけ増えて欠損率が大きく見えるため。
          if (g_completedNodes >= g_totalNodes) {
            Simulator::Stop (Simulator::Now ());
          }
        }
      }
    }
  }

  virtual void StartApplication (void) override {
    m_startTime = Simulator::Now ();

    // セグメントIDに対応するポートを待ち受ける
    for (uint32_t i = 0; i < m_totalSegments; ++i) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9001 + i));
      sock->SetRecvCallback (MakeCallback (&ClientApp::HandleRead, this));
      m_sockets.push_back (sock);
    }
  }

  virtual void StopApplication (void) override {
    double finalArvRate = 0.0;
    if (m_totalSegments > 0) {
      finalArvRate = ((double)m_receivedCount / m_totalSegments) * 100.0;
    }

    NS_LOG_UNCOND ("================================================");
    NS_LOG_UNCOND ("【ノード " << GetNode ()->GetId () << " 最終報告】");
    NS_LOG_UNCOND ("受信済み: " << m_receivedCount << " / " << m_totalSegments);
    NS_LOG_UNCOND ("セグメント到達率: " << finalArvRate << " %");

    if (m_receivedCount == m_totalSegments) {
      NS_LOG_UNCOND ("状態: 完全受信成功");
    } else {
      NS_LOG_UNCOND ("状態: 不完全（残り " << (m_totalSegments - m_receivedCount) << "個）");
      std::cout << "Node " << GetNode ()->GetId () << " 未受信ID一覧: [ ";
      for (uint32_t i = 1; i <= m_totalSegments; ++i) {
        if (!m_receivedMap[i]) {
          std::cout << i << " ";
        }
      }
      std::cout << "]" << std::endl;
    }
    NS_LOG_UNCOND ("================================================");

    for (auto& sock : m_sockets) {
      if (sock) {
        sock->Close ();
      }
    }
  }
};

// --- ServerApp ---
class ServerApp : public Application {
public:
  ServerApp ()
    : m_packetSize (1024),
      m_totalSegments (0),
      m_interval (Seconds (1.0)),
      m_running (false),
      m_sentCount (0) {}

  void Setup (const std::vector<Ipv4Address>& destinations,
              uint32_t packetSize,
              uint32_t numChannels,
              Time interval,
              uint32_t mode) {
    m_destinations = destinations;
    m_packetSize = packetSize;
    m_interval = interval;

    uint32_t nSegments = std::pow (2, numChannels) - 1;
    m_totalSegments = nSegments;

    if (mode == 0) {
      // FB法: チャネルiに 2^i 個のセグメントを割り当てる
      uint32_t cur = 1;
      for (uint32_t i = 0; i < numChannels; ++i) {
        for (uint32_t j = 0; j < std::pow (2, i); ++j) {
          m_channelData[i].push_back (cur++);
        }
        m_indices[i] = 0;
      }
    } else if (mode == 1) {
      // 放送配信: 1チャネルで全セグメントを順番に送信
      for (uint32_t j = 1; j <= nSegments; ++j) {
        m_channelData[0].push_back (j);
      }
      m_indices[0] = 0;
    } else {
      // 等分割法: 各セグメントを個別チャネルへ割り当てる
      for (uint32_t i = 0; i < nSegments; ++i) {
        m_channelData[i].push_back (i + 1);
        m_indices[i] = 0;
      }
    }
  }

  uint32_t GetTotalSegments () const {
    return m_totalSegments;
  }

  uint32_t GetTotalSent () const {
    return m_sentCount;
  }

private:
  std::vector<Ipv4Address> m_destinations;
  uint32_t m_packetSize;
  uint32_t m_totalSegments;
  Time m_interval;
  bool m_running;
  uint32_t m_sentCount;
  std::map<uint32_t, Ptr<Socket>> m_socketsMap;
  std::map<uint32_t, std::vector<uint32_t>> m_channelData;
  std::map<uint32_t, uint32_t> m_indices;

  virtual void StartApplication (void) override {
    m_running = true;

    for (auto const& item : m_channelData) {
      uint32_t chanIdx = item.first;
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_socketsMap[chanIdx] = sock;
      Simulator::Schedule (Seconds (chanIdx * 0.001), &ServerApp::SendPacket, this, chanIdx);
    }
  }

  virtual void StopApplication (void) override {
    m_running = false;
    for (auto& item : m_socketsMap) {
      if (item.second) {
        item.second->Close ();
      }
    }
  }

  void SendPacket (uint32_t chanIdx) {
    if (!m_running) {
      return;
    }

    uint32_t segId = m_channelData[chanIdx][m_indices[chanIdx]];

    // 有線版では、疑似マルチキャストとして各クライアントへ同じセグメントをユニキャスト送信する
    for (const auto& dest : m_destinations) {
      Ptr<Packet> packet = Create<Packet> (m_packetSize);
      SeqTsHeader header;
      header.SetSeq (segId);
      packet->AddHeader (header);

      m_socketsMap[chanIdx]->SendTo (packet, 0, InetSocketAddress (dest, 9001 + chanIdx));
      m_sentCount++;
    }

    m_indices[chanIdx] = (m_indices[chanIdx] + 1) % m_channelData[chanIdx].size ();
    Simulator::Schedule (m_interval, &ServerApp::SendPacket, this, chanIdx);
  }
};

int main (int argc, char *argv[]) {
  uint32_t mode = 0;
  uint32_t nNodes = 3;
  uint32_t kChannels = 4;
  double interval = 1.0;
  double simTime = 400.0;
  uint32_t packetSize = 1024;

  std::string dataRate = "11Mbps";

  // 有線距離[m]。距離は伝搬遅延としてのみ反映する。
  double dist = 0.0;

  // 有線中の信号伝搬速度。おおよそ光速の2/3程度。
  double propagationSpeed = 2.0e8;

  uint32_t seed = 1;
  uint32_t run = 1;
  bool enablePcap = false;

  CommandLine cmd;
  cmd.AddValue ("mode", "0:FB, 1:Simple, 2:Full", mode);
  cmd.AddValue ("nodes", "Number of client nodes", nNodes);
  cmd.AddValue ("k", "Number of channels", kChannels);
  cmd.AddValue ("int", "Interval (s)", interval);
  cmd.AddValue ("simTime", "Simulation time (s)", simTime);
  cmd.AddValue ("packetSize", "Packet size in bytes", packetSize);
  cmd.AddValue ("rate", "PointToPoint data rate", dataRate);
  cmd.AddValue ("dist", "Cable distance in meters", dist);
  cmd.AddValue ("propSpeed", "Propagation speed in cable (m/s)", propagationSpeed);
  cmd.AddValue ("seed", "Random seed", seed);
  cmd.AddValue ("run", "Run number", run);
  cmd.AddValue ("pcap", "Enable PCAP output", enablePcap);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (seed);
  RngSeedManager::SetRun (run);
  g_totalNodes = nNodes;

  // 距離を伝搬遅延に変換
  double delaySeconds = 0.0;
  if (propagationSpeed > 0.0) {
    delaySeconds = dist / propagationSpeed;
  }

  std::ostringstream delayStream;
  delayStream << delaySeconds << "s";
  std::string delay = delayStream.str ();

  NodeContainer serverNode;
  NodeContainer clientNodes;
  serverNode.Create (1);
  clientNodes.Create (nNodes);

  NodeContainer allNodes;
  allNodes.Add (serverNode);
  allNodes.Add (clientNodes);

  // 有線PointToPoint環境
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue (dataRate));
  p2p.SetChannelAttribute ("Delay", StringValue (delay));

  InternetStackHelper stack;
  stack.Install (allNodes);

  std::vector<Ipv4Address> clientAddresses;
  NetDeviceContainer allDevices;

  for (uint32_t i = 0; i < nNodes; ++i) {
    NodeContainer pair (serverNode.Get (0), clientNodes.Get (i));
    NetDeviceContainer linkDevices = p2p.Install (pair);
    allDevices.Add (linkDevices);

    std::ostringstream subnet;
    subnet << "10.1." << (i + 1) << ".0";

    Ipv4AddressHelper address;
    address.SetBase (subnet.str ().c_str (), "255.255.255.0");
    Ipv4InterfaceContainer ifc = address.Assign (linkDevices);

    // ifc.GetAddress(0): server側, ifc.GetAddress(1): client側
    clientAddresses.push_back (ifc.GetAddress (1));
  }

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // 位置は有線では性能に影響しないが、NetAnim等で見やすいように配置
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (uint32_t i = 0; i < nNodes; ++i) {
    positionAlloc->Add (Vector (20.0, i * 10.0, 0.0));
  }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (clientAddresses, packetSize, kChannels, Seconds (interval), mode);
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds (interval));

  uint32_t totalSegs = serverApp->GetTotalSegments ();
  std::string modeName = (mode == 0) ? "FB法" : (mode == 1 ? "放送配信" : "等分割法");

  uint32_t actualk;
  if (mode == 0) {
    actualk = kChannels;
  } else if (mode == 1) {
    actualk = 1;
  } else {
    actualk = totalSegs;
  }

  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("【実験設定：シミュレーションパラメータ】");
  NS_LOG_UNCOND ("環境            : 有線 PointToPoint");
  NS_LOG_UNCOND ("配信手法        : " << modeName);
  NS_LOG_UNCOND ("ノード数        : " << nNodes);
  NS_LOG_UNCOND ("チャネル数 (k)  : " << actualk);
  NS_LOG_UNCOND ("総セグメント数(N): " << totalSegs);
  NS_LOG_UNCOND ("スロット間隔    : " << interval << "s");
  NS_LOG_UNCOND ("パケットサイズ  : " << packetSize << " bytes");
  NS_LOG_UNCOND ("------------------------------------------------");
  NS_LOG_UNCOND ("【有線設定】");
  NS_LOG_UNCOND ("DataRate        : " << dataRate);
  NS_LOG_UNCOND ("距離            : " << dist << "m");
  NS_LOG_UNCOND ("伝搬速度        : " << propagationSpeed << "m/s");
  NS_LOG_UNCOND ("Delay           : " << delay);
  NS_LOG_UNCOND ("乱数Seed        : " << seed);
  NS_LOG_UNCOND ("乱数Run         : " << run);
  NS_LOG_UNCOND ("================================================");

  for (uint32_t i = 0; i < nNodes; ++i) {
    Ptr<ClientApp> clientApp = CreateObject<ClientApp> ();
    clientApp->Setup (totalSegs);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (2.0 + i * 0.01));
  }

  if (enablePcap) {
    p2p.EnablePcapAll ("wired-seg-experiment");
  }

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  double totalArvRateSum = 0.0;
  uint32_t totalReceivedByAllClients = 0;

  NS_LOG_UNCOND ("\n==== 個別ノード受信状況報告 ====");
  for (uint32_t i = 0; i < clientNodes.GetN (); ++i) {
    Ptr<ClientApp> app = DynamicCast<ClientApp> (clientNodes.Get (i)->GetApplication (0));
    uint32_t received = app->GetReceivedCount ();
    double arvRate = ((double)received / totalSegs) * 100.0;
    totalArvRateSum += arvRate;
    totalReceivedByAllClients += app->GetRawReceivedCount ();
  }

  double sum = 0.0;
  for (double t : g_waitTimes) {
    sum += t;
  }
  double avgWait = g_waitTimes.empty () ? 0.0 : sum / g_waitTimes.size ();
  double avgArv = totalArvRateSum / nNodes;

  uint32_t totalSentByServer = serverApp->GetTotalSent ();

  NS_LOG_UNCOND ("【総送信数】        : " << totalSentByServer);
  NS_LOG_UNCOND ("【実受信数】        : " << totalReceivedByAllClients);
  NS_LOG_UNCOND ("【平均到達率】      : " << avgArv << "%");
  NS_LOG_UNCOND ("【平均待ち時間】    : " << avgWait << "s");

  NS_LOG_UNCOND ("\n=== 最終結果サマリー (Excel用) ===");
  NS_LOG_UNCOND ("環境,手法,ノード数,チャネル数(k),総セグメント数(N),スロット間隔,帯域幅,距離,伝搬速度,遅延,パケットサイズ,平均待ち時間,平均到達率,Seed,Run");
  NS_LOG_UNCOND ("有線," << modeName << "," << nNodes << "," << actualk << "," << totalSegs << ","
                 << interval << "," << dataRate << "," << dist << "," << propagationSpeed << ","
                 << delay << "," << packetSize << "," << avgWait << "," << avgArv << "%" << ","
                 << seed << "," << run);

  Simulator::Destroy ();
  return 0;
}
