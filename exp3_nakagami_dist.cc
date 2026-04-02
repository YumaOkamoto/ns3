#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include <map>
#include <vector>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiBroadcastingVariable");

// 平均算出用のグローバル変数
std::vector<double> g_waitTimes;
uint32_t g_completedNodes = 0;
uint32_t g_totalNodes = 3;

// --- ClientApp ---
class ClientApp : public Application {
public:
  ClientApp () : m_receivedCount (0), m_allReceived (false) {}
  void Setup (uint32_t totalSegments) {
    m_totalSegments = totalSegments;
  }
  
  uint32_t GetReceivedCount() const {
    return m_receivedCount;
  }

  virtual void StopApplication(void){
    double finalLossRate = (1.0 - (double)m_receivedCount / m_totalSegments) * 100;

    // コンソールへの出力
    NS_LOG_UNCOND ("================================================");
    NS_LOG_UNCOND ("【ノード " << GetNode()->GetId() << " 最終報告】");
    NS_LOG_UNCOND ("受信済み: " << m_receivedCount << " / " << m_totalSegments);
    NS_LOG_UNCOND ("パケット欠損率: " << finalLossRate << " %");

    if (m_receivedCount == m_totalSegments) {
      NS_LOG_UNCOND ("状態: 完全受信成功");
    } else {
      NS_LOG_UNCOND ("状態: 不完全（残り " << (m_totalSegments - m_receivedCount) << "個）");
    }
    NS_LOG_UNCOND ("================================================");

    // 既存の終了処理
    for (std::vector<Ptr<Socket>>::iterator it = m_sockets.begin(); it != m_sockets.end(); ++it) {
        if (*it) {
            (*it)->Close();
        }
    }
  }


private:
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
      SeqTsHeader header;
      packet->RemoveHeader(header);
      uint32_t segmentId = header.GetSeq(); 
      if (segmentId >= 1 && segmentId <= m_totalSegments && !m_receivedMap[segmentId]) {
        m_receivedMap[segmentId] = true;
        m_receivedCount++;

        // --- パケット欠損率と進捗の表示を追加 ---
        //double lossRate = (1.0 - (double)m_receivedCount / m_totalSegments) * 100;
        //NS_LOG_UNCOND ("Node " << GetNode()->GetId() << " 受信: " << m_receivedCount
        //               << "/" << m_totalSegments << " (ID:" << segmentId
        //               << ") 未取得率: " << lossRate << "%");
        // ------------------------------------

        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          Time finishTime = Simulator::Now ();
          double wait = (finishTime - m_startTime).GetSeconds();
          g_waitTimes.push_back(wait);
          g_completedNodes++;
          if(g_completedNodes >= g_totalNodes){
            Simulator::Stop(Simulator::Now() + Seconds(1.0));
          }

          NS_LOG_UNCOND (" Node " << GetNode()->GetId() << " 完了! (待ち時間: " << wait << "s)");
        }
      }
    }
  }

  virtual void StartApplication (void) {
    m_startTime = Simulator::Now ();
    for (uint32_t i = 0; i < 2048; ++i) { 
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9001 + i));
      Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (sock);
      if (udpSocket) udpSocket->MulticastJoinGroup (0, InetSocketAddress (Ipv4Address ("224.1.1.1"), 9001 + i));
      sock->SetRecvCallback (MakeCallback (&ClientApp::HandleRead, this));
      m_sockets.push_back (sock);
    }
  }
};

// --- ServerApp ---
class ServerApp : public Application {
public:
  ServerApp () : m_running (false) {}
  void Setup (Ipv4Address dest, uint32_t packetSize, uint32_t numChannels, Time interval, int mode) {
    m_peer = dest; m_packetSize = packetSize; m_interval = interval;
    uint32_t N_fb = std::pow(2, numChannels) - 1;
    m_totalSegments = N_fb;

    if (mode == 0) {
      uint32_t cur = 1;
      for (uint32_t i = 0; i < numChannels; ++i) {
        for (uint32_t j = 0; j < std::pow(2, i); ++j) m_channelData[i].push_back(cur++);
        m_indices[i] = 0;
      }
      m_totalSegments = N_fb;
    } else if (mode == 1) {
      for (uint32_t j = 1; j <= N_fb; ++j) m_channelData[0].push_back(j);
      m_indices[0] = 0; 
    } else {
        for (uint32_t i = 0; i < N_fb; ++i) {
          m_channelData[i].clear();
          m_channelData[i].push_back(i + 1); // 各チャネルiにセグメントi+1を割り当て
          m_indices[i] = 0;
        }
      } 
  }
  
  uint32_t GetTotalSegments() { return m_totalSegments; }
private:
  virtual void StartApplication (void) {
    m_running = true;
    for (auto const& [chanIdx, data] : m_channelData) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_sockets_map[chanIdx] = sock;
      Simulator::Schedule (Seconds (chanIdx * 0.001), &ServerApp::SendPacket, this, chanIdx);
    }
  }
  void SendPacket (uint32_t chanIdx) {
    if (!m_running) return;
    uint32_t segId = m_channelData[chanIdx][m_indices[chanIdx]];
    Ptr<Packet> packet = Create<Packet> (m_packetSize);
    SeqTsHeader header; header.SetSeq(segId); packet->AddHeader(header);
    m_sockets_map[chanIdx]->SendTo (packet, 0, InetSocketAddress (m_peer, 9001 + chanIdx));
    m_indices[chanIdx] = (m_indices[chanIdx] + 1) % m_channelData[chanIdx].size();
    Simulator::Schedule (m_interval, &ServerApp::SendPacket, this, chanIdx);
  }
  Ipv4Address m_peer; uint32_t m_packetSize; uint32_t m_totalSegments; Time m_interval;
  bool m_running; std::map<uint32_t, Ptr<Socket>> m_sockets_map;
  std::map<uint32_t, std::vector<uint32_t>> m_channelData;
  std::map<uint32_t, uint32_t> m_indices;
};

int main (int argc, char *argv[]) {
  uint32_t mode = 0;
  uint32_t nNodes = 3;
  uint32_t kChannels = 4;
  double interval = 1.0;
  double dist = 30.0;
  std::string dataRate = "DsssRate1Mbps";

  CommandLine cmd;
  cmd.AddValue ("mode", "0:FB, 1:Simple, 2:Full", mode);
  cmd.AddValue ("nodes", "Number of client nodes", nNodes);
  cmd.AddValue ("k", "Number of channels", kChannels);
  cmd.AddValue ("int", "Interval (s)", interval);
  cmd.AddValue ("dist", "Distance (m)", dist);
  cmd.AddValue ("rate", "Data Rate", dataRate);
  cmd.Parse (argc, argv);
  g_totalNodes = nNodes;

  NodeContainer serverNode, clientNodes;
  serverNode.Create (1); clientNodes.Create (nNodes);
  NodeContainer allNodes = NodeContainer (serverNode, clientNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (dataRate), "ControlMode", StringValue (dataRate));
  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel",
                             "Distance1", DoubleValue (40.0), // 境界付近でのゆらぎ強度
                             "Distance2", DoubleValue (50.0),
                             "m0", DoubleValue (1.0),
                             "m1", DoubleValue (0.75),
                             "m2", DoubleValue (0.5));
  phy.SetChannel (channel.Create ());
  WifiMacHelper mac; mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (phy, mac, allNodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (uint32_t i = 0; i < nNodes; ++i) positionAlloc->Add (Vector (dist, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  InternetStackHelper stack; stack.Install (allNodes);
  Ipv4AddressHelper address; address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);
  Ipv4StaticRoutingHelper staticRouting;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i) {
    staticRouting.GetStaticRouting (allNodes.Get (i)->GetObject<Ipv4> ())->SetDefaultMulticastRoute (1);
  }

  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (Ipv4Address ("224.1.1.1"), 1024, kChannels, Seconds(interval), mode);
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds(interval));

  uint32_t totalSegs = serverApp->GetTotalSegments();
  std::string modeName = (mode == 0) ? "FB法" : (mode == 1 ? "集中型巡回放送" : "等分割法");

  // --- ネットワーク設定の表示 ---
  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("【実験設定：シミュレーションパラメータ】");
  NS_LOG_UNCOND ("配信手法        : " << modeName);
  NS_LOG_UNCOND ("ノード数        : " << nNodes);
  NS_LOG_UNCOND ("チャネル数 (k)  : " << kChannels);
  NS_LOG_UNCOND ("総セグメント数(N): " << totalSegs);
  NS_LOG_UNCOND ("スロット間隔    : " << interval << "s");
  NS_LOG_UNCOND ("マルチキャストIP: 224.1.1.1");
  NS_LOG_UNCOND ("------------------------------------------------");
  NS_LOG_UNCOND ("【無線設定】");
  NS_LOG_UNCOND ("Wi-Fi規格       : IEEE 802.11b (2.4GHz帯)");
  NS_LOG_UNCOND ("データレート    : " << dataRate);
  NS_LOG_UNCOND ("物理層モデル    : YansWifiPhy");
  NS_LOG_UNCOND ("距離 (S-Node)   : " << dist << "m");
  NS_LOG_UNCOND ("================================================");

  for (uint32_t i = 0; i < nNodes; ++i) {
    Ptr<ClientApp> clientApp = CreateObject<ClientApp> ();
    clientApp->Setup (totalSegs);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (2.0 + i * 0.01)); 
  }

  Simulator::Stop (Seconds (400.0));
  Simulator::Run ();
  
  // --- 追加：全ノードから受信状況を回収 ---
  double totalLossRateSum = 0;
  NS_LOG_UNCOND ("\n==== 個別ノード受信状況報告 ====");
  for (uint32_t i = 0; i < clientNodes.GetN(); ++i) {
      Ptr<ClientApp> app = DynamicCast<ClientApp>(clientNodes.Get(i)->GetApplication(0));
      // ここで直接メンバ変数にアクセスできない場合は、ClientAppにGetterを追加するか、
      // ログが出るのを待つ代わりにここで計算します。
      app->StopApplication();

      uint32_t received = app->GetReceivedCount();
      double lossRate = (1.0 - (double)received / totalSegs) * 100;
      totalLossRateSum += lossRate;
  }

  // 平均計算
  double sum = 0;
  for (double t : g_waitTimes) sum += t;
  double avg = g_waitTimes.empty() ? 0 : sum / g_waitTimes.size();

  // 平均欠損率の計算
  double avgLoss = totalLossRateSum / nNodes;
  uint32_t actualk = (mode == 2) ? totalSegs : kChannels;

  // Excel用最終出力
  NS_LOG_UNCOND ("\n" << "=== 最終結果サマリー (Excel用) ===");
  NS_LOG_UNCOND ("手法,ノード数,チャネル数(k),総セグメント数(N),帯域幅,平均待ち時間,平均欠損率");
  NS_LOG_UNCOND (modeName << "," << nNodes << "," << actualk << "," << totalSegs << "," << dataRate << "," << avg << "," << avgLoss << "%");

  Simulator::Destroy ();
  return 0;
}

