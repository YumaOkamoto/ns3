#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include <map>
#include <vector>
#include <cmath>
#include <numeric>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiBroadcastingVariable");

std::vector<double> g_waitTimes;
uint32_t g_nNodes = 0;

// --- ClientApp ---
class ClientApp : public Application {
public:
  ClientApp () : m_receivedCount (0), m_allReceived (false) {}
  void Setup (uint32_t totalSegments) {
    m_totalSegments = totalSegments;
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
        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          Time finishTime = Simulator::Now ();
          double wait = (finishTime - m_startTime).GetSeconds();
          g_waitTimes.push_back(wait); 
          NS_LOG_UNCOND (" Node " << GetNode()->GetId() << " 完了! (待ち時間: " << wait << "s)");
          if (g_waitTimes.size() == g_nNodes) {
            NS_LOG_UNCOND ("全ノードの受信が完了しました。サマリーを表示します。");
            Simulator::Stop (Seconds(0.1)); // 0.1秒後に停止してサマリーへ進む
          }
        }
      }
    }
  }

  virtual void StartApplication (void) {
    m_startTime = Simulator::Now ();
    for (uint32_t i = 0; i < 200; ++i) { 
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
  void Setup (Ipv4Address dest, uint32_t packetSize, uint32_t numChannels, Time interval, int mode, uint32_t calculatedN) {
    m_peer = dest; m_packetSize = packetSize; m_interval = interval;
    uint32_t N_val = calculatedN; 

    if (mode == 0) { // FB法
      uint32_t cur = 1;
      for (uint32_t i = 0; i < numChannels; ++i) {
        for (uint32_t j = 0; j < std::pow(2, i); ++j) {
           if (cur <= N_val) m_channelData[i].push_back(cur++);
        }
        m_indices[i] = 0;
      }
      m_totalSegments = N_val;
    } else if (mode == 1) { // 集中型
      for (uint32_t j = 1; j <= N_val; ++j) m_channelData[0].push_back(j);
      m_indices[0] = 0; m_totalSegments = N_val;
    } else { // 等分割
      uint32_t segsPerChan = std::ceil((double)N_val / numChannels);
      uint32_t cur = 1;
      for (uint32_t i = 0; i < numChannels; ++i) {
        for (uint32_t j = 0; j < segsPerChan && cur <= N_val; ++j) {
          m_channelData[i].push_back(cur++);
        }
        m_indices[i] = 0;
      }
      m_totalSegments = N_val;
    }
  }
private:
  virtual void StartApplication (void) {
    m_running = true;
    for (auto const& [chanIdx, data] : m_channelData) {
      if (data.empty()) continue;
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_sockets_map[chanIdx] = sock;
      Simulator::Schedule (Seconds (chanIdx * 0.001), &ServerApp::SendPacket, this, chanIdx);
    }
  }
  void SendPacket (uint32_t chanIdx) {
    if (!m_running) return;
    uint32_t segId = m_channelData[chanIdx][m_indices[chanIdx]];
    // --- 【修正箇所】分割送信ロジック ---
    uint32_t mtu = 1024; // 無線で安全に送れるサイズ
    uint32_t remaining = m_packetSize;
    
    while (remaining > 0) {
      uint32_t sendSize = std::min(remaining, mtu);
      Ptr<Packet> packet = Create<Packet> (sendSize);
      
      SeqTsHeader header;
      header.SetSeq(segId); // 同じセグメントIDを付与
      packet->AddHeader(header);
      
      m_sockets_map[chanIdx]->SendTo (packet, 0, InetSocketAddress (m_peer, 9001 + chanIdx));
      remaining -= sendSize;
    }
    // ----------------------------------

    m_indices[chanIdx] = (m_indices[chanIdx] + 1) % m_channelData[chanIdx].size();
    Simulator::Schedule (m_interval, &ServerApp::SendPacket, this, chanIdx);
  }
  Ipv4Address m_peer;
  uint32_t m_packetSize;
  uint32_t m_totalSegments;
  Time m_interval;
  bool m_running;
  std::map<uint32_t, Ptr<Socket>> m_sockets_map;
  std::map<uint32_t, std::vector<uint32_t>> m_channelData;
  std::map<uint32_t, uint32_t> m_indices;
};

int main (int argc, char *argv[]) {
  uint32_t mode = 0;
  uint32_t nNodes = 30;
  uint32_t kChannels = 4;
  double videoTime = 60.0;
  double videoRate = 1000000.0;
  double interval = 1.0;
  double dist = 30.0;
  std::string dataRate = "DsssRate11Mbps";

  CommandLine cmd;
  cmd.AddValue ("mode", "0:FB, 1:Simple, 2:Full", mode);
  cmd.AddValue ("nodes", "Number of client nodes", nNodes);
  cmd.AddValue ("k", "Number of channels", kChannels);
  cmd.AddValue ("time", "Video duration (s)", videoTime);
  cmd.AddValue ("int", "Interval (s)", interval);
  cmd.AddValue ("dist", "Distance (m)", dist);
  cmd.AddValue ("rate", "Data Rate", dataRate);
  cmd.Parse (argc, argv);
  g_nNodes = nNodes;

  // --- 【厳密モデル計算】N = 2^k - 1 とし、1Mbps相当のパケットサイズを決定 ---
  uint32_t totalSegs = std::pow(2, kChannels) - 1;
  uint32_t pSize = std::ceil((videoTime * videoRate) / (totalSegs * 8.0));

  NodeContainer serverNode, clientNodes;
  serverNode.Create (1); clientNodes.Create (nNodes);
  NodeContainer allNodes = NodeContainer (serverNode, clientNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (dataRate), "ControlMode", StringValue (dataRate));
  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
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
  serverApp->Setup (Ipv4Address ("224.1.1.1"), pSize, kChannels, Seconds(interval), mode, totalSegs);
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds(interval));

  // --- 出力形式の維持 ---
  std::string modeName = (mode == 0) ? "FB法" : (mode == 1 ? "集中型巡回放送" : "等分割法");
  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("【実験設定：シミュレーションパラメータ】");
  NS_LOG_UNCOND ("配信手法        : " << modeName);
  NS_LOG_UNCOND ("ノード数        : " << nNodes);
  NS_LOG_UNCOND ("チャネル数 (k)  : " << kChannels);
  NS_LOG_UNCOND ("総セグメント数(N): " << totalSegs);
  NS_LOG_UNCOND ("スロット間隔    : " << interval << "s");
  NS_LOG_UNCOND ("動画再生時間    : " << videoTime << "s (1Mbps)");
  NS_LOG_UNCOND ("------------------------------------------------");
  NS_LOG_UNCOND ("【無線設定】");
  NS_LOG_UNCOND ("Wi-Fi規格       : IEEE 802.11b (2.4GHz帯)");
  NS_LOG_UNCOND ("データレート    : " << dataRate);
  NS_LOG_UNCOND ("距離 (S-Node)   : " << dist << "m");
  NS_LOG_UNCOND ("================================================");

  for (uint32_t i = 0; i < nNodes; ++i) {
    Ptr<ClientApp> clientApp = CreateObject<ClientApp> ();
    clientApp->Setup (totalSegs);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (2.0 + i * 0.01)); 
  }

  Simulator::Stop (Seconds (videoTime * 20.0 + 100)); // タイムアウト防止のため少し長めに設定
  Simulator::Run ();

  // 未完了ノードのチェック
  uint32_t completedNodes = g_waitTimes.size();
  NS_LOG_UNCOND ("\n--- 各ノードの最終ステータス ---");
  if (completedNodes < nNodes) {
    NS_LOG_UNCOND ("【警告】" << (nNodes - completedNodes) << " 個のノードが受信完了できませんでした。");
  } else {
    NS_LOG_UNCOND ("【成功】全ノードが正常に受信完了しました。");
  }

  double sum = 0;
  for (double t : g_waitTimes) sum += t;
  double avg = g_waitTimes.empty() ? 0 : sum / g_waitTimes.size();

  NS_LOG_UNCOND ("\n=== 最終結果サマリー (Excel用) ===");
  NS_LOG_UNCOND ("手法,ノード数,チャネル数(k),総セグメント数(N),帯域幅,平均待ち時間");
  NS_LOG_UNCOND (modeName << "," << nNodes << "," << kChannels << "," << totalSegs << "," << dataRate << "," << avg);

  Simulator::Destroy ();
  return 0;
}
