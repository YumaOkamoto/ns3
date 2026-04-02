#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include <map>
#include <vector>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiBroadcastingExample");

// --- クライアントアプリ ---
class BroadcastingClientApp : public Application {
public:
  BroadcastingClientApp () : m_receivedCount (0), m_allReceived (false) {}
  void Setup (uint32_t totalSegments) {
    m_totalSegments = totalSegments;
    for (uint32_t i = 0; i < totalSegments; ++i) m_receivedMap[9001 + i] = false;
  }
private:
  Time m_startTime;
  void HandleRead (Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {
      Address localAddress;
      socket->GetSockName (localAddress);
      uint16_t port = InetSocketAddress::ConvertFrom (localAddress).GetPort ();
      if (m_receivedMap.count (port) && !m_receivedMap[port]) {
        m_receivedMap[port] = true;
        m_receivedCount++;
        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          Time finishTime = Simulator::Now ();
          NS_LOG_UNCOND ("*** Node " << GetNode()->GetId() << " 完了! ***");
          NS_LOG_UNCOND (std::fixed << std::setprecision(6)
                         << "  完了時刻: " << finishTime.GetSeconds () << "s" << std::endl
                         << "  待ち時間: " << (finishTime - m_startTime).GetSeconds () << "s");
          NS_LOG_UNCOND ("----------------------------------");
        }
      }
    }
  }
  virtual void StartApplication (void) {
    m_startTime = Simulator::Now ();
    NS_LOG_UNCOND ("Node " << GetNode()->GetId() << ": 参加 (時刻: " << std::fixed << std::setprecision(6) << m_startTime.GetSeconds() << "s)");
    for (auto const& [port, received] : m_receivedMap) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), port));
      Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (sock);
      if (udpSocket) udpSocket->MulticastJoinGroup (0, InetSocketAddress (Ipv4Address ("224.1.1.1"), port));
      sock->SetRecvCallback (MakeCallback (&BroadcastingClientApp::HandleRead, this));
      m_sockets.push_back (sock);
    }
  }
  uint32_t m_totalSegments; uint32_t m_receivedCount; bool m_allReceived;
  std::map<uint16_t, bool> m_receivedMap; std::vector<Ptr<Socket>> m_sockets;
};

// --- サーバーアプリ ---
class BroadcastingServerApp : public Application {
public:
  BroadcastingServerApp () : m_running (false) {}
  void Setup (Ipv4Address dest, uint32_t packetSize, uint32_t totalSegments, Time interval) {
    m_peer = dest; m_packetSize = packetSize; m_totalSegments = totalSegments; m_interval = interval;
  }
private:
  virtual void StartApplication (void) {
    m_running = true;
    for (uint32_t i = 0; i < m_totalSegments; ++i) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->SetAllowBroadcast (true);
      m_sockets.push_back (sock);
      Simulator::Schedule (Seconds (i * 0.1), &BroadcastingServerApp::SendPacket, this, i);
    }
  }
  void SendPacket (uint32_t index) {
    if (!m_running) return;
    m_sockets[index]->SendTo (Create<Packet> (m_packetSize), 0, InetSocketAddress (m_peer, 9001 + index));
    Simulator::Schedule (m_interval, &BroadcastingServerApp::SendPacket, this, index);
  }
  Ipv4Address m_peer; uint32_t m_packetSize; uint32_t m_totalSegments; Time m_interval;
  bool m_running; std::vector<Ptr<Socket>> m_sockets;
};

int main (int argc, char *argv[]) {
  uint32_t nClients = 3;
  uint32_t nSegments = 30;
  double videoDuration = 30.0;
  double cycleTime = videoDuration / nSegments;

  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("ネットワーク形式: 無線 (Wi-Fi 802.11b Adhoc)");
  NS_LOG_UNCOND ("セグメント数    : " << nSegments);
  NS_LOG_UNCOND ("================================================");

  NodeContainer serverNode; serverNode.Create (1);
  NodeContainer clientNodes; clientNodes.Create (nClients);
  NodeContainer allNodes = NodeContainer (serverNode, clientNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("DsssRate1Mbps"),
                                "ControlMode", StringValue ("DsssRate1Mbps"));

  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());
  phy.Set ("TxPowerStart", DoubleValue(25.0));
  phy.Set ("TxPowerEnd", DoubleValue(25.0));

  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (phy, mac, allNodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));   // Server
  positionAlloc->Add (Vector (30.0, 0.0, 0.0));  // Node 1
  positionAlloc->Add (Vector (60.0, 0.0, 0.0));  // Node 2
  positionAlloc->Add (Vector (90.0, 0.0, 0.0));  // Node 3
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  InternetStackHelper stack;
  stack.Install (allNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  Ipv4StaticRoutingHelper staticRouting;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i) {
    Ptr<Ipv4StaticRouting> routing = staticRouting.GetStaticRouting (allNodes.Get (i)->GetObject<Ipv4> ());
    routing->SetDefaultMulticastRoute (1);
  }

  Ptr<BroadcastingServerApp> serverApp = CreateObject<BroadcastingServerApp> ();
  serverApp->Setup (Ipv4Address ("224.1.1.1"), 1024, nSegments, Seconds (cycleTime));
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds (1.0));

  double joinTimes[] = {2.0, 7.0, 12.0};
  for (uint32_t i = 0; i < nClients; ++i) {
    Ptr<BroadcastingClientApp> clientApp = CreateObject<BroadcastingClientApp> ();
    clientApp->Setup (nSegments);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (joinTimes[i]));
  }

  // --- ノード配置情報の表示 ---
  NS_LOG_UNCOND ("\n--- ノード配置情報 ---");
  Ptr<MobilityModel> serverMob = allNodes.Get (0)->GetObject<MobilityModel> ();
  for (uint32_t i = 1; i < allNodes.GetN (); ++i) {
    Ptr<MobilityModel> clientMob = allNodes.Get (i)->GetObject<MobilityModel> ();
    double distance = clientMob->GetDistanceFrom (serverMob);
    NS_LOG_UNCOND ("Node " << i << ": 距離 " << distance << " m");
  }
  NS_LOG_UNCOND ("----------------------\n");

  NS_LOG_UNCOND ("無線シミュレーションを開始します...");
  Simulator::Stop (Seconds (100.0));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_UNCOND ("シミュレーションが終了しました。");

  return 0;
}
