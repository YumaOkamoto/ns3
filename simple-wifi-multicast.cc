#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiMulticastBroadcasting");

// --- 受信アプリ ---
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
        NS_LOG_UNCOND ("Node " << GetNode()->GetId() << ": セグメント " << (port-9000) << " 受信");
        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          NS_LOG_UNCOND ("*** Node " << GetNode()->GetId() << " 完了! (待ち時間: " << (Simulator::Now() - m_startTime).GetSeconds() << "s) ***");
        }
      }
    }
  }
  virtual void StartApplication (void) {
    m_startTime = Simulator::Now ();
    NS_LOG_UNCOND ("Node " << GetNode()->GetId() << ": 参加 (" << m_startTime.GetSeconds() << "s)");
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

// --- 送信アプリ ---
class BroadcastingServerApp : public Application {
public:
  BroadcastingServerApp () : m_running (false) {}
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, Time interval) {
    m_socket = socket; m_peer = address; m_packetSize = packetSize; m_interval = interval;
  }
private:
  virtual void StartApplication (void) { m_running = true; SendPacket (); }
  void SendPacket (void) {
    m_socket->SendTo (Create<Packet> (m_packetSize), 0, m_peer);
    if (m_running) m_sendEvent = Simulator::Schedule (m_interval, &BroadcastingServerApp::SendPacket, this);
  }
  Ptr<Socket> m_socket; Address m_peer; uint32_t m_packetSize; Time m_interval; bool m_running; EventId m_sendEvent;
};

int main (int argc, char *argv[]) {
  NodeContainer allNodes; allNodes.Create (4);
  NodeContainer serverNode = allNodes.Get (0);
  NodeContainer clientNodes = NodeContainer (allNodes.Get (1), allNodes.Get (2), allNodes.Get (3));

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("DsssRate1Mbps"), "ControlMode", StringValue ("DsssRate1Mbps"));

  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());
  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (phy, mac, allNodes);

  InternetStackHelper stack;
  stack.Install (allNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  // 【重要】静的マルチキャストルーティングの設定
  // これがないと、無線アダプタはマルチキャストパケットを「どこに送ればいいか不明」として破棄します
  Ipv4StaticRoutingHelper staticRouting;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i) {
    Ptr<Ipv4StaticRouting> routing = staticRouting.GetStaticRouting (allNodes.Get (i)->GetObject<Ipv4> ());
    // インターフェース 1 (Wi-Fi) を通じて 224.0.0.0/4 (マルチキャスト全般) を送受信可能にする
    routing->AddMulticastRoute (Ipv4Address ("10.1.1.1"), Ipv4Address ("224.1.1.1"), 1, {1});
  }

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  positionAlloc->Add (Vector (15.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  for (uint32_t i = 0; i < 3; ++i) {
    Ptr<Socket> sock = Socket::CreateSocket (serverNode.Get (0), UdpSocketFactory::GetTypeId ());
    sock->BindToNetDevice (devices.Get (0));
    Ptr<BroadcastingServerApp> app = CreateObject<BroadcastingServerApp> ();
    app->Setup (sock, InetSocketAddress (Ipv4Address ("224.1.1.1"), 9001 + i), 1024, Seconds (1.0));
    serverNode.Get (0)->AddApplication (app);
    app->SetStartTime (Seconds (1.0 + (i * 0.1)));
  }

  for (uint32_t i = 0; i < 3; ++i) {
    Ptr<BroadcastingClientApp> clientApp = CreateObject<BroadcastingClientApp> ();
    clientApp->Setup (3);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (2.0 + (i * 5.0))); 
  }

  NS_LOG_UNCOND ("=== 配信開始 (ルーティング修正版) ===");
  Simulator::Stop (Seconds (40.0));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
