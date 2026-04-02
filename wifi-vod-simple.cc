#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include <map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiVoDExample");

// --- クライアントアプリ ---
class VoDClientApp : public Application {
public:
  VoDClientApp () : m_receivedCount (0), m_allReceived (false) {}
  void Setup (Ipv4Address serverAddr, uint32_t totalSegments) {
    m_serverAddr = serverAddr;
    m_totalSegments = totalSegments;
    for (uint32_t i = 0; i < totalSegments; ++i) m_receivedMap[9001 + i] = false;
  }
private:
  void HandleRead (Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {
      Address localAddress;
      // 修正ポイント：GetSockNameは引数にAddressを渡して受け取る形式
      socket->GetSockName (localAddress); 
      uint16_t port = InetSocketAddress::ConvertFrom (localAddress).GetPort ();

      if (m_receivedMap.count (port) && !m_receivedMap[port]) {
        m_receivedMap[port] = true;
        m_receivedCount++;
        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          NS_LOG_UNCOND ("*** Node " << GetNode()->GetId() << " 完了! (待ち時間: " 
                         << (Simulator::Now() - m_startTime).GetSeconds() << "s) ***");
        }
      }
    }
  }
  virtual void StartApplication (void) {
    m_startTime = Simulator::Now ();
    NS_LOG_UNCOND ("Node " << GetNode()->GetId() << ": リクエストを送信します");

    for (uint32_t i = 0; i < m_totalSegments; ++i) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9001 + i));
      sock->SetRecvCallback (MakeCallback (&VoDClientApp::HandleRead, this));
      m_sockets.push_back (sock);
    }

    Ptr<Socket> reqSock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    reqSock->Connect (InetSocketAddress (m_serverAddr, 9999));
    reqSock->Send (Create<Packet> (100)); 
  }
  Ipv4Address m_serverAddr; 
  uint32_t m_totalSegments; 
  uint32_t m_receivedCount;
  bool m_allReceived; 
  Time m_startTime;
  std::map<uint16_t, bool> m_receivedMap; 
  std::vector<Ptr<Socket>> m_sockets;
};

// --- サーバーアプリ ---
class VoDServerApp : public Application {
public:
  VoDServerApp () {}
  void Setup (uint32_t packetSize, uint32_t totalSegments, Time interval) {
    m_packetSize = packetSize; 
    m_totalSegments = totalSegments; 
    m_interval = interval;
  }

  // クライアントからの要求を受け取った時の処理
  void HandleRequest (Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {
      Ipv4Address clientIp = InetSocketAddress::ConvertFrom (from).GetIpv4 ();
      NS_LOG_UNCOND ("サーバー: Node " << clientIp << " からの要求を確認。配信開始...");
      for (uint32_t i = 0; i < m_totalSegments; ++i) {
        // 各セグメントの送信開始時間を少しずつずらしてスケジューリング
        Simulator::Schedule (Seconds (i * 0.05), &VoDServerApp::SendToClient, this, i, from);
      }
    }
  }

  // クライアントへ定期的にデータを送信する処理
  void SendToClient (uint32_t index, Address dest) {
    m_sendSockets[index]->SendTo (Create<Packet> (m_packetSize), 0, dest);
    Simulator::Schedule (m_interval, &VoDServerApp::SendToClient, this, index, dest);
  }

private:
  virtual void StartApplication (void) {
    Ptr<Socket> recvSock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    recvSock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9999));
    recvSock->SetRecvCallback (MakeCallback (&VoDServerApp::HandleRequest, this));

    for (uint32_t i = 0; i < m_totalSegments; ++i) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_sendSockets.push_back (sock);
    }
  }

  uint32_t m_packetSize; 
  uint32_t m_totalSegments; 
  Time m_interval;
  std::vector<Ptr<Socket>> m_sendSockets;
}; // ここで正しくクラスを閉じます

// --- Main関数 ---
int main (int argc, char *argv[]) {
  NodeContainer serverNode; serverNode.Create (1);
  NodeContainer clientNodes; clientNodes.Create (3);
  NodeContainer allNodes = NodeContainer (serverNode, clientNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (phy, mac, allNodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));   
  positionAlloc->Add (Vector (50.0, 0.0, 0.0));  
  positionAlloc->Add (Vector (100.0, 0.0, 0.0)); 
  positionAlloc->Add (Vector (130.0, 0.0, 0.0)); 
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  InternetStackHelper stack;
  stack.Install (allNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  Ptr<VoDServerApp> serverApp = CreateObject<VoDServerApp> ();
  serverApp->Setup (1024, 30, Seconds (1.0));
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds (1.0));

  double joinTimes[] = {2.0, 7.0, 12.0};
  for (uint32_t i = 0; i < 3; ++i) {
    Ptr<VoDClientApp> clientApp = CreateObject<VoDClientApp> ();
    clientApp->Setup (interfaces.GetAddress (0), 30);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (joinTimes[i]));
  }

  Simulator::Stop (Seconds (100.0));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
