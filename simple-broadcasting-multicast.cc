#include "ns4/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/mobility-module.h"

#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MulticastBroadcasting");

// --- 受信アプリ ---
class BroadcastingClientApp : public Application {
public:
  BroadcastingClientApp () : m_receivedCount (0), m_allReceived (false) {}
  void Setup (uint32_t totalSegments) {
    m_totalSegments = totalSegments;
    for (uint32_t i = 0; i < totalSegments; ++i) m_receivedMap[9001 + i] = false;
  }
private:
  // ---　参加時刻を保存する変数を追加 ---
  Time m_startTime;
  void HandleRead (Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {
      Address localAddress;
      socket->GetSockName (localAddress);
      uint16_t port = InetSocketAddress::ConvertFrom (localAddress).GetPort ();

      /*
      // デバッグログ：パケットが届いたら必ず表示
      NS_LOG_UNCOND ("Node " << GetNode()->GetId() << ": ポート " << port << " からパケット受信");
      */
      if (m_receivedMap.count (port) && !m_receivedMap[port]) {
        m_receivedMap[port] = true;
        m_receivedCount++;
        if (m_receivedCount == m_totalSegments && !m_allReceived) {
          m_allReceived = true;
          Time finishTime = Simulator::Now ();
          // --- ここで参加時刻と待ち時間を出力 ---
          NS_LOG_UNCOND ("*** Node " << GetNode()->GetId() << " 完了! *** ");
          NS_LOG_UNCOND (std::fixed << std::setprecision(6) // 桁数を固定
                 << "  参加時刻: " << m_startTime.GetSeconds () << "s" << std::endl
                 << "  完了時刻: " << finishTime.GetSeconds () << "s" << std::endl
                 << "  待ち時間: " << (finishTime - m_startTime).GetSeconds () << "s");
          NS_LOG_UNCOND ("----------------------------------");
        }
      }
    }
  }
  virtual void StartApplication (void) {
    // --- アプリが起動した(参加した)時刻を記録 ---
    m_startTime = Simulator::Now ();
    NS_LOG_UNCOND("Node " << GetNode()->GetId() << ": シミュレーションに参加しました (時刻: " << std::fixed << std::setprecision(6) << m_startTime.GetSeconds () << "s)");

    for (auto const& [port, received] : m_receivedMap) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), port));
      Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (sock);
      if (udpSocket) {
        udpSocket->MulticastJoinGroup (0, InetSocketAddress (Ipv4Address ("224.1.1.1"), port));
      }
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
  uint32_t nClients = 3;
  uint32_t nSegments = 3;
  double videoDuration = 30.0;
  double cycleTime = videoDuration / nSegments;
  double errorRate = 0.2; 

  // シミュレーション設定の表示
  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("分割法  : 単純分割法 ");
  NS_LOG_UNCOND ("ビデオ総時間  : " << videoDuration << " 秒");
  NS_LOG_UNCOND ("セグメント数  : " << nSegments << " 分割");
  NS_LOG_UNCOND ("セグメント間隔: " << cycleTime << " 秒 (1サイクル)");
  NS_LOG_UNCOND ("パケットロス率: "<< std::fixed << std::setprecision(2) << (errorRate * 100) << " %");
  NS_LOG_UNCOND ("================================================");

  NodeContainer serverNode; serverNode.Create (1);
  NodeContainer clientNodes; clientNodes.Create (nClients);
  NodeContainer allNodes = NodeContainer (serverNode, clientNodes);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("10Mbps"));
  csma.SetChannelAttribute ("Delay", StringValue ("2ms"));
  NetDeviceContainer devices = csma.Install (allNodes);

  //パケット欠損設定
 Ptr<RateErrorModel> em = CreateObject<RateErrorModel> ();
  em->SetAttribute ("ErrorRate", DoubleValue (errorRate)); // 10%の確率でロス
  em->SetUnit (RateErrorModel::ERROR_UNIT_PACKET);
  for (uint32_t i = 0; i < devices.GetN (); ++i) {
    devices.Get (i)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
  }

  InternetStackHelper stack;
  stack.Install (allNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);
/* 
  // --- 重要：マルチキャストルーティングの設定 ---
  Ipv4StaticRoutingHelper staticRouting;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i) {
    Ptr<Ipv4StaticRouting> multicast = staticRouting.GetStaticRouting (allNodes.Get (i)->GetObject<Ipv4> ());
    // インターフェース 1 (CSMA) を通じてマルチキャスト 224.1.1.1 を送受信可能にする
    // 出力インタフェースのリストを作成(1番目のインタフェース=CSMA)
    std::vector<uint32_t> outputInterfaces;
    outputInterfaces.push_back(1);
    multicast->AddMulticastRoute (interfaces.GetAddress(0), Ipv4Address("224.1.1.1"), 1,outputInterfaces);
  }
*/
  
  // --- これまでのルーティング設定を消して、これに差し替え ---
  Ipv4StaticRoutingHelper staticRouting;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i) {
    Ptr<Ipv4> ipv4 = allNodes.Get (i)->GetObject<Ipv4> ();
    Ptr<Ipv4StaticRouting> routing = staticRouting.GetStaticRouting (ipv4);
    // 全インターフェースでマルチキャストパケットを「とりあえず外へ出す」設定
    routing->SetDefaultMulticastRoute (1);
  }

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (50.0, 10.0, 0.0));
  for (uint32_t i = 0; i < nClients; ++i) positionAlloc->Add (Vector (20.0 + (i * 30.0), 50.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  Ipv4Address multicastAddr ("224.1.1.1");
  for (uint32_t i = 0; i < nSegments; ++i) {
    Ptr<Socket> sock = Socket::CreateSocket (serverNode.Get (0), UdpSocketFactory::GetTypeId ());
    Ptr<BroadcastingServerApp> app = CreateObject<BroadcastingServerApp> ();
    app->Setup (sock, InetSocketAddress (multicastAddr, 9001 + i), 1024, Seconds (videoDuration / nSegments));
    serverNode.Get (0)->AddApplication (app);
    app->SetStartTime (Seconds (1.0 + (i * 0.1)));
  }

  for (uint32_t i = 0; i < nClients; ++i) {
    Ptr<BroadcastingClientApp> clientApp = CreateObject<BroadcastingClientApp> ();
    clientApp->Setup (nSegments);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (2.0 + (i * 5.0))); 
  }

  AnimationInterface anim ("broadcasting-multicast.xml");
  anim.SetMaxPktsPerTraceFile (1000000);

  NS_LOG_UNCOND ("シミュレーションを開始します...");
  Simulator::Stop (Seconds (40.0));
  Simulator::Run ();
  NS_LOG_UNCOND ("シミュレーションが終了しました。");
  Simulator::Destroy ();
  return 0;
}
