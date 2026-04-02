#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include <map>
#include <vector>
#include <iomanip>
#include <cmath> 

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiBroadcastingClassicalFB");

// --- ClientApp ---
class ClientApp : public Application {
public:
  ClientApp () : m_receivedCount (0), m_allReceived (false) {}
  void Setup (uint32_t totalSegments) {
    m_totalSegments = totalSegments;
    for (uint32_t i = 1; i <= totalSegments; ++i) m_receivedMap[i] = false;
  }
private:
  Time m_startTime;
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
          NS_LOG_UNCOND (" Node " << GetNode()->GetId() << " 完了!  ");
          NS_LOG_UNCOND (std::fixed << std::setprecision(6)
                         << "   完了時刻: " << finishTime.GetSeconds () << "s" << std::endl
                         << "   待ち時間: " << (finishTime - m_startTime).GetSeconds () << "s");
          NS_LOG_UNCOND ("----------------------------------");
        }
      }
    }
  }
  virtual void StartApplication (void) {
    m_startTime = Simulator::Now ();
    NS_LOG_UNCOND ("Node " << GetNode()->GetId() << ": 参加 (時刻: " << m_startTime.GetSeconds() << "s)");
    for (uint32_t i = 0; i < 10; ++i) { 
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9001 + i));
      Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (sock);
      if (udpSocket) udpSocket->MulticastJoinGroup (0, InetSocketAddress (Ipv4Address ("224.1.1.1"), 9001 + i));
      sock->SetRecvCallback (MakeCallback (&ClientApp::HandleRead, this));
      m_sockets.push_back (sock);
    }
  }
  uint32_t m_totalSegments; uint32_t m_receivedCount; bool m_allReceived;
  std::map<uint32_t, bool> m_receivedMap; std::vector<Ptr<Socket>> m_sockets;
};

// --- ServerApp ---
class ServerApp : public Application {
public:
  ServerApp () : m_running (false) {}
  void Setup (Ipv4Address dest, uint32_t packetSize, uint32_t numChannels, Time interval) {
    m_peer = dest; m_packetSize = packetSize; m_interval = interval;
    
    uint32_t currentSegId = 1; 
    
    for (uint32_t i = 0; i < numChannels; ++i) {
      uint32_t numInThisChannel = std::pow(2, i); 
      std::vector<uint32_t> segs;
      for (uint32_t j = 0; j < numInThisChannel; ++j) {
        segs.push_back(currentSegId++);
      }
      m_channelData[i] = segs;
      m_indices[i] = 0; 
    }
    m_totalSegments = currentSegId - 1; 
  }
  uint32_t GetTotalSegments() { return m_totalSegments; }

private:
  virtual void StartApplication (void) {
    m_running = true;
    for (uint32_t i = 0; i < m_channelData.size(); ++i) {
      Ptr<Socket> sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      sock->SetAllowBroadcast (true);
      m_sockets.push_back (sock);
      Simulator::Schedule (Seconds (i * 0.01), &ServerApp::SendPacket, this, i);
    }
  }
  void SendPacket (uint32_t chanIdx) {
    if (!m_running) return;

    uint32_t listIdx = m_indices[chanIdx];
    uint32_t segmentId = m_channelData[chanIdx][listIdx];

    Ptr<Packet> packet = Create<Packet> (m_packetSize);
    SeqTsHeader header;
    header.SetSeq(segmentId); 
    packet->AddHeader(header);

    m_sockets[chanIdx]->SendTo (packet, 0, InetSocketAddress (m_peer, 9001 + chanIdx));
    m_indices[chanIdx] = (listIdx + 1) % m_channelData[chanIdx].size();

    Simulator::Schedule (m_interval, &ServerApp::SendPacket, this, chanIdx);
  }
  Ipv4Address m_peer; uint32_t m_packetSize; uint32_t m_totalSegments; Time m_interval;
  bool m_running; std::vector<Ptr<Socket>> m_sockets;
  std::map<uint32_t, std::vector<uint32_t>> m_channelData;
  std::map<uint32_t, uint32_t> m_indices;
};

int main (int argc, char *argv[]) {
  uint32_t nClients = 3;
  uint32_t kChannels = 5; 
  Time baseInterval = Seconds (0.1); 

  NodeContainer serverNode; serverNode.Create (1);
  NodeContainer clientNodes; clientNodes.Create (nClients);
  NodeContainer allNodes = NodeContainer (serverNode, clientNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("DsssRate1Mbps"), "ControlMode", StringValue ("DsssRate1Mbps"));
  YansWifiPhyHelper phy;
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());
  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (phy, mac, allNodes);
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));  // Server
  positionAlloc->Add (Vector (30.0, 0.0, 0.0)); // Node 1
  positionAlloc->Add (Vector (30.0, 0.0, 0.0)); // Node 2
  positionAlloc->Add (Vector (30.0, 0.0, 0.0)); // Node 3
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

  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (Ipv4Address ("224.1.1.1"), 1024, kChannels, baseInterval);
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds (1.0));

  uint32_t totalSegs = serverApp->GetTotalSegments();
  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("スケジューリング手法: FB法 ");
  NS_LOG_UNCOND ("チャネル数(k)    : " << kChannels);
  NS_LOG_UNCOND ("総セグメント数(N): " << totalSegs);
  NS_LOG_UNCOND ("------------------------------------------------");
  NS_LOG_UNCOND ("Wi-Fi規格: 802.11b");
  NS_LOG_UNCOND ("データレート: DsssRate1Mbps");
  
  Ptr<MobilityModel> serverMobility = serverNode.Get(0)->GetObject<MobilityModel>();
  Vector serverPos = serverMobility->GetPosition();
  
  for (uint32_t i = 0; i < clientNodes.GetN(); ++i) {
    Ptr<MobilityModel> clientMobility = clientNodes.Get(i)->GetObject<MobilityModel>();
    Vector clientPos = clientMobility->GetPosition();
    double distance = CalculateDistance(serverPos, clientPos);
    NS_LOG_UNCOND ("Server <-> Node " << i + 1 << " 距離: " << distance << "m");
  }
  NS_LOG_UNCOND ("================================================");

  double joinTimes[] = {2.0, 7.0, 12.0};
  for (uint32_t i = 0; i < nClients; ++i) {
    Ptr<ClientApp> clientApp = CreateObject<ClientApp> ();
    clientApp->Setup (totalSegs);
    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (joinTimes[i]));
  }

  Simulator::Stop (Seconds (100.0));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
