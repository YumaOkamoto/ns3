#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("WifiSegmentedBroadcastPlaybackFixed");
namespace
{
constexpr uint16_t RECEIVE_PORT = 9001;
const Ipv4Address MULTICAST_GROUP ("224.1.1.1");
uint32_t g_completedNodes = 0;
uint32_t g_totalNodes = 0;
}

enum class DeliveryMethod : uint32_t
{
  FB = 0,
  SIMPLE = 1,
  FULL = 2
};

static std::string MethodName (DeliveryMethod method)
{
  switch (method)
  {
    case DeliveryMethod::FB: return "FB";
    case DeliveryMethod::SIMPLE: return "Simple";
    case DeliveryMethod::FULL: return "Full";
  }
  return "Unknown";
}

static std::string MethodDisplayName (DeliveryMethod method)
{
  switch (method)
  {
    case DeliveryMethod::FB: return "FB法";
    case DeliveryMethod::SIMPLE: return "単純放送";
    case DeliveryMethod::FULL: return "等分割法";
  }
  return "Unknown";
}

class SegmentPacketHeader : public Header
{
public:
  SegmentPacketHeader () : m_segmentId (0), m_packetIndex (0), m_packetsInSegment (0) {}
  SegmentPacketHeader (uint32_t segmentId, uint32_t packetIndex, uint32_t packetsInSegment)
      : m_segmentId (segmentId), m_packetIndex (packetIndex), m_packetsInSegment (packetsInSegment) {}

  static TypeId GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::SegmentPacketHeader")
      .SetParent<Header> ()
      .AddConstructor<SegmentPacketHeader> ();
    return tid;
  }
  TypeId GetInstanceTypeId () const override { return GetTypeId (); }
  uint32_t GetSerializedSize () const override { return 12; }
  void Serialize (Buffer::Iterator i) const override
  {
    i.WriteHtonU32 (m_segmentId); i.WriteHtonU32 (m_packetIndex); i.WriteHtonU32 (m_packetsInSegment);
  }
  uint32_t Deserialize (Buffer::Iterator i) override
  {
    m_segmentId = i.ReadNtohU32 (); m_packetIndex = i.ReadNtohU32 (); m_packetsInSegment = i.ReadNtohU32 (); return 12;
  }
  void Print (std::ostream& os) const override
  {
    os << "seg=" << m_segmentId << " pkt=" << m_packetIndex << "/" << m_packetsInSegment;
  }
  uint32_t GetSegmentId () const { return m_segmentId; }
  uint32_t GetPacketIndex () const { return m_packetIndex; }
  uint32_t GetPacketsInSegment () const { return m_packetsInSegment; }
private:
  uint32_t m_segmentId, m_packetIndex, m_packetsInSegment;
};
NS_OBJECT_ENSURE_REGISTERED (SegmentPacketHeader);

struct PacketDescriptor
{
  uint32_t segmentId = 0;
  uint32_t packetIndex = 0;
  uint32_t packetsInSegment = 0;
  uint32_t payloadBytes = 0;
};

class ServerApp : public Application
{
public:
  ServerApp () : m_totalSegments(0),m_channelRateBps(1e6),m_channelOffset(MilliSeconds(1)),m_running(false),m_sentCount(0),m_totalVideoBytes(0){}
  void Setup(Ipv4Address peer,uint32_t payloadSize,uint32_t numChannels,double videoDuration,double videoBitrateMbps,double channelRateMbps,Time channelOffset,Time measurementStart,DeliveryMethod method)
  {
    m_peer=peer;m_payloadSize=payloadSize;m_channelRateBps=channelRateMbps*1e6;m_channelOffset=channelOffset;m_measurementStart=measurementStart;m_totalSegments=(1u<<numChannels)-1u;
    m_totalVideoBytes=static_cast<uint64_t>(std::llround(videoDuration*videoBitrateMbps*1e6/8.0)); NS_ABORT_MSG_IF(m_totalVideoBytes<m_totalSegments,"Video too small");
    m_packetsPerSegment.assign(m_totalSegments+1,0); std::vector<std::vector<PacketDescriptor>> seg(m_totalSegments+1);
    uint64_t base=m_totalVideoBytes/m_totalSegments,rem=m_totalVideoBytes%m_totalSegments;
    for(uint32_t s=1;s<=m_totalSegments;++s){uint64_t bytes=base+(s<=rem?1:0);uint32_t count=static_cast<uint32_t>((bytes+payloadSize-1)/payloadSize);m_packetsPerSegment[s]=count;uint64_t left=bytes;
      for(uint32_t p=0;p<count;++p){uint32_t b=static_cast<uint32_t>(std::min<uint64_t>(payloadSize,left));seg[s].push_back({s,p,count,b});left-=b;}}
    uint32_t actual=method==DeliveryMethod::FB?numChannels:(method==DeliveryMethod::SIMPLE?1u:m_totalSegments);m_channelPackets.assign(actual,{});m_indices.assign(actual,0);
    auto add=[&](uint32_t ch,uint32_t s){m_channelPackets[ch].insert(m_channelPackets[ch].end(),seg[s].begin(),seg[s].end());};
    if(method==DeliveryMethod::FB){uint32_t cur=1;for(uint32_t ch=0;ch<numChannels;++ch)for(uint32_t j=0;j<(1u<<ch);++j)add(ch,cur++);}else if(method==DeliveryMethod::SIMPLE){for(uint32_t s=1;s<=m_totalSegments;++s)add(0,s);}else{for(uint32_t ch=0;ch<m_totalSegments;++ch)add(ch,ch+1);}
  }
  uint32_t GetTotalSegments()const{return m_totalSegments;} uint32_t GetLogicalChannelCount()const{return static_cast<uint32_t>(m_channelPackets.size());}
  const std::vector<uint32_t>& GetPacketsPerSegment()const{return m_packetsPerSegment;} uint64_t GetTotalVideoBytes()const{return m_totalVideoBytes;} uint64_t GetTotalSent()const{return m_sentCount;}
  void StopSending(){m_running=false;if(m_socket){m_socket->Close();m_socket=nullptr;}}
private:
  void StartApplication()override{m_running=true;m_socket=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_socket->SetPriority(7);for(uint32_t ch=0;ch<m_channelPackets.size();++ch)Simulator::Schedule(Seconds(m_channelOffset.GetSeconds()*ch),&ServerApp::SendPacket,this,ch);}
  void StopApplication()override{StopSending();}
  void SendPacket(uint32_t ch){if(!m_running||!m_socket)return;PacketDescriptor d=m_channelPackets[ch][m_indices[ch]];Ptr<Packet> p=Create<Packet>(d.payloadBytes);p->AddHeader(SegmentPacketHeader(d.segmentId,d.packetIndex,d.packetsInSegment));bool measured=Simulator::Now()>=m_measurementStart;if(m_socket->SendTo(p,0,InetSocketAddress(m_peer,RECEIVE_PORT))>=0&&measured)++m_sentCount;m_indices[ch]=(m_indices[ch]+1)%m_channelPackets[ch].size();double sec=static_cast<double>(d.payloadBytes)*8.0/m_channelRateBps;Simulator::Schedule(Seconds(sec),&ServerApp::SendPacket,this,ch);}
  Ipv4Address m_peer;uint32_t m_payloadSize=1024,m_totalSegments;double m_channelRateBps;Time m_channelOffset;Time m_measurementStart;bool m_running;Ptr<Socket> m_socket;
  std::vector<std::vector<PacketDescriptor>> m_channelPackets;std::vector<uint32_t> m_indices,m_packetsPerSegment;uint64_t m_sentCount,m_totalVideoBytes;
};

class ClientApp : public Application
{
public:
  ClientApp () : m_totalSegments (0), m_receivedUnique (0), m_rawReceived (0),
      m_segmentDuration (Seconds (1)), m_playbackStarted (false), m_playbackComplete (false),
      m_stalled (false), m_currentSegment (0), m_initialWait (0), m_totalStall (0),
      m_maxStall (0), m_stallCount (0), m_finishFromJoin (0), m_verbose (false),
      m_drainTime (Seconds (0.1)) {}

  void Setup (const std::vector<uint32_t>& packetsPerSegment, Time segmentDuration,
              bool verbose, Time drainTime, Callback<void> allCompletedCallback)
  {
    m_packetsPerSegment = packetsPerSegment;
    m_totalSegments = static_cast<uint32_t> (packetsPerSegment.size () - 1);
    m_segmentDuration = segmentDuration; m_verbose = verbose; m_drainTime = drainTime;
    m_allCompletedCallback = allCompletedCallback;
    m_segmentComplete.assign (m_totalSegments + 1, 0);
    m_packetReceived.resize (m_totalSegments + 1);
    m_packetCounts.assign (m_totalSegments + 1, 0);
    for (uint32_t s = 1; s <= m_totalSegments; ++s)
      m_packetReceived[s].assign (m_packetsPerSegment[s], 0);
  }

  uint32_t GetReceivedCount () const { return m_receivedUnique; }
  uint64_t GetRawReceivedCount () const { return m_rawReceived; }
  bool PlaybackStarted () const { return m_playbackStarted; }
  bool PlaybackComplete () const { return m_playbackComplete; }
  double GetInitialWait () const { return m_initialWait; }
  uint32_t GetStallCount () const { return m_stallCount; }
  double GetMaxStall () const
  {
    return m_stalled ? std::max (m_maxStall, (Simulator::Now () - m_stallStart).GetSeconds ()) : m_maxStall;
  }
  double GetFinishFromJoin () const { return m_finishFromJoin; }
  double GetObservedTotalStall () const
  {
    double value = m_totalStall;
    if (m_stalled) value += (Simulator::Now () - m_stallStart).GetSeconds ();
    return value;
  }

private:
  void StartApplication () override
  {
    m_joinTime=Simulator::Now();m_socket=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());NS_ABORT_MSG_IF(m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(),RECEIVE_PORT))<0,"bind failed");
    Ptr<UdpSocket> udp=DynamicCast<UdpSocket>(m_socket);NS_ABORT_MSG_IF(!udp,"UDP socket failed");udp->MulticastJoinGroup(0,InetSocketAddress(MULTICAST_GROUP,RECEIVE_PORT));m_socket->SetRecvCallback(MakeCallback(&ClientApp::HandleRead,this));
  }
  void StopApplication()override{if(m_socket){m_socket->SetRecvCallback(MakeNullCallback<void,Ptr<Socket>>());m_socket->Close();m_socket=nullptr;}}
  void HandleRead(Ptr<Socket> socket){Ptr<Packet> p;Address from;while((p=socket->RecvFrom(from)))ProcessReceivedPacket(p);}

  void ProcessReceivedPacket (Ptr<Packet> packet)
  {
    ++m_rawReceived;
    SegmentPacketHeader h;
    if (packet->RemoveHeader (h) == 0) return;
    const uint32_t s = h.GetSegmentId (), p = h.GetPacketIndex ();
    if (s == 0 || s > m_totalSegments || p >= m_packetsPerSegment[s]) return;

    if (!m_packetReceived[s][p])
    {
      m_packetReceived[s][p] = 1;
      ++m_packetCounts[s];
      if (m_packetCounts[s] == m_packetsPerSegment[s])
      {
        m_segmentComplete[s] = 1; ++m_receivedUnique;
        if (!m_playbackStarted && s == 1) StartPlayback ();
        else if (m_stalled && s == m_currentSegment) ResumePlayback ();
      }
    }
  }

  void StartPlayback ()
  {
    if (m_playbackStarted || m_playbackComplete) return;
    m_playbackStarted = true; m_currentSegment = 1;
    m_initialWait = (Simulator::Now () - m_joinTime).GetSeconds ();
    if (m_verbose) NS_LOG_UNCOND ("Node " << GetNode ()->GetId () << " 再生開始 wait=" << m_initialWait);
    Simulator::Schedule (m_segmentDuration, &ClientApp::OnSegmentPlaybackFinished, this);
  }

  void OnSegmentPlaybackFinished ()
  {
    if (!m_playbackStarted || m_playbackComplete) return;
    if (m_currentSegment >= m_totalSegments) { FinishPlayback (); return; }
    ++m_currentSegment;
    if (m_segmentComplete[m_currentSegment])
      Simulator::Schedule (m_segmentDuration, &ClientApp::OnSegmentPlaybackFinished, this);
    else
    {
      m_stalled = true; m_stallStart = Simulator::Now (); ++m_stallCount;
      if (m_verbose) NS_LOG_UNCOND ("Node " << GetNode ()->GetId () << " 途切れ seg=" << m_currentSegment);
    }
  }

  void ResumePlayback ()
  {
    if (!m_stalled) return;
    const double stall = (Simulator::Now () - m_stallStart).GetSeconds ();
    m_totalStall += stall; m_maxStall = std::max (m_maxStall, stall); m_stalled = false;
    Simulator::Schedule (m_segmentDuration, &ClientApp::OnSegmentPlaybackFinished, this);
  }

  void FinishPlayback ()
  {
    if (m_playbackComplete) return;
    m_playbackComplete = true; m_finishFromJoin = (Simulator::Now () - m_joinTime).GetSeconds (); ++g_completedNodes;
    if (m_verbose) NS_LOG_UNCOND ("Node " << GetNode ()->GetId () << " 再生完了 " << m_finishFromJoin << "s");
    if (g_completedNodes >= g_totalNodes && !m_allCompletedCallback.IsNull ())
    {
      m_allCompletedCallback (); Simulator::Stop (Simulator::Now () + m_drainTime);
    }
  }

  Ptr<Socket> m_socket;
  uint32_t m_totalSegments, m_receivedUnique; uint64_t m_rawReceived;
  std::vector<uint32_t> m_packetsPerSegment, m_packetCounts;
  std::vector<std::vector<uint8_t>> m_packetReceived;
  std::vector<uint8_t> m_segmentComplete;
  Time m_segmentDuration, m_joinTime, m_stallStart;
  bool m_playbackStarted, m_playbackComplete, m_stalled;
  uint32_t m_currentSegment; double m_initialWait, m_totalStall, m_maxStall; uint32_t m_stallCount;
  double m_finishFromJoin; bool m_verbose; Time m_drainTime; Callback<void> m_allCompletedCallback;
};

struct SimulationConfig
{
  uint32_t nNodes=5,kChannels=6,packetSize=1024; double videoDuration=60.0,videoBitrateMbps=1.0,channelRateMbps=1.0,channelOffset=0.001,distance=30.0;
  double serverStart=1.0,clientStart=5.0,simulationTime=400.0,drainTime=0.1; std::string dataRate="DsssRate11Mbps";uint32_t seed=1,run=1;bool enablePcap=false,verbose=false;std::string outputDir="results",outputTag="";
};

struct NodeResult
{
  std::string method;
  uint32_t index = 0;
  uint32_t nodeId = 0;
  bool playbackStarted = false;
  bool playbackComplete = false;
  double initialWaitS = 0.0;
  uint32_t stallCount = 0;
  double totalStallS = 0.0;
  double maxStallS = 0.0;
  double finishFromJoinS = 0.0;
  uint32_t receivedUniqueSegments = 0;
  uint32_t totalSegments = 0;
  uint64_t rawReceivedPackets = 0;
};

struct ExperimentResult
{
  std::string method;
  uint32_t nodes = 0;
  uint32_t logicalChannels = 0;
  uint32_t totalSegments = 0;
  uint32_t startedNodes = 0;
  uint32_t completedNodes = 0;
  double completionRatePct = 0.0;
  double avgInitialWaitS = 0.0;
  double avgTotalStallS = 0.0;
  double avgStallCount = 0.0;
  double avgFinishFromJoinS = 0.0;
  uint64_t logicalSentPackets = 0;
  uint64_t attemptedPackets = 0;
  uint64_t sentPackets = 0;
  uint64_t actualReceptions = 0;
  double packetLossRatePct = 0.0;
};

struct ExperimentOutput
{
  ExperimentResult summary;
  std::vector<NodeResult> nodes;
};

static std::string CompactNumber (double value)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision (6) << value;
  std::string s = oss.str ();
  while (!s.empty () && s.back () == '0') s.pop_back ();
  if (!s.empty () && s.back () == '.') s.pop_back ();
  for (char& c : s)
  {
    if (c == '.') c = 'p';
    else if (c == '-') c = 'm';
  }
  return s;
}

ExperimentOutput RunExperiment (const SimulationConfig& config, DeliveryMethod method)
{
  // 同じ Seed / Run を各方式に設定
  RngSeedManager::SetSeed (config.seed);
  RngSeedManager::SetRun (config.run);

  NodeContainer serverNode;
  NodeContainer clientNodes;

  serverNode.Create (1);
  clientNodes.Create (config.nNodes);

  NodeContainer allNodes;
  allNodes.Add (serverNode);
  allNodes.Add (clientNodes);

  // ------------------------------------------------------------
  // Wi-Fi
  // ------------------------------------------------------------
  WifiHelper wifi;
  wifi.SetStandard (
      WIFI_STANDARD_80211b);

  // multicast/broadcastも指定レートで送信する
  wifi.SetRemoteStationManager (
      "ns3::ConstantRateWifiManager",
      "DataMode",
      StringValue (config.dataRate),
      "ControlMode",
      StringValue (config.dataRate),
      "NonUnicastMode",
      StringValue (config.dataRate));

  YansWifiChannelHelper channel;

  channel.SetPropagationDelay (
      "ns3::ConstantSpeedPropagationDelayModel");

  channel.AddPropagationLoss (
      "ns3::LogDistancePropagationLossModel",
      "Exponent",
      DoubleValue (3.0));

  channel.AddPropagationLoss (
      "ns3::NakagamiPropagationLossModel",
      "m0",
      DoubleValue (5.0),
      "m1",
      DoubleValue (5.0),
      "m2",
      DoubleValue (5.0));

  YansWifiPhyHelper phy;
  phy.SetChannel (
      channel.Create ());

  WifiMacHelper mac;

  mac.SetType (
      "ns3::AdhocWifiMac",
      "QosSupported",
      BooleanValue (true));

  NetDeviceContainer devices =
      wifi.Install (
          phy,
          mac,
          allNodes);

  // ------------------------------------------------------------
  // Mobility
  // ------------------------------------------------------------
  MobilityHelper mobility;

  Ptr<ListPositionAllocator> positionAllocator =
      CreateObject<ListPositionAllocator> ();

  // Server
  positionAllocator->Add (
      Vector (0.0, 0.0, 0.0));

  // ノード数だけの影響を見る基礎実験:
  // 全クライアントを同じ距離に配置
  for (uint32_t i = 0;
       i < config.nNodes;
       ++i)
  {
    positionAllocator->Add (
        Vector (
            config.distance,
            0.0,
            0.0));
  }

  mobility.SetPositionAllocator (
      positionAllocator);

  mobility.SetMobilityModel (
      "ns3::ConstantPositionMobilityModel");

  mobility.Install (
      allNodes);

  // ------------------------------------------------------------
  // Internet
  // ------------------------------------------------------------
  InternetStackHelper stack;
  stack.Install (allNodes);

  TrafficControlHelper trafficControl;

  trafficControl.SetRootQueueDisc (
      "ns3::FifoQueueDisc");

  trafficControl.Install (
      devices);

  Ipv4AddressHelper address;

  // /16 にして大規模ノード数にも対応
  address.SetBase (
      "10.1.0.0",
      "255.255.0.0");

  address.Assign (
      devices);

  Ipv4StaticRoutingHelper staticRouting;

  for (uint32_t i = 0;
       i < allNodes.GetN ();
       ++i)
  {
    Ptr<Ipv4> ipv4 =
        allNodes.Get (i)
            ->GetObject<Ipv4> ();

    staticRouting
        .GetStaticRouting (ipv4)
        ->SetDefaultMulticastRoute (1);
  }

  Ptr<ServerApp> serverApp=CreateObject<ServerApp>();serverApp->Setup(MULTICAST_GROUP,config.packetSize,config.kChannels,config.videoDuration,config.videoBitrateMbps,config.channelRateMbps,Seconds(config.channelOffset),Seconds(config.clientStart),method);
  serverNode.Get(0)->AddApplication(serverApp);serverApp->SetStartTime(Seconds(config.serverStart));const uint32_t totalSegments=serverApp->GetTotalSegments();double segmentDuration=config.videoDuration/totalSegments;g_totalNodes=config.nNodes;g_completedNodes=0;
  for(uint32_t i=0;i<config.nNodes;++i){Ptr<ClientApp> c=CreateObject<ClientApp>();c->Setup(serverApp->GetPacketsPerSegment(),Seconds(segmentDuration),config.verbose,Seconds(config.drainTime),MakeCallback(&ServerApp::StopSending,serverApp));clientNodes.Get(i)->AddApplication(c);c->SetStartTime(Seconds(config.clientStart));}
  if(config.enablePcap)phy.EnablePcap(config.outputDir+"/wifi_playback_fixed",devices.Get(0));
  NS_LOG_UNCOND("\n================================================");NS_LOG_UNCOND("【Wi-Fi・再生評価（固定動画）："<<MethodDisplayName(method)<<"】");NS_LOG_UNCOND("動画時間/bitrate       : "<<config.videoDuration<<"s / "<<config.videoBitrateMbps<<"Mbps");
  NS_LOG_UNCOND("動画サイズ             : "<<serverApp->GetTotalVideoBytes()/1e6<<"MB");NS_LOG_UNCOND("N / SegmentDuration    : "<<totalSegments<<" / "<<segmentDuration<<"s");NS_LOG_UNCOND("論理ch数 / rate        : "<<serverApp->GetLogicalChannelCount()<<" / "<<config.channelRateMbps<<"Mbps/ch");NS_LOG_UNCOND("放送開始/視聴参加      : "<<config.serverStart<<"s / "<<config.clientStart<<"s");NS_LOG_UNCOND("================================================");
  Simulator::Stop(Seconds(config.simulationTime));Simulator::Run();ExperimentOutput output;uint64_t totalRaw=0;double initialSum=0,stallSum=0,stallCountSum=0,finishSum=0;uint32_t started=0,completed=0;
  for(uint32_t i=0;i<clientNodes.GetN();++i){Ptr<ClientApp> app=DynamicCast<ClientApp>(clientNodes.Get(i)->GetApplication(0));NodeResult n;n.method=MethodName(method);n.index=i;n.nodeId=clientNodes.Get(i)->GetId();n.playbackStarted=app->PlaybackStarted();n.playbackComplete=app->PlaybackComplete();n.initialWaitS=app->GetInitialWait();n.stallCount=app->GetStallCount();n.totalStallS=app->GetObservedTotalStall();n.maxStallS=app->GetMaxStall();n.finishFromJoinS=app->GetFinishFromJoin();n.receivedUniqueSegments=app->GetReceivedCount();n.totalSegments=totalSegments;n.rawReceivedPackets=app->GetRawReceivedCount();output.nodes.push_back(n);totalRaw+=n.rawReceivedPackets;if(n.playbackStarted){++started;initialSum+=n.initialWaitS;stallSum+=n.totalStallS;stallCountSum+=n.stallCount;}if(n.playbackComplete){++completed;finishSum+=n.finishFromJoinS;}}
  ExperimentResult&r=output.summary;r.method=MethodName(method);r.nodes=config.nNodes;r.logicalChannels=serverApp->GetLogicalChannelCount();r.totalSegments=totalSegments;r.startedNodes=started;r.completedNodes=completed;r.completionRatePct=100.0*completed/config.nNodes;r.avgInitialWaitS=started?initialSum/started:0;r.avgTotalStallS=started?stallSum/started:0;r.avgStallCount=started?stallCountSum/started:0;r.avgFinishFromJoinS=completed?finishSum/completed:0;r.logicalSentPackets=serverApp->GetTotalSent();r.sentPackets=serverApp->GetTotalSent();r.attemptedPackets=r.sentPackets*static_cast<uint64_t>(config.nNodes);r.actualReceptions=totalRaw;r.packetLossRatePct=r.attemptedPackets?std::max(0.0,100.0*(1.0-static_cast<double>(totalRaw)/r.attemptedPackets)):0;
  NS_LOG_UNCOND("==== 再生結果: "<<MethodDisplayName(method)<<" ====");NS_LOG_UNCOND("開始/完了             : "<<started<<"/"<<completed<<" (全"<<config.nNodes<<")");NS_LOG_UNCOND("平均初期待ち時間      : "<<(started?std::to_string(r.avgInitialWaitS):"N/A")<<"s");NS_LOG_UNCOND("平均総途切れ時間      : "<<(started?std::to_string(r.avgTotalStallS):"N/A")<<"s");NS_LOG_UNCOND("平均途切れ回数        : "<<(started?std::to_string(r.avgStallCount):"N/A"));Simulator::Destroy();return output;
}

static void WriteCsv (const std::vector<ExperimentResult>& summaries,
                      const std::vector<NodeResult>& nodes,
                      const std::string& base)
{
  std::ofstream out (base + "_summary.csv");
  NS_ABORT_MSG_IF (!out, "Cannot open summary CSV.");
  out << "Method,Nodes,LogicalChannels,Segments,Started,Completed,CompletionRatePct,AvgInitialWaitS,AvgTotalStallS,AvgStallCount,AvgFinishFromJoinS,LogicalSent,Attempted,Sent,ActualRx,LossRatePct\n";
  for (const auto& r : summaries)
  {
    out << r.method << "," << r.nodes << "," << r.logicalChannels << "," << r.totalSegments << ","
        << r.startedNodes << "," << r.completedNodes << "," << r.completionRatePct << ","
        << r.avgInitialWaitS << "," << r.avgTotalStallS << "," << r.avgStallCount << ","
        << r.avgFinishFromJoinS << "," << r.logicalSentPackets << "," << r.attemptedPackets << ","
        << r.sentPackets << "," << r.actualReceptions << "," << r.packetLossRatePct << "\n";
  }
  out.close ();

  std::ofstream nodeOut (base + "_nodes.csv");
  NS_ABORT_MSG_IF (!nodeOut, "Cannot open node CSV.");
  nodeOut << "Method,Index,NodeId,PlaybackStarted,PlaybackComplete,InitialWaitS,StallCount,TotalStallS,MaxStallS,FinishFromJoinS,ReceivedUniqueSegments,TotalSegments,RawReceivedPackets\n";
  for (const auto& n : nodes)
  {
    nodeOut << n.method << "," << n.index << "," << n.nodeId << "," << n.playbackStarted << ","
            << n.playbackComplete << ",";
    if (n.playbackStarted) nodeOut << n.initialWaitS;
    nodeOut << "," << n.stallCount << "," << n.totalStallS << "," << n.maxStallS << ",";
    if (n.playbackComplete) nodeOut << n.finishFromJoinS;
    nodeOut << "," << n.receivedUniqueSegments << "," << n.totalSegments << "," << n.rawReceivedPackets << "\n";
  }
}

int main(int argc,char*argv[]){SimulationConfig config;CommandLine cmd;cmd.AddValue("nodes","clients",config.nNodes);cmd.AddValue("k","FB k",config.kChannels);cmd.AddValue("packetSize","video UDP payload",config.packetSize);cmd.AddValue("videoDuration","fixed video seconds",config.videoDuration);cmd.AddValue("videoBitrate","fixed video Mbps",config.videoBitrateMbps);cmd.AddValue("channelRate","per logical channel Mbps",config.channelRateMbps);cmd.AddValue("channelOffset","channel offset",config.channelOffset);cmd.AddValue("dist","distance",config.distance);cmd.AddValue("serverStart","broadcast start",config.serverStart);cmd.AddValue("clientStart","viewer join",config.clientStart);cmd.AddValue("simTime","sim time",config.simulationTime);cmd.AddValue("drainTime","drain",config.drainTime);cmd.AddValue("dataRate","802.11b rate",config.dataRate);cmd.AddValue("seed","seed",config.seed);cmd.AddValue("run","run",config.run);cmd.AddValue("pcap","pcap",config.enablePcap);cmd.AddValue("verbose","logs",config.verbose);cmd.AddValue("outputDir","output",config.outputDir);cmd.AddValue("tag","tag",config.outputTag);cmd.Parse(argc,argv);
  NS_ABORT_MSG_IF(config.nNodes==0,"nodes>=1");NS_ABORT_MSG_IF(config.kChannels==0||config.kChannels>=31,"k=1..30");NS_ABORT_MSG_IF(config.videoDuration<=0||config.videoBitrateMbps<=0||config.channelRateMbps<=0,"video settings >0");NS_ABORT_MSG_IF(config.clientStart<config.serverStart,"clientStart>=serverStart");NS_ABORT_MSG_IF(config.simulationTime<=config.clientStart,"simTime>clientStart");std::error_code ec;std::filesystem::create_directories(config.outputDir,ec);NS_ABORT_MSG_IF(ec,ec.message());std::vector<ExperimentResult> summaries;std::vector<NodeResult> nodes;for(auto m:{DeliveryMethod::FB,DeliveryMethod::SIMPLE,DeliveryMethod::FULL}){auto o=RunExperiment(config,m);summaries.push_back(o.summary);nodes.insert(nodes.end(),o.nodes.begin(),o.nodes.end());}std::ostringstream name;name<<config.outputDir<<"/wifi_playback_fixed_nodes"<<config.nNodes<<"_k"<<config.kChannels<<"_vdur"<<CompactNumber(config.videoDuration)<<"_vbr"<<CompactNumber(config.videoBitrateMbps)<<"_chrate"<<CompactNumber(config.channelRateMbps)<<"_dist"<<CompactNumber(config.distance)<<"_seed"<<config.seed<<"_run"<<config.run;if(!config.outputTag.empty())name<<"_"<<config.outputTag;WriteCsv(summaries,nodes,name.str());NS_LOG_UNCOND("\nCSV: "<<name.str()<<"_summary.csv");return 0;}
