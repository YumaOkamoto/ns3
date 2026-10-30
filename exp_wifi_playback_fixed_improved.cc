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
  SegmentPacketHeader () : m_segmentId (0), m_packetIndex (0), m_packetsInSegment (0), m_measured (0) {}
  SegmentPacketHeader (uint32_t segmentId, uint32_t packetIndex, uint32_t packetsInSegment, bool measured)
      : m_segmentId (segmentId), m_packetIndex (packetIndex), m_packetsInSegment (packetsInSegment), m_measured (measured ? 1u : 0u) {}

  static TypeId GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::SegmentPacketHeader")
      .SetParent<Header> ()
      .AddConstructor<SegmentPacketHeader> ();
    return tid;
  }
  TypeId GetInstanceTypeId () const override { return GetTypeId (); }
  uint32_t GetSerializedSize () const override { return 16; }
  void Serialize (Buffer::Iterator i) const override
  {
    i.WriteHtonU32 (m_segmentId); i.WriteHtonU32 (m_packetIndex); i.WriteHtonU32 (m_packetsInSegment); i.WriteHtonU32 (m_measured);
  }
  uint32_t Deserialize (Buffer::Iterator i) override
  {
    m_segmentId = i.ReadNtohU32 (); m_packetIndex = i.ReadNtohU32 (); m_packetsInSegment = i.ReadNtohU32 (); m_measured = i.ReadNtohU32 (); return 16;
  }
  void Print (std::ostream& os) const override
  {
    os << "seg=" << m_segmentId << " pkt=" << m_packetIndex << "/" << m_packetsInSegment;
  }
  uint32_t GetSegmentId () const { return m_segmentId; }
  uint32_t GetPacketIndex () const { return m_packetIndex; }
  uint32_t GetPacketsInSegment () const { return m_packetsInSegment; }
  bool GetMeasured () const { return m_measured != 0; }
private:
  uint32_t m_segmentId, m_packetIndex, m_packetsInSegment, m_measured;
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
  ServerApp () : m_totalSegments(0),m_channelRateBps(1e6),m_channelOffset(MilliSeconds(1)),m_running(false),m_sentCount(0),m_totalVideoBytes(0),m_sentVideoBytes(0),m_attempted(0){}
  void Setup(Ipv4Address peer,uint32_t payloadSize,uint32_t numChannels,double videoDuration,double videoBitrateMbps,double channelRateMbps,bool totalRateMode,Time channelOffset,Time measurementStart,Time measurementEnd,DeliveryMethod method)
  {
    m_peer=peer;m_payloadSize=payloadSize;m_channelRateBps=channelRateMbps*1e6;m_channelOffset=channelOffset;m_measurementStart=measurementStart;m_measurementEnd=measurementEnd;m_totalSegments=(1u<<numChannels)-1u;
    m_totalVideoBytes=static_cast<uint64_t>(std::llround(videoDuration*videoBitrateMbps*1e6/8.0)); NS_ABORT_MSG_IF(m_totalVideoBytes<m_totalSegments,"Video too small");
    m_packetsPerSegment.assign(m_totalSegments+1,0); std::vector<std::vector<PacketDescriptor>> seg(m_totalSegments+1);
    uint64_t base=m_totalVideoBytes/m_totalSegments,rem=m_totalVideoBytes%m_totalSegments;
    for(uint32_t s=1;s<=m_totalSegments;++s){uint64_t bytes=base+(s<=rem?1:0);uint32_t count=static_cast<uint32_t>((bytes+payloadSize-1)/payloadSize);m_packetsPerSegment[s]=count;uint64_t left=bytes;
      for(uint32_t p=0;p<count;++p){uint32_t b=static_cast<uint32_t>(std::min<uint64_t>(payloadSize,left));seg[s].push_back({s,p,count,b});left-=b;}}
    uint32_t actual=method==DeliveryMethod::FB?numChannels:(method==DeliveryMethod::SIMPLE?1u:m_totalSegments);m_channelPackets.assign(actual,{});m_indices.assign(actual,0);if (totalRateMode) { m_channelRateBps /= actual; }
    auto add=[&](uint32_t ch,uint32_t s){m_channelPackets[ch].insert(m_channelPackets[ch].end(),seg[s].begin(),seg[s].end());};
    if(method==DeliveryMethod::FB){uint32_t cur=1;for(uint32_t ch=0;ch<numChannels;++ch)for(uint32_t j=0;j<(1u<<ch);++j)add(ch,cur++);}else if(method==DeliveryMethod::SIMPLE){for(uint32_t s=1;s<=m_totalSegments;++s)add(0,s);}else{for(uint32_t ch=0;ch<m_totalSegments;++ch)add(ch,ch+1);}
  }
  uint32_t GetTotalSegments()const{return m_totalSegments;} uint32_t GetLogicalChannelCount()const{return static_cast<uint32_t>(m_channelPackets.size());}
  const std::vector<uint32_t>& GetPacketsPerSegment()const{return m_packetsPerSegment;} uint64_t GetTotalVideoBytes()const{return m_totalVideoBytes;} uint64_t GetTotalSent()const{return m_sentCount;} uint64_t GetSentVideoBytes()const{return m_sentVideoBytes;}
  uint64_t GetAttempted()const{return m_attempted;}
  void StopSending(){m_running=false;if(m_socket){m_socket->Close();m_socket=nullptr;}}
private:
  void StartApplication()override{m_running=true;m_socket=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_socket->SetPriority(7);for(uint32_t ch=0;ch<m_channelPackets.size();++ch)Simulator::Schedule(Seconds(m_channelOffset.GetSeconds()*ch),&ServerApp::SendPacket,this,ch);}
  void StopApplication()override{StopSending();}
  void SendPacket (uint32_t ch)
  {
    if (!m_running || !m_socket)
    {
      return;
    }
    const PacketDescriptor d = m_channelPackets[ch][m_indices[ch]];
    const bool measured = Simulator::Now () >= m_measurementStart &&
                          Simulator::Now () < m_measurementEnd;
    Ptr<Packet> packet = Create<Packet> (d.payloadBytes);
    packet->AddHeader (SegmentPacketHeader (d.segmentId, d.packetIndex,
                                           d.packetsInSegment, measured));
    if (measured)
    {
      ++m_attempted;
    }
    const int sent = m_socket->SendTo (packet, 0, InetSocketAddress (m_peer, RECEIVE_PORT));
    if (sent >= 0 && measured)
    {
      ++m_sentCount;
      m_sentVideoBytes += d.payloadBytes;
    }
    m_indices[ch] = (m_indices[ch] + 1) % m_channelPackets[ch].size ();
    const double seconds = static_cast<double> (d.payloadBytes) * 8.0 / m_channelRateBps;
    Simulator::Schedule (Seconds (seconds), &ServerApp::SendPacket, this, ch);
  }
  Ipv4Address m_peer;uint32_t m_payloadSize=1024,m_totalSegments;double m_channelRateBps;Time m_channelOffset;Time m_measurementStart;Time m_measurementEnd;bool m_running;Ptr<Socket> m_socket;
  std::vector<std::vector<PacketDescriptor>> m_channelPackets;std::vector<uint32_t> m_indices,m_packetsPerSegment;uint64_t m_sentCount,m_totalVideoBytes,m_sentVideoBytes,m_attempted;
};

#include "playback_receiver_model.h"

struct SimulationConfig
{
  uint32_t nNodes=5,kChannels=6,packetSize=1024; double startupBuffer=0.5,resumeBuffer=0.5,playbackUnit=0.1,joinSpread=0.0;std::string rateMode="perChannel",methodSelect="all";
  double videoDuration=60.0,videoBitrateMbps=1.0,channelRateMbps=1.0,channelOffset=0.001,distance=30.0;
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
  uint64_t measuredRxPackets = 0, uniquePackets = 0, duplicatePackets = 0, contiguousBytes = 0;
  double playableSeconds = 0.0, playedSeconds = 0.0, joinTime = 0.0;
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
  double packetLossRatePct = 0.0; // App delivery deficit (not PHY packet loss)
  uint64_t txPayloadBytes = 0, measuredRxPackets = 0;
  double avgPlayedSeconds = 0.0;
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

  Ptr<ServerApp> serverApp=CreateObject<ServerApp>();serverApp->Setup(MULTICAST_GROUP,config.packetSize,config.kChannels,config.videoDuration,config.videoBitrateMbps,config.channelRateMbps,config.rateMode=="total",Seconds(config.channelOffset),Seconds(config.clientStart+(config.nNodes==1?0.0:config.joinSpread)),Seconds(config.simulationTime-config.drainTime),method);
  serverNode.Get(0)->AddApplication(serverApp);serverApp->SetStartTime(Seconds(config.serverStart));const uint32_t totalSegments=serverApp->GetTotalSegments();double segmentDuration=config.videoDuration/totalSegments;for (uint32_t i = 0; i < config.nNodes; ++i)
  {
    Ptr<ClientApp> c = CreateObject<ClientApp> ();
    c->Setup (serverApp->GetPacketsPerSegment (), serverApp->GetTotalVideoBytes (), config.packetSize,
              config.videoDuration, config.startupBuffer, config.resumeBuffer, config.playbackUnit,
              true, MULTICAST_GROUP, config.verbose);
    clientNodes.Get (i)->AddApplication (c);
    const double join = config.clientStart + (config.nNodes == 1 ? 0.0 :
                          config.joinSpread * static_cast<double> (i) / (config.nNodes - 1));
    c->SetStartTime (Seconds (join));
  }
  if(config.enablePcap)phy.EnablePcap(config.outputDir+"/wifi_playback_fixed",devices.Get(0));
  NS_LOG_UNCOND("\n================================================");NS_LOG_UNCOND("【Wi-Fi・再生評価（固定動画）："<<MethodDisplayName(method)<<"】");NS_LOG_UNCOND("動画時間/bitrate       : "<<config.videoDuration<<"s / "<<config.videoBitrateMbps<<"Mbps");
  NS_LOG_UNCOND("動画サイズ             : "<<serverApp->GetTotalVideoBytes()/1e6<<"MB");NS_LOG_UNCOND("N / SegmentDuration    : "<<totalSegments<<" / "<<segmentDuration<<"s");NS_LOG_UNCOND("論理ch数 / rate        : "<<serverApp->GetLogicalChannelCount()<<" / "<<config.channelRateMbps<<"Mbps/ch");NS_LOG_UNCOND("放送開始/視聴参加      : "<<config.serverStart<<"s / "<<config.clientStart<<"s");NS_LOG_UNCOND("================================================");
  Simulator::Stop(Seconds(config.simulationTime));Simulator::Run();ExperimentOutput output;double initialSum=0,stallSum=0,stallCountSum=0,finishSum=0;uint32_t started=0,completed=0;
  for(uint32_t i=0;i<clientNodes.GetN();++i){Ptr<ClientApp> app=DynamicCast<ClientApp>(clientNodes.Get(i)->GetApplication(0));NodeResult n;n.method=MethodName(method);n.index=i;n.nodeId=clientNodes.Get(i)->GetId();n.playbackStarted=app->PlaybackStarted();n.playbackComplete=app->PlaybackComplete();n.initialWaitS=app->GetInitialWait();n.stallCount=app->GetStallCount();n.totalStallS=app->GetObservedTotalStall();n.maxStallS=app->GetMaxStall();n.finishFromJoinS=app->GetFinishFromJoin();n.receivedUniqueSegments=app->GetReceivedCount();n.totalSegments=totalSegments;n.rawReceivedPackets=app->GetRawReceivedCount();
    n.measuredRxPackets=app->GetMeasuredReceivedCount(); n.uniquePackets=app->GetUniquePacketCount();
    n.duplicatePackets=app->GetDuplicatePacketCount(); n.contiguousBytes=app->GetContiguousBytes();
    n.playableSeconds=app->GetPlayableSeconds(); n.playedSeconds=app->GetPlayedSeconds(); n.joinTime=app->GetJoinTime();
    output.nodes.push_back(n);if(n.playbackStarted){++started;initialSum+=n.initialWaitS;stallSum+=n.totalStallS;stallCountSum+=n.stallCount;}if(n.playbackComplete){++completed;finishSum+=n.finishFromJoinS;}}
  ExperimentResult&r=output.summary;r.method=MethodName(method);r.nodes=config.nNodes;r.logicalChannels=serverApp->GetLogicalChannelCount();r.totalSegments=totalSegments;r.startedNodes=started;r.completedNodes=completed;r.completionRatePct=100.0*completed/config.nNodes;r.avgInitialWaitS=started?initialSum/started:0;r.avgTotalStallS=started?stallSum/started:0;r.avgStallCount=started?stallCountSum/started:0;r.avgFinishFromJoinS=completed?finishSum/completed:0;r.logicalSentPackets=serverApp->GetTotalSent();r.sentPackets=serverApp->GetTotalSent();r.attemptedPackets=serverApp->GetAttempted();r.actualReceptions=0;for (const auto& nd : output.nodes) { r.actualReceptions += nd.measuredRxPackets; }
  r.measuredRxPackets=r.actualReceptions;r.txPayloadBytes=serverApp->GetSentVideoBytes();
  for (const auto& nd : output.nodes) { r.avgPlayedSeconds += nd.playedSeconds; }
  r.avgPlayedSeconds /= config.nNodes;
  r.packetLossRatePct=r.sentPackets?std::max(0.0,100.0*(1.0-static_cast<double>(r.actualReceptions)/(r.sentPackets*static_cast<uint64_t>(config.nNodes)))):0;
  NS_LOG_UNCOND("==== 再生結果: "<<MethodDisplayName(method)<<" ====");NS_LOG_UNCOND("開始/完了             : "<<started<<"/"<<completed<<" (全"<<config.nNodes<<")");NS_LOG_UNCOND("平均初期待ち時間      : "<<(started?std::to_string(r.avgInitialWaitS):"N/A")<<"s");NS_LOG_UNCOND("平均総途切れ時間      : "<<(started?std::to_string(r.avgTotalStallS):"N/A")<<"s");NS_LOG_UNCOND("平均途切れ回数        : "<<(started?std::to_string(r.avgStallCount):"N/A"));Simulator::Destroy();return output;
}

static void WriteCsv (const std::vector<ExperimentResult>& summaries,
                      const std::vector<NodeResult>& nodes,
                      const std::string& base)
{
  std::ofstream out (base + "_summary.csv");
  NS_ABORT_MSG_IF (!out, "Cannot open summary CSV.");
  out << "Method,Nodes,LogicalChannels,Segments,Started,Completed,CompletionRatePct,AvgInitialWaitS,AvgTotalStallS,AvgStallCount,AvgFinishFromJoinS,LogicalSent,Attempted,Sent,ActualRx,AppDeliveryDeficitPct,TxPayloadBytes,MeasuredRx,AvgPlayedSeconds\n";
  for (const auto& r : summaries)
  {
    out << r.method << "," << r.nodes << "," << r.logicalChannels << "," << r.totalSegments << ","
        << r.startedNodes << "," << r.completedNodes << "," << r.completionRatePct << ","
        << (r.startedNodes ? std::to_string (r.avgInitialWaitS) : "") << ","
        << (r.startedNodes ? std::to_string (r.avgTotalStallS) : "") << ","
        << (r.startedNodes ? std::to_string (r.avgStallCount) : "") << ","
        << (r.completedNodes ? std::to_string (r.avgFinishFromJoinS) : "") << ","
        << r.logicalSentPackets << "," << r.attemptedPackets << ","
        << r.sentPackets << "," << r.actualReceptions << "," << r.packetLossRatePct << ","
        << r.txPayloadBytes << "," << r.measuredRxPackets << "," << r.avgPlayedSeconds << "\n";
  }
  out.close ();

  std::ofstream nodeOut (base + "_nodes.csv");
  NS_ABORT_MSG_IF (!nodeOut, "Cannot open node CSV.");
  nodeOut << "Method,Index,NodeId,PlaybackStarted,PlaybackComplete,InitialWaitS,StallCount,TotalStallS,MaxStallS,FinishFromJoinS,ReceivedUniqueSegments,TotalSegments,RawReceivedPackets,MeasuredRx,UniquePackets,DuplicatePackets,ContiguousBytes,PlayableSeconds,PlayedSeconds,JoinTime\n";
  for (const auto& n : nodes)
  {
    nodeOut << n.method << "," << n.index << "," << n.nodeId << "," << n.playbackStarted << ","
            << n.playbackComplete << ",";
    if (n.playbackStarted) nodeOut << n.initialWaitS;
    nodeOut << "," << n.stallCount << "," << n.totalStallS << "," << n.maxStallS << ",";
    if (n.playbackComplete) nodeOut << n.finishFromJoinS;
    nodeOut << "," << n.receivedUniqueSegments << "," << n.totalSegments << "," << n.rawReceivedPackets << "," << n.measuredRxPackets << "," << n.uniquePackets << ","
            << n.duplicatePackets << "," << n.contiguousBytes << "," << n.playableSeconds << ","
            << n.playedSeconds << "," << n.joinTime << "\n";
  }
}

int main(int argc,char*argv[]){SimulationConfig config;CommandLine cmd;cmd.AddValue("nodes","clients",config.nNodes);cmd.AddValue("k","FB k",config.kChannels);cmd.AddValue("packetSize","video UDP payload",config.packetSize);cmd.AddValue("videoDuration","fixed video seconds",config.videoDuration);cmd.AddValue("videoBitrate","fixed video Mbps",config.videoBitrateMbps);cmd.AddValue("channelRate","per logical channel Mbps",config.channelRateMbps);cmd.AddValue("channelOffset","channel offset",config.channelOffset);cmd.AddValue("dist","distance",config.distance);cmd.AddValue("serverStart","broadcast start",config.serverStart);cmd.AddValue("clientStart","viewer join",config.clientStart);cmd.AddValue("simTime","sim time",config.simulationTime);cmd.AddValue("drainTime","drain",config.drainTime);cmd.AddValue("dataRate","802.11b rate",config.dataRate);cmd.AddValue("seed","seed",config.seed);cmd.AddValue("run","run",config.run);cmd.AddValue("pcap","pcap",config.enablePcap);cmd.AddValue("verbose","logs",config.verbose);cmd.AddValue("outputDir","output",config.outputDir);cmd.AddValue("tag","tag",config.outputTag);cmd.AddValue("startupBuffer", "Continuous playable seconds needed at startup", config.startupBuffer);
  cmd.AddValue("resumeBuffer", "Continuous playable seconds needed after stall", config.resumeBuffer);
  cmd.AddValue("playbackUnit", "Playback-check unit seconds", config.playbackUnit);
  cmd.AddValue("joinSpread", "Spread join times evenly across N nodes (seconds)", config.joinSpread);
  cmd.AddValue("rateMode", "perChannel or total; channelRate is applied accordingly", config.rateMode);
  cmd.AddValue("method", "all, FB, Simple, or Full", config.methodSelect);
  cmd.Parse(argc,argv);
  NS_ABORT_MSG_IF (config.rateMode != "total" && config.rateMode != "perChannel", "rateMode must be total or perChannel");
  NS_ABORT_MSG_IF (config.packetSize == 0 || config.startupBuffer <= 0 || config.resumeBuffer <= 0 || config.playbackUnit <= 0, "Invalid packet size or buffer/playback unit");
  NS_ABORT_MSG_IF (config.joinSpread < 0 || config.clientStart + config.joinSpread >= config.simulationTime - config.drainTime, "Join times must precede end of measurement");
  NS_ABORT_MSG_IF(config.nNodes==0,"nodes>=1");NS_ABORT_MSG_IF(config.kChannels==0||config.kChannels>=31,"k=1..30");NS_ABORT_MSG_IF(config.videoDuration<=0||config.videoBitrateMbps<=0||config.channelRateMbps<=0,"video settings >0");NS_ABORT_MSG_IF(config.clientStart<config.serverStart,"clientStart>=serverStart");NS_ABORT_MSG_IF(config.simulationTime<=config.clientStart,"simTime>clientStart");std::error_code ec;std::filesystem::create_directories(config.outputDir,ec);NS_ABORT_MSG_IF(ec,ec.message());std::vector<ExperimentResult> summaries;std::vector<NodeResult> nodes;for (auto m : {DeliveryMethod::FB, DeliveryMethod::SIMPLE, DeliveryMethod::FULL})
  {
    if (config.methodSelect != "all" && config.methodSelect != MethodName (m))
    {
      continue;
    }
    auto o = RunExperiment (config, m);
    summaries.push_back (o.summary);
    nodes.insert (nodes.end (), o.nodes.begin (), o.nodes.end ());
  }
  NS_ABORT_MSG_IF (summaries.empty (), "method must be all, FB, Simple, or Full");std::ostringstream name;name<<config.outputDir<<"/wifi_playback_fixed_improved_nodes"<<config.nNodes<<"_k"<<config.kChannels<<"_vdur"<<CompactNumber(config.videoDuration)<<"_vbr"<<CompactNumber(config.videoBitrateMbps)<<"_chrate"<<CompactNumber(config.channelRateMbps)<<"_dist"<<CompactNumber(config.distance)<<"_rateMode"<<config.rateMode<<"_startup"<<CompactNumber(config.startupBuffer)<<"_resume"<<CompactNumber(config.resumeBuffer)<<"_unit"<<CompactNumber(config.playbackUnit)<<"_join"<<CompactNumber(config.joinSpread)<<"_method"<<config.methodSelect<<"_seed"<<config.seed<<"_run"<<config.run;if (!config.outputTag.empty()) { name << "_" << config.outputTag; }
  WriteCsv(summaries,nodes,name.str());NS_LOG_UNCOND("\nCSV: "<<name.str()<<"_summary.csv");return 0;}
