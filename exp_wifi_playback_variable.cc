#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("WifiSegmentedBroadcastPlaybackVariable");
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

class ServerApp : public Application
{
public:
  ServerApp () : m_packetSize (1024), m_totalSegments (0), m_interval (Seconds (0.1)),
                 m_channelOffset (MilliSeconds (1)), m_running (false), m_sentCount (0) {}

  void Setup (Ipv4Address destination, uint32_t packetSize, uint32_t numChannels,
              Time interval, Time channelOffset, Time measurementStart, DeliveryMethod method)
  {
    m_peer = destination; m_packetSize = packetSize; m_interval = interval; m_channelOffset = channelOffset; m_measurementStart = measurementStart;
    m_totalSegments = (1u << numChannels) - 1u;
    uint32_t actualChannels = method == DeliveryMethod::FB ? numChannels :
                              (method == DeliveryMethod::SIMPLE ? 1u : m_totalSegments);
    m_channelData.assign (actualChannels, {}); m_indices.assign (actualChannels, 0);
    if (method == DeliveryMethod::FB)
    {
      uint32_t current = 1;
      for (uint32_t ch = 0; ch < numChannels; ++ch)
        for (uint32_t j = 0; j < (1u << ch); ++j) m_channelData[ch].push_back (current++);
    }
    else if (method == DeliveryMethod::SIMPLE)
    {
      for (uint32_t s = 1; s <= m_totalSegments; ++s) m_channelData[0].push_back (s);
    }
    else
    {
      for (uint32_t ch = 0; ch < m_totalSegments; ++ch) m_channelData[ch].push_back (ch + 1);
    }
  }

  uint32_t GetTotalSegments () const { return m_totalSegments; }
  uint32_t GetLogicalChannelCount () const { return static_cast<uint32_t> (m_channelData.size ()); }
  uint64_t GetTotalSent () const { return m_sentCount; }
  void StopSending () { m_running = false; if (m_socket) { m_socket->Close (); m_socket = nullptr; } }

private:
  void StartApplication () override
  {
    m_running = true;
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    m_socket->SetPriority (7);
    for (uint32_t ch = 0; ch < m_channelData.size (); ++ch)
      Simulator::Schedule (Seconds (m_channelOffset.GetSeconds () * ch), &ServerApp::SendPacket, this, ch);
  }
  void StopApplication () override { StopSending (); }
  void SendPacket (uint32_t channelId)
  {
    if (!m_running || !m_socket) return;
    const uint32_t segmentId = m_channelData[channelId][m_indices[channelId]];
    Ptr<Packet> packet = Create<Packet> (m_packetSize);
    SeqTsHeader h; h.SetSeq (segmentId); packet->AddHeader (h);
    const bool measured = Simulator::Now () >= m_measurementStart;
    if (m_socket->SendTo (packet, 0, InetSocketAddress (m_peer, RECEIVE_PORT)) >= 0 && measured) ++m_sentCount;
    m_indices[channelId] = (m_indices[channelId] + 1) % m_channelData[channelId].size ();
    Simulator::Schedule (m_interval, &ServerApp::SendPacket, this, channelId);
  }
  Ipv4Address m_peer; uint32_t m_packetSize; uint32_t m_totalSegments; Time m_interval; Time m_channelOffset; Time m_measurementStart;
  bool m_running; uint64_t m_sentCount; Ptr<Socket> m_socket;
  std::vector<std::vector<uint32_t>> m_channelData; std::vector<uint32_t> m_indices;
};

class ClientApp : public Application
{
public:
  ClientApp ()
      : m_totalSegments (0),
        m_receivedUnique (0),
        m_rawReceived (0),
        m_segmentDuration (Seconds (1.0)),
        m_playbackStarted (false),
        m_playbackComplete (false),
        m_stalled (false),
        m_currentSegment (0),
        m_initialWait (0.0),
        m_totalStall (0.0),
        m_maxStall (0.0),
        m_stallCount (0),
        m_finishFromJoin (0.0),
        m_verbose (false),
        m_drainTime (Seconds (0.1))
  {
  }

  void Setup (uint32_t totalSegments,
              Time segmentDuration,
              bool verbose,
              Time drainTime,
              Callback<void> allCompletedCallback)
  {
    m_totalSegments = totalSegments;
    m_segmentDuration = segmentDuration;
    m_verbose = verbose;
    m_drainTime = drainTime;
    m_allCompletedCallback = allCompletedCallback;
    m_received.assign (m_totalSegments + 1, 0);
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
    if (m_stalled)
    {
      value += (Simulator::Now () - m_stallStart).GetSeconds ();
    }
    return value;
  }

private:
  void StartApplication () override
  {
    m_joinTime = Simulator::Now ();
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    NS_ABORT_MSG_IF (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), RECEIVE_PORT)) < 0,
                     "Failed to bind Wi-Fi receive socket.");
    Ptr<UdpSocket> udp = DynamicCast<UdpSocket> (m_socket);
    NS_ABORT_MSG_IF (!udp, "Failed to create UDP socket.");
    udp->MulticastJoinGroup (0, InetSocketAddress (MULTICAST_GROUP, RECEIVE_PORT));
    m_socket->SetRecvCallback (MakeCallback (&ClientApp::HandleRead, this));
  }

  void StopApplication () override
  {
    if (m_socket)
    {
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket>> ());
      m_socket->Close ();
      m_socket = nullptr;
    }
  }

  void HandleRead (Ptr<Socket> socket)
  {
    Ptr<Packet> packet; Address from;
    while ((packet = socket->RecvFrom (from))) ProcessReceivedPacket (packet);
  }

  void ProcessReceivedPacket (Ptr<Packet> packet)
  {
    ++m_rawReceived;

    SeqTsHeader header;
    if (packet->RemoveHeader (header) == 0)
    {
      return;
    }

    const uint32_t segmentId = header.GetSeq ();
    if (segmentId == 0 || segmentId > m_totalSegments)
    {
      return;
    }

    if (m_received[segmentId] == 0)
    {
      m_received[segmentId] = 1;
      ++m_receivedUnique;
    }

    // 初期待ち時間: 視聴参加から Segment 1 を取得するまで。
    if (!m_playbackStarted && segmentId == 1)
    {
      StartPlayback ();
      return;
    }

    // 再生途切れ中に、まさに必要なセグメントが到着したら再開。
    if (m_stalled && segmentId == m_currentSegment)
    {
      ResumePlayback ();
    }
  }

  void StartPlayback ()
  {
    if (m_playbackStarted || m_playbackComplete)
    {
      return;
    }

    m_playbackStarted = true;
    m_currentSegment = 1;
    m_initialWait = (Simulator::Now () - m_joinTime).GetSeconds ();

    if (m_verbose)
    {
      NS_LOG_UNCOND ("Node " << GetNode ()->GetId ()
                     << " 再生開始: initialWait=" << m_initialWait << "s");
    }

    Simulator::Schedule (m_segmentDuration,
                         &ClientApp::OnSegmentPlaybackFinished,
                         this);
  }

  void OnSegmentPlaybackFinished ()
  {
    if (m_playbackComplete || !m_playbackStarted)
    {
      return;
    }

    if (m_currentSegment >= m_totalSegments)
    {
      FinishPlayback ();
      return;
    }

    ++m_currentSegment;

    if (m_received[m_currentSegment] != 0)
    {
      Simulator::Schedule (m_segmentDuration,
                           &ClientApp::OnSegmentPlaybackFinished,
                           this);
    }
    else
    {
      m_stalled = true;
      m_stallStart = Simulator::Now ();
      ++m_stallCount;

      if (m_verbose)
      {
        NS_LOG_UNCOND ("Node " << GetNode ()->GetId ()
                       << " 再生途切れ: Segment " << m_currentSegment);
      }
    }
  }

  void ResumePlayback ()
  {
    if (!m_stalled)
    {
      return;
    }

    const double stall = (Simulator::Now () - m_stallStart).GetSeconds ();
    m_totalStall += stall;
    m_maxStall = std::max (m_maxStall, stall);
    m_stalled = false;

    if (m_verbose)
    {
      NS_LOG_UNCOND ("Node " << GetNode ()->GetId ()
                     << " 再生再開: stall=" << stall << "s");
    }

    Simulator::Schedule (m_segmentDuration,
                         &ClientApp::OnSegmentPlaybackFinished,
                         this);
  }

  void FinishPlayback ()
  {
    if (m_playbackComplete)
    {
      return;
    }

    m_playbackComplete = true;
    m_finishFromJoin = (Simulator::Now () - m_joinTime).GetSeconds ();
    ++g_completedNodes;

    if (m_verbose)
    {
      NS_LOG_UNCOND ("Node " << GetNode ()->GetId ()
                     << " 再生完了: finish=" << m_finishFromJoin
                     << "s, stalls=" << m_stallCount
                     << ", totalStall=" << m_totalStall << "s");
    }

    if (g_completedNodes >= g_totalNodes && !m_allCompletedCallback.IsNull ())
    {
      m_allCompletedCallback ();
      Simulator::Stop (Simulator::Now () + m_drainTime);
    }
  }

  Ptr<Socket> m_socket;
  uint32_t m_totalSegments;
  uint32_t m_receivedUnique;
  uint64_t m_rawReceived;
  std::vector<uint8_t> m_received;

  Time m_segmentDuration;
  Time m_joinTime;
  bool m_playbackStarted;
  bool m_playbackComplete;
  bool m_stalled;
  uint32_t m_currentSegment;
  double m_initialWait;
  double m_totalStall;
  double m_maxStall;
  uint32_t m_stallCount;
  double m_finishFromJoin;
  Time m_stallStart;
  bool m_verbose;
  Time m_drainTime;
  Callback<void> m_allCompletedCallback;
};

struct SimulationConfig
{
  uint32_t nNodes = 5;
  uint32_t kChannels = 6;
  uint32_t packetSize = 1024;
  double interval = 0.1;
  double channelOffset = 0.001;
  double segmentPlaybackDuration = 1.0;
  double distance = 30.0;
  double serverStart = 1.0;
  double clientStart = 5.0;
  double simulationTime = 400.0;
  double drainTime = 0.1;
  std::string dataRate = "DsssRate11Mbps";
  uint32_t seed = 1;
  uint32_t run = 1;
  bool enablePcap = false;
  bool verbose = false;
  std::string outputDir = "results";
  std::string outputTag = "";
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

  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (MULTICAST_GROUP, config.packetSize, config.kChannels,
                    Seconds (config.interval), Seconds (config.channelOffset), Seconds (config.clientStart), method);
  serverNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds (config.serverStart));

  const uint32_t totalSegments = serverApp->GetTotalSegments ();
  g_totalNodes = config.nNodes; g_completedNodes = 0;
  for (uint32_t i = 0; i < config.nNodes; ++i)
  {
    Ptr<ClientApp> client = CreateObject<ClientApp> ();
    client->Setup (totalSegments, Seconds (config.segmentPlaybackDuration), config.verbose,
                   Seconds (config.drainTime), MakeCallback (&ServerApp::StopSending, serverApp));
    clientNodes.Get (i)->AddApplication (client);
    client->SetStartTime (Seconds (config.clientStart));
  }

  if (config.enablePcap) phy.EnablePcap (config.outputDir + "/wifi_playback_variable", devices.Get (0));

  NS_LOG_UNCOND ("\n================================================");
  NS_LOG_UNCOND ("【Wi-Fi・再生評価（動画非固定）：" << MethodDisplayName (method) << "】");
  NS_LOG_UNCOND ("STA数                  : " << config.nNodes);
  NS_LOG_UNCOND ("論理チャネル数        : " << serverApp->GetLogicalChannelCount ());
  NS_LOG_UNCOND ("総セグメント数(N)     : " << totalSegments);
  NS_LOG_UNCOND ("1セグメント再生時間   : " << config.segmentPlaybackDuration << "s");
  NS_LOG_UNCOND ("総動画時間            : " << totalSegments * config.segmentPlaybackDuration << "s (Nで可変)");
  NS_LOG_UNCOND ("1セグメントサイズ     : " << config.packetSize << " bytes (1 packet)");
  NS_LOG_UNCOND ("放送開始 / 視聴参加   : " << config.serverStart << "s / " << config.clientStart << "s");
  NS_LOG_UNCOND ("Wi-Fi                  : 802.11b " << config.dataRate << ", multicast");
  NS_LOG_UNCOND ("距離                   : " << config.distance << "m");
  NS_LOG_UNCOND ("================================================");

  Simulator::Stop (Seconds (config.simulationTime)); Simulator::Run ();

  ExperimentOutput output;
  uint64_t totalRaw = 0; double initialSum = 0, stallSum = 0, stallCountSum = 0, finishSum = 0;
  uint32_t started = 0, completed = 0;
  for (uint32_t i = 0; i < clientNodes.GetN (); ++i)
  {
    Ptr<ClientApp> app = DynamicCast<ClientApp> (clientNodes.Get (i)->GetApplication (0));
    NodeResult nr; nr.method = MethodName (method); nr.index = i; nr.nodeId = clientNodes.Get (i)->GetId ();
    nr.playbackStarted = app->PlaybackStarted (); nr.playbackComplete = app->PlaybackComplete ();
    nr.initialWaitS = app->GetInitialWait (); nr.stallCount = app->GetStallCount ();
    nr.totalStallS = app->GetObservedTotalStall (); nr.maxStallS = app->GetMaxStall ();
    nr.finishFromJoinS = app->GetFinishFromJoin (); nr.receivedUniqueSegments = app->GetReceivedCount ();
    nr.totalSegments = totalSegments; nr.rawReceivedPackets = app->GetRawReceivedCount (); output.nodes.push_back (nr);
    totalRaw += nr.rawReceivedPackets;
    if (nr.playbackStarted) { ++started; initialSum += nr.initialWaitS; stallSum += nr.totalStallS; stallCountSum += nr.stallCount; }
    if (nr.playbackComplete) { ++completed; finishSum += nr.finishFromJoinS; }
  }
  ExperimentResult& r = output.summary;
  r.method = MethodName (method); r.nodes = config.nNodes; r.logicalChannels = serverApp->GetLogicalChannelCount ();
  r.totalSegments = totalSegments; r.startedNodes = started; r.completedNodes = completed;
  r.completionRatePct = 100.0 * completed / config.nNodes;
  r.avgInitialWaitS = started ? initialSum / started : 0; r.avgTotalStallS = started ? stallSum / started : 0;
  r.avgStallCount = started ? stallCountSum / started : 0; r.avgFinishFromJoinS = completed ? finishSum / completed : 0;
  r.logicalSentPackets = serverApp->GetTotalSent (); r.sentPackets = serverApp->GetTotalSent ();
  r.attemptedPackets = r.sentPackets * static_cast<uint64_t> (config.nNodes);
  r.actualReceptions = totalRaw;
  r.packetLossRatePct = r.attemptedPackets ? std::max (0.0, 100.0 * (1.0 - static_cast<double> (totalRaw) / r.attemptedPackets)) : 0;

  NS_LOG_UNCOND ("==== 再生結果: " << MethodDisplayName (method) << " ====");
  NS_LOG_UNCOND ("再生開始STA数         : " << started << " / " << config.nNodes);
  NS_LOG_UNCOND ("再生完了STA数         : " << completed << " / " << config.nNodes);
  NS_LOG_UNCOND ("平均初期待ち時間      : " << (started ? std::to_string (r.avgInitialWaitS) : "N/A") << "s");
  NS_LOG_UNCOND ("平均総途切れ時間      : " << (started ? std::to_string (r.avgTotalStallS) : "N/A") << "s");
  NS_LOG_UNCOND ("平均途切れ回数        : " << (started ? std::to_string (r.avgStallCount) : "N/A"));
  NS_LOG_UNCOND ("観測パケット欠損率    : " << r.packetLossRatePct << "%");
  Simulator::Destroy (); return output;
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

int main (int argc, char* argv[])
{
  SimulationConfig config; CommandLine cmd;
  cmd.AddValue ("nodes", "Number of client nodes", config.nNodes);
  cmd.AddValue ("k", "FB logical channels; N=2^k-1", config.kChannels);
  cmd.AddValue ("int", "Per-logical-channel send interval", config.interval);
  cmd.AddValue ("channelOffset", "Initial channel offset", config.channelOffset);
  cmd.AddValue ("packetSize", "Bytes per segment (1 segment=1 packet)", config.packetSize);
  cmd.AddValue ("segmentDuration", "Playback seconds per segment; total video time varies with N", config.segmentPlaybackDuration);
  cmd.AddValue ("dist", "Distance (m)", config.distance);
  cmd.AddValue ("serverStart", "Broadcast start (s)", config.serverStart);
  cmd.AddValue ("clientStart", "Viewer join (s)", config.clientStart);
  cmd.AddValue ("simTime", "Simulation time", config.simulationTime);
  cmd.AddValue ("drainTime", "Drain time", config.drainTime);
  cmd.AddValue ("dataRate", "802.11b rate", config.dataRate);
  cmd.AddValue ("seed", "Seed", config.seed); cmd.AddValue ("run", "Run", config.run);
  cmd.AddValue ("pcap", "Enable PCAP", config.enablePcap); cmd.AddValue ("verbose", "Playback log", config.verbose);
  cmd.AddValue ("outputDir", "CSV output dir", config.outputDir); cmd.AddValue ("tag", "Output tag", config.outputTag);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (config.nNodes == 0, "nodes must be >=1");
  NS_ABORT_MSG_IF (config.kChannels == 0 || config.kChannels >= 31, "k must be 1..30");
  NS_ABORT_MSG_IF (config.interval <= 0 || config.segmentPlaybackDuration <= 0, "int and segmentDuration must be >0");
  NS_ABORT_MSG_IF (config.clientStart < config.serverStart, "clientStart must be >= serverStart");
  NS_ABORT_MSG_IF (config.simulationTime <= config.clientStart, "simTime must be > clientStart");
  std::error_code ec; std::filesystem::create_directories (config.outputDir, ec); NS_ABORT_MSG_IF (ec, ec.message ());

  std::vector<ExperimentResult> summaries; std::vector<NodeResult> nodes;
  for (auto method : {DeliveryMethod::FB, DeliveryMethod::SIMPLE, DeliveryMethod::FULL})
  {
    auto o = RunExperiment (config, method); summaries.push_back (o.summary); nodes.insert (nodes.end (), o.nodes.begin (), o.nodes.end ());
  }
  std::ostringstream name; name << config.outputDir << "/wifi_playback_variable_nodes" << config.nNodes
    << "_k" << config.kChannels << "_int" << CompactNumber (config.interval)
    << "_segdur" << CompactNumber (config.segmentPlaybackDuration) << "_dist" << CompactNumber (config.distance)
    << "_seed" << config.seed << "_run" << config.run; if (!config.outputTag.empty ()) name << "_" << config.outputTag;
  WriteCsv (summaries, nodes, name.str ());
  NS_LOG_UNCOND ("\nCSV: " << name.str () << "_summary.csv");
  NS_LOG_UNCOND ("CSV: " << name.str () << "_nodes.csv");
  return 0;
}
