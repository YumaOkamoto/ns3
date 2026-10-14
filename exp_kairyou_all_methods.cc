#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cctype>
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

NS_LOG_COMPONENT_DEFINE ("WifiSegmentedBroadcastingAllMethods");

namespace
{
constexpr uint16_t RECEIVE_PORT = 9001;
const Ipv4Address MULTICAST_GROUP ("224.1.1.1");

// 1方式のシミュレーションごとに必ず ResetGlobalStats() で初期化する
uint32_t g_completedNodes = 0;
uint32_t g_totalNodes = 0;
double g_waitTimeSum = 0.0;
double g_minWaitTime = std::numeric_limits<double>::max ();
double g_maxWaitTime = 0.0;

void
ResetGlobalStats (uint32_t totalNodes)
{
  g_completedNodes = 0;
  g_totalNodes = totalNodes;
  g_waitTimeSum = 0.0;
  g_minWaitTime = std::numeric_limits<double>::max ();
  g_maxWaitTime = 0.0;
}
} // namespace

enum class DeliveryMethod : uint32_t
{
  FB = 0,
  SIMPLE = 1,
  FULL = 2
};

std::string
MethodName (DeliveryMethod method)
{
  switch (method)
  {
  case DeliveryMethod::FB:
    return "FB";
  case DeliveryMethod::SIMPLE:
    return "Simple";
  case DeliveryMethod::FULL:
    return "Full";
  }
  return "Unknown";
}

std::string
MethodDisplayName (DeliveryMethod method)
{
  switch (method)
  {
  case DeliveryMethod::FB:
    return "FB法";
  case DeliveryMethod::SIMPLE:
    return "単純放送";
  case DeliveryMethod::FULL:
    return "等分割法";
  }
  return "Unknown";
}

// ============================================================
// ServerApp
// ============================================================
class ServerApp : public Application
{
public:
  ServerApp ()
    : m_packetSize (1024),
      m_totalSegments (0),
      m_interval (Seconds (1.0)),
      m_channelOffset (MilliSeconds (1)),
      m_running (false),
      m_sentCount (0)
  {
  }

  void Setup (Ipv4Address destination,
              uint32_t packetSize,
              uint32_t numChannels,
              Time interval,
              Time channelOffset,
              DeliveryMethod method)
  {
    m_peer = destination;
    m_packetSize = packetSize;
    m_interval = interval;
    m_channelOffset = channelOffset;

    NS_ABORT_MSG_IF (numChannels == 0 || numChannels >= 31,
                     "k must be between 1 and 30.");

    const uint32_t nSegments = (1u << numChannels) - 1u;
    m_totalSegments = nSegments;

    uint32_t actualChannels = 0;

    if (method == DeliveryMethod::FB)
    {
      actualChannels = numChannels;
    }
    else if (method == DeliveryMethod::SIMPLE)
    {
      actualChannels = 1;
    }
    else
    {
      actualChannels = nSegments;
    }

    m_channelData.assign (actualChannels, {});
    m_indices.assign (actualChannels, 0);

    if (method == DeliveryMethod::FB)
    {
      // FB法: 論理チャネル i に 2^i セグメントを割り当てる
      uint32_t currentSegment = 1;

      for (uint32_t channelId = 0; channelId < numChannels; ++channelId)
      {
        const uint32_t segmentsInChannel = (1u << channelId);

        for (uint32_t j = 0; j < segmentsInChannel; ++j)
        {
          m_channelData[channelId].push_back (currentSegment++);
        }
      }
    }
    else if (method == DeliveryMethod::SIMPLE)
    {
      // 単純放送: 1論理チャネルで全Nセグメントを巡回
      for (uint32_t segmentId = 1; segmentId <= nSegments; ++segmentId)
      {
        m_channelData[0].push_back (segmentId);
      }
    }
    else
    {
      // 等分割法: 1論理チャネルにつき1セグメント
      for (uint32_t channelId = 0; channelId < nSegments; ++channelId)
      {
        m_channelData[channelId].push_back (channelId + 1);
      }
    }
  }

  uint32_t GetTotalSegments () const
  {
    return m_totalSegments;
  }

  uint64_t GetTotalSent () const
  {
    return m_sentCount;
  }

  uint32_t GetLogicalChannelCount () const
  {
    return static_cast<uint32_t> (m_channelData.size ());
  }

  void StopSending ()
  {
    m_running = false;

    if (m_socket)
    {
      m_socket->Close ();
      m_socket = nullptr;
    }
  }

private:
  void StartApplication () override
  {
    m_running = true;

    // 全論理チャネルで1個のUDPソケットを共有
    m_socket =
        Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());

    m_socket->SetPriority (7);

    for (uint32_t channelId = 0; channelId < m_channelData.size (); ++channelId)
    {
      Simulator::Schedule (
          Seconds (m_channelOffset.GetSeconds () * channelId),
          &ServerApp::SendPacket,
          this,
          channelId);
    }
  }

  void StopApplication () override
  {
    StopSending ();
  }

  void SendPacket (uint32_t channelId)
  {
    if (!m_running || !m_socket)
    {
      return;
    }

    const uint32_t segmentId =
        m_channelData[channelId][m_indices[channelId]];

    Ptr<Packet> packet = Create<Packet> (m_packetSize);

    SeqTsHeader header;
    header.SetSeq (segmentId);
    packet->AddHeader (header);

    const int result =
        m_socket->SendTo (
            packet,
            0,
            InetSocketAddress (m_peer, RECEIVE_PORT));

    if (result >= 0)
    {
      ++m_sentCount;
    }

    m_indices[channelId] =
        (m_indices[channelId] + 1) %
        m_channelData[channelId].size ();

    Simulator::Schedule (
        m_interval,
        &ServerApp::SendPacket,
        this,
        channelId);
  }

  Ipv4Address m_peer;
  uint32_t m_packetSize;
  uint32_t m_totalSegments;
  Time m_interval;
  Time m_channelOffset;
  bool m_running;
  uint64_t m_sentCount;

  Ptr<Socket> m_socket;
  std::vector<std::vector<uint32_t>> m_channelData;
  std::vector<uint32_t> m_indices;
};

// ============================================================
// ClientApp
// ============================================================
class ClientApp : public Application
{
public:
  ClientApp ()
    : m_rawReceivedCount (0),
      m_totalSegments (0),
      m_receivedCount (0),
      m_allReceived (false),
      m_waitTime (0.0),
      m_verbose (false),
      m_drainTime (Seconds (1.0))
  {
  }

  void Setup (uint32_t totalSegments,
              bool verbose,
              Time drainTime,
              Callback<void> allCompletedCallback)
  {
    m_totalSegments = totalSegments;
    m_verbose = verbose;
    m_drainTime = drainTime;
    m_allCompletedCallback = allCompletedCallback;

    m_received.assign (m_totalSegments + 1, 0);
  }

  uint32_t GetReceivedCount () const
  {
    return m_receivedCount;
  }

  uint64_t GetRawReceivedCount () const
  {
    return m_rawReceivedCount;
  }

  bool IsComplete () const
  {
    return m_allReceived;
  }

  double GetWaitTime () const
  {
    return m_waitTime;
  }

private:
  void StartApplication () override
  {
    m_startTime = Simulator::Now ();

    // 1クライアントにつき1 UDPソケット
    m_socket =
        Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());

    const int bindResult =
        m_socket->Bind (
            InetSocketAddress (
                Ipv4Address::GetAny (),
                RECEIVE_PORT));

    NS_ABORT_MSG_IF (
        bindResult < 0,
        "Failed to bind UDP receive socket.");

    Ptr<UdpSocket> udpSocket =
        DynamicCast<UdpSocket> (m_socket);

    NS_ABORT_MSG_IF (
        !udpSocket,
        "Failed to create UDP socket.");

    udpSocket->MulticastJoinGroup (
        0,
        InetSocketAddress (
            MULTICAST_GROUP,
            RECEIVE_PORT));

    m_socket->SetRecvCallback (
        MakeCallback (
            &ClientApp::HandleRead,
            this));
  }

  void StopApplication () override
  {
    if (m_socket)
    {
      m_socket->SetRecvCallback (
          MakeNullCallback<void, Ptr<Socket>> ());

      m_socket->Close ();
      m_socket = nullptr;
    }
  }

  void HandleRead (Ptr<Socket> socket)
  {
    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom (from)))
    {
      ++m_rawReceivedCount;

      SeqTsHeader header;

      if (packet->RemoveHeader (header) == 0)
      {
        continue;
      }

      const uint32_t segmentId =
          header.GetSeq ();

      if (segmentId == 0 ||
          segmentId > m_totalSegments)
      {
        continue;
      }

      if (m_received[segmentId] != 0)
      {
        continue;
      }

      m_received[segmentId] = 1;
      ++m_receivedCount;

      if (m_receivedCount == m_totalSegments &&
          !m_allReceived)
      {
        m_allReceived = true;

        m_waitTime =
            (Simulator::Now () - m_startTime)
                .GetSeconds ();

        ++g_completedNodes;
        g_waitTimeSum += m_waitTime;

        g_minWaitTime =
            std::min (
                g_minWaitTime,
                m_waitTime);

        g_maxWaitTime =
            std::max (
                g_maxWaitTime,
                m_waitTime);

        if (m_verbose)
        {
          NS_LOG_UNCOND (
              "Node "
              << GetNode ()->GetId ()
              << " 完了! (待ち時間: "
              << m_waitTime
              << "s)");
        }

        if (g_completedNodes >= g_totalNodes)
        {
          // 新規送信停止後、キュー内パケットを処理
          if (!m_allCompletedCallback.IsNull ())
          {
            m_allCompletedCallback ();
          }

          Simulator::Stop (
              Simulator::Now () +
              m_drainTime);
        }
      }
    }
  }

  Ptr<Socket> m_socket;
  uint64_t m_rawReceivedCount;
  Time m_startTime;
  uint32_t m_totalSegments;
  uint32_t m_receivedCount;
  bool m_allReceived;
  double m_waitTime;
  bool m_verbose;
  Time m_drainTime;
  Callback<void> m_allCompletedCallback;
  std::vector<uint8_t> m_received;
};

// ============================================================
// 実験設定・結果
// ============================================================
struct SimulationConfig
{
  uint32_t nNodes = 3;
  uint32_t kChannels = 4;
  uint32_t packetSize = 1024;

  double interval = 1.0;
  double channelOffset = 0.001;
  double distance = 30.0;
  double clientStart = 2.0;
  double simulationTime = 400.0;
  double drainTime = 1.0;

  std::string dataRate = "DsssRate11Mbps";

  uint32_t seed = 1;
  uint32_t run = 1;

  bool enablePcap = false;
  bool verbose = false;

  // 結果保存先
  std::string outputDir = "results";
  std::string outputTag = "";
};

struct ExperimentResult
{
  std::string environment;
  std::string method;

  uint32_t nodes = 0;
  uint32_t logicalChannels = 0;
  uint32_t totalSegments = 0;

  double intervalS = 0.0;
  double channelOffsetS = 0.0;
  uint16_t receivePort = 0;
  double clientStartS = 0.0;
  double simulationTimeS = 0.0;
  double drainTimeS = 0.0;

  uint32_t packetSizeBytes = 0;
  std::string wifiStandard;
  std::string dataRate;
  double distanceM = 0.0;

  bool pcapEnabled = false;
  bool verboseEnabled = false;

  uint32_t seed = 0;
  uint32_t run = 0;

  uint32_t completedNodes = 0;
  double completionRatePct = 0.0;

  double averageWaitS = 0.0;
  double minimumWaitS = 0.0;
  double maximumWaitS = 0.0;

  double averageSegmentArrivalRatePct = 0.0;

  uint64_t totalSent = 0;
  uint64_t expectedReceptions = 0;
  uint64_t actualReceptions = 0;
  uint64_t lostPackets = 0;
  double packetLossRatePct = 0.0;
};

struct NodeResult
{
  std::string method;

  uint32_t clientIndex = 0;
  uint32_t nodeId = 0;

  bool completed = false;
  double waitTimeS = 0.0;

  uint32_t receivedUniqueSegments = 0;
  uint32_t missingUniqueSegments = 0;
  uint32_t totalSegments = 0;

  double segmentArrivalRatePct = 0.0;

  uint64_t rawReceivedPackets = 0;
};

struct ExperimentOutput
{
  ExperimentResult summary;
  std::vector<NodeResult> nodes;
};

// ファイル名に使いにくい文字を '_' にする
std::string
SanitizeFilePart (const std::string& input)
{
  std::string output = input;

  for (char& c : output)
  {
    const unsigned char uc =
        static_cast<unsigned char> (c);

    if (!(std::isalnum (uc) ||
          c == '-' ||
          c == '_' ||
          c == '.'))
    {
      c = '_';
    }
  }

  return output;
}

std::string
CompactNumber (double value)
{
  std::ostringstream oss;
  oss << std::fixed
      << std::setprecision (6)
      << value;

  std::string s = oss.str ();

  while (!s.empty () &&
         s.back () == '0')
  {
    s.pop_back ();
  }

  if (!s.empty () &&
      s.back () == '.')
  {
    s.pop_back ();
  }

  for (char& c : s)
  {
    if (c == '.')
    {
      c = 'p';
    }
    else if (c == '-')
    {
      c = 'm';
    }
  }

  return s;
}

std::string
BuildOutputBaseName (const SimulationConfig& config)
{
  std::ostringstream oss;

  oss << "wifi"
      << "_nodes" << config.nNodes
      << "_k" << config.kChannels
      << "_int" << CompactNumber (config.interval)
      << "_dist" << CompactNumber (config.distance)
      << "_seed" << config.seed
      << "_run" << config.run;

  if (!config.outputTag.empty ())
  {
    oss << "_"
        << SanitizeFilePart (
               config.outputTag);
  }

  return oss.str ();
}

// ============================================================
// 1方式の実験を実行
// ============================================================
ExperimentOutput
RunExperiment (const SimulationConfig& config,
               DeliveryMethod method)
{
  ResetGlobalStats (config.nNodes);

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

  // ------------------------------------------------------------
  // Application
  // ------------------------------------------------------------
  Ptr<ServerApp> serverApp =
      CreateObject<ServerApp> ();

  serverApp->Setup (
      MULTICAST_GROUP,
      config.packetSize,
      config.kChannels,
      Seconds (config.interval),
      Seconds (config.channelOffset),
      method);

  serverNode.Get (0)
      ->AddApplication (serverApp);

  // クライアントと同時刻から送信を開始
  serverApp->SetStartTime (
      Seconds (config.clientStart));

  const uint32_t totalSegments =
      serverApp->GetTotalSegments ();

  for (uint32_t i = 0;
       i < config.nNodes;
       ++i)
  {
    Ptr<ClientApp> clientApp =
        CreateObject<ClientApp> ();

    clientApp->Setup (
        totalSegments,
        config.verbose,
        Seconds (config.drainTime),
        MakeCallback (
            &ServerApp::StopSending,
            serverApp));

    clientNodes.Get (i)
        ->AddApplication (clientApp);

    clientApp->SetStartTime (
        Seconds (config.clientStart));
  }

  // PCAPを方式別に分けて保存
  if (config.enablePcap)
  {
    const std::string pcapPrefix =
        config.outputDir +
        "/" +
        BuildOutputBaseName (config) +
        "_" +
        MethodName (method);

    phy.EnablePcap (
        pcapPrefix,
        devices.Get (0));
  }

  // ------------------------------------------------------------
  // 設定表示
  // ------------------------------------------------------------
  NS_LOG_UNCOND (
      "\n================================================");

  NS_LOG_UNCOND (
      "【実験設定："
      << MethodDisplayName (method)
      << "】");

  NS_LOG_UNCOND (
      "配信手法          : "
      << MethodDisplayName (method));

  NS_LOG_UNCOND (
      "ノード数          : "
      << config.nNodes);

  NS_LOG_UNCOND (
      "論理チャネル数    : "
      << serverApp->GetLogicalChannelCount ());

  NS_LOG_UNCOND (
      "総セグメント数(N) : "
      << totalSegments);

  NS_LOG_UNCOND (
      "チャネル送信間隔  : "
      << config.interval
      << "s");

  NS_LOG_UNCOND (
      "チャネル初期ずれ  : "
      << config.channelOffset
      << "s");

  NS_LOG_UNCOND (
      "受信ポート        : "
      << RECEIVE_PORT);

  NS_LOG_UNCOND (
      "クライアント開始  : "
      << config.clientStart
      << "s");

  NS_LOG_UNCOND (
      "パケットサイズ    : "
      << config.packetSize
      << " bytes");

  NS_LOG_UNCOND (
      "------------------------------------------------");

  NS_LOG_UNCOND (
      "Wi-Fi規格         : IEEE 802.11b");

  NS_LOG_UNCOND (
      "データレート      : "
      << config.dataRate);

  NS_LOG_UNCOND (
      "距離              : "
      << config.distance
      << "m");

  NS_LOG_UNCOND (
      "PCAP              : "
      << (config.enablePcap ? "ON" : "OFF"));

  NS_LOG_UNCOND (
      "個別ログ          : "
      << (config.verbose ? "ON" : "OFF"));

  NS_LOG_UNCOND (
      "乱数Seed          : "
      << config.seed);

  NS_LOG_UNCOND (
      "乱数Run           : "
      << config.run);

  NS_LOG_UNCOND (
      "================================================");

  // ------------------------------------------------------------
  // Run
  // ------------------------------------------------------------
  Simulator::Stop (
      Seconds (
          config.simulationTime));

  Simulator::Run ();

  // ------------------------------------------------------------
  // 結果回収
  // ------------------------------------------------------------
  ExperimentOutput output;

  output.nodes.reserve (
      config.nNodes);

  double totalArrivalRate = 0.0;
  uint64_t totalRawReceived = 0;

  for (uint32_t i = 0;
       i < clientNodes.GetN ();
       ++i)
  {
    Ptr<ClientApp> app =
        DynamicCast<ClientApp> (
            clientNodes.Get (i)
                ->GetApplication (0));

    const uint32_t receivedUnique =
        app->GetReceivedCount ();

    const uint32_t missingUnique =
        totalSegments - receivedUnique;

    const double arrivalRate =
        (static_cast<double> (
             receivedUnique) /
         totalSegments) *
        100.0;

    totalArrivalRate += arrivalRate;
    totalRawReceived +=
        app->GetRawReceivedCount ();

    NodeResult nodeResult;

    nodeResult.method =
        MethodName (method);

    nodeResult.clientIndex = i;

    nodeResult.nodeId =
        clientNodes.Get (i)
            ->GetId ();

    nodeResult.completed =
        app->IsComplete ();

    nodeResult.waitTimeS =
        app->GetWaitTime ();

    nodeResult.receivedUniqueSegments =
        receivedUnique;

    nodeResult.missingUniqueSegments =
        missingUnique;

    nodeResult.totalSegments =
        totalSegments;

    nodeResult.segmentArrivalRatePct =
        arrivalRate;

    nodeResult.rawReceivedPackets =
        app->GetRawReceivedCount ();

    output.nodes.push_back (
        nodeResult);

    if (config.verbose)
    {
      NS_LOG_UNCOND (
          "Node "
          << nodeResult.nodeId
          << ": unique="
          << receivedUnique
          << "/"
          << totalSegments
          << ", raw="
          << nodeResult.rawReceivedPackets
          << ", complete="
          << (nodeResult.completed
                  ? "yes"
                  : "no"));
    }
  }

  const double averageArrivalRate =
      totalArrivalRate /
      config.nNodes;

  const double completionRate =
      (static_cast<double> (
           g_completedNodes) /
       config.nNodes) *
      100.0;

  const double averageWaitTime =
      (g_completedNodes == 0)
          ? 0.0
          : g_waitTimeSum /
                g_completedNodes;

  const double minimumWaitTime =
      (g_completedNodes == 0)
          ? 0.0
          : g_minWaitTime;

  const double maximumWaitTime =
      (g_completedNodes == 0)
          ? 0.0
          : g_maxWaitTime;

  const uint64_t totalSent =
      serverApp->GetTotalSent ();

  const uint64_t expectedReceptions =
      totalSent *
      static_cast<uint64_t> (
          config.nNodes);

  const uint64_t lostPackets =
      (expectedReceptions >=
       totalRawReceived)
          ? expectedReceptions -
                totalRawReceived
          : 0;

  double packetLossRate = 0.0;

  if (expectedReceptions > 0)
  {
    packetLossRate =
        (1.0 -
         static_cast<double> (
             totalRawReceived) /
             expectedReceptions) *
        100.0;
  }

  packetLossRate =
      std::max (
          0.0,
          packetLossRate);

  // ------------------------------------------------------------
  // summary格納
  // ------------------------------------------------------------
  ExperimentResult& result =
      output.summary;

  result.environment =
      "WiFi-802.11b";

  result.method =
      MethodName (method);

  result.nodes =
      config.nNodes;

  result.logicalChannels =
      serverApp->GetLogicalChannelCount ();

  result.totalSegments =
      totalSegments;

  result.intervalS =
      config.interval;

  result.channelOffsetS =
      config.channelOffset;

  result.receivePort =
      RECEIVE_PORT;

  result.clientStartS =
      config.clientStart;

  result.simulationTimeS =
      config.simulationTime;

  result.drainTimeS =
      config.drainTime;

  result.packetSizeBytes =
      config.packetSize;

  result.wifiStandard =
      "IEEE802.11b";

  result.dataRate =
      config.dataRate;

  result.distanceM =
      config.distance;

  result.pcapEnabled =
      config.enablePcap;

  result.verboseEnabled =
      config.verbose;

  result.seed =
      config.seed;

  result.run =
      config.run;

  result.completedNodes =
      g_completedNodes;

  result.completionRatePct =
      completionRate;

  result.averageWaitS =
      averageWaitTime;

  result.minimumWaitS =
      minimumWaitTime;

  result.maximumWaitS =
      maximumWaitTime;

  result.averageSegmentArrivalRatePct =
      averageArrivalRate;

  result.totalSent =
      totalSent;

  result.expectedReceptions =
      expectedReceptions;

  result.actualReceptions =
      totalRawReceived;

  result.lostPackets =
      lostPackets;

  result.packetLossRatePct =
      packetLossRate;

  // ------------------------------------------------------------
  // 結果表示
  // ------------------------------------------------------------
  NS_LOG_UNCOND (
      "\n==== 集計結果: "
      << MethodDisplayName (method)
      << " ====");

  NS_LOG_UNCOND (
      "完了ノード数          : "
      << result.completedNodes
      << " / "
      << result.nodes);

  NS_LOG_UNCOND (
      "完了率                : "
      << result.completionRatePct
      << "%");

  NS_LOG_UNCOND (
      "完了ノード平均待ち時間: "
      << result.averageWaitS
      << "s");

  NS_LOG_UNCOND (
      "最小待ち時間          : "
      << result.minimumWaitS
      << "s");

  NS_LOG_UNCOND (
      "最大待ち時間          : "
      << result.maximumWaitS
      << "s");

  NS_LOG_UNCOND (
      "全ノード平均到達率    : "
      << result.averageSegmentArrivalRatePct
      << "%");

  NS_LOG_UNCOND (
      "総送信数              : "
      << result.totalSent);

  NS_LOG_UNCOND (
      "理論受信数            : "
      << result.expectedReceptions);

  NS_LOG_UNCOND (
      "実受信数              : "
      << result.actualReceptions);

  NS_LOG_UNCOND (
      "観測損失パケット数    : "
      << result.lostPackets);

  NS_LOG_UNCOND (
      "観測パケット欠損率    : "
      << result.packetLossRatePct
      << "%");

  // この方式のシミュレーション状態を破棄してから次方式へ
  Simulator::Destroy ();

  return output;
}

// ============================================================
// CSV出力
// ============================================================
void
WriteSummaryCsv (
    const std::vector<ExperimentResult>& results,
    const std::string& filename)
{
  std::ofstream file (
      filename,
      std::ios::out |
          std::ios::trunc);

  NS_ABORT_MSG_IF (
      !file.is_open (),
      "Cannot open summary CSV: "
          << filename);

  file << std::setprecision (15);

  file
      << "environment,"
      << "method,"
      << "nodes,"
      << "logical_channels,"
      << "total_segments,"
      << "interval_s,"
      << "channel_offset_s,"
      << "receive_port,"
      << "client_start_s,"
      << "simulation_time_s,"
      << "drain_time_s,"
      << "packet_size_bytes,"
      << "wifi_standard,"
      << "data_rate,"
      << "distance_m,"
      << "pcap_enabled,"
      << "verbose_enabled,"
      << "seed,"
      << "run,"
      << "completed_nodes,"
      << "completion_rate_pct,"
      << "avg_wait_s,"
      << "min_wait_s,"
      << "max_wait_s,"
      << "avg_segment_arrival_rate_pct,"
      << "total_sent_packets,"
      << "expected_receptions,"
      << "actual_receptions,"
      << "lost_packets,"
      << "packet_loss_rate_pct"
      << "\n";

  for (const auto& r : results)
  {
    file
        << r.environment << ","
        << r.method << ","
        << r.nodes << ","
        << r.logicalChannels << ","
        << r.totalSegments << ","
        << r.intervalS << ","
        << r.channelOffsetS << ","
        << r.receivePort << ","
        << r.clientStartS << ","
        << r.simulationTimeS << ","
        << r.drainTimeS << ","
        << r.packetSizeBytes << ","
        << r.wifiStandard << ","
        << r.dataRate << ","
        << r.distanceM << ","
        << (r.pcapEnabled ? 1 : 0) << ","
        << (r.verboseEnabled ? 1 : 0) << ","
        << r.seed << ","
        << r.run << ","
        << r.completedNodes << ","
        << r.completionRatePct << ","
        << r.averageWaitS << ","
        << r.minimumWaitS << ","
        << r.maximumWaitS << ","
        << r.averageSegmentArrivalRatePct << ","
        << r.totalSent << ","
        << r.expectedReceptions << ","
        << r.actualReceptions << ","
        << r.lostPackets << ","
        << r.packetLossRatePct
        << "\n";
  }
}

void
WriteNodeResultsCsv (
    const std::vector<NodeResult>& results,
    const std::string& filename)
{
  std::ofstream file (
      filename,
      std::ios::out |
          std::ios::trunc);

  NS_ABORT_MSG_IF (
      !file.is_open (),
      "Cannot open node CSV: "
          << filename);

  file << std::setprecision (15);

  file
      << "method,"
      << "client_index,"
      << "node_id,"
      << "completed,"
      << "wait_time_s,"
      << "received_unique_segments,"
      << "missing_unique_segments,"
      << "total_segments,"
      << "segment_arrival_rate_pct,"
      << "raw_received_packets"
      << "\n";

  for (const auto& r : results)
  {
    file
        << r.method << ","
        << r.clientIndex << ","
        << r.nodeId << ","
        << (r.completed ? 1 : 0) << ",";

    // 未完了ノードは待ち時間を空欄にする。
    // 0秒完了と「未完了」を区別するため。
    if (r.completed)
    {
      file << r.waitTimeS;
    }

    file
        << ","
        << r.receivedUniqueSegments << ","
        << r.missingUniqueSegments << ","
        << r.totalSegments << ","
        << r.segmentArrivalRatePct << ","
        << r.rawReceivedPackets
        << "\n";
  }
}

// ============================================================
// main
// ============================================================
int
main (int argc, char* argv[])
{
  SimulationConfig config;

  CommandLine cmd;

  // mode は廃止。
  // 1回の実行で FB -> Simple -> Full を順番に実行する。
  cmd.AddValue (
      "nodes",
      "Number of client nodes",
      config.nNodes);

  cmd.AddValue (
      "k",
      "Number of logical channels for FB",
      config.kChannels);

  cmd.AddValue (
      "int",
      "Per-logical-channel transmission interval (s)",
      config.interval);

  cmd.AddValue (
      "channelOffset",
      "Initial offset between logical channels (s)",
      config.channelOffset);

  cmd.AddValue (
      "dist",
      "Server-client distance (m)",
      config.distance);

  cmd.AddValue (
      "rate",
      "Wi-Fi data rate",
      config.dataRate);

  cmd.AddValue (
      "packetSize",
      "Application payload size in bytes",
      config.packetSize);

  cmd.AddValue (
      "clientStart",
      "Client/server application start time (s)",
      config.clientStart);

  cmd.AddValue (
      "simTime",
      "Maximum simulation time for each method (s)",
      config.simulationTime);

  cmd.AddValue (
      "drainTime",
      "Queue drain time after all clients complete (s)",
      config.drainTime);

  cmd.AddValue (
      "seed",
      "Random seed",
      config.seed);

  cmd.AddValue (
      "run",
      "Run number",
      config.run);

  cmd.AddValue (
      "pcap",
      "Enable PCAP output",
      config.enablePcap);

  cmd.AddValue (
      "verbose",
      "Enable per-node console output",
      config.verbose);

  cmd.AddValue (
      "outputDir",
      "Directory for CSV/PCAP results",
      config.outputDir);

  cmd.AddValue (
      "tag",
      "Optional tag appended to output file names",
      config.outputTag);

  cmd.Parse (
      argc,
      argv);

  // ------------------------------------------------------------
  // 入力チェック
  // ------------------------------------------------------------
  NS_ABORT_MSG_IF (
      config.nNodes == 0,
      "nodes must be at least 1.");

  NS_ABORT_MSG_IF (
      config.kChannels == 0 ||
          config.kChannels >= 31,
      "k must be between 1 and 30.");

  NS_ABORT_MSG_IF (
      config.interval <= 0.0,
      "int must be greater than 0.");

  NS_ABORT_MSG_IF (
      config.channelOffset < 0.0,
      "channelOffset must be 0 or greater.");

  NS_ABORT_MSG_IF (
      config.packetSize == 0,
      "packetSize must be at least 1.");

  NS_ABORT_MSG_IF (
      config.simulationTime <=
          config.clientStart,
      "simTime must be later than clientStart.");

  NS_ABORT_MSG_IF (
      config.drainTime < 0.0,
      "drainTime must be 0 or greater.");

  NS_ABORT_MSG_IF (
      config.outputDir.empty (),
      "outputDir must not be empty.");

  // ------------------------------------------------------------
  // Ubuntu上に結果ディレクトリを作る
  // ------------------------------------------------------------
  std::error_code directoryError;

  std::filesystem::create_directories (
      config.outputDir,
      directoryError);

  NS_ABORT_MSG_IF (
      directoryError,
      "Failed to create output directory '"
          << config.outputDir
          << "': "
          << directoryError.message ());

  // ------------------------------------------------------------
  // 3方式を独立に順次実行
  // ------------------------------------------------------------
  const std::vector<DeliveryMethod> methods = {
      DeliveryMethod::FB,
      DeliveryMethod::SIMPLE,
      DeliveryMethod::FULL};

  std::vector<ExperimentResult> summaries;
  std::vector<NodeResult> allNodeResults;

  summaries.reserve (
      methods.size ());

  allNodeResults.reserve (
      static_cast<size_t> (
          config.nNodes) *
      methods.size ());

  for (DeliveryMethod method : methods)
  {
    ExperimentOutput output =
        RunExperiment (
            config,
            method);

    summaries.push_back (
        output.summary);

    allNodeResults.insert (
        allNodeResults.end (),
        output.nodes.begin (),
        output.nodes.end ());
  }

  // ------------------------------------------------------------
  // CSVファイル名
  // ------------------------------------------------------------
  const std::string baseName =
      BuildOutputBaseName (
          config);

  const std::string summaryFilename =
      config.outputDir +
      "/" +
      baseName +
      "_summary.csv";

  const std::string nodeFilename =
      config.outputDir +
      "/" +
      baseName +
      "_nodes.csv";

  // ------------------------------------------------------------
  // CSV書き込み
  // ------------------------------------------------------------
  WriteSummaryCsv (
      summaries,
      summaryFilename);

  WriteNodeResultsCsv (
      allNodeResults,
      nodeFilename);

  // ------------------------------------------------------------
  // 最終表示
  // ------------------------------------------------------------
  NS_LOG_UNCOND (
      "\n================================================");

  NS_LOG_UNCOND (
      "3方式すべての実験が完了しました。");

  NS_LOG_UNCOND (
      "Summary CSV : "
      << summaryFilename);

  NS_LOG_UNCOND (
      "Node CSV    : "
      << nodeFilename);

  NS_LOG_UNCOND (
      "================================================");

  NS_LOG_UNCOND (
      "\n=== 3方式比較 ===");

  NS_LOG_UNCOND (
      "Method,Completed,CompletionRate(%),"
      "AvgWait(s),MinWait(s),MaxWait(s),"
      "AvgArrival(%),Sent,ExpectedRx,ActualRx,"
      "Lost,LossRate(%)");

  for (const auto& r : summaries)
  {
    NS_LOG_UNCOND (
        r.method << ","
                 << r.completedNodes
                 << "/"
                 << r.nodes
                 << ","
                 << r.completionRatePct
                 << ","
                 << r.averageWaitS
                 << ","
                 << r.minimumWaitS
                 << ","
                 << r.maximumWaitS
                 << ","
                 << r.averageSegmentArrivalRatePct
                 << ","
                 << r.totalSent
                 << ","
                 << r.expectedReceptions
                 << ","
                 << r.actualReceptions
                 << ","
                 << r.lostPackets
                 << ","
                 << r.packetLossRatePct);
  }

  return 0;
}
