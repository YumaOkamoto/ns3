#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
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

NS_LOG_COMPONENT_DEFINE ("Wifi5gComparisonAllMethods");

namespace
{
constexpr uint16_t RECEIVE_PORT = 9001;
constexpr double PI = 3.14159265358979323846;

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
// 比較用 ServerApp
// - 5G版に合わせ、1回の論理送信を各STAへのUDP unicastへ展開する
// - logicalSent と unicastAttempted / unicastSent を分離して計数する
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
      m_logicalSent (0),
      m_unicastAttempted (0),
      m_unicastSent (0),
      m_sendFailures (0)
  {
  }

  void Setup (const std::vector<Ipv4Address>& destinations,
              uint32_t packetSize,
              uint32_t numChannels,
              Time interval,
              Time channelOffset,
              DeliveryMethod method)
  {
    m_destinations = destinations;
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
      for (uint32_t segmentId = 1; segmentId <= nSegments; ++segmentId)
      {
        m_channelData[0].push_back (segmentId);
      }
    }
    else
    {
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

  uint32_t GetLogicalChannelCount () const
  {
    return static_cast<uint32_t> (m_channelData.size ());
  }

  uint64_t GetLogicalSent () const
  {
    return m_logicalSent;
  }

  uint64_t GetUnicastAttempted () const
  {
    return m_unicastAttempted;
  }

  uint64_t GetUnicastSent () const
  {
    return m_unicastSent;
  }

  uint64_t GetSendFailures () const
  {
    return m_sendFailures;
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
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    m_socket->SetPriority (7);

    for (uint32_t channelId = 0; channelId < m_channelData.size (); ++channelId)
    {
      Simulator::Schedule (Seconds (m_channelOffset.GetSeconds () * channelId),
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

    const uint32_t segmentId = m_channelData[channelId][m_indices[channelId]];
    ++m_logicalSent;

    for (const auto& destination : m_destinations)
    {
      Ptr<Packet> packet = Create<Packet> (m_packetSize);
      SeqTsHeader header;
      header.SetSeq (segmentId);
      packet->AddHeader (header);

      ++m_unicastAttempted;
      const int result =
          m_socket->SendTo (packet,
                            0,
                            InetSocketAddress (destination, RECEIVE_PORT));

      if (result >= 0)
      {
        ++m_unicastSent;
      }
      else
      {
        ++m_sendFailures;
      }
    }

    m_indices[channelId] =
        (m_indices[channelId] + 1) % m_channelData[channelId].size ();

    Simulator::Schedule (m_interval,
                         &ServerApp::SendPacket,
                         this,
                         channelId);
  }

  std::vector<Ipv4Address> m_destinations;
  uint32_t m_packetSize;
  uint32_t m_totalSegments;
  Time m_interval;
  Time m_channelOffset;
  bool m_running;

  uint64_t m_logicalSent;
  uint64_t m_unicastAttempted;
  uint64_t m_unicastSent;
  uint64_t m_sendFailures;

  Ptr<Socket> m_socket;
  std::vector<std::vector<uint32_t>> m_channelData;
  std::vector<uint32_t> m_indices;
};

// ============================================================
// ClientApp
// - multicast join は行わず、5G版と同様に各STAがUDP unicastを受信
// - 待ち時間の基準は送信開始時刻（measurementStart）
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
      m_drainTime (Seconds (1.0)),
      m_measurementStart (Seconds (1.0))
  {
  }

  void Setup (uint32_t totalSegments,
              bool verbose,
              Time drainTime,
              Time measurementStart,
              Callback<void> allCompletedCallback)
  {
    m_totalSegments = totalSegments;
    m_verbose = verbose;
    m_drainTime = drainTime;
    m_measurementStart = measurementStart;
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
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());

    const int bindResult =
        m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), RECEIVE_PORT));

    NS_ABORT_MSG_IF (bindResult < 0, "Failed to bind UDP receive socket.");

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

      const uint32_t segmentId = header.GetSeq ();
      if (segmentId == 0 || segmentId > m_totalSegments)
      {
        continue;
      }

      if (m_received[segmentId] != 0)
      {
        continue;
      }

      m_received[segmentId] = 1;
      ++m_receivedCount;

      if (m_receivedCount == m_totalSegments && !m_allReceived)
      {
        m_allReceived = true;
        m_waitTime = (Simulator::Now () - m_measurementStart).GetSeconds ();

        ++g_completedNodes;
        g_waitTimeSum += m_waitTime;
        g_minWaitTime = std::min (g_minWaitTime, m_waitTime);
        g_maxWaitTime = std::max (g_maxWaitTime, m_waitTime);

        if (m_verbose)
        {
          NS_LOG_UNCOND ("Node " << GetNode ()->GetId ()
                                  << " 完了! (待ち時間: "
                                  << m_waitTime << "s)");
        }

        if (g_completedNodes >= g_totalNodes)
        {
          if (!m_allCompletedCallback.IsNull ())
          {
            m_allCompletedCallback ();
          }
          Simulator::Stop (Simulator::Now () + m_drainTime);
        }
      }
    }
  }

  Ptr<Socket> m_socket;
  uint64_t m_rawReceivedCount;
  uint32_t m_totalSegments;
  uint32_t m_receivedCount;
  bool m_allReceived;
  double m_waitTime;
  bool m_verbose;
  Time m_drainTime;
  Time m_measurementStart;
  Callback<void> m_allCompletedCallback;
  std::vector<uint8_t> m_received;
};

// ============================================================
// 設定 / 結果
// ============================================================
struct SimulationConfig
{
  uint32_t nNodes = 5;
  uint32_t kChannels = 6;
  uint32_t packetSize = 1024;

  double interval = 0.1;
  double channelOffset = 0.001;
  double distance = 30.0;

  // 5G版に合わせる
  double receiverStart = 0.9;
  double serverStart = 1.0;
  double simulationTime = 20.0;
  double drainTime = 1.0;

  // 比較用に5Gと送信電力・高さ・大規模伝搬モデルをできるだけ揃える
  double apTxPowerDbm = 30.0;
  double staTxPowerDbm = 23.0;
  double propagationFrequencyGHz = 2.4; // 802.11bとして物理的に整合する周波数
  bool shadowing = false;
  double apHeight = 10.0;
  double staHeight = 1.5;

  // 5GのAMCに近づけるため、Wi-Fi側もデフォルトはレート適応
  // "ideal" または "constant"
  std::string rateManager = "ideal";
  std::string constantDataRate = "DsssRate11Mbps";

  uint32_t seed = 1;
  uint32_t run = 1;

  bool enablePcap = false;
  bool verbose = false;

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
  double distanceM = 0.0;
  double apTxPowerDbm = 0.0;
  double staTxPowerDbm = 0.0;
  double propagationFrequencyGHz = 0.0;
  std::string rateManager;

  uint32_t completedNodes = 0;
  double completionRatePct = 0.0;

  double averageWaitS = 0.0;
  double minimumWaitS = 0.0;
  double maximumWaitS = 0.0;

  double averageSegmentArrivalRatePct = 0.0;

  uint64_t logicalSent = 0;
  uint64_t unicastAttempted = 0;
  uint64_t unicastSent = 0;
  uint64_t sendFailures = 0;
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

std::string
SanitizeFilePart (const std::string& input)
{
  std::string output = input;
  for (char& c : output)
  {
    const unsigned char uc = static_cast<unsigned char> (c);
    if (!(std::isalnum (uc) || c == '-' || c == '_' || c == '.'))
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
  oss << std::fixed << std::setprecision (6) << value;
  std::string s = oss.str ();

  while (!s.empty () && s.back () == '0')
  {
    s.pop_back ();
  }
  if (!s.empty () && s.back () == '.')
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
  oss << "wifi_cmp"
      << "_nodes" << config.nNodes
      << "_k" << config.kChannels
      << "_int" << CompactNumber (config.interval)
      << "_dist" << CompactNumber (config.distance)
      << "_aptx" << CompactNumber (config.apTxPowerDbm)
      << "_statx" << CompactNumber (config.staTxPowerDbm)
      << "_" << SanitizeFilePart (config.rateManager)
      << "_seed" << config.seed
      << "_run" << config.run;

  if (!config.outputTag.empty ())
  {
    oss << "_" << SanitizeFilePart (config.outputTag);
  }
  return oss.str ();
}

// ============================================================
// 1方式の実験
// ============================================================
ExperimentOutput
RunExperiment (const SimulationConfig& config,
               DeliveryMethod method)
{
  ResetGlobalStats (config.nNodes);

  RngSeedManager::SetSeed (config.seed);
  RngSeedManager::SetRun (config.run);

  NodeContainer apNode;
  NodeContainer staNodes;
  apNode.Create (1);
  staNodes.Create (config.nNodes);

  // ------------------------------------------------------------
  // Mobility: 5G版と同様に中心AP + 円周上STA
  // AP 10m / STA 1.5m
  // ------------------------------------------------------------
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, config.apHeight));

  for (uint32_t i = 0; i < config.nNodes; ++i)
  {
    const double angle = 2.0 * PI * static_cast<double> (i) /
                         static_cast<double> (config.nNodes);
    positions->Add (Vector (config.distance * std::cos (angle),
                            config.distance * std::sin (angle),
                            config.staHeight));
  }

  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  NodeContainer allNodes;
  allNodes.Add (apNode);
  allNodes.Add (staNodes);
  mobility.Install (allNodes);

  // ------------------------------------------------------------
  // Wi-Fi PHY/MAC
  // 主要な比較条件を5G版へ寄せる:
  //   - AP TxPower = 30 dBm / STA TxPower = 23 dBm
  //   - UMi Street Canyon path loss
  //   - shadowing OFF
  //   - AP/STAインフラ構成
  //   - per-STA unicast
  // ------------------------------------------------------------
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);

  if (config.rateManager == "ideal")
  {
    wifi.SetRemoteStationManager ("ns3::IdealWifiManager");
  }
  else if (config.rateManager == "constant")
  {
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",
                                  StringValue (config.constantDataRate),
                                  "ControlMode",
                                  StringValue (config.constantDataRate));
  }
  else
  {
    NS_ABORT_MSG ("rateManager must be 'ideal' or 'constant'.");
  }

  Ptr<ThreeGppUmiStreetCanyonChannelConditionModel> conditionModel =
      CreateObject<ThreeGppUmiStreetCanyonChannelConditionModel> ();

  YansWifiChannelHelper channel;
  channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss (
      "ns3::ThreeGppUmiStreetCanyonPropagationLossModel",
      "Frequency",
      DoubleValue (config.propagationFrequencyGHz * 1e9),
      "ShadowingEnabled",
      BooleanValue (config.shadowing),
      "ChannelConditionModel",
      PointerValue (conditionModel));

  YansWifiPhyHelper phy;
  phy.SetChannel (channel.Create ());
  phy.Set ("TxGain", DoubleValue (0.0));
  phy.Set ("RxGain", DoubleValue (0.0));

  WifiMacHelper mac;
  Ssid ssid = Ssid ("wifi-5g-comparison");

  // STA側: 5G UEで一般的な23 dBmを比較用デフォルトにする
  phy.Set ("TxPowerStart", DoubleValue (config.staTxPowerDbm));
  phy.Set ("TxPowerEnd", DoubleValue (config.staTxPowerDbm));
  mac.SetType ("ns3::StaWifiMac",
               "Ssid",
               SsidValue (ssid),
               "ActiveProbing",
               BooleanValue (false));
  NetDeviceContainer staDevices = wifi.Install (phy, mac, staNodes);

  // AP側: 5G gNB設定の30 dBmに揃える
  phy.Set ("TxPowerStart", DoubleValue (config.apTxPowerDbm));
  phy.Set ("TxPowerEnd", DoubleValue (config.apTxPowerDbm));
  mac.SetType ("ns3::ApWifiMac",
               "Ssid",
               SsidValue (ssid));
  NetDeviceContainer apDevice = wifi.Install (phy, mac, apNode);

  // ------------------------------------------------------------
  // Internet
  // ------------------------------------------------------------
  InternetStackHelper stack;
  stack.Install (allNodes);

  TrafficControlHelper trafficControl;
  trafficControl.SetRootQueueDisc ("ns3::FifoQueueDisc");
  trafficControl.Install (apDevice);
  trafficControl.Install (staDevices);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.0.0", "255.255.0.0");

  Ipv4InterfaceContainer apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterfaces = address.Assign (staDevices);

  std::vector<Ipv4Address> destinations;
  destinations.reserve (config.nNodes);
  for (uint32_t i = 0; i < config.nNodes; ++i)
  {
    destinations.push_back (staInterfaces.GetAddress (i));
  }

  // ------------------------------------------------------------
  // Application
  // ------------------------------------------------------------
  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (destinations,
                    config.packetSize,
                    config.kChannels,
                    Seconds (config.interval),
                    Seconds (config.channelOffset),
                    method);

  apNode.Get (0)->AddApplication (serverApp);
  serverApp->SetStartTime (Seconds (config.serverStart));

  const uint32_t totalSegments = serverApp->GetTotalSegments ();

  for (uint32_t i = 0; i < config.nNodes; ++i)
  {
    Ptr<ClientApp> clientApp = CreateObject<ClientApp> ();
    clientApp->Setup (totalSegments,
                      config.verbose,
                      Seconds (config.drainTime),
                      Seconds (config.serverStart),
                      MakeCallback (&ServerApp::StopSending, serverApp));

    staNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (config.receiverStart));
  }

  if (config.enablePcap)
  {
    const std::string pcapPrefix =
        config.outputDir + "/" + BuildOutputBaseName (config) + "_" +
        MethodName (method);
    phy.EnablePcap (pcapPrefix, apDevice.Get (0));
  }

  // ------------------------------------------------------------
  // 設定表示
  // ------------------------------------------------------------
  NS_LOG_UNCOND ("\n================================================");
  NS_LOG_UNCOND ("【Wi-Fi比較実験設定：" << MethodDisplayName (method) << "】");
  NS_LOG_UNCOND ("配信手法              : " << MethodDisplayName (method));
  NS_LOG_UNCOND ("構成                  : AP -> STA (Infrastructure)");
  NS_LOG_UNCOND ("アプリ通信            : DL UDP unicast / STA");
  NS_LOG_UNCOND ("AP数                  : 1");
  NS_LOG_UNCOND ("STA数                 : " << config.nNodes);
  NS_LOG_UNCOND ("論理チャネル数        : " << serverApp->GetLogicalChannelCount ());
  NS_LOG_UNCOND ("総セグメント数(N)     : " << totalSegments);
  NS_LOG_UNCOND ("チャネル送信間隔      : " << config.interval << "s");
  NS_LOG_UNCOND ("チャネル初期ずれ      : " << config.channelOffset << "s");
  NS_LOG_UNCOND ("受信ポート            : " << RECEIVE_PORT);
  NS_LOG_UNCOND ("STA受信App開始        : " << config.receiverStart << "s");
  NS_LOG_UNCOND ("配信・待ち時間測定開始: " << config.serverStart << "s");
  NS_LOG_UNCOND ("Payload               : " << config.packetSize << " bytes");
  NS_LOG_UNCOND ("------------------------------------------------");
  NS_LOG_UNCOND ("Wi-Fi規格             : IEEE 802.11b");
  NS_LOG_UNCOND ("Rate manager          : " << config.rateManager);
  if (config.rateManager == "constant")
  {
    NS_LOG_UNCOND ("固定データレート      : " << config.constantDataRate);
  }
  NS_LOG_UNCOND ("Propagation           : 3GPP UMi-StreetCanyon");
  NS_LOG_UNCOND ("Propagation frequency : " << config.propagationFrequencyGHz << " GHz");
  NS_LOG_UNCOND ("Shadowing             : " << (config.shadowing ? "ON" : "OFF"));
  NS_LOG_UNCOND ("AP TxPower            : " << config.apTxPowerDbm << " dBm");
  NS_LOG_UNCOND ("STA TxPower           : " << config.staTxPowerDbm << " dBm");
  NS_LOG_UNCOND ("STA配置               : AP中心の円周上に等間隔");
  NS_LOG_UNCOND ("全STAの水平半径       : " << config.distance << "m");
  NS_LOG_UNCOND ("AP高さ / STA高さ      : " << config.apHeight << "m / "
                                                   << config.staHeight << "m");
  NS_LOG_UNCOND ("Seed / Run            : " << config.seed << " / " << config.run);
  NS_LOG_UNCOND ("================================================");

  Simulator::Stop (Seconds (config.simulationTime));
  Simulator::Run ();

  // ------------------------------------------------------------
  // 結果回収
  // ------------------------------------------------------------
  ExperimentOutput output;
  output.nodes.reserve (config.nNodes);

  double totalArrivalRate = 0.0;
  uint64_t totalRawReceived = 0;

  for (uint32_t i = 0; i < staNodes.GetN (); ++i)
  {
    Ptr<ClientApp> app =
        DynamicCast<ClientApp> (staNodes.Get (i)->GetApplication (0));

    const uint32_t receivedUnique = app->GetReceivedCount ();
    const uint32_t missingUnique = totalSegments - receivedUnique;
    const double arrivalRate =
        (static_cast<double> (receivedUnique) / totalSegments) * 100.0;

    totalArrivalRate += arrivalRate;
    totalRawReceived += app->GetRawReceivedCount ();

    NodeResult nodeResult;
    nodeResult.method = MethodName (method);
    nodeResult.clientIndex = i;
    nodeResult.nodeId = staNodes.Get (i)->GetId ();
    nodeResult.completed = app->IsComplete ();
    nodeResult.waitTimeS = app->GetWaitTime ();
    nodeResult.receivedUniqueSegments = receivedUnique;
    nodeResult.missingUniqueSegments = missingUnique;
    nodeResult.totalSegments = totalSegments;
    nodeResult.segmentArrivalRatePct = arrivalRate;
    nodeResult.rawReceivedPackets = app->GetRawReceivedCount ();
    output.nodes.push_back (nodeResult);
  }

  const double averageArrivalRate = totalArrivalRate / config.nNodes;
  const double completionRate =
      (static_cast<double> (g_completedNodes) / config.nNodes) * 100.0;

  const double averageWaitTime =
      (g_completedNodes == 0) ? 0.0 : g_waitTimeSum / g_completedNodes;
  const double minimumWaitTime =
      (g_completedNodes == 0) ? 0.0 : g_minWaitTime;
  const double maximumWaitTime =
      (g_completedNodes == 0) ? 0.0 : g_maxWaitTime;

  const uint64_t unicastSent = serverApp->GetUnicastSent ();
  const uint64_t lostPackets =
      (unicastSent >= totalRawReceived) ? unicastSent - totalRawReceived : 0;

  double packetLossRate = 0.0;
  if (unicastSent > 0)
  {
    packetLossRate =
        (1.0 - static_cast<double> (totalRawReceived) / unicastSent) * 100.0;
  }
  packetLossRate = std::max (0.0, packetLossRate);

  ExperimentResult& result = output.summary;
  result.environment = "WiFi-802.11b-comparison";
  result.method = MethodName (method);
  result.nodes = config.nNodes;
  result.logicalChannels = serverApp->GetLogicalChannelCount ();
  result.totalSegments = totalSegments;
  result.intervalS = config.interval;
  result.channelOffsetS = config.channelOffset;
  result.distanceM = config.distance;
  result.apTxPowerDbm = config.apTxPowerDbm;
  result.staTxPowerDbm = config.staTxPowerDbm;
  result.propagationFrequencyGHz = config.propagationFrequencyGHz;
  result.rateManager = config.rateManager;
  result.completedNodes = g_completedNodes;
  result.completionRatePct = completionRate;
  result.averageWaitS = averageWaitTime;
  result.minimumWaitS = minimumWaitTime;
  result.maximumWaitS = maximumWaitTime;
  result.averageSegmentArrivalRatePct = averageArrivalRate;
  result.logicalSent = serverApp->GetLogicalSent ();
  result.unicastAttempted = serverApp->GetUnicastAttempted ();
  result.unicastSent = unicastSent;
  result.sendFailures = serverApp->GetSendFailures ();
  result.actualReceptions = totalRawReceived;
  result.lostPackets = lostPackets;
  result.packetLossRatePct = packetLossRate;

  NS_LOG_UNCOND ("\n==== Wi-Fi比較 集計結果: " << MethodDisplayName (method) << " ====");
  NS_LOG_UNCOND ("完了STA数               : " << result.completedNodes << " / " << result.nodes);
  NS_LOG_UNCOND ("完了率                  : " << result.completionRatePct << "%");
  if (result.completedNodes == 0)
  {
    NS_LOG_UNCOND ("平均待ち時間            : N/A");
    NS_LOG_UNCOND ("最小待ち時間            : N/A");
    NS_LOG_UNCOND ("最大待ち時間            : N/A");
  }
  else
  {
    NS_LOG_UNCOND ("平均待ち時間            : " << result.averageWaitS << "s");
    NS_LOG_UNCOND ("最小待ち時間            : " << result.minimumWaitS << "s");
    NS_LOG_UNCOND ("最大待ち時間            : " << result.maximumWaitS << "s");
  }
  NS_LOG_UNCOND ("平均セグメント到達率    : " << result.averageSegmentArrivalRatePct << "%");
  NS_LOG_UNCOND ("論理セグメント送信回数  : " << result.logicalSent);
  NS_LOG_UNCOND ("STA向けUnicast試行数    : " << result.unicastAttempted);
  NS_LOG_UNCOND ("STA向けUnicast送信成功数: " << result.unicastSent);
  NS_LOG_UNCOND ("Application送信失敗数   : " << result.sendFailures);
  NS_LOG_UNCOND ("実受信パケット数        : " << result.actualReceptions);
  NS_LOG_UNCOND ("観測損失パケット数      : " << result.lostPackets);
  NS_LOG_UNCOND ("観測パケット欠損率      : " << result.packetLossRatePct << "%");

  Simulator::Destroy ();
  return output;
}

// ============================================================
// CSV
// ============================================================
void
WriteSummaryCsv (const std::vector<ExperimentResult>& results,
                 const std::string& filename)
{
  std::ofstream file (filename, std::ios::out | std::ios::trunc);
  NS_ABORT_MSG_IF (!file.is_open (), "Cannot open summary CSV: " << filename);

  file << std::setprecision (15);
  file << "environment,method,nodes,logical_channels,total_segments,"
       << "interval_s,channel_offset_s,distance_m,ap_tx_power_dbm,sta_tx_power_dbm,"
       << "propagation_frequency_ghz,rate_manager,"
       << "completed_nodes,completion_rate_pct,avg_wait_s,min_wait_s,max_wait_s,"
       << "avg_segment_arrival_rate_pct,logical_sent,unicast_attempted,"
       << "unicast_sent,send_failures,actual_rx,lost,loss_rate_pct\n";

  for (const auto& r : results)
  {
    file << r.environment << ","
         << r.method << ","
         << r.nodes << ","
         << r.logicalChannels << ","
         << r.totalSegments << ","
         << r.intervalS << ","
         << r.channelOffsetS << ","
         << r.distanceM << ","
         << r.apTxPowerDbm << ","
         << r.staTxPowerDbm << ","
         << r.propagationFrequencyGHz << ","
         << r.rateManager << ","
         << r.completedNodes << ","
         << r.completionRatePct << ",";

    if (r.completedNodes > 0)
    {
      file << r.averageWaitS << ","
           << r.minimumWaitS << ","
           << r.maximumWaitS;
    }
    else
    {
      file << ",,";
    }

    file << ","
         << r.averageSegmentArrivalRatePct << ","
         << r.logicalSent << ","
         << r.unicastAttempted << ","
         << r.unicastSent << ","
         << r.sendFailures << ","
         << r.actualReceptions << ","
         << r.lostPackets << ","
         << r.packetLossRatePct << "\n";
  }
}

void
WriteNodeResultsCsv (const std::vector<NodeResult>& results,
                     const std::string& filename)
{
  std::ofstream file (filename, std::ios::out | std::ios::trunc);
  NS_ABORT_MSG_IF (!file.is_open (), "Cannot open node CSV: " << filename);

  file << std::setprecision (15);
  file << "method,client_index,node_id,completed,wait_time_s,"
       << "received_unique_segments,missing_unique_segments,total_segments,"
       << "segment_arrival_rate_pct,raw_received_packets\n";

  for (const auto& r : results)
  {
    file << r.method << ","
         << r.clientIndex << ","
         << r.nodeId << ","
         << (r.completed ? 1 : 0) << ",";

    if (r.completed)
    {
      file << r.waitTimeS;
    }

    file << ","
         << r.receivedUniqueSegments << ","
         << r.missingUniqueSegments << ","
         << r.totalSegments << ","
         << r.segmentArrivalRatePct << ","
         << r.rawReceivedPackets << "\n";
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
  cmd.AddValue ("nodes", "Number of STA nodes", config.nNodes);
  cmd.AddValue ("k", "Number of logical channels for FB", config.kChannels);
  cmd.AddValue ("int", "Per-logical-channel transmission interval (s)", config.interval);
  cmd.AddValue ("channelOffset", "Initial offset between logical channels (s)", config.channelOffset);
  cmd.AddValue ("dist", "AP-STA horizontal distance (m)", config.distance);
  cmd.AddValue ("packetSize", "Application payload size in bytes", config.packetSize);
  cmd.AddValue ("receiverStart", "STA receive application start time (s)", config.receiverStart);
  cmd.AddValue ("serverStart", "Server transmission / measurement start time (s)", config.serverStart);
  cmd.AddValue ("simTime", "Maximum simulation time for each method (s)", config.simulationTime);
  cmd.AddValue ("drainTime", "Queue drain time after all clients complete (s)", config.drainTime);

  cmd.AddValue ("apTxPower", "Wi-Fi AP Tx power (dBm)", config.apTxPowerDbm);
  cmd.AddValue ("staTxPower", "Wi-Fi STA Tx power (dBm)", config.staTxPowerDbm);
  cmd.AddValue ("frequencyGHz", "Frequency used by UMi pathloss model (GHz)", config.propagationFrequencyGHz);
  cmd.AddValue ("shadowing", "Enable UMi shadowing", config.shadowing);
  cmd.AddValue ("apHeight", "AP height (m)", config.apHeight);
  cmd.AddValue ("staHeight", "STA height (m)", config.staHeight);
  cmd.AddValue ("rateManager", "Wi-Fi rate manager: ideal or constant", config.rateManager);
  cmd.AddValue ("rate", "Data rate when rateManager=constant", config.constantDataRate);

  cmd.AddValue ("seed", "Random seed", config.seed);
  cmd.AddValue ("run", "Run number", config.run);
  cmd.AddValue ("pcap", "Enable PCAP output", config.enablePcap);
  cmd.AddValue ("verbose", "Enable per-node console output", config.verbose);
  cmd.AddValue ("outputDir", "Directory for CSV/PCAP results", config.outputDir);
  cmd.AddValue ("tag", "Optional tag appended to output file names", config.outputTag);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (config.nNodes == 0, "nodes must be at least 1.");
  NS_ABORT_MSG_IF (config.kChannels == 0 || config.kChannels >= 31,
                   "k must be between 1 and 30.");
  NS_ABORT_MSG_IF (config.interval <= 0.0, "int must be greater than 0.");
  NS_ABORT_MSG_IF (config.channelOffset < 0.0,
                   "channelOffset must be 0 or greater.");
  NS_ABORT_MSG_IF (config.packetSize == 0, "packetSize must be at least 1.");
  NS_ABORT_MSG_IF (config.receiverStart < 0.0, "receiverStart must be >= 0.");
  NS_ABORT_MSG_IF (config.serverStart <= config.receiverStart,
                   "serverStart must be later than receiverStart.");
  NS_ABORT_MSG_IF (config.simulationTime <= config.serverStart,
                   "simTime must be later than serverStart.");
  NS_ABORT_MSG_IF (config.drainTime < 0.0, "drainTime must be >= 0.");
  NS_ABORT_MSG_IF (config.apTxPowerDbm <= 0.0, "apTxPower must be > 0 dBm.");
  NS_ABORT_MSG_IF (config.staTxPowerDbm <= 0.0, "staTxPower must be > 0 dBm.");
  NS_ABORT_MSG_IF (config.propagationFrequencyGHz < 0.5 ||
                       config.propagationFrequencyGHz > 100.0,
                   "frequencyGHz must be in the UMi model range [0.5, 100].");
  NS_ABORT_MSG_IF (config.distance < 10.0,
                   "UMi Street Canyon comparison should use dist >= 10 m.");
  NS_ABORT_MSG_IF (config.outputDir.empty (), "outputDir must not be empty.");

  std::error_code directoryError;
  std::filesystem::create_directories (config.outputDir, directoryError);
  NS_ABORT_MSG_IF (directoryError,
                   "Failed to create output directory '"
                       << config.outputDir << "': " << directoryError.message ());

  const std::vector<DeliveryMethod> methods = {
      DeliveryMethod::FB,
      DeliveryMethod::SIMPLE,
      DeliveryMethod::FULL};

  std::vector<ExperimentResult> summaries;
  std::vector<NodeResult> allNodeResults;
  summaries.reserve (methods.size ());
  allNodeResults.reserve (static_cast<size_t> (config.nNodes) * methods.size ());

  for (DeliveryMethod method : methods)
  {
    ExperimentOutput output = RunExperiment (config, method);
    summaries.push_back (output.summary);
    allNodeResults.insert (allNodeResults.end (),
                           output.nodes.begin (),
                           output.nodes.end ());
  }

  const std::string baseName = BuildOutputBaseName (config);
  const std::string summaryFilename =
      config.outputDir + "/" + baseName + "_summary.csv";
  const std::string nodeFilename =
      config.outputDir + "/" + baseName + "_nodes.csv";

  WriteSummaryCsv (summaries, summaryFilename);
  WriteNodeResultsCsv (allNodeResults, nodeFilename);

  NS_LOG_UNCOND ("\n================================================");
  NS_LOG_UNCOND ("Wi-Fi比較版: 3方式すべての実験が完了しました。");
  NS_LOG_UNCOND ("Summary CSV : " << summaryFilename);
  NS_LOG_UNCOND ("Node CSV    : " << nodeFilename);
  NS_LOG_UNCOND ("================================================");

  NS_LOG_UNCOND ("\n=== Wi-Fi比較版 3方式比較 ===");
  NS_LOG_UNCOND ("Method,Completed,CompletionRate(%),AvgWait(s),MinWait(s),MaxWait(s),"
                 "AvgArrival(%),LogicalSent,UnicastAttempted,UnicastSent,"
                 "SendFailures,ActualRx,Lost,LossRate(%)");

  for (const auto& r : summaries)
  {
    std::ostringstream waitFields;
    if (r.completedNodes == 0)
    {
      waitFields << "N/A,N/A,N/A";
    }
    else
    {
      waitFields << r.averageWaitS << "," << r.minimumWaitS << "," << r.maximumWaitS;
    }

    NS_LOG_UNCOND (r.method << ","
                            << r.completedNodes << "/" << r.nodes << ","
                            << r.completionRatePct << ","
                            << waitFields.str () << ","
                            << r.averageSegmentArrivalRatePct << ","
                            << r.logicalSent << ","
                            << r.unicastAttempted << ","
                            << r.unicastSent << ","
                            << r.sendFailures << ","
                            << r.actualReceptions << ","
                            << r.lostPackets << ","
                            << r.packetLossRatePct);
  }

  return 0;
}
