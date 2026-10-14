#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <cctype>
#include <cmath>
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

NS_LOG_COMPONENT_DEFINE ("NrSegmentedBroadcastingAllMethods");

namespace
{
constexpr uint16_t RECEIVE_PORT = 9001;

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
//
// 5G-LENA の通常の gNB -> UE ダウンリンクを使う基礎版。
// Wi-Fi版の「1回のIP multicast」をそのまま再現するのではなく、
// 1つの論理セグメントを全UEへ個別UDP unicast送信する。
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
      m_logicalSentCount (0),
      m_unicastAttemptCount (0),
      m_unicastSentCount (0)
  {
  }

  void Setup (const std::vector<Ipv4Address>& destinations,
              uint32_t packetSize,
              uint32_t numChannels,
              Time interval,
              Time channelOffset,
              DeliveryMethod method)
  {
    NS_ABORT_MSG_IF (destinations.empty (),
                     "No UE destination addresses.");
    NS_ABORT_MSG_IF (numChannels == 0 || numChannels >= 31,
                     "k must be between 1 and 30.");

    m_destinations = destinations;
    m_packetSize = packetSize;
    m_interval = interval;
    m_channelOffset = channelOffset;

    const uint32_t nSegments =
        (1u << numChannels) - 1u;

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

      for (uint32_t channelId = 0;
           channelId < numChannels;
           ++channelId)
      {
        const uint32_t segmentsInChannel =
            (1u << channelId);

        for (uint32_t j = 0;
             j < segmentsInChannel;
             ++j)
        {
          m_channelData[channelId]
              .push_back (currentSegment++);
        }
      }
    }
    else if (method == DeliveryMethod::SIMPLE)
    {
      for (uint32_t segmentId = 1;
           segmentId <= nSegments;
           ++segmentId)
      {
        m_channelData[0]
            .push_back (segmentId);
      }
    }
    else
    {
      for (uint32_t channelId = 0;
           channelId < nSegments;
           ++channelId)
      {
        m_channelData[channelId]
            .push_back (channelId + 1);
      }
    }
  }

  uint32_t GetTotalSegments () const
  {
    return m_totalSegments;
  }

  uint32_t GetLogicalChannelCount () const
  {
    return static_cast<uint32_t> (
        m_channelData.size ());
  }

  uint64_t GetLogicalSentCount () const
  {
    return m_logicalSentCount;
  }

  uint64_t GetUnicastAttemptCount () const
  {
    return m_unicastAttemptCount;
  }

  uint64_t GetUnicastSentCount () const
  {
    return m_unicastSentCount;
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

    m_socket =
        Socket::CreateSocket (
            GetNode (),
            UdpSocketFactory::GetTypeId ());

    const int bindResult =
        m_socket->Bind ();

    NS_ABORT_MSG_IF (
        bindResult < 0,
        "Failed to bind server UDP socket.");

    for (uint32_t channelId = 0;
         channelId < m_channelData.size ();
         ++channelId)
    {
      Simulator::Schedule (
          Seconds (
              m_channelOffset.GetSeconds () *
              channelId),
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
        m_channelData[channelId]
                     [m_indices[channelId]];

    // 「論理的には1セグメントを1回送った」と数える
    ++m_logicalSentCount;

    // 通常NRのDL基礎版では、各UEへ個別unicast
    for (const auto& destination :
         m_destinations)
    {
      ++m_unicastAttemptCount;

      Ptr<Packet> packet =
          Create<Packet> (m_packetSize);

      SeqTsHeader header;
      header.SetSeq (segmentId);
      packet->AddHeader (header);

      const int result =
          m_socket->SendTo (
              packet,
              0,
              InetSocketAddress (
                  destination,
                  RECEIVE_PORT));

      if (result >= 0)
      {
        ++m_unicastSentCount;
      }
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

  std::vector<Ipv4Address> m_destinations;

  uint32_t m_packetSize;
  uint32_t m_totalSegments;

  Time m_interval;
  Time m_channelOffset;

  bool m_running;

  uint64_t m_logicalSentCount;
  uint64_t m_unicastAttemptCount;
  uint64_t m_unicastSentCount;

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
      m_drainTime (Seconds (0.1)),
      m_measurementStart (Seconds (0))
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
    m_allCompletedCallback =
        allCompletedCallback;

    m_received.assign (
        m_totalSegments + 1,
        0);
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
    m_socket =
        Socket::CreateSocket (
            GetNode (),
            UdpSocketFactory::GetTypeId ());

    const int bindResult =
        m_socket->Bind (
            InetSocketAddress (
                Ipv4Address::GetAny (),
                RECEIVE_PORT));

    NS_ABORT_MSG_IF (
        bindResult < 0,
        "Failed to bind UE UDP receive socket.");

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

    while ((packet =
                socket->RecvFrom (from)))
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

      if (m_receivedCount ==
              m_totalSegments &&
          !m_allReceived)
      {
        m_allReceived = true;

        m_waitTime =
            (Simulator::Now () -
             m_measurementStart)
                .GetSeconds ();

        ++g_completedNodes;

        g_waitTimeSum +=
            m_waitTime;

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
              "UE Node "
              << GetNode ()->GetId ()
              << " 完了! (待ち時間: "
              << m_waitTime
              << "s)");
        }

        if (g_completedNodes >=
            g_totalNodes)
        {
          if (!m_allCompletedCallback
                   .IsNull ())
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
// 設定・結果構造体
// ============================================================
struct SimulationConfig
{
  // 配信
  uint32_t nUes = 5;
  uint32_t kChannels = 6;
  uint32_t packetSize = 1024;

  double interval = 0.1;
  double channelOffset = 0.001;

  // NR無線
  double centralFrequency = 3.5e9; // Hz
  double bandwidth = 20e6;         // Hz
  uint16_t numerology = 1;

  double gnbTxPowerDbm = 30.0;

  bool enableOfdma = true;

  // 明示的に固定して再現性を確保
  std::string tddPattern =
      "DL|S|UL|UL|DL|DL|S|UL|UL|DL|";

  // 配置
  double distance = 30.0;
  double gnbHeight = 10.0;
  double ueHeight = 1.5;

  // 時刻
  double clientStart = 0.9;
  double serverStart = 1.0;
  double simulationTime = 400.0;
  double drainTime = 0.1;

  // EPC / RemoteHost
  std::string remoteHostRate = "100Gb/s";
  uint32_t remoteHostMtu = 2500;
  double remoteHostDelay = 0.0; // s
  double s1uDelayMs = 0.0;

  // 乱数
  uint32_t seed = 1;
  uint32_t run = 1;

  bool verbose = false;

  // CSV
  std::string outputDir = "results";
  std::string outputTag = "";
};

struct ExperimentResult
{
  std::string environment;
  std::string architecture;
  std::string trafficType;
  std::string method;

  uint32_t gnbCount = 1;
  uint32_t ueCount = 0;

  uint32_t logicalChannels = 0;
  uint32_t totalSegments = 0;

  double intervalS = 0.0;
  double channelOffsetS = 0.0;
  uint16_t receivePort = 0;

  double clientStartS = 0.0;
  double serverStartS = 0.0;
  double simulationTimeS = 0.0;
  double drainTimeS = 0.0;

  uint32_t packetSizeBytes = 0;

  double centralFrequencyHz = 0.0;
  double bandwidthHz = 0.0;
  uint16_t numerology = 0;

  double gnbTxPowerDbm = 0.0;
  std::string scheduler;
  std::string tddPattern;

  std::string channelScenario;
  bool shadowingEnabled = false;
  std::string beamforming;

  uint32_t gnbAntennaRows = 1;
  uint32_t gnbAntennaColumns = 1;
  uint32_t ueAntennaRows = 1;
  uint32_t ueAntennaColumns = 1;

  double distanceM = 0.0;
  double gnbHeightM = 0.0;
  double ueHeightM = 0.0;

  std::string remoteHostRate;
  uint32_t remoteHostMtu = 0;
  double remoteHostDelayS = 0.0;
  double s1uDelayMs = 0.0;

  uint32_t seed = 0;
  uint32_t run = 0;

  uint32_t completedUes = 0;
  double completionRatePct = 0.0;

  double averageWaitS = 0.0;
  double minimumWaitS = 0.0;
  double maximumWaitS = 0.0;

  double averageSegmentArrivalRatePct = 0.0;

  uint64_t logicalSentPackets = 0;
  uint64_t unicastAttemptedPackets = 0;
  uint64_t unicastSentPackets = 0;
  uint64_t applicationSendFailures = 0;

  uint64_t actualReceptions = 0;
  uint64_t lostPackets = 0;
  double packetLossRatePct = 0.0;
};

struct NodeResult
{
  std::string method;

  uint32_t ueIndex = 0;
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

// ============================================================
// 補助関数
// ============================================================
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

  std::string s =
      oss.str ();

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
BuildOutputBaseName (
    const SimulationConfig& config)
{
  std::ostringstream oss;

  oss << "5gnr"
      << "_ues" << config.nUes
      << "_k" << config.kChannels
      << "_int"
      << CompactNumber (config.interval)
      << "_dist"
      << CompactNumber (config.distance)
      << "_fGHz"
      << CompactNumber (
             config.centralFrequency /
             1e9)
      << "_bwMHz"
      << CompactNumber (
             config.bandwidth /
             1e6)
      << "_mu"
      << config.numerology
      << "_seed"
      << config.seed
      << "_run"
      << config.run;

  if (!config.outputTag.empty ())
  {
    oss << "_"
        << SanitizeFilePart (
               config.outputTag);
  }

  return oss.str ();
}

// ============================================================
// 1方式の5G NRシミュレーション
// ============================================================
ExperimentOutput
RunExperiment (
    const SimulationConfig& config,
    DeliveryMethod method)
{
  ResetGlobalStats (
      config.nUes);

  RngSeedManager::SetSeed (
      config.seed);

  RngSeedManager::SetRun (
      config.run);

  // ------------------------------------------------------------
  // Node
  // ------------------------------------------------------------
  NodeContainer gnbNodes;
  NodeContainer ueNodes;

  gnbNodes.Create (1);
  ueNodes.Create (
      config.nUes);

  // ------------------------------------------------------------
  // Mobility
  //
  // gNB = (0,0,h)
  //
  // 3GPP Spectrum channel modelでは、異なる無線デバイスを
  // 完全に同一座標へ配置できない。
  //
  // そこで全UEをgNBを中心とする円周上へ均等配置する。
  // これにより、
  //   - 全UEのgNBからの水平距離は同じ config.distance
  //   - UE同士の座標は重複しない
  // を両立する。
  // ------------------------------------------------------------
  MobilityHelper gnbMobility;

  Ptr<ListPositionAllocator>
      gnbPosition =
          CreateObject<ListPositionAllocator> ();

  gnbPosition->Add (
      Vector (
          0.0,
          0.0,
          config.gnbHeight));

  gnbMobility.SetPositionAllocator (
      gnbPosition);

  gnbMobility.SetMobilityModel (
      "ns3::ConstantPositionMobilityModel");

  gnbMobility.Install (
      gnbNodes);

  MobilityHelper ueMobility;

  Ptr<ListPositionAllocator>
      uePosition =
          CreateObject<ListPositionAllocator> ();

  const double pi =
      std::acos (-1.0);

  for (uint32_t i = 0;
       i < config.nUes;
       ++i)
  {
    const double angle =
        (config.nUes == 1)
            ? 0.0
            : (2.0 * pi *
               static_cast<double> (i) /
               static_cast<double> (config.nUes));

    const double x =
        config.distance *
        std::cos (angle);

    const double y =
        config.distance *
        std::sin (angle);

    uePosition->Add (
        Vector (
            x,
            y,
            config.ueHeight));
  }

  ueMobility.SetPositionAllocator (
      uePosition);

  ueMobility.SetMobilityModel (
      "ns3::ConstantPositionMobilityModel");

  ueMobility.Install (
      ueNodes);

  // ------------------------------------------------------------
  // 5G-LENA helper
  // ------------------------------------------------------------
  Ptr<NrPointToPointEpcHelper>
      epcHelper =
          CreateObject<NrPointToPointEpcHelper> ();

  Ptr<IdealBeamformingHelper>
      beamformingHelper =
          CreateObject<IdealBeamformingHelper> ();

  Ptr<NrHelper>
      nrHelper =
          CreateObject<NrHelper> ();

  nrHelper->SetBeamformingHelper (
      beamformingHelper);

  nrHelper->SetEpcHelper (
      epcHelper);

  // 初期評価ではシャドウイングをOFF
  nrHelper->SetPathlossAttribute (
      "ShadowingEnabled",
      BooleanValue (false));

  epcHelper->SetAttribute (
      "S1uLinkDelay",
      TimeValue (
          MilliSeconds (
              config.s1uDelayMs)));

  Config::SetDefault (
      "ns3::ThreeGppChannelModel::UpdatePeriod",
      TimeValue (
          MilliSeconds (0)));

  nrHelper->SetChannelConditionModelAttribute (
      "UpdatePeriod",
      TimeValue (
          MilliSeconds (0)));

  // Scheduler
  if (config.enableOfdma)
  {
    nrHelper->SetSchedulerTypeId (
        TypeId::LookupByName (
            "ns3::NrMacSchedulerOfdmaRR"));
  }
  else
  {
    nrHelper->SetSchedulerTypeId (
        TypeId::LookupByName (
            "ns3::NrMacSchedulerTdmaRR"));
  }

  // Direct-path ideal beamforming
  beamformingHelper->SetAttribute (
      "BeamformingMethod",
      TypeIdValue (
          DirectPathBeamforming::GetTypeId ()));

  // 基礎比較: 1x1 isotropic antenna
  nrHelper->SetUeAntennaAttribute (
      "NumRows",
      UintegerValue (1));

  nrHelper->SetUeAntennaAttribute (
      "NumColumns",
      UintegerValue (1));

  nrHelper->SetUeAntennaAttribute (
      "AntennaElement",
      PointerValue (
          CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetGnbAntennaAttribute (
      "NumRows",
      UintegerValue (1));

  nrHelper->SetGnbAntennaAttribute (
      "NumColumns",
      UintegerValue (1));

  nrHelper->SetGnbAntennaAttribute (
      "AntennaElement",
      PointerValue (
          CreateObject<IsotropicAntennaModel> ()));

  // ------------------------------------------------------------
  // Spectrum:
  // 1 Operation Band / 1 CC / 1 BWP
  // UMi Street Canyon
  // ------------------------------------------------------------
  BandwidthPartInfoPtrVector allBwps;

  CcBwpCreator ccBwpCreator;

  OperationBandInfo band;

  const uint8_t numOfCcs = 1;

  CcBwpCreator::SimpleOperationBandConf
      bandConf (
          config.centralFrequency,
          config.bandwidth,
          numOfCcs,
          BandwidthPartInfo::
              UMi_StreetCanyon);

  bandConf.m_numBwp = 1;

  band =
      ccBwpCreator
          .CreateOperationBandContiguousCc (
              bandConf);

  nrHelper->InitializeOperationBand (
      &band);

  allBwps =
      CcBwpCreator::GetAllBwps (
          {band});

  // ------------------------------------------------------------
  // NR Device install
  // ------------------------------------------------------------
  NetDeviceContainer gnbNetDev =
      nrHelper->InstallGnbDevice (
          gnbNodes,
          allBwps);

  NetDeviceContainer ueNetDev =
      nrHelper->InstallUeDevice (
          ueNodes,
          allBwps);

  int64_t stream = 1;

  stream +=
      nrHelper->AssignStreams (
          gnbNetDev,
          stream);

  stream +=
      nrHelper->AssignStreams (
          ueNetDev,
          stream);

  // gNB PHY
  nrHelper
      ->GetGnbPhy (
          gnbNetDev.Get (0),
          0)
      ->SetAttribute (
          "Numerology",
          UintegerValue (
              config.numerology));

  nrHelper
      ->GetGnbPhy (
          gnbNetDev.Get (0),
          0)
      ->SetAttribute (
          "TxPower",
          DoubleValue (
              config.gnbTxPowerDbm));

  nrHelper
      ->GetGnbPhy (
          gnbNetDev.Get (0),
          0)
      ->SetAttribute (
          "Pattern",
          StringValue (
              config.tddPattern));

  // v2.5系では設定後にUpdateConfigを明示
  for (auto it =
           gnbNetDev.Begin ();
       it != gnbNetDev.End ();
       ++it)
  {
    DynamicCast<NrGnbNetDevice> (*it)
        ->UpdateConfig ();
  }

  for (auto it =
           ueNetDev.Begin ();
       it != ueNetDev.End ();
       ++it)
  {
    DynamicCast<NrUeNetDevice> (*it)
        ->UpdateConfig ();
  }

  // ------------------------------------------------------------
  // EPC + RemoteHost
  // ------------------------------------------------------------
  Ptr<Node> pgw =
      epcHelper->GetPgwNode ();

  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);

  Ptr<Node> remoteHost =
      remoteHostContainer.Get (0);

  InternetStackHelper internet;

  internet.Install (
      remoteHostContainer);

  internet.Install (
      ueNodes);

  PointToPointHelper p2p;

  p2p.SetDeviceAttribute (
      "DataRate",
      DataRateValue (
          DataRate (
              config.remoteHostRate)));

  p2p.SetDeviceAttribute (
      "Mtu",
      UintegerValue (
          config.remoteHostMtu));

  p2p.SetChannelAttribute (
      "Delay",
      TimeValue (
          Seconds (
              config.remoteHostDelay)));

  NetDeviceContainer internetDevices =
      p2p.Install (
          pgw,
          remoteHost);

  Ipv4AddressHelper ipv4h;

  ipv4h.SetBase (
      "1.0.0.0",
      "255.0.0.0");

  Ipv4InterfaceContainer internetIfaces =
      ipv4h.Assign (
          internetDevices);

  (void) internetIfaces;

  Ipv4StaticRoutingHelper
      ipv4RoutingHelper;

  Ptr<Ipv4StaticRouting>
      remoteHostStaticRouting =
          ipv4RoutingHelper
              .GetStaticRouting (
                  remoteHost
                      ->GetObject<Ipv4> ());

  // EPC側UE IPv4 pool
  remoteHostStaticRouting
      ->AddNetworkRouteTo (
          Ipv4Address (
              "7.0.0.0"),
          Ipv4Mask (
              "255.0.0.0"),
          1);

  Ipv4InterfaceContainer ueIpIface =
      epcHelper
          ->AssignUeIpv4Address (
              NetDeviceContainer (
                  ueNetDev));

  // UE default route
  for (uint32_t i = 0;
       i < ueNodes.GetN ();
       ++i)
  {
    Ptr<Ipv4StaticRouting>
        ueStaticRouting =
            ipv4RoutingHelper
                .GetStaticRouting (
                    ueNodes.Get (i)
                        ->GetObject<Ipv4> ());

    ueStaticRouting
        ->SetDefaultRoute (
            epcHelper
                ->GetUeDefaultGatewayAddress (),
            1);
  }

  // 5G-LENA v2.5 / ns-3.39 API
  nrHelper->AttachToClosestEnb (
      ueNetDev,
      gnbNetDev);

  // ------------------------------------------------------------
  // Application addresses
  // ------------------------------------------------------------
  std::vector<Ipv4Address>
      ueAddresses;

  ueAddresses.reserve (
      config.nUes);

  for (uint32_t i = 0;
       i < config.nUes;
       ++i)
  {
    ueAddresses.push_back (
        ueIpIface.GetAddress (i));
  }

  // ------------------------------------------------------------
  // ServerApp
  // ------------------------------------------------------------
  Ptr<ServerApp> serverApp =
      CreateObject<ServerApp> ();

  serverApp->Setup (
      ueAddresses,
      config.packetSize,
      config.kChannels,
      Seconds (
          config.interval),
      Seconds (
          config.channelOffset),
      method);

  remoteHost
      ->AddApplication (
          serverApp);

  serverApp->SetStartTime (
      Seconds (
          config.serverStart));

  const uint32_t totalSegments =
      serverApp->GetTotalSegments ();

  // ------------------------------------------------------------
  // UE ClientApp
  // 受信socketをserverより先に起動して、
  // 待ち時間はserverStartから測定する。
  // ------------------------------------------------------------
  for (uint32_t i = 0;
       i < config.nUes;
       ++i)
  {
    Ptr<ClientApp> clientApp =
        CreateObject<ClientApp> ();

    clientApp->Setup (
        totalSegments,
        config.verbose,
        Seconds (
            config.drainTime),
        Seconds (
            config.serverStart),
        MakeCallback (
            &ServerApp::StopSending,
            serverApp));

    ueNodes.Get (i)
        ->AddApplication (
            clientApp);

    clientApp->SetStartTime (
        Seconds (
            config.clientStart));
  }

  // ------------------------------------------------------------
  // 設定表示
  // ------------------------------------------------------------
  NS_LOG_UNCOND (
      "\n================================================");

  NS_LOG_UNCOND (
      "【5G NR 実験設定："
      << MethodDisplayName (method)
      << "】");

  NS_LOG_UNCOND (
      "配信手法              : "
      << MethodDisplayName (method));

  NS_LOG_UNCOND (
      "構成                  : RemoteHost -> PGW -> gNB -> UE");

  NS_LOG_UNCOND (
      "アプリ通信            : DL UDP unicast / UE");

  NS_LOG_UNCOND (
      "gNB数                 : 1");

  NS_LOG_UNCOND (
      "UE数                  : "
      << config.nUes);

  NS_LOG_UNCOND (
      "論理チャネル数        : "
      << serverApp
             ->GetLogicalChannelCount ());

  NS_LOG_UNCOND (
      "総セグメント数(N)     : "
      << totalSegments);

  NS_LOG_UNCOND (
      "チャネル送信間隔      : "
      << config.interval
      << "s");

  NS_LOG_UNCOND (
      "チャネル初期ずれ      : "
      << config.channelOffset
      << "s");

  NS_LOG_UNCOND (
      "受信ポート            : "
      << RECEIVE_PORT);

  NS_LOG_UNCOND (
      "UE受信App開始         : "
      << config.clientStart
      << "s");

  NS_LOG_UNCOND (
      "配信・待ち時間測定開始: "
      << config.serverStart
      << "s");

  NS_LOG_UNCOND (
      "Payload               : "
      << config.packetSize
      << " bytes");

  NS_LOG_UNCOND (
      "------------------------------------------------");

  NS_LOG_UNCOND (
      "中心周波数            : "
      << config.centralFrequency /
             1e9
      << " GHz");

  NS_LOG_UNCOND (
      "帯域幅                : "
      << config.bandwidth /
             1e6
      << " MHz");

  NS_LOG_UNCOND (
      "Numerology            : "
      << config.numerology);

  NS_LOG_UNCOND (
      "gNB TxPower           : "
      << config.gnbTxPowerDbm
      << " dBm");

  NS_LOG_UNCOND (
      "Scheduler             : "
      << (config.enableOfdma
              ? "OFDMA-RR"
              : "TDMA-RR"));

  NS_LOG_UNCOND (
      "TDD Pattern           : "
      << config.tddPattern);

  NS_LOG_UNCOND (
      "Channel scenario      : UMi-StreetCanyon");

  NS_LOG_UNCOND (
      "Shadowing             : OFF");

  NS_LOG_UNCOND (
      "Beamforming           : DirectPathBeamforming");

  NS_LOG_UNCOND (
      "Antenna               : gNB 1x1 / UE 1x1 isotropic");

  NS_LOG_UNCOND (
      "UE配置                : gNB中心の円周上に等間隔");

  NS_LOG_UNCOND (
      "全UEの水平半径        : "
      << config.distance
      << "m");

  NS_LOG_UNCOND (
      "gNB高さ / UE高さ      : "
      << config.gnbHeight
      << "m / "
      << config.ueHeight
      << "m");

  NS_LOG_UNCOND (
      "RemoteHost-PGW        : "
      << config.remoteHostRate
      << ", MTU "
      << config.remoteHostMtu
      << ", delay "
      << config.remoteHostDelay
      << "s");

  NS_LOG_UNCOND (
      "S1-U delay            : "
      << config.s1uDelayMs
      << "ms");

  NS_LOG_UNCOND (
      "Seed / Run            : "
      << config.seed
      << " / "
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
  // Collect
  // ------------------------------------------------------------
  ExperimentOutput output;

  output.nodes.reserve (
      config.nUes);

  double totalArrivalRate = 0.0;
  uint64_t totalRawReceived = 0;

  for (uint32_t i = 0;
       i < ueNodes.GetN ();
       ++i)
  {
    Ptr<ClientApp> app =
        DynamicCast<ClientApp> (
            ueNodes.Get (i)
                ->GetApplication (0));

    const uint32_t
        receivedUnique =
            app->GetReceivedCount ();

    const uint32_t
        missingUnique =
            totalSegments -
            receivedUnique;

    const double arrivalRate =
        (static_cast<double> (
             receivedUnique) /
         totalSegments) *
        100.0;

    totalArrivalRate +=
        arrivalRate;

    totalRawReceived +=
        app->GetRawReceivedCount ();

    NodeResult nodeResult;

    nodeResult.method =
        MethodName (method);

    nodeResult.ueIndex = i;

    nodeResult.nodeId =
        ueNodes.Get (i)
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
  }

  const double averageArrivalRate =
      totalArrivalRate /
      config.nUes;

  const double completionRate =
      (static_cast<double> (
           g_completedNodes) /
       config.nUes) *
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

  const uint64_t logicalSent =
      serverApp
          ->GetLogicalSentCount ();

  const uint64_t unicastAttempted =
      serverApp
          ->GetUnicastAttemptCount ();

  const uint64_t unicastSent =
      serverApp
          ->GetUnicastSentCount ();

  const uint64_t sendFailures =
      (unicastAttempted >=
       unicastSent)
          ? unicastAttempted -
                unicastSent
          : 0;

  // SendTo()成功パケットは各1UE宛なので
  // 理論受信数 = successful unicast sends
  const uint64_t expectedReceptions =
      unicastSent;

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
  // Summary
  // ------------------------------------------------------------
  ExperimentResult& result =
      output.summary;

  result.environment =
      "5G-NR";

  result.architecture =
      "RemoteHost-PGW-gNB-UE";

  result.trafficType =
      "DL-UDP-Unicast-per-UE";

  result.method =
      MethodName (method);

  result.gnbCount = 1;
  result.ueCount =
      config.nUes;

  result.logicalChannels =
      serverApp
          ->GetLogicalChannelCount ();

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

  result.serverStartS =
      config.serverStart;

  result.simulationTimeS =
      config.simulationTime;

  result.drainTimeS =
      config.drainTime;

  result.packetSizeBytes =
      config.packetSize;

  result.centralFrequencyHz =
      config.centralFrequency;

  result.bandwidthHz =
      config.bandwidth;

  result.numerology =
      config.numerology;

  result.gnbTxPowerDbm =
      config.gnbTxPowerDbm;

  result.scheduler =
      config.enableOfdma
          ? "OFDMA-RR"
          : "TDMA-RR";

  result.tddPattern =
      config.tddPattern;

  result.channelScenario =
      "UMi-StreetCanyon";

  result.shadowingEnabled =
      false;

  result.beamforming =
      "DirectPathBeamforming";

  result.distanceM =
      config.distance;

  result.gnbHeightM =
      config.gnbHeight;

  result.ueHeightM =
      config.ueHeight;

  result.remoteHostRate =
      config.remoteHostRate;

  result.remoteHostMtu =
      config.remoteHostMtu;

  result.remoteHostDelayS =
      config.remoteHostDelay;

  result.s1uDelayMs =
      config.s1uDelayMs;

  result.seed =
      config.seed;

  result.run =
      config.run;

  result.completedUes =
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

  result.logicalSentPackets =
      logicalSent;

  result.unicastAttemptedPackets =
      unicastAttempted;

  result.unicastSentPackets =
      unicastSent;

  result.applicationSendFailures =
      sendFailures;

  result.actualReceptions =
      totalRawReceived;

  result.lostPackets =
      lostPackets;

  result.packetLossRatePct =
      packetLossRate;

  // ------------------------------------------------------------
  // Console result
  // ------------------------------------------------------------
  NS_LOG_UNCOND (
      "\n==== 5G NR 集計結果: "
      << MethodDisplayName (method)
      << " ====");

  NS_LOG_UNCOND (
      "完了UE数                : "
      << result.completedUes
      << " / "
      << result.ueCount);

  NS_LOG_UNCOND (
      "完了率                  : "
      << result.completionRatePct
      << "%");

  NS_LOG_UNCOND (
      "平均待ち時間            : "
      << result.averageWaitS
      << "s");

  NS_LOG_UNCOND (
      "最小待ち時間            : "
      << result.minimumWaitS
      << "s");

  NS_LOG_UNCOND (
      "最大待ち時間            : "
      << result.maximumWaitS
      << "s");

  NS_LOG_UNCOND (
      "平均セグメント到達率    : "
      << result.averageSegmentArrivalRatePct
      << "%");

  NS_LOG_UNCOND (
      "論理セグメント送信回数  : "
      << result.logicalSentPackets);

  NS_LOG_UNCOND (
      "UE向けUnicast試行数     : "
      << result.unicastAttemptedPackets);

  NS_LOG_UNCOND (
      "UE向けUnicast送信成功数 : "
      << result.unicastSentPackets);

  NS_LOG_UNCOND (
      "Application送信失敗数   : "
      << result.applicationSendFailures);

  NS_LOG_UNCOND (
      "実受信パケット数        : "
      << result.actualReceptions);

  NS_LOG_UNCOND (
      "観測損失パケット数      : "
      << result.lostPackets);

  NS_LOG_UNCOND (
      "観測パケット欠損率      : "
      << result.packetLossRatePct
      << "%");

  Simulator::Destroy ();

  return output;
}

// ============================================================
// CSV
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
      << "architecture,"
      << "traffic_type,"
      << "method,"
      << "gnb_count,"
      << "ue_count,"
      << "logical_channels,"
      << "total_segments,"
      << "interval_s,"
      << "channel_offset_s,"
      << "receive_port,"
      << "client_start_s,"
      << "server_start_s,"
      << "simulation_time_s,"
      << "drain_time_s,"
      << "packet_size_bytes,"
      << "central_frequency_hz,"
      << "bandwidth_hz,"
      << "numerology,"
      << "gnb_tx_power_dbm,"
      << "scheduler,"
      << "tdd_pattern,"
      << "channel_scenario,"
      << "shadowing_enabled,"
      << "beamforming,"
      << "gnb_antenna_rows,"
      << "gnb_antenna_columns,"
      << "ue_antenna_rows,"
      << "ue_antenna_columns,"
      << "distance_m,"
      << "gnb_height_m,"
      << "ue_height_m,"
      << "remote_host_rate,"
      << "remote_host_mtu,"
      << "remote_host_delay_s,"
      << "s1u_delay_ms,"
      << "seed,"
      << "run,"
      << "completed_ues,"
      << "completion_rate_pct,"
      << "avg_wait_s,"
      << "min_wait_s,"
      << "max_wait_s,"
      << "avg_segment_arrival_rate_pct,"
      << "logical_sent_packets,"
      << "unicast_attempted_packets,"
      << "unicast_sent_packets,"
      << "application_send_failures,"
      << "actual_receptions,"
      << "lost_packets,"
      << "packet_loss_rate_pct"
      << "\n";

  for (const auto& r : results)
  {
    file
        << r.environment << ","
        << r.architecture << ","
        << r.trafficType << ","
        << r.method << ","
        << r.gnbCount << ","
        << r.ueCount << ","
        << r.logicalChannels << ","
        << r.totalSegments << ","
        << r.intervalS << ","
        << r.channelOffsetS << ","
        << r.receivePort << ","
        << r.clientStartS << ","
        << r.serverStartS << ","
        << r.simulationTimeS << ","
        << r.drainTimeS << ","
        << r.packetSizeBytes << ","
        << r.centralFrequencyHz << ","
        << r.bandwidthHz << ","
        << r.numerology << ","
        << r.gnbTxPowerDbm << ","
        << r.scheduler << ","
        << "\"" << r.tddPattern << "\"" << ","
        << r.channelScenario << ","
        << (r.shadowingEnabled ? 1 : 0) << ","
        << r.beamforming << ","
        << r.gnbAntennaRows << ","
        << r.gnbAntennaColumns << ","
        << r.ueAntennaRows << ","
        << r.ueAntennaColumns << ","
        << r.distanceM << ","
        << r.gnbHeightM << ","
        << r.ueHeightM << ","
        << r.remoteHostRate << ","
        << r.remoteHostMtu << ","
        << r.remoteHostDelayS << ","
        << r.s1uDelayMs << ","
        << r.seed << ","
        << r.run << ","
        << r.completedUes << ","
        << r.completionRatePct << ","
        << r.averageWaitS << ","
        << r.minimumWaitS << ","
        << r.maximumWaitS << ","
        << r.averageSegmentArrivalRatePct << ","
        << r.logicalSentPackets << ","
        << r.unicastAttemptedPackets << ","
        << r.unicastSentPackets << ","
        << r.applicationSendFailures << ","
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
      << "ue_index,"
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
        << r.ueIndex << ","
        << r.nodeId << ","
        << (r.completed ? 1 : 0)
        << ",";

    // 未完了と0秒完了を区別
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

  // --mode は使わない。
  // FB -> Simple -> Full の3方式を1コマンドで順次実行する。
  cmd.AddValue (
      "nodes",
      "Number of UE nodes",
      config.nUes);

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
      "Initial logical-channel offset (s)",
      config.channelOffset);

  cmd.AddValue (
      "packetSize",
      "Application payload size in bytes",
      config.packetSize);

  cmd.AddValue (
      "frequency",
      "NR central frequency (Hz)",
      config.centralFrequency);

  cmd.AddValue (
      "bandwidth",
      "NR bandwidth (Hz)",
      config.bandwidth);

  cmd.AddValue (
      "numerology",
      "NR numerology",
      config.numerology);

  cmd.AddValue (
      "txPower",
      "gNB Tx power (dBm)",
      config.gnbTxPowerDbm);

  cmd.AddValue (
      "ofdma",
      "true: OFDMA-RR, false: TDMA-RR",
      config.enableOfdma);

  cmd.AddValue (
      "tddPattern",
      "NR TDD pattern",
      config.tddPattern);

  cmd.AddValue (
      "dist",
      "Horizontal gNB-UE distance (m)",
      config.distance);

  cmd.AddValue (
      "gnbHeight",
      "gNB height (m)",
      config.gnbHeight);

  cmd.AddValue (
      "ueHeight",
      "UE height (m)",
      config.ueHeight);

  cmd.AddValue (
      "clientStart",
      "UE receive application start time (s)",
      config.clientStart);

  cmd.AddValue (
      "serverStart",
      "Traffic / waiting-time measurement start (s)",
      config.serverStart);

  cmd.AddValue (
      "simTime",
      "Maximum simulation time per method (s)",
      config.simulationTime);

  cmd.AddValue (
      "drainTime",
      "Drain time after all UEs complete (s)",
      config.drainTime);

  cmd.AddValue (
      "remoteRate",
      "RemoteHost-PGW PointToPoint rate",
      config.remoteHostRate);

  cmd.AddValue (
      "remoteMtu",
      "RemoteHost-PGW MTU",
      config.remoteHostMtu);

  cmd.AddValue (
      "remoteDelay",
      "RemoteHost-PGW delay (s)",
      config.remoteHostDelay);

  cmd.AddValue (
      "s1uDelayMs",
      "S1-U link delay (ms)",
      config.s1uDelayMs);

  cmd.AddValue (
      "seed",
      "Random seed",
      config.seed);

  cmd.AddValue (
      "run",
      "Run number",
      config.run);

  cmd.AddValue (
      "verbose",
      "Enable per-UE console logs",
      config.verbose);

  cmd.AddValue (
      "outputDir",
      "Directory for CSV results",
      config.outputDir);

  cmd.AddValue (
      "tag",
      "Optional output file tag",
      config.outputTag);

  cmd.Parse (
      argc,
      argv);

  // ------------------------------------------------------------
  // Check
  // ------------------------------------------------------------
  NS_ABORT_MSG_IF (
      config.nUes == 0,
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
      "channelOffset must be >= 0.");

  NS_ABORT_MSG_IF (
      config.packetSize == 0,
      "packetSize must be at least 1.");

  NS_ABORT_MSG_IF (
      config.centralFrequency <= 0.0,
      "frequency must be > 0.");

  NS_ABORT_MSG_IF (
      config.bandwidth <= 0.0,
      "bandwidth must be > 0.");

  NS_ABORT_MSG_IF (
      config.distance < 0.0,
      "dist must be >= 0.");

  NS_ABORT_MSG_IF (
      config.nUes > 1 &&
          config.distance == 0.0,
      "For 5G NR with multiple UEs, dist must be greater than 0 "
      "because the 3GPP spectrum channel model does not allow "
      "different radio devices at exactly the same position.");

  NS_ABORT_MSG_IF (
      config.clientStart < 0.0,
      "clientStart must be >= 0.");

  NS_ABORT_MSG_IF (
      config.serverStart <
          config.clientStart,
      "serverStart must be >= clientStart.");

  NS_ABORT_MSG_IF (
      config.simulationTime <=
          config.serverStart,
      "simTime must be later than serverStart.");

  NS_ABORT_MSG_IF (
      config.drainTime < 0.0,
      "drainTime must be >= 0.");

  NS_ABORT_MSG_IF (
      config.outputDir.empty (),
      "outputDir must not be empty.");

  // ------------------------------------------------------------
  // Ubuntu/Linux result directory
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
  // 3 methods
  // ------------------------------------------------------------
  const std::vector<DeliveryMethod>
      methods = {
          DeliveryMethod::FB,
          DeliveryMethod::SIMPLE,
          DeliveryMethod::FULL};

  std::vector<ExperimentResult>
      summaries;

  std::vector<NodeResult>
      allNodeResults;

  summaries.reserve (
      methods.size ());

  allNodeResults.reserve (
      static_cast<size_t> (
          config.nUes) *
      methods.size ());

  for (DeliveryMethod method :
       methods)
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
  // Write CSV
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

  WriteSummaryCsv (
      summaries,
      summaryFilename);

  WriteNodeResultsCsv (
      allNodeResults,
      nodeFilename);

  // ------------------------------------------------------------
  // Final console table
  // ------------------------------------------------------------
  NS_LOG_UNCOND (
      "\n================================================");

  NS_LOG_UNCOND (
      "5G NR: 3方式すべての実験が完了しました。");

  NS_LOG_UNCOND (
      "Summary CSV : "
      << summaryFilename);

  NS_LOG_UNCOND (
      "Node CSV    : "
      << nodeFilename);

  NS_LOG_UNCOND (
      "================================================");

  NS_LOG_UNCOND (
      "\n=== 5G NR 3方式比較 ===");

  NS_LOG_UNCOND (
      "Method,Completed,CompletionRate(%),"
      "AvgWait(s),MinWait(s),MaxWait(s),"
      "AvgArrival(%),LogicalSent,"
      "UnicastAttempted,UnicastSent,"
      "SendFailures,ActualRx,Lost,LossRate(%)");

  for (const auto& r :
       summaries)
  {
    NS_LOG_UNCOND (
        r.method << ","
                 << r.completedUes
                 << "/"
                 << r.ueCount
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
                 << r.logicalSentPackets
                 << ","
                 << r.unicastAttemptedPackets
                 << ","
                 << r.unicastSentPackets
                 << ","
                 << r.applicationSendFailures
                 << ","
                 << r.actualReceptions
                 << ","
                 << r.lostPackets
                 << ","
                 << r.packetLossRatePct);
  }

  return 0;
}
