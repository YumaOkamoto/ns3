#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
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
NS_LOG_COMPONENT_DEFINE ("NrSegmentedBroadcastPlaybackFixed");

namespace
{
constexpr uint16_t RECEIVE_PORT = 9001;
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
  ServerApp () : m_totalSegments (0), m_channelRateBps (1e6), m_channelOffset (MilliSeconds (1)),
      m_running (false), m_logicalSentCount (0), m_unicastAttemptCount (0), m_unicastSentCount (0),
      m_totalVideoBytes (0) {}

  void Setup (const std::vector<Ipv4Address>& destinations, uint32_t payloadSize, uint32_t numChannels,
              double videoDuration, double videoBitrateMbps, double channelRateMbps,
              Time channelOffset, Time measurementStart, DeliveryMethod method)
  {
    m_destinations = destinations; m_payloadSize = payloadSize; m_channelOffset = channelOffset; m_measurementStart = measurementStart;
    m_channelRateBps = channelRateMbps * 1e6;
    m_totalSegments = (1u << numChannels) - 1u;
    m_totalVideoBytes = static_cast<uint64_t> (std::llround (videoDuration * videoBitrateMbps * 1e6 / 8.0));
    NS_ABORT_MSG_IF (m_totalVideoBytes < m_totalSegments, "Video is too small for the selected N.");

    m_packetsPerSegment.assign (m_totalSegments + 1, 0);
    std::vector<std::vector<PacketDescriptor>> segmentPackets (m_totalSegments + 1);
    const uint64_t baseBytes = m_totalVideoBytes / m_totalSegments;
    const uint64_t remainder = m_totalVideoBytes % m_totalSegments;
    for (uint32_t s = 1; s <= m_totalSegments; ++s)
    {
      const uint64_t bytes = baseBytes + (s <= remainder ? 1 : 0);
      const uint32_t count = static_cast<uint32_t> ((bytes + payloadSize - 1) / payloadSize);
      m_packetsPerSegment[s] = count;
      uint64_t left = bytes;
      for (uint32_t p = 0; p < count; ++p)
      {
        const uint32_t thisBytes = static_cast<uint32_t> (std::min<uint64_t> (payloadSize, left));
        segmentPackets[s].push_back ({s, p, count, thisBytes}); left -= thisBytes;
      }
    }

    const uint32_t actualChannels = method == DeliveryMethod::FB ? numChannels :
      (method == DeliveryMethod::SIMPLE ? 1u : m_totalSegments);
    m_channelPackets.assign (actualChannels, {}); m_indices.assign (actualChannels, 0);
    auto appendSegment = [&] (uint32_t ch, uint32_t s)
    {
      m_channelPackets[ch].insert (m_channelPackets[ch].end (), segmentPackets[s].begin (), segmentPackets[s].end ());
    };
    if (method == DeliveryMethod::FB)
    {
      uint32_t current = 1;
      for (uint32_t ch = 0; ch < numChannels; ++ch)
        for (uint32_t j = 0; j < (1u << ch); ++j) appendSegment (ch, current++);
    }
    else if (method == DeliveryMethod::SIMPLE)
    {
      for (uint32_t s = 1; s <= m_totalSegments; ++s) appendSegment (0, s);
    }
    else
    {
      for (uint32_t ch = 0; ch < m_totalSegments; ++ch) appendSegment (ch, ch + 1);
    }
  }

  uint32_t GetTotalSegments () const { return m_totalSegments; }
  uint32_t GetLogicalChannelCount () const { return static_cast<uint32_t> (m_channelPackets.size ()); }
  const std::vector<uint32_t>& GetPacketsPerSegment () const { return m_packetsPerSegment; }
  uint64_t GetTotalVideoBytes () const { return m_totalVideoBytes; }
  uint64_t GetLogicalSentCount () const { return m_logicalSentCount; }
  uint64_t GetUnicastAttemptCount () const { return m_unicastAttemptCount; }
  uint64_t GetUnicastSentCount () const { return m_unicastSentCount; }
  void StopSending () { m_running = false; if (m_socket) { m_socket->Close (); m_socket = nullptr; } }

private:
  void StartApplication () override
  {
    m_running = true; m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    NS_ABORT_MSG_IF (m_socket->Bind () < 0, "Failed to bind server socket.");
    for (uint32_t ch = 0; ch < m_channelPackets.size (); ++ch)
      Simulator::Schedule (Seconds (m_channelOffset.GetSeconds () * ch), &ServerApp::SendPacket, this, ch);
  }
  void StopApplication () override { StopSending (); }
  void SendPacket (uint32_t ch)
  {
    if (!m_running || !m_socket) return;
    const PacketDescriptor d = m_channelPackets[ch][m_indices[ch]]; const bool measured = Simulator::Now () >= m_measurementStart; if (measured) ++m_logicalSentCount;
    for (const auto& dst : m_destinations)
    {
      if (measured)
      {
        ++m_unicastAttemptCount;
      }
      Ptr<Packet> p = Create<Packet> (d.payloadBytes);
      p->AddHeader (SegmentPacketHeader (d.segmentId, d.packetIndex, d.packetsInSegment));
      if (m_socket->SendTo (p, 0, InetSocketAddress (dst, RECEIVE_PORT)) >= 0 && measured) ++m_unicastSentCount;
    }
    m_indices[ch] = (m_indices[ch] + 1) % m_channelPackets[ch].size ();
    const double seconds = (static_cast<double> (d.payloadBytes) * 8.0) / m_channelRateBps;
    Simulator::Schedule (Seconds (seconds), &ServerApp::SendPacket, this, ch);
  }

  std::vector<Ipv4Address> m_destinations; uint32_t m_payloadSize = 1024, m_totalSegments;
  double m_channelRateBps; Time m_channelOffset; Time m_measurementStart; bool m_running; Ptr<Socket> m_socket;
  std::vector<std::vector<PacketDescriptor>> m_channelPackets; std::vector<uint32_t> m_indices, m_packetsPerSegment;
  uint64_t m_logicalSentCount, m_unicastAttemptCount, m_unicastSentCount, m_totalVideoBytes;
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
    m_joinTime = Simulator::Now (); m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    NS_ABORT_MSG_IF (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), RECEIVE_PORT)) < 0, "UE bind failed");
    m_socket->SetRecvCallback (MakeCallback (&ClientApp::HandleRead, this));
  }
  void StopApplication () override
  {
    if (m_socket) { m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket>> ()); m_socket->Close (); m_socket = nullptr; }
  }
  void HandleRead (Ptr<Socket> socket)
  {
    Ptr<Packet> p; Address from; while ((p = socket->RecvFrom (from))) ProcessReceivedPacket (p);
  }

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
  uint32_t nUes = 5, kChannels = 6, packetSize = 1024;
  double videoDuration = 60.0;
  double videoBitrateMbps = 1.0;
  double channelRateMbps = 1.0;
  double channelOffset = 0.001;
  double centralFrequency = 3.5e9, bandwidth = 20e6; uint16_t numerology = 1; double gnbTxPowerDbm = 30.0;
  bool enableOfdma = true; std::string tddPattern = "DL|S|UL|UL|DL|DL|S|UL|UL|DL|";
  double distance = 30.0, gnbHeight = 10.0, ueHeight = 1.5;
  double serverStart = 1.0, clientStart = 5.0, simulationTime = 400.0, drainTime = 0.1;
  std::string remoteHostRate = "100Gb/s"; uint32_t remoteHostMtu = 2500; double remoteHostDelay = 0.0, s1uDelayMs = 0.0;
  uint32_t seed = 1, run = 1; bool verbose = false; std::string outputDir = "results", outputTag = "";
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
  RngSeedManager::SetSeed(config.seed);
  RngSeedManager::SetRun(config.run);
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

  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (ueAddresses, config.packetSize, config.kChannels, config.videoDuration,
                    config.videoBitrateMbps, config.channelRateMbps, Seconds (config.channelOffset), Seconds (config.clientStart), method);
  remoteHost->AddApplication (serverApp); serverApp->SetStartTime (Seconds (config.serverStart));
  const uint32_t totalSegments = serverApp->GetTotalSegments ();
  const double segmentDuration = config.videoDuration / totalSegments;
  g_totalNodes = config.nUes; g_completedNodes = 0;
  for (uint32_t i = 0; i < config.nUes; ++i)
  {
    Ptr<ClientApp> c = CreateObject<ClientApp> ();
    c->Setup (serverApp->GetPacketsPerSegment (), Seconds (segmentDuration), config.verbose,
              Seconds (config.drainTime), MakeCallback (&ServerApp::StopSending, serverApp));
    ueNodes.Get (i)->AddApplication (c); c->SetStartTime (Seconds (config.clientStart));
  }

  NS_LOG_UNCOND ("\n================================================");
  NS_LOG_UNCOND ("【5G NR・再生評価（固定動画）：" << MethodDisplayName (method) << "】");
  NS_LOG_UNCOND ("動画時間              : " << config.videoDuration << "s (固定)");
  NS_LOG_UNCOND ("動画ビットレート      : " << config.videoBitrateMbps << "Mbps (固定)");
  NS_LOG_UNCOND ("動画サイズ            : " << serverApp->GetTotalVideoBytes () / 1e6 << "MB (固定)");
  NS_LOG_UNCOND ("N / SegmentDuration   : " << totalSegments << " / " << segmentDuration << "s");
  NS_LOG_UNCOND ("論理チャネル数        : " << serverApp->GetLogicalChannelCount ());
  NS_LOG_UNCOND ("1論理ch送信レート     : " << config.channelRateMbps << "Mbps");
  NS_LOG_UNCOND ("放送開始 / 視聴参加   : " << config.serverStart << "s / " << config.clientStart << "s");
  NS_LOG_UNCOND ("================================================");

  Simulator::Stop (Seconds (config.simulationTime)); Simulator::Run ();
  ExperimentOutput output; uint64_t totalRaw = 0; double initialSum=0, stallSum=0, stallCountSum=0, finishSum=0; uint32_t started=0, completed=0;
  for (uint32_t i=0;i<ueNodes.GetN ();++i)
  {
    Ptr<ClientApp> app=DynamicCast<ClientApp> (ueNodes.Get(i)->GetApplication(0)); NodeResult n;
    n.method=MethodName(method); n.index=i; n.nodeId=ueNodes.Get(i)->GetId(); n.playbackStarted=app->PlaybackStarted(); n.playbackComplete=app->PlaybackComplete();
    n.initialWaitS=app->GetInitialWait(); n.stallCount=app->GetStallCount(); n.totalStallS=app->GetObservedTotalStall(); n.maxStallS=app->GetMaxStall();
    n.finishFromJoinS=app->GetFinishFromJoin(); n.receivedUniqueSegments=app->GetReceivedCount(); n.totalSegments=totalSegments; n.rawReceivedPackets=app->GetRawReceivedCount(); output.nodes.push_back(n);
    totalRaw+=n.rawReceivedPackets; if(n.playbackStarted){++started;initialSum+=n.initialWaitS;stallSum+=n.totalStallS;stallCountSum+=n.stallCount;} if(n.playbackComplete){++completed;finishSum+=n.finishFromJoinS;}
  }
  ExperimentResult& r=output.summary; r.method=MethodName(method); r.nodes=config.nUes; r.logicalChannels=serverApp->GetLogicalChannelCount(); r.totalSegments=totalSegments;
  r.startedNodes=started; r.completedNodes=completed; r.completionRatePct=100.0*completed/config.nUes; r.avgInitialWaitS=started?initialSum/started:0; r.avgTotalStallS=started?stallSum/started:0;
  r.avgStallCount=started?stallCountSum/started:0; r.avgFinishFromJoinS=completed?finishSum/completed:0; r.logicalSentPackets=serverApp->GetLogicalSentCount(); r.attemptedPackets=serverApp->GetUnicastAttemptCount();
  r.sentPackets=serverApp->GetUnicastSentCount(); r.actualReceptions=totalRaw; r.packetLossRatePct=r.sentPackets?std::max(0.0,100.0*(1.0-static_cast<double>(totalRaw)/r.sentPackets)):0;
  NS_LOG_UNCOND ("==== 再生結果: " << MethodDisplayName(method) << " ====");
  NS_LOG_UNCOND ("開始/完了             : " << started << "/" << completed << " (全" << config.nUes << ")");
  NS_LOG_UNCOND ("平均初期待ち時間      : " << (started?std::to_string(r.avgInitialWaitS):"N/A") << "s");
  NS_LOG_UNCOND ("平均総途切れ時間      : " << (started?std::to_string(r.avgTotalStallS):"N/A") << "s");
  NS_LOG_UNCOND ("平均途切れ回数        : " << (started?std::to_string(r.avgStallCount):"N/A"));
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

int main(int argc,char* argv[])
{
  SimulationConfig config; CommandLine cmd;
  cmd.AddValue("nodes","UE count",config.nUes); cmd.AddValue("k","FB k; N=2^k-1",config.kChannels);
  cmd.AddValue("packetSize","UDP video payload bytes",config.packetSize); cmd.AddValue("videoDuration","Fixed video duration seconds",config.videoDuration);
  cmd.AddValue("videoBitrate","Fixed video bitrate Mbps",config.videoBitrateMbps); cmd.AddValue("channelRate","Per-logical-channel offered rate Mbps",config.channelRateMbps);
  cmd.AddValue("channelOffset","Initial channel offset s",config.channelOffset); cmd.AddValue("frequency","NR frequency Hz",config.centralFrequency); cmd.AddValue("bandwidth","NR bandwidth Hz",config.bandwidth);
  cmd.AddValue("numerology","NR numerology",config.numerology); cmd.AddValue("txPower","gNB Tx dBm",config.gnbTxPowerDbm); cmd.AddValue("ofdma","OFDMA-RR",config.enableOfdma); cmd.AddValue("tddPattern","TDD",config.tddPattern);
  cmd.AddValue("dist","distance m",config.distance); cmd.AddValue("gnbHeight","gNB height",config.gnbHeight); cmd.AddValue("ueHeight","UE height",config.ueHeight);
  cmd.AddValue("serverStart","broadcast start",config.serverStart); cmd.AddValue("clientStart","viewer join",config.clientStart); cmd.AddValue("simTime","simulation time",config.simulationTime); cmd.AddValue("drainTime","drain",config.drainTime);
  cmd.AddValue("remoteRate","RemoteHost rate",config.remoteHostRate); cmd.AddValue("remoteMtu","MTU",config.remoteHostMtu); cmd.AddValue("remoteDelay","remote delay",config.remoteHostDelay); cmd.AddValue("s1uDelayMs","S1U ms",config.s1uDelayMs);
  cmd.AddValue("seed","seed",config.seed); cmd.AddValue("run","run",config.run); cmd.AddValue("verbose","logs",config.verbose); cmd.AddValue("outputDir","output",config.outputDir); cmd.AddValue("tag","tag",config.outputTag); cmd.Parse(argc,argv);
  NS_ABORT_MSG_IF(config.nUes==0,"nodes>=1"); NS_ABORT_MSG_IF(config.kChannels==0||config.kChannels>=31,"k=1..30");
  NS_ABORT_MSG_IF(config.videoDuration<=0||config.videoBitrateMbps<=0||config.channelRateMbps<=0,"videoDuration/videoBitrate/channelRate >0");
  NS_ABORT_MSG_IF(config.clientStart<config.serverStart,"clientStart must be >= serverStart"); NS_ABORT_MSG_IF(config.simulationTime<=config.clientStart,"simTime > clientStart");
  NS_ABORT_MSG_IF(config.nUes>1&&config.distance==0,"dist>0 for multiple UEs");
  std::error_code ec; std::filesystem::create_directories(config.outputDir,ec); NS_ABORT_MSG_IF(ec,ec.message());
  std::vector<ExperimentResult> summaries; std::vector<NodeResult> nodes;
  for(auto m:{DeliveryMethod::FB,DeliveryMethod::SIMPLE,DeliveryMethod::FULL}){auto o=RunExperiment(config,m);summaries.push_back(o.summary);nodes.insert(nodes.end(),o.nodes.begin(),o.nodes.end());}
  std::ostringstream name; name<<config.outputDir<<"/5gnr_playback_fixed_ues"<<config.nUes<<"_k"<<config.kChannels<<"_vdur"<<CompactNumber(config.videoDuration)
      <<"_vbr"<<CompactNumber(config.videoBitrateMbps)<<"_chrate"<<CompactNumber(config.channelRateMbps)<<"_dist"<<CompactNumber(config.distance)<<"_seed"<<config.seed<<"_run"<<config.run;
  if (!config.outputTag.empty ())
  {
    name << "_" << config.outputTag;
  }
  WriteCsv (summaries, nodes, name.str ());
  NS_LOG_UNCOND ("\nCSV: " << name.str () << "_summary.csv");
  return 0;
}
