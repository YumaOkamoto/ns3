#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSegmentedBroadcastingLargeScale");

namespace
{
constexpr uint16_t RECEIVE_PORT = 9001;
const Ipv4Address MULTICAST_GROUP ("224.1.1.1");

uint32_t g_completedNodes = 0;
uint32_t g_totalNodes = 0;
double g_waitTimeSum = 0.0;
double g_minWaitTime = std::numeric_limits<double>::max ();
double g_maxWaitTime = 0.0;
} // namespace

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
              uint32_t mode)
  {
    m_peer = destination;
    m_packetSize = packetSize;
    m_interval = interval;
    m_channelOffset = channelOffset;

    NS_ABORT_MSG_IF (numChannels == 0 || numChannels >= 31,
                     "k must be between 1 and 30.");
    NS_ABORT_MSG_IF (mode > 2, "mode must be 0, 1, or 2.");

    const uint32_t nSegments = (1u << numChannels) - 1u;
    m_totalSegments = nSegments;

    uint32_t actualChannels = 0;
    if (mode == 0)
    {
      actualChannels = numChannels;
    }
    else if (mode == 1)
    {
      actualChannels = 1;
    }
    else
    {
      actualChannels = nSegments;
    }

    m_channelData.assign (actualChannels, {});
    m_indices.assign (actualChannels, 0);

    if (mode == 0)
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
    else if (mode == 1)
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

    // 全論理チャネルで1個のUDPソケットを共有する
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
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

    // 受信側はSeq番号からセグメントIDを識別する
    SeqTsHeader header;
    header.SetSeq (segmentId);
    packet->AddHeader (header);

    const int result =
        m_socket->SendTo (packet,
                          0,
                          InetSocketAddress (m_peer, RECEIVE_PORT));

    if (result >= 0)
    {
      ++m_sentCount;
    }

    m_indices[channelId] =
        (m_indices[channelId] + 1) % m_channelData[channelId].size ();

    Simulator::Schedule (m_interval,
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

private:
  void StartApplication () override
  {
    m_startTime = Simulator::Now ();

    // クライアント1台につきソケットは1個
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());

    const int bindResult =
        m_socket->Bind (
            InetSocketAddress (Ipv4Address::GetAny (), RECEIVE_PORT));

    NS_ABORT_MSG_IF (bindResult < 0,
                     "Failed to bind UDP receive socket.");

    Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (m_socket);
    NS_ABORT_MSG_IF (!udpSocket, "Failed to create UDP socket.");

    udpSocket->MulticastJoinGroup (
        0,
        InetSocketAddress (MULTICAST_GROUP, RECEIVE_PORT));

    m_socket->SetRecvCallback (
        MakeCallback (&ClientApp::HandleRead, this));
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
        m_waitTime = (Simulator::Now () - m_startTime).GetSeconds ();

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
          // 新規送信を止めた後、キュー内パケットだけを処理する
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

int
main (int argc, char* argv[])
{
  uint32_t mode = 0;
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

  CommandLine cmd;
  cmd.AddValue ("mode", "0:FB, 1:Simple, 2:Full", mode);
  cmd.AddValue ("nodes", "Number of client nodes", nNodes);
  cmd.AddValue ("k", "Number of logical channels", kChannels);
  cmd.AddValue ("int", "Per-channel transmission interval (s)", interval);
  cmd.AddValue ("channelOffset",
                "Initial offset between logical channels (s)",
                channelOffset);
  cmd.AddValue ("dist", "Server-client distance (m)", distance);
  cmd.AddValue ("rate", "Wi-Fi data rate", dataRate);
  cmd.AddValue ("packetSize", "Payload size in bytes", packetSize);
  cmd.AddValue ("clientStart", "Client application start time (s)", clientStart);
  cmd.AddValue ("simTime", "Maximum simulation time (s)", simulationTime);
  cmd.AddValue ("drainTime",
                "Queue drain time after every client completes (s)",
                drainTime);
  cmd.AddValue ("seed", "Random seed", seed);
  cmd.AddValue ("run", "Run number", run);
  cmd.AddValue ("pcap", "Enable PCAP output", enablePcap);
  cmd.AddValue ("verbose", "Enable per-node completion output", verbose);
  cmd.Parse (argc, argv);

  NS_ABORT_MSG_IF (nNodes == 0, "nodes must be at least 1.");
  NS_ABORT_MSG_IF (interval <= 0.0, "int must be greater than 0.");
  NS_ABORT_MSG_IF (channelOffset < 0.0,
                   "channelOffset must be 0 or greater.");
  NS_ABORT_MSG_IF (packetSize == 0, "packetSize must be at least 1.");
  NS_ABORT_MSG_IF (simulationTime <= clientStart,
                   "simTime must be later than clientStart.");
  NS_ABORT_MSG_IF (drainTime < 0.0, "drainTime must be 0 or greater.");

  RngSeedManager::SetSeed (seed);
  RngSeedManager::SetRun (run);

  g_totalNodes = nNodes;

  NodeContainer serverNode;
  NodeContainer clientNodes;
  serverNode.Create (1);
  clientNodes.Create (nNodes);

  NodeContainer allNodes;
  allNodes.Add (serverNode);
  allNodes.Add (clientNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager (
      "ns3::ConstantRateWifiManager",
      "DataMode",
      StringValue (dataRate),
      "ControlMode",
      StringValue (dataRate),
      "NonUnicastMode",
      StringValue (dataRate));

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
  phy.SetChannel (channel.Create ());

  WifiMacHelper mac;
  mac.SetType (
      "ns3::AdhocWifiMac",
      "QosSupported",
      BooleanValue (true));

  NetDeviceContainer devices =
      wifi.Install (phy, mac, allNodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAllocator =
      CreateObject<ListPositionAllocator> ();

  positionAllocator->Add (Vector (0.0, 0.0, 0.0));

  // ノード数だけの影響を見るため、全クライアントを同じ距離へ置く
  for (uint32_t i = 0; i < nNodes; ++i)
  {
    positionAllocator->Add (Vector (distance, 0.0, 0.0));
  }

  mobility.SetPositionAllocator (positionAllocator);
  mobility.SetMobilityModel (
      "ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);

  InternetStackHelper stack;
  stack.Install (allNodes);

  // 送信キュー設定。帯域自体を論理分割する処理ではない
  TrafficControlHelper trafficControl;
  trafficControl.SetRootQueueDisc ("ns3::FifoQueueDisc");
  trafficControl.Install (devices);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.0.0", "255.255.0.0");
  address.Assign (devices);

  Ipv4StaticRoutingHelper staticRouting;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i)
  {
    Ptr<Ipv4> ipv4 = allNodes.Get (i)->GetObject<Ipv4> ();
    staticRouting.GetStaticRouting (ipv4)
        ->SetDefaultMulticastRoute (1);
  }

  Ptr<ServerApp> serverApp = CreateObject<ServerApp> ();
  serverApp->Setup (
      MULTICAST_GROUP,
      packetSize,
      kChannels,
      Seconds (interval),
      Seconds (channelOffset),
      mode);

  serverNode.Get (0)->AddApplication (serverApp);

  // クライアント開始時刻以降に送信したパケットだけを計算する
  serverApp->SetStartTime (Seconds (clientStart));

  const uint32_t totalSegments =
      serverApp->GetTotalSegments ();

  const std::string modeName =
      (mode == 0) ? "FB法"
                  : ((mode == 1) ? "放送配信" : "等分割法");

  // 全クライアントを同じ時刻に参加させる
  for (uint32_t i = 0; i < nNodes; ++i)
  {
    Ptr<ClientApp> clientApp = CreateObject<ClientApp> ();

    clientApp->Setup (
        totalSegments,
        verbose,
        Seconds (drainTime),
        MakeCallback (&ServerApp::StopSending, serverApp));

    clientNodes.Get (i)->AddApplication (clientApp);
    clientApp->SetStartTime (Seconds (clientStart));
  }

  if (enablePcap)
  {
    phy.EnablePcap ("dps-large-scale", devices.Get (0));
  }

  NS_LOG_UNCOND ("================================================");
  NS_LOG_UNCOND ("【実験設定：大規模化対応版】");
  NS_LOG_UNCOND ("配信手法          : " << modeName);
  NS_LOG_UNCOND ("ノード数          : " << nNodes);
  NS_LOG_UNCOND ("論理チャネル数    : "
                 << serverApp->GetLogicalChannelCount ());
  NS_LOG_UNCOND ("総セグメント数(N) : " << totalSegments);
  NS_LOG_UNCOND ("チャネル送信間隔  : " << interval << "s");
  NS_LOG_UNCOND ("チャネル初期ずれ  : " << channelOffset << "s");
  NS_LOG_UNCOND ("受信ポート        : " << RECEIVE_PORT << "（全チャネル共通）");
  NS_LOG_UNCOND ("クライアント開始  : " << clientStart << "s（全台共通）");
  NS_LOG_UNCOND ("パケットサイズ    : " << packetSize << " bytes");
  NS_LOG_UNCOND ("------------------------------------------------");
  NS_LOG_UNCOND ("【無線設定】");
  NS_LOG_UNCOND ("Wi-Fi規格         : IEEE 802.11b");
  NS_LOG_UNCOND ("データレート      : " << dataRate);
  NS_LOG_UNCOND ("距離              : " << distance << "m");
  NS_LOG_UNCOND ("PCAP              : " << (enablePcap ? "ON" : "OFF"));
  NS_LOG_UNCOND ("個別ログ          : " << (verbose ? "ON" : "OFF"));
  NS_LOG_UNCOND ("乱数Seed          : " << seed);
  NS_LOG_UNCOND ("乱数Run           : " << run);
  NS_LOG_UNCOND ("================================================");

  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();

  double totalArrivalRate = 0.0;
  uint64_t totalRawReceived = 0;

  for (uint32_t i = 0; i < clientNodes.GetN (); ++i)
  {
    Ptr<ClientApp> app =
        DynamicCast<ClientApp> (
            clientNodes.Get (i)->GetApplication (0));

    const double arrivalRate =
        (static_cast<double> (app->GetReceivedCount ()) /
         totalSegments) *
        100.0;

    totalArrivalRate += arrivalRate;
    totalRawReceived += app->GetRawReceivedCount ();

    if (verbose)
    {
      NS_LOG_UNCOND (
          "Node " << clientNodes.Get (i)->GetId ()
          << ": unique=" << app->GetReceivedCount ()
          << "/" << totalSegments
          << ", raw=" << app->GetRawReceivedCount ()
          << ", complete=" << (app->IsComplete () ? "yes" : "no"));
    }
  }

  const double averageArrivalRate =
      totalArrivalRate / nNodes;

  const double completionRate =
      (static_cast<double> (g_completedNodes) / nNodes) * 100.0;

  const double averageWaitTime =
      (g_completedNodes == 0)
          ? 0.0
          : g_waitTimeSum / g_completedNodes;

  const double minimumWaitTime =
      (g_completedNodes == 0) ? 0.0 : g_minWaitTime;

  const double maximumWaitTime =
      (g_completedNodes == 0) ? 0.0 : g_maxWaitTime;

  const uint64_t totalSent =
      serverApp->GetTotalSent ();

  const uint64_t expectedReceptions =
      totalSent * static_cast<uint64_t> (nNodes);

  const uint64_t lostPackets =
      (expectedReceptions >= totalRawReceived)
          ? expectedReceptions - totalRawReceived
          : 0;

  double packetLossRate = 0.0;
  if (expectedReceptions > 0)
  {
    packetLossRate =
        (1.0 -
         static_cast<double> (totalRawReceived) /
             expectedReceptions) *
        100.0;
  }

  packetLossRate = std::max (0.0, packetLossRate);

  NS_LOG_UNCOND ("\n==== 集計結果 ====");
  NS_LOG_UNCOND ("完了ノード数          : "
                 << g_completedNodes << " / " << nNodes);
  NS_LOG_UNCOND ("完了率                : "
                 << completionRate << "%");
  NS_LOG_UNCOND ("完了ノード平均待ち時間: "
                 << averageWaitTime << "s");
  NS_LOG_UNCOND ("最小待ち時間          : "
                 << minimumWaitTime << "s");
  NS_LOG_UNCOND ("最大待ち時間          : "
                 << maximumWaitTime << "s");
  NS_LOG_UNCOND ("全ノード平均到達率    : "
                 << averageArrivalRate << "%");
  NS_LOG_UNCOND ("総送信数              : " << totalSent);
  NS_LOG_UNCOND ("理論受信数            : " << expectedReceptions);
  NS_LOG_UNCOND ("実受信数              : " << totalRawReceived);
  NS_LOG_UNCOND ("観測損失パケット数    : " << lostPackets);
  NS_LOG_UNCOND ("観測パケット欠損率    : "
                 << packetLossRate << "%");

  NS_LOG_UNCOND ("\n=== 最終結果サマリー (CSV) ===");
  NS_LOG_UNCOND (
      "手法,ノード数,論理チャネル数,総セグメント数,"
      "送信間隔,チャネル初期ずれ,帯域幅,パケットサイズ,"
      "平均待ち時間,最小待ち時間,最大待ち時間,"
      "完了ノード数,完了率,平均到達率,観測パケット欠損率,"
      "距離,Seed,Run");

  NS_LOG_UNCOND (
      modeName << ","
               << nNodes << ","
               << serverApp->GetLogicalChannelCount () << ","
               << totalSegments << ","
               << interval << ","
               << channelOffset << ","
               << dataRate << ","
               << packetSize << ","
               << averageWaitTime << ","
               << minimumWaitTime << ","
               << maximumWaitTime << ","
               << g_completedNodes << ","
               << completionRate << "%,"
               << averageArrivalRate << "%,"
               << packetLossRate << "%,"
               << distance << ","
               << seed << ","
               << run);

  Simulator::Destroy ();
  return 0;
}