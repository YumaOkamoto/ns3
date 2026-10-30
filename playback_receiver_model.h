#ifndef SEGMENTED_PLAYBACK_RECEIVER_MODEL_H
#define SEGMENTED_PLAYBACK_RECEIVER_MODEL_H

// Included by the fixed-video experiments AFTER SegmentPacketHeader has been defined.
// Common playback logic for NR unicast and 802.11b IP multicast.
class ClientApp : public ns3::Application
{
public:
  ClientApp () = default;

  void Setup (const std::vector<uint32_t>& packetsPerSegment,
              uint64_t totalVideoBytes, uint32_t payloadSize,
              double videoDuration, double startupBuffer,
              double resumeBuffer, double playbackUnit,
              bool useMulticast, ns3::Ipv4Address multicastGroup,
              bool verbose)
  {
    m_packetsPerSegment = packetsPerSegment;
    m_totalSegments = static_cast<uint32_t> (packetsPerSegment.size () - 1);
    m_totalVideoBytes = totalVideoBytes;
    m_payloadSize = payloadSize;
    m_videoDuration = videoDuration;
    m_startupBuffer = startupBuffer;
    m_resumeBuffer = resumeBuffer;
    m_playbackUnit = playbackUnit;
    m_useMulticast = useMulticast;
    m_multicastGroup = multicastGroup;
    m_verbose = verbose;
    m_packetReceived.resize (m_totalSegments + 1);
    m_packetCounts.assign (m_totalSegments + 1, 0);
    m_segmentBytes.assign (m_totalSegments + 1, 0);
    const uint64_t base = totalVideoBytes / m_totalSegments;
    const uint64_t rem = totalVideoBytes % m_totalSegments;
    for (uint32_t s = 1; s <= m_totalSegments; ++s)
    {
      m_segmentBytes[s] = base + (s <= rem ? 1u : 0u);
      m_packetReceived[s].assign (m_packetsPerSegment[s], 0);
    }
    m_totalUnits = static_cast<uint32_t> (std::ceil (m_videoDuration / m_playbackUnit));
  }

  bool PlaybackStarted () const { return m_playbackStarted; }
  bool PlaybackComplete () const { return m_playbackComplete; }
  double GetInitialWait () const { return m_initialWait; }
  double GetFinishFromJoin () const { return m_finishFromJoin; }
  uint32_t GetStallCount () const { return m_stallCount; }
  uint32_t GetReceivedCount () const { return m_completedSegments; }
  uint64_t GetRawReceivedCount () const { return m_rawReceived; }
  uint64_t GetMeasuredReceivedCount () const { return m_measuredReceived; }
  uint64_t GetUniquePacketCount () const { return m_uniquePackets; }
  uint64_t GetDuplicatePacketCount () const { return m_duplicatePackets; }
  uint64_t GetContiguousBytes () const { return m_contiguousBytes; }
  double GetPlayableSeconds () const
  {
    return m_videoDuration * static_cast<double> (m_contiguousBytes) /
           static_cast<double> (m_totalVideoBytes);
  }
  double GetPlayedSeconds () const
  {
    return std::min (m_videoDuration, m_playedUnits * m_playbackUnit);
  }
  double GetObservedTotalStall () const
  {
    return m_totalStall + (m_stalled ? (ns3::Simulator::Now () - m_stallStart).GetSeconds () : 0.0);
  }
  double GetMaxStall () const
  {
    return m_stalled ? std::max (m_maxStall, (ns3::Simulator::Now () - m_stallStart).GetSeconds ()) : m_maxStall;
  }
  double GetJoinTime () const { return m_joinTime.GetSeconds (); }

private:
  void StartApplication () override
  {
    m_joinTime = ns3::Simulator::Now ();
    m_socket = ns3::Socket::CreateSocket (GetNode (), ns3::UdpSocketFactory::GetTypeId ());
    NS_ABORT_MSG_IF (m_socket->Bind (ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), RECEIVE_PORT)) < 0,
                     "Cannot bind playback UDP receiver");
    if (m_useMulticast)
    {
      ns3::Ptr<ns3::UdpSocket> udp = ns3::DynamicCast<ns3::UdpSocket> (m_socket);
      NS_ABORT_MSG_IF (!udp, "Expected UdpSocket for multicast membership");
      udp->MulticastJoinGroup (0, ns3::InetSocketAddress (m_multicastGroup, RECEIVE_PORT));
    }
    m_socket->SetRecvCallback (ns3::MakeCallback (&ClientApp::HandleRead, this));
  }

  void StopApplication () override
  {
    if (m_socket)
    {
      m_socket->SetRecvCallback (ns3::MakeNullCallback<void, ns3::Ptr<ns3::Socket>> ());
      m_socket->Close ();
      m_socket = nullptr;
    }
  }

  void HandleRead (ns3::Ptr<ns3::Socket> socket)
  {
    ns3::Ptr<ns3::Packet> packet;
    ns3::Address source;
    while ((packet = socket->RecvFrom (source)))
    {
      ++m_rawReceived;
      SegmentPacketHeader header;
      if (packet->RemoveHeader (header) == 0)
      {
        continue;
      }
      const uint32_t segment = header.GetSegmentId ();
      const uint32_t index = header.GetPacketIndex ();
      if (segment == 0 || segment > m_totalSegments ||
          index >= m_packetsPerSegment[segment] ||
          header.GetPacketsInSegment () != m_packetsPerSegment[segment])
      {
        continue;
      }
      // This flag was set by the sender for the common measurement interval.
      if (header.GetMeasured ())
      {
        ++m_measuredReceived;
      }
      if (m_packetReceived[segment][index])
      {
        ++m_duplicatePackets;
        continue;
      }
      m_packetReceived[segment][index] = 1;
      ++m_uniquePackets;
      ++m_packetCounts[segment];
      if (m_packetCounts[segment] == m_packetsPerSegment[segment])
      {
        ++m_completedSegments;
      }
      // Only consecutive video data from byte zero can be played.
      while (m_nextSegment <= m_totalSegments &&
             m_nextPacket < m_packetsPerSegment[m_nextSegment] &&
             m_packetReceived[m_nextSegment][m_nextPacket])
      {
        const uint64_t offset = static_cast<uint64_t> (m_nextPacket) * m_payloadSize;
        const uint64_t bytes = std::min<uint64_t> (m_payloadSize, m_segmentBytes[m_nextSegment] - offset);
        m_contiguousBytes += bytes;
        ++m_nextPacket;
        if (m_nextPacket == m_packetsPerSegment[m_nextSegment])
        {
          ++m_nextSegment;
          m_nextPacket = 0;
        }
      }
      TryStartOrResume ();
    }
  }

  uint64_t ByteThreshold (double playbackSeconds) const
  {
    const double fraction = std::min (1.0, std::max (0.0, playbackSeconds / m_videoDuration));
    return static_cast<uint64_t> (std::ceil (fraction * m_totalVideoBytes - 1e-7));
  }

  bool HasBufferAhead (double seconds) const
  {
    const double playhead = GetPlayedSeconds ();
    const double target = std::min (m_videoDuration, playhead + seconds);
    return m_contiguousBytes >= ByteThreshold (target);
  }

  void TryStartOrResume ()
  {
    if (m_playbackComplete)
    {
      return;
    }
    if (!m_playbackStarted)
    {
      if (!HasBufferAhead (std::min (m_startupBuffer, m_videoDuration)))
      {
        return;
      }
      m_playbackStarted = true;
      m_initialWait = (ns3::Simulator::Now () - m_joinTime).GetSeconds ();
      if (m_verbose)
      {
        NS_LOG_UNCOND ("UE " << GetNode ()->GetId () << " START wait=" << m_initialWait
                               << " contiguous=" << GetPlayableSeconds ());
      }
      ScheduleNextUnitOrStall ();
    }
    else if (m_stalled && HasBufferAhead (std::min (m_resumeBuffer, m_videoDuration - GetPlayedSeconds ())))
    {
      const double delay = (ns3::Simulator::Now () - m_stallStart).GetSeconds ();
      m_totalStall += delay;
      m_maxStall = std::max (m_maxStall, delay);
      m_stalled = false;
      if (m_verbose)
      {
        NS_LOG_UNCOND ("UE " << GetNode ()->GetId () << " RESUME stall=" << delay);
      }
      ScheduleNextUnitOrStall ();
    }
  }

  void ScheduleNextUnitOrStall ()
  {
    if (m_playbackComplete || m_stalled)
    {
      return;
    }
    if (m_playedUnits >= m_totalUnits)
    {
      m_playbackComplete = true;
      m_finishFromJoin = (ns3::Simulator::Now () - m_joinTime).GetSeconds ();
      if (m_verbose)
      {
        NS_LOG_UNCOND ("UE " << GetNode ()->GetId () << " FINISH " << m_finishFromJoin);
      }
      return;
    }
    const double nextEnd = std::min (m_videoDuration, (m_playedUnits + 1) * m_playbackUnit);
    if (m_contiguousBytes >= ByteThreshold (nextEnd))
    {
      const double step = nextEnd - GetPlayedSeconds ();
      ns3::Simulator::Schedule (ns3::Seconds (step), &ClientApp::OnUnitFinished, this);
    }
    else
    {
      m_stalled = true;
      ++m_stallCount;
      m_stallStart = ns3::Simulator::Now ();
      if (m_verbose)
      {
        NS_LOG_UNCOND ("UE " << GetNode ()->GetId () << " STALL at " << GetPlayedSeconds ());
      }
    }
  }

  void OnUnitFinished ()
  {
    ++m_playedUnits;
    ScheduleNextUnitOrStall ();
  }

  ns3::Ptr<ns3::Socket> m_socket;
  ns3::Time m_joinTime = ns3::Seconds (0);
  ns3::Time m_stallStart = ns3::Seconds (0);
  std::vector<uint32_t> m_packetsPerSegment, m_packetCounts;
  std::vector<uint64_t> m_segmentBytes;
  std::vector<std::vector<uint8_t>> m_packetReceived;
  uint32_t m_totalSegments = 0, m_payloadSize = 1024;
  uint32_t m_nextSegment = 1, m_nextPacket = 0;
  uint32_t m_completedSegments = 0, m_stallCount = 0;
  uint32_t m_totalUnits = 0, m_playedUnits = 0;
  uint64_t m_totalVideoBytes = 0, m_contiguousBytes = 0;
  uint64_t m_rawReceived = 0, m_measuredReceived = 0, m_uniquePackets = 0, m_duplicatePackets = 0;
  double m_videoDuration = 0.0, m_startupBuffer = 0.5, m_resumeBuffer = 0.5, m_playbackUnit = 0.1;
  double m_initialWait = 0.0, m_totalStall = 0.0, m_maxStall = 0.0, m_finishFromJoin = 0.0;
  bool m_useMulticast = false, m_verbose = false;
  bool m_playbackStarted = false, m_playbackComplete = false, m_stalled = false;
  ns3::Ipv4Address m_multicastGroup;
};
#endif
