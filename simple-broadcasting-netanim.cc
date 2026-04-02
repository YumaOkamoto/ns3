#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h" // NetAnim ヘッダーを追加

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SimpleBroadcastingNetAnimExample");

// --- カスタム送信アプリケーション ---
class BroadcastingServerApp : public Application {
public:
  BroadcastingServerApp () : m_running (false) {}
  
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, Time interval, uint32_t segmentId) {
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_interval = interval;
    m_segmentId = segmentId; // NetAnimでセグメントIDを追跡するために追加
  }

private:
  virtual void StartApplication (void) {
    m_running = true;
    SendPacket ();
  }

  virtual void StopApplication (void) {
    m_running = false;
    if (m_sendEvent.IsRunning ()) {
      Simulator::Cancel (m_sendEvent);
    }
  }

  void SendPacket (void) {
    // パケットに情報を付与するためのカスタムヘッダーの代わりに、今回は簡易的にサイズで識別
    // 実際にはカスタムヘッダーを使う方が望ましい
    Ptr<Packet> packet = Create<Packet> (m_packetSize + m_segmentId); // セグメントIDをパケットサイズに含めてNetAnimで識別しやすくする（あくまで例）
    m_socket->SendTo (packet, 0, m_peer);
    
    if (m_running) {
      m_sendEvent = Simulator::Schedule (m_interval, &BroadcastingServerApp::SendPacket, this);
    }
  }

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  Time            m_interval;
  bool            m_running;
  EventId         m_sendEvent;
  uint32_t        m_segmentId; // 各アプリがどのセグメントを放送しているか
};

int main (int argc, char *argv[]) {
  LogComponentEnable ("SimpleBroadcastingNetAnimExample", LOG_LEVEL_INFO);

  uint32_t nSegments = 3;      // 分割数 L
  double videoDuration = 30.0; // 動画の長さ（秒）
  uint32_t basePacketSize = 100; // 基本パケットサイズ (バイト)

  NodeContainer nodes;
  nodes.Create (2); // 0: Server, 1: Client

  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer devices = p2p.Install (nodes);

  InternetStackHelper stack;
  stack.Install (nodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  // --- NetAnimHelperの設定 ---
  AnimationInterface anim("simple-broadcasting.xml");
  anim.EnablePacketMetadata (); // パケットのメタデータを記録
 //anim.EnableIpv4RouteTracking (); // ルーティング情報を記録 (今回のP2Pではあまり意味ないが、一般的な設定)

  // ノードの位置を設定 (NetAnim上での表示位置)
  anim.SetConstantPosition (nodes.Get (0), 10.0, 50.0); // サーバー
  anim.SetConstantPosition (nodes.Get (1), 90.0, 50.0); // クライアント

  // --- サーバー側の設定 ---
  // 各セグメントを異なるポートで送信 (9001, 9002, 9003...)
  for (uint32_t i = 0; i < nSegments; ++i) {
    uint16_t port = 9001 + i;
    Ptr<Socket> sock = Socket::CreateSocket (nodes.Get (0), UdpSocketFactory::GetTypeId ());
    InetSocketAddress destAddr (interfaces.GetAddress (1), port); // クライアントのIPアドレスを指定
    
    Ptr<BroadcastingServerApp> app = CreateObject<BroadcastingServerApp> ();
    Time interval = Seconds (videoDuration / nSegments); // 各セグメントの放送周期
    app->Setup (sock, destAddr, basePacketSize, interval, i); // segmentId を追加
    
    nodes.Get (0)->AddApplication (app);
    app->SetStartTime (Seconds (1.0 + (i * 0.05))); // 少しずらして開始 (NetAnimで識別しやすくするため)
    app->SetStopTime (Seconds (videoDuration + 5.0));
    
    NS_LOG_INFO ("Segment " << i << " will be broadcasted on port " << port << " every " << interval.GetSeconds() << "s");
  }

  // --- クライアント側の設定 (受信ログ) ---
  // NetAnimで可視化するために、全ポートの受信トラフィックを記録する必要がある
  // PacketSinkは一つのポートしかListenできないため、複数のPacketSinkをインストールするか、カスタムレシーバーを作成する
  for (uint32_t i = 0; i < nSegments; ++i) {
    uint16_t port = 9001 + i;
    PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = sink.Install (nodes.Get (1));
    sinkApps.Start (Seconds (0.0));
    sinkApps.Stop (Seconds (videoDuration + 6.0));
  }


  Simulator::Stop (Seconds (videoDuration + 7.0)); // シミュレーション終了時間
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
