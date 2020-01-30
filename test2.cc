#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/netanim-module.h"

#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor-helper.h"
using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");

class RoutingExperiment
{
public:
  RoutingExperiment ();
  void Run (int nSinks, double txp, std::string CSVfileName);
  //static void SetMACParam (ns3::NetDeviceContainer & devices,
  //                                 int slotDistance);
  std::string CommandSetup (int argc, char **argv);

private:
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node); //POinter to the socket, template library
  void ReceivePacket (Ptr<Socket> socket); //packet is received through a packet number
  void CheckThroughput ();

  uint32_t port;
  uint32_t bytesTotal;
  uint32_t packetsReceived;

  std::string m_CSVfileName;    // file where we are going to record the data
  int m_nSinks;                 // No. of sinks available
  std::string m_protocolName;   // name of the protocal
  double m_txp;                 
  bool m_traceMobility;         // trace the mobility -> one method is by generating PCAP files while can be through .flowmon
  uint32_t m_protocol;          // there are 4 protocols being used -> AODV, DSDV, OLSR, DSR
};

RoutingExperiment::RoutingExperiment ()         // CONSTRUCTOR
  : port (9),                                   // port no. 9 has been used
    bytesTotal (0),                             // initialising the values to 0 for all the variables
    packetsReceived (0),
    m_CSVfileName ("DSR.csv"),                 // for different protocols. different files can be generated and then compare finally at the last
    m_traceMobility (false),
    m_protocol (4) // AODV                      1->OLSR 2->AODV 3->DSDV 4->DSR
{
}

static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)     // how many packets has been received
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 (); // print the address from where the packet has been received
    }
  else
    {
      oss << " received one packet!";   // If it doesn't know from where has the packet has been received, then it prints just the packet has been received
    }
  return oss.str ();
}

void
RoutingExperiment::ReceivePacket (Ptr<Socket> socket) // the number of bytes that has been received
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();         // total byted received, it keeps on adding
      packetsReceived += 1;                     // no . of packets received, it keeps a track on it.
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

void
RoutingExperiment::CheckThroughput ()
{
  double kbs = (bytesTotal * 8.0) / 1000;      // 1 byte = 8 bits so (*8.0) has been done. Also we need to convert it to kbits so (/1000) has been done
  bytesTotal = 0;

  std::ofstream out (m_CSVfileName.c_str (), std::ios::app); // data has been recorded -> SimulationTime, kbs, packetsReceived, sinks, protocol, txp

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << m_nSinks << ","
      << m_protocolName << ","
      << m_txp << ""
      << std::endl;             // all of these has been saved into the file AODV.csv

  out.close ();
  packetsReceived = 0;
  Simulator::Schedule (Seconds (1.0), &RoutingExperiment::CheckThroughput, this);
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

  return sink;
}

std::string
RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd;
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.Parse (argc, argv);
  return m_CSVfileName;
}

int
main (int argc, char *argv[])
{
  RoutingExperiment experiment; // object is created
  std::string CSVfileName = experiment.CommandSetup (argc,argv);

  //blank out the last output file and write the column headers
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  "RoutingProtocol," <<
  "TransmissionPower" <<
  std::endl;
  out.close ();

  int nSinks = 10;
  double txp = 7.5;

  experiment.Run (nSinks, txp, CSVfileName);
}

/*

        function (int a , int b)
        {
                this.a = a;     // y=a;
                this.b = b;     // y=b;
        }

*/
void
RoutingExperiment::Run (int nSinks, double txp, std::string CSVfileName)
{
  Packet::EnablePrinting ();    // data will be printed
  m_nSinks = nSinks;
  m_txp = txp;
  m_CSVfileName = CSVfileName;

  int nWifis = 3;
  int nWifi = 4;
  bool verbose = true;
  uint32_t nCsma = 3;
  //bool tracing = false;

     if ((nWifis > 18) && (nWifi > 18))
    {
      std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box" << std::endl;
      //return 1;
    }

    if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  NodeContainer p2pNodes;
  p2pNodes.Create (2);

  NodeContainer p2pNode;
  p2pNode.Create (2);

  PointToPointHelper pointToPoint, pointToPoint2;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));
  pointToPoint2.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint2.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer p2pDevices,p2pDevices2;
  p2pDevices = pointToPoint.Install (p2pNodes);
  p2pDevices2 = pointToPoint2.Install (p2pNode);

  NodeContainer csmaNodes;
  csmaNodes.Add (p2pNodes.Get (1));
  csmaNodes.Add (p2pNode.Get (1));
  csmaNodes.Create (nCsma-1);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

  NetDeviceContainer csmaDevices;
  csmaDevices = csma.Install (csmaNodes);

  double TotalTime = 200.0;
  std::string rate ("2048bps");
  std::string phyMode ("DsssRate11Mbps");
  std::string tr_name ("DSR");
  int nodeSpeed = 20; //in m/s
  int nodePause = 0; //in s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("64"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));

  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

  NodeContainer adhocNodes;
  adhocNodes.Create (nWifis);
  NodeContainer wifiApNode = p2pNodes.Get (0); // Access Point Node

  NodeContainer adhocNodes2;
  adhocNodes2.Create (nWifi);
  NodeContainer wifiApNode2 = p2pNode.Get (0); // Access Point Node

//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------1st set of WifiNodes Connected with Csma LAN Node no.1---------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

        // YANS - YET ANOTHER NETWORK SIMULATOR
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));   

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  NetDeviceContainer apDevices;
  apDevices = wifi.Install (wifiPhy, wifiMac, wifiApNode);

  MobilityHelper mobilityAdhoc; // what kind of moility helper this network would  work
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios


  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",          // nodes are positioned on grid system
                                 "MinX", DoubleValue (0.0),             // minimum x
                                 "MinY", DoubleValue (0.0),             // minimum y
                                 "DeltaX", DoubleValue (5.0),           // difference in x
                                 "DeltaY", DoubleValue (10.0),          // difference in y
                                 "GridWidth", UintegerValue (3),        // width in grid
                                 "LayoutType", StringValue ("RowFirst"));// RowFirst is the LayoutType
mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel"); // the nodes are position is fixed and no RectangleValue has to be mentioned.
  mobility.Install (wifiApNode);
InternetStackHelper stack;
  stack.Install (csmaNodes);
  stack.Install (wifiApNode);
  stack.Install (adhocNodes);

 Ipv4AddressHelper address;

  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces;
  p2pInterfaces = address.Assign (p2pDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer csmaInterfaces;
  csmaInterfaces = address.Assign (csmaDevices);

  address.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = address.Assign (adhocDevices);
  address.Assign (apDevices);


  Ipv4GlobalRoutingHelper::PopulateRoutingTables (); // ns3 provides routing tables algorithm to make GlobalRoutingTable

//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------2nd set of WifiNodes Connected with Csma LAN Node no.2---------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------

/* we need to set up 3 things -> Channel, physical layer, MAC- medium access control*/

  // YANS -> Yet Another Network Simulator
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();// we are using default channel location or default channel properties such as frequency modulation. The whole coding behind this is done by an Electronics student or telecom student.
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ()); // Creating a communication path


// we browse internet through wifiManager or a router

  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");// AarfWifiManager is one of SetRemoteStationManager

  WifiMacHelper mac;
  Ssid ssid = Ssid ("ns-3-ssid");       // ssid is the broadcast id. when we switch on our wifi on phone , the names of wifi signals are the ssid.
// what kind of mac has been saved
  mac.SetType ("ns3::StaWifiMac",               // StationWifiMac
               "Ssid", SsidValue (ssid),        // 
               "ActiveProbing", BooleanValue (false)); // two types of nodes in wifi -> station nodes(moving nodes) & access point nodes (fixed due to ethernet connectivity)
        // ActiveProbing keeps on checking whether there is any wifi or not in the nearby area.

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, adhocNodes2);

// AP another model of wifi
  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));

  NetDeviceContainer apDevices2;
  apDevices2 = wifi.Install (phy, mac, wifiApNode2);



  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",          // nodes are positioned on grid system
                                 "MinX", DoubleValue (0.0),             // minimum x
                                 "MinY", DoubleValue (0.0),             // minimum y
                                 "DeltaX", DoubleValue (5.0),           // difference in x
                                 "DeltaY", DoubleValue (10.0),          // difference in y
                                 "GridWidth", UintegerValue (3),        // width in grid
                                 "LayoutType", StringValue ("RowFirst"));// RowFirst is the LayoutType

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", // z axis is missing
                             "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobility.Install (adhocNodes2); // moving model

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel"); // the nodes are position is fixed and no RectangleValue has to be mentioned.
  mobility.Install (wifiApNode2);


  stack.Install (csmaNodes);
  stack.Install (wifiApNode2);
  stack.Install (adhocNodes2);

  Ipv4AddressHelper address2;

  address2.SetBase ("10.1.4.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces2;
  p2pInterfaces2 = address.Assign (p2pDevices2);

  address2.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer csmaInterfaces2;
  csmaInterfaces2 = address2.Assign (csmaDevices);

  address2.SetBase ("10.1.5.0", "255.255.255.0");
  address2.Assign (staDevices);
  address2.Assign (apDevices2);

//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator"); // nodes are allocated in a rectangular fashion that too in random. In this case a rectangle of 300 X 1500 has been setup.
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]")); // x_min =0 and x_max = 300
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]")); // y_min = 0 and y_max = 1500

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]"; // nodes can move with a random speed of min 0 m/s and a max of 20 m/s
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (adhocNodes);
  streamIndex += mobilityAdhoc.AssignStreams (adhocNodes, streamIndex);
  NS_UNUSED (streamIndex); // From this point, streamIndex is unused

// PROTOCOL SECTION
  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  switch (m_protocol)
    {
    case 1:
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2:
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

  if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);
      internet.Install (adhocNodes);
    }
  else if (m_protocol == 4)
    {
      internet.Install (adhocNodes);
      dsrMain.Install (dsr, adhocNodes);
    }

  NS_LOG_INFO ("assigning ip address");

  /*Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);*/

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  for (int i = 0; i < nSinks; i++)
    {
      Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));

      AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), port)); // nodes need to remember which port no it has to be received.
      onoff1.SetAttribute ("Remote", remoteAddress);

      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (adhocNodes.Get (i + nSinks));
      temp.Start (Seconds (var->GetValue (100.0,101.0)));
      temp.Stop (Seconds (TotalTime));
    }

  std::stringstream ss;
  ss << nWifis;
  std::string nodes = ss.str ();

  std::stringstream ss2;
  ss2 << nodeSpeed;
  std::string sNodeSpeed = ss2.str ();

  std::stringstream ss3;
  ss3 << nodePause;
  std::string sNodePause = ss3.str ();

  std::stringstream ss4;
  ss4 << rate;
  std::string sRate = ss4.str ();

  //NS_LOG_INFO ("Configure Tracing.");
  //tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sNodeSpeed + "speed_" + sNodePause + "pause_" + sRate + "rate";

  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (tr_name + ".tr").c_str());
  wifiPhy.EnableAsciiAll (osw);
  //AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

/* FlowMonitor generates the XML file and this would help to trace the packets for different nodes*/
  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();


  NS_LOG_INFO ("Run Simulation.");

  CheckThroughput ();

  Simulator::Stop (Seconds (TotalTime));
  Simulator::Run ();

  flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);

  Simulator::Destroy ();
}

