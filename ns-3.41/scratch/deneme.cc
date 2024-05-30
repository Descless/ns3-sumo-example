#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/propagation-module.h"
#include "ns3/lora-helper.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/ns2-mobility-helper.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("SumoLorawanIntegrationExample");

int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.Parse(argc, argv);


    // Set up logging
    LogComponentEnable("SumoLorawanIntegrationExample", LOG_LEVEL_ALL);
    LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
    LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
    LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
    LogComponentEnable("OneShotSenderHelper", LOG_LEVEL_ALL);
    LogComponentEnable("OneShotSender", LOG_LEVEL_ALL);
    LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
    LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnableAll(LOG_PREFIX_NODE);
    LogComponentEnableAll(LOG_PREFIX_TIME);


    /************************
     *  Create the channel  *
     ************************/
    NS_LOG_INFO("Creating the channel...");

    // Create the lora channel object
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/
    NS_LOG_INFO("Setting up helpers...");

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    allocator->Add(Vector(1000, 0, 0));
    allocator->Add(Vector(0, 0, 0));
    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper();

    // Create the LoraHelper
    LoraHelper helper = LoraHelper();

    /************************
     *  Create End Devices  *
     ************************/
    NS_LOG_INFO("Creating the end devices...");

    // Create a set of nodes
    NodeContainer endDevices;
    endDevices.Create(5);

    // Assign a mobility model to the nodes
    mobility.Install(endDevices);

    // Create the LoraNetDevices of the end devices
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    helper.Install(phyHelper, macHelper, endDevices);

    // Assign mobility to some devices
    MobilityHelper mobilityRandom;
    mobilityRandom.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                    "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
                                    "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                                    "PositionAllocator", PointerValue(allocator));
    NodeContainer mobileDevices;
    mobileDevices.Add(endDevices.Get(1));
    mobileDevices.Add(endDevices.Get(2));
    mobilityRandom.Install(mobileDevices);

    /*********************
     *  Create Gateways  *
     *********************/
    NS_LOG_INFO("Creating the gateway...");
    NodeContainer gateways;
    gateways.Create(1);

    mobility.Install(gateways);

    // Create a netdevice for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/
    OneShotSenderHelper oneShotSenderHelper;
    oneShotSenderHelper.SetSendTime(Seconds(2));

    oneShotSenderHelper.Install(endDevices);

    /******************
     * Set Data Rates *
     ******************/
    std::vector<int> sfQuantity(6);
    sfQuantity = LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    /**************************
     * Integrate SUMO mobility*
     **************************/
    NS_LOG_INFO("Integrating SUMO mobility...");

    // Create nodes
    NodeContainer sumoNodes;
    sumoNodes.Create(10); // Adjust based on your simulation

    // Install mobility model from SUMO trace
    std::string traceFile = "sumoTrace.xml"; // Adjust this path if necessary
    Ns2MobilityHelper ns2(traceFile);
    ns2.Install();

    // Install Internet Stack
    InternetStackHelper internet;
    internet.Install(sumoNodes);

    // Create point-to-point links between pairs of nodes
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

    std::vector<NetDeviceContainer> devices;
    for (uint32_t i = 0; i < sumoNodes.GetN() - 1; ++i) {
        NodeContainer pair(sumoNodes.Get(i), sumoNodes.Get(i + 1));
        devices.push_back(pointToPoint.Install(pair));
    }

    // Assign IP addresses
    Ipv4AddressHelper ipv4;
    std::vector<Ipv4InterfaceContainer> interfaces;
    for (uint32_t i = 0; i < devices.size(); ++i) {
        std::ostringstream subnet;
        subnet << "10.1." << i + 1 << ".0";
        ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
        interfaces.push_back(ipv4.Assign(devices[i]));
    }

    // Set up applications (e.g., a simple ping application)
    uint16_t port = 9;
    UdpEchoServerHelper echoServer(port);

    ApplicationContainer serverApps = echoServer.Install(sumoNodes.Get(1));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    UdpEchoClientHelper echoClient(interfaces[0].GetAddress(1), port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(sumoNodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));

    // Enable tracing
    AsciiTraceHelper ascii;
    pointToPoint.EnableAsciiAll(ascii.CreateFileStream("sumo_lorawan_integration.tr"));

    /****************
     *  Simulation  *
     ****************/
    Simulator::Stop(Hours(2));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
