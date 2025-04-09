/*
* Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
* Copyright (c) 2019, University of Padova, Dep. of Information Engineering, SIGNET lab
*
* SPDX-License-Identifier: GPL-2.0-only
*
* Author: Nicola Baldo <nbaldo@cttc.es> for the code adapted from the lena-dual-stripe.cc example
* Author: Michele Polese <michele.polese@gmail.com> for this version
*/

#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/eht-phy.h"
#include "ns3/enum.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-server.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

#include <array>
#include <functional>
#include <numeric>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RandomWalk2dIndoor");

/**
 * Print the buildings list in a format that can be used by Gnuplot to draw them.
 *
 * \param filename The output filename.
 */
void
PrintGnuplottableBuildingListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    uint32_t index = 0;
    for (auto it = BuildingList::Begin(); it != BuildingList::End(); ++it)
    {
        ++index;
        Box box = (*it)->GetBoundaries();
        outFile << "set object " << index << " rect from " << box.xMin << "," << box.yMin << " to "
                << box.xMax << "," << box.yMax << std::endl;
    }
}

static void
TimePasses ()
{
time_t t = time (nullptr);
struct tm tm = *localtime (&t);

std::cout << "Simulation Time: " << Simulator::Now ().As (Time::S) << " real time: "
            << tm.tm_hour << "h, " << tm.tm_min << "m, " << tm.tm_sec << "s." << std::endl;
Simulator::Schedule (MilliSeconds (100), &TimePasses);
}

/**
 * \param udp true if UDP is used, false if TCP is used
 * \param serverApp a container of server applications
 * \param payloadSize the size in bytes of the packets
 * \return the bytes received by each server application
 */
std::vector<uint64_t>
GetRxBytes(bool udp, const ApplicationContainer& serverApp, uint32_t payloadSize)
{
    std::vector<uint64_t> rxBytes(serverApp.GetN(), 0);
    if (udp)
    {
        for (uint32_t i = 0; i < serverApp.GetN(); i++)
        {
            rxBytes[i] = payloadSize * DynamicCast<UdpServer>(serverApp.Get(i))->GetReceived();
        }
    }
    else
    {
        for (uint32_t i = 0; i < serverApp.GetN(); i++)
        {
            rxBytes[i] = DynamicCast<PacketSink>(serverApp.Get(i))->GetTotalRx();
        }
    }
    return rxBytes;
}

/**
 * Print average throughput over an intermediate time interval.
 * \param rxBytes a vector of the amount of bytes received by each server application
 * \param udp true if UDP is used, false if TCP is used
 * \param serverApp a container of server applications
 * \param payloadSize the size in bytes of the packets
 * \param tputInterval the duration of an intermediate time interval
 * \param simulationTime the simulation time in seconds
 */
void
PrintIntermediateTput(std::vector<uint64_t>& rxBytes,
                    bool udp,
                    const ApplicationContainer& serverApp,
                    uint32_t payloadSize,
                    Time tputInterval,
                    Time simulationTime)
{
    auto newRxBytes = GetRxBytes(udp, serverApp, payloadSize);
    Time now = Simulator::Now();

    std::cout << "[" << (now - tputInterval).As(Time::S) << " - " << now.As(Time::S)
            << "] Per-STA Throughput (Mbit/s):";

    for (std::size_t i = 0; i < newRxBytes.size(); i++)
    {
        std::cout << "\t\t(" << i << ") "
                << (newRxBytes[i] - rxBytes[i]) * 8. / tputInterval.GetMicroSeconds(); // Mbit/s
    }
    std::cout << std::endl;

    rxBytes.swap(newRxBytes);

    if (now < (simulationTime - NanoSeconds(1)))
    {
        Simulator::Schedule(Min(tputInterval, simulationTime - now - NanoSeconds(1)),
                            &PrintIntermediateTput,
                            rxBytes,
                            udp,
                            serverApp,
                            payloadSize,
                            tputInterval,
                            simulationTime);
    }
}

bool findSubstring(const std::string& text, const std::string& substring) 
{
    return text.find(substring) != std::string::npos;
}

/**
 * This is an example on how to use the RandomWalk2dOutdoorMobilityModel class.
 * The script outdoor-random-walk-example.sh can be used to visualize the
 * positions visited by the random walk.
 */
int
main(int argc, char* argv[])
{
    // General parameters
    std::string outputDir = "./scratch/wifi-eht-random-walk/";
    Time simulationTime{"30s"};
    std::size_t nStations{1};

    // Building parameters
    double officeSizeX = 40; // m
    double officeSizeY = 20;  // m
    double officeHeight = 3; // m

    // Wifi Simulation Parameters
    bool udp{true};
    bool downlink{true};
    bool useRts{false};
    bool use80Plus80{false};
    uint16_t mpduBufferSize{512};
    std::string emlsrLinks;//="0,1,2";
    uint16_t paddingDelayUsec{32};
    uint16_t transitionDelayUsec{128};
    uint16_t channelSwitchDelayUsec{100};
    bool switchAuxPhy{true};
    uint16_t auxPhyChWidth{20};
    bool auxPhyTxCapable{true};
    std::vector<double> frequency = {5,6};
    dBm_u powSta{10.0};
    dBm_u powAp{21.0};
    dBm_u ccaEdTr{-62};
    dBm_u minimumRssi{-82};
    int channelWidth = 20;
    int gi = 3200;
    std::string dlAckSeqType{"MU-BAR"};//(NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    int mcs{0}; // -1 indicates an unset value
    uint32_t payloadSize =
        700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    double poissonLambda = 1000;
    double minExpectedThroughput{0};
    double maxExpectedThroughput{0};
    Time accessReqInterval{0};
    bool enableMultiApCoordination = false;
    bool reliabilityMode = false;
    bool enablePoisson = true;

    LogComponentEnable("RandomWalk2dIndoor", LOG_LEVEL_LOGIC);
    CommandLine cmd(__FILE__);
    cmd.AddValue("simulationTime", "Simulation time", simulationTime);
    cmd.AddValue("emlsrLinks",
                "The comma separated list of IDs of EMLSR links (for MLDs only)",
                emlsrLinks);
    cmd.AddValue("emlsrPaddingDelay",
                "The EMLSR padding delay in microseconds (0, 32, 64, 128 or 256)",
                paddingDelayUsec);
    cmd.AddValue("emlsrTransitionDelay",
                "The EMLSR transition delay in microseconds (0, 16, 32, 64, 128 or 256)",
                transitionDelayUsec);
    cmd.AddValue("emlsrAuxSwitch",
                "Whether Aux PHY should switch channel to operate on the link on which "
                "the Main PHY was operating before moving to the link of the Aux PHY. ",
                switchAuxPhy);
    cmd.AddValue("emlsrAuxChWidth",
                "The maximum channel width (MHz) supported by Aux PHYs.",
                auxPhyChWidth);
    cmd.AddValue("emlsrAuxTxCapable",
                "Whether Aux PHYs are capable of transmitting.",
                auxPhyTxCapable);
    cmd.AddValue("channelSwitchDelay",
                "The PHY channel switch delay in microseconds",
                channelSwitchDelayUsec);
    cmd.AddValue("simulationTime", "Simulation time", simulationTime);
    cmd.AddValue("udp", "UDP if set to 1, TCP otherwise", udp);
    cmd.AddValue("downlink",
                "Generate downlink flows if set to 1, uplink flows otherwise",
                downlink);
    cmd.AddValue("useRts", "Enable/disable RTS/CTS", useRts);
    cmd.AddValue("use80Plus80", "Enable/disable use of 80+80 MHz", use80Plus80);
    cmd.AddValue("mpduBufferSize",
                "Size (in number of MPDUs) of the BlockAck buffer",
                mpduBufferSize);
    cmd.AddValue("nStations", "Number of non-AP EHT stations", nStations);
    cmd.AddValue("dlAckType",
                "Ack sequence type for DL OFDMA (NO-OFDMA, ACK-SU-FORMAT, MU-BAR, AGGR-MU-BAR)",
                dlAckSeqType);
    cmd.AddValue("enableUlOfdma",
                "Enable UL OFDMA (useful if DL OFDMA is enabled and TCP is used)",
                enableUlOfdma);
    cmd.AddValue("enableBsrp",
                "Enable BSRP (useful if DL and UL OFDMA are enabled and TCP is used)",
                enableBsrp);
    cmd.AddValue(
        "muSchedAccessReqInterval",
        "Duration of the interval between two requests for channel access made by the MU scheduler",
        accessReqInterval);
    cmd.AddValue("mcs", "if set, limit testing to a specific MCS (0-11)", mcs);
    cmd.AddValue("payloadSize", "The application payload size in bytes", payloadSize);
    cmd.AddValue("tputInterval", "duration of intervals for throughput measurement", tputInterval);
    cmd.AddValue("minExpectedThroughput",
                "if set, simulation fails if the lowest throughput is below this value",
                minExpectedThroughput);
    cmd.AddValue("maxExpectedThroughput",
                "if set, simulation fails if the highest throughput is above this value",
                maxExpectedThroughput);
    cmd.AddValue("enableMultiApCoordination",
                "Enable WiFi-8 Multi AP Coordination",
                enableMultiApCoordination);
    cmd.AddValue("reliabilityMode",
                "Enable WiFi-8 reliability mode",
                reliabilityMode);
    cmd.Parse(argc, argv);
    
    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    if (dlAckSeqType == "ACK-SU-FORMAT")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                        EnumValue(WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
    }
    else if (dlAckSeqType == "MU-BAR")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                        EnumValue(WifiAcknowledgment::DL_MU_TF_MU_BAR));
    }
    else if (dlAckSeqType == "AGGR-MU-BAR")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                        EnumValue(WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    }
    else if (dlAckSeqType != "NO-OFDMA")
    {
        NS_ABORT_MSG("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or "
                    "AGGR-MU-BAR)");
    }

    double prevThroughput[12] = {0};
    bool has24GHz = false;
    for (double freq : frequency) 
    {
        if (freq == 2.4) {
            has24GHz = true;
            break;
        }
    }
    uint16_t maxChannelWidth = (!has24GHz) ? 160 : 40;
    const auto is80Plus80 = (use80Plus80 && (channelWidth == 160));
    const std::string widthStr = is80Plus80 ? "80+80" : std::to_string(channelWidth);
    const auto segmentWidthStr = is80Plus80 ? "80" : widthStr;

    if (!udp)
    {
        Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
    }

    // Install office paraneters
    std::vector<Ptr<Building>> buildingVector;
    Ptr<Building> office;
    office = CreateObject<Building>();
    office->SetBoundaries(Box(0.0,
                            officeSizeX,
                            0.0,
                            officeSizeY,
                            0.0,
                            officeHeight));
    office->SetNRoomsX(1);
    office->SetNRoomsY(1);
    office->SetNFloors(1);
    buildingVector.push_back(office);

    // print the list of buildings to file
    std::string outputDirBuilding = outputDir + "buildings.txt";
    PrintGnuplottableBuildingListToFile(outputDirBuilding);

    // Create Wifi nodes
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nStations);
    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);

    NetDeviceContainer apDevices;
    NetDeviceContainer staDevices;

    WifiMacHelper mac;
    WifiHelper wifi;

    wifi.SetStandard(WIFI_STANDARD_80211be);
    std::array<std::string, 3> channelStr;
    std::array<FrequencyRange, 3> freqRanges;
    uint8_t nLinks = 0;

    std::string ctrlRateStr;
    std::string dataModeStr;
    uint64_t nonHtRefRateMbps;
    if (mcs >= 0)
    {
        dataModeStr = "EhtMcs" + std::to_string(mcs);
        nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;  
    }

    for (auto freq : frequency)
    {
        if (nLinks > 0 && freq == 0)
        {
            break;
        }
        if (freq == 6)
        {
            channelStr[nLinks] = "{1, " + segmentWidthStr + ", ";
            channelStr[nLinks] += "BAND_6GHZ, 0}";
            freqRanges[nLinks] = WIFI_SPECTRUM_6_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(48));
            if (mcs >= 0)
            {
                wifi.SetRemoteStationManager(nLinks,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifi.SetRemoteStationManager(nLinks,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 5)
        {
            channelStr[nLinks] = "{36, " + segmentWidthStr + ", ";
            channelStr[nLinks] += "BAND_5GHZ, 0}";
            freqRanges[nLinks] = WIFI_SPECTRUM_5_GHZ;
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifi.SetRemoteStationManager(nLinks,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifi.SetRemoteStationManager(nLinks,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 2.4)
        {
            channelStr[nLinks] = "{1, " + segmentWidthStr + ", ";
            channelStr[nLinks] += "BAND_2_4GHZ, 0}";
            freqRanges[nLinks] = WIFI_SPECTRUM_2_4_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(40));
            ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifi.SetRemoteStationManager(nLinks,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifi.SetRemoteStationManager(nLinks,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else
        {
            NS_FATAL_ERROR("Wrong frequency value!");
        }

        if (is80Plus80)
        {
            channelStr[nLinks] += std::string(";") + channelStr[nLinks];
        }

        nLinks++;
    }

    // Check if multi-link is used!!
    if (nLinks > 1 && !emlsrLinks.empty())
    {
        wifi.ConfigEhtOptions("EmlsrActivated", BooleanValue(true));
    }

    // Setup the EMLSR manager!!!
    mac.SetEmlsrManager("ns3::DefaultEmlsrManager",
        "EmlsrLinkSet",
        StringValue(emlsrLinks),
        "EmlsrPaddingDelay",
        TimeValue(MicroSeconds(paddingDelayUsec)),
        "EmlsrTransitionDelay",
        TimeValue(MicroSeconds(transitionDelayUsec)),
        "SwitchAuxPhy",
        BooleanValue(switchAuxPhy),
        "AuxPhyTxCapable",
        BooleanValue(auxPhyTxCapable),
        "AuxPhyChannelWidth",
        UintegerValue(auxPhyChWidth));

    auto spectrumChannel5 = CreateObject<MultiModelSpectrumChannel>();
    auto spectrumChannel6 = CreateObject<MultiModelSpectrumChannel>();
    auto spectrumChannel2_4 = CreateObject<MultiModelSpectrumChannel>();
    auto lossModel = CreateObject<FriisPropagationLossModel>();
    spectrumChannel5->AddPropagationLossModel(lossModel);
    spectrumChannel6->AddPropagationLossModel(lossModel);
    spectrumChannel2_4->AddPropagationLossModel(lossModel);
    
    Ssid ssid = Ssid("ns3-80211be");

    SpectrumWifiPhyHelper phy(nLinks);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phy.Set("ChannelSwitchDelay", TimeValue(MicroSeconds(channelSwitchDelayUsec)));

    // Sets up multiple link in the physical layer
    for (uint8_t linkId = 0; linkId < nLinks; linkId++)
    {
        phy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId]));
        if (findSubstring(channelStr[linkId],"5GHZ"))
        {
            phy.AddChannel(spectrumChannel5, freqRanges[linkId]);
        }
        else if (findSubstring(channelStr[linkId],"6GHZ"))
        {
            phy.AddChannel(spectrumChannel6, freqRanges[linkId]);
        }
        else if (findSubstring(channelStr[linkId],"2_4GHZ"))
        {
            phy.AddChannel(spectrumChannel2_4, freqRanges[linkId]);
        }
    }

    phy.Set ("TxPowerStart", DoubleValue (powSta));
    phy.Set ("TxPowerEnd", DoubleValue (powSta));
    phy.Set ("TxPowerLevels", UintegerValue (1));
    phy.Set("CcaEdThreshold", DoubleValue(ccaEdTr));
    phy.Set("RxSensitivity", DoubleValue(-92.0));

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices.Add(wifi.Install(phy, mac, wifiStaNodes));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));

    if (dlAckSeqType != "NO-OFDMA")
    {
        mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                    "EnableUlOfdma",
                                    BooleanValue(enableUlOfdma),
                                    "EnableBsrp",
                                    BooleanValue(enableBsrp),
                                    "AccessReqInterval",
                                    TimeValue(accessReqInterval));
    }

    phy.Set ("TxPowerStart", DoubleValue (powAp));
    phy.Set ("TxPowerEnd", DoubleValue (powAp));
    phy.Set ("TxPowerLevels", UintegerValue (1));
    phy.Set("CcaEdThreshold", DoubleValue(ccaEdTr));
    phy.Set("RxSensitivity", DoubleValue(-92.0));

    apDevices.Add(wifi.Install(phy, mac, wifiApNodes));

    // Set guard interval and MPDU buffer size
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
        TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                UintegerValue(mpduBufferSize));

    //Install AP mobility
    MobilityHelper apMobility;
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(10.0, 10.0, 1.5));
    apPositionAlloc->Add(Vector(30.0, 10.0, 1.5));
    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobility.SetPositionAllocator(apPositionAlloc);
    apMobility.Install(wifiApNodes);

    //Install STA mobility
    MobilityHelper staMobility;
    staMobility.SetMobilityModel(
            "ns3::RandomWalk2dMobilityModel",
            "Mode",
            StringValue("Time"),
            "Time",
            StringValue("2s"),
            "Speed",
            StringValue("ns3::ConstantRandomVariable[Constant=6.0]"),
            "Bounds",
            RectangleValue(Rectangle(0.0, officeSizeX, 0.0, officeSizeY)));
    staMobility.Install(wifiStaNodes);
    
    /* Internet stack*/
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staNodeInterfaces;
    Ipv4InterfaceContainer apNodeInterface;

    staNodeInterfaces = address.Assign(staDevices);
    apNodeInterface = address.Assign(apDevices);

    /* Setting applications */
    ApplicationContainer serverApp;
    ApplicationContainer clientApp;

    uint64_t maxLoad;
    // Calculate maxLoad as
    if (mcs >= 0)
    {
        maxLoad = nLinks * EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1) / nStations;
        std::cout << "Data rate " << EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1) << std::endl;
        std::cout << "Total data rate " << maxLoad << std::endl;
    }
    else
    {
        maxLoad = nLinks * EhtPhy::GetDataRate(13, channelWidth, NanoSeconds(gi), 1) / nStations;
    }

    if (udp)
    {
        if(reliabilityMode)
        {
            clientApp.EnableReliabilityMode();
        }
        if(enablePoisson)
        {
            clientApp.EnablePoissonTraffic();
        }

        // UDP flow
        uint16_t port = 9;
        UdpServerHelper server(port);
        for (std::size_t i = 0; i < nStations; i++)
        {   
            serverApp.Add(server.Install(wifiStaNodes.Get(i)));
        }

        serverApp.Start(Seconds(0.0));
        serverApp.Stop(simulationTime + Seconds(1.0));
        const auto packetInterval = payloadSize * 8.0 / (maxLoad);
        if (!poissonLambda)
        {
            poissonLambda = 1/packetInterval;
        }
        std::cout << "The poissonLambda is " << poissonLambda << std::endl;

        for (std::size_t i = 0; i < nStations; i++)
        {
            UdpClientHelper client(staNodeInterfaces.GetAddress(i), port);
            client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
            if(enablePoisson)
            {
                client.SetAttribute("EnablePoisson", BooleanValue(enablePoisson));
                client.SetAttribute("PoissonLambda", DoubleValue(poissonLambda));
            }
            else
            {
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
            }
            client.SetAttribute("PacketSize", UintegerValue(payloadSize));
            
            for (std::size_t i = 0; i < wifiApNodes.GetN(); i++)
            {   
                clientApp.Add(client.Install(wifiApNodes.Get(i)));
            }
            
            clientApp.Start(Seconds(1.0));
            clientApp.Stop(simulationTime + Seconds(1.0));
        }
    }

    // cumulative number of bytes received by each server application
    std::vector<uint64_t> cumulRxBytes(nStations, 0);

    if (tputInterval.IsStrictlyPositive())
    {
        Simulator::Schedule(Seconds(1) + tputInterval,
                            &PrintIntermediateTput,
                            cumulRxBytes,
                            udp,
                            serverApp,
                            payloadSize,
                            tputInterval,
                            simulationTime + Seconds(1.0));
    }
    Simulator::Schedule (MicroSeconds (100), &TimePasses);

    // Enable tracing
    // Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpClient/Tx", MakeCallback(&AppTxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&DevTxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&DevTxDropTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&DevRxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&DevRxDropTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback(&DevAckedMpduTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/DroppedMpdu", MakeCallback(&DevDroppedMpduTrace));
    // Config::Connect("/NodeList/[i]/DeviceList/[i]/$ns3::WifiNetDevice/Mac/Txop/Queue/Dequeue", MakeCallback(&DevTxDequeueTrace));

    // enable the traces for the mobility model
    AsciiTraceHelper ascii;
    std::string outputDirMobilityTrace = outputDir + "mobility-trace-example.mob";
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream(outputDirMobilityTrace));

    
    Simulator::Stop(simulationTime + Seconds(1.0));
    Simulator::Run();

    cumulRxBytes = GetRxBytes(udp, serverApp, payloadSize);
    auto rxBytes = std::accumulate(cumulRxBytes.cbegin(), cumulRxBytes.cend(), 0.0);
    auto throughput = (rxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s

    // std::cout << "TxPackets : " << m_txPackets << std::endl;
    // std::cout << "TxPacketsDequeue : " << m_txDequeue << std::endl;
    // std::cout << "RxPackets : " << m_rxPackets << std::endl;

    std::cout << +mcs << "\t\t\t" << widthStr << " MHz\t\t"
                        << (widthStr.size() > 3 ? "" : "\t") << gi << " ns\t\t\t" << throughput
                        << " Mbit/s" << std::endl;

    Simulator::Destroy();

    return 0;
}
