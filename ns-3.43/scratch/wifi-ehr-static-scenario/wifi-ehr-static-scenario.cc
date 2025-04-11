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
#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiEhrStaticScenario");

Time m_sumLatency{"0s"};
Time m_avgLatency{"0s"};
uint64_t m_rxPackets{0};
uint64_t m_txPackets{0};
uint64_t m_txDequeue{0};

Ptr<ListPositionAllocator> 
ReadPositionAllocatorFromFiles(const std::string& layoutDir, uint16_t numBss, const std::vector<uint16_t>& numApsPerBss)
{
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    std::vector<std::vector<double>> staPositions;
    std::vector<std::vector<double>> apPositions;

    // Read STA positions
    std::ifstream staFile(layoutDir + "sta-layout-matrix.txt");
    if (!staFile.is_open()) 
    {
        NS_LOG_ERROR("Can't open STA position file!");
        return positionAlloc;
    }

    std::string line;
    while (std::getline(staFile, line)) 
    {
        if (line.find("STA") != std::string::npos) 
        {
            // Skip the STA line, read coordinates in next line
            if (std::getline(staFile, line)) 
            {
                std::istringstream iss(line);
                double x, y, z;
                if (iss >> x >> y >> z) 
                {
                staPositions.push_back({x, y, z});
                }
            }
        }
    }
    staFile.close();

    // Read AP positions (grouped)
    std::ifstream apFile(layoutDir + "ap-layout-matrix.txt");
    if (!apFile.is_open()) 
    {
        NS_LOG_ERROR("Can't open AP position file!");
        return positionAlloc;
    }

    while (std::getline(apFile, line)) 
    {
        if (line.find("Group") != std::string::npos) 
        {
            // For each AP in the group
            for (int i = 0; i < 3; i++) 
            {
                if (std::getline(apFile, line)) 
                {
                    std::istringstream iss(line);
                    double x, y, z;
                    if (iss >> x >> y >> z) 
                    {
                        apPositions.push_back({x, y, z});
                    }
                }
            }
        }
    }
    apFile.close();

    for (uint32_t i = 0; i < numBss; i++) 
    {
        positionAlloc->Add(Vector(staPositions[i][0], staPositions[i][1], staPositions[i][2]));
    }

    for (uint32_t i = 0; i < numBss; i++) 
    {
        for (uint32_t j = 0; j < numApsPerBss[i]; j++) 
        {
            positionAlloc->Add(Vector(apPositions[i*3+j][0], apPositions[i*3+j][1], apPositions[i*3+j][2]));
        }
    }

    for (uint32_t i = 0; i < numBss; i++) 
    {
        std::cout << "BSS " << i+1 << std::endl; 
        for (uint32_t j = 0; j < numApsPerBss[i]; j++)
        {
            std::cout << "AP " << j+1 << " x: " << apPositions[i*3+j][0] 
            << ", y: " << apPositions[i*3+j][1] 
            << ", z: " << apPositions[i*3+j][2] << std::endl;
        }
        std::cout << "STA " << i+1 << " x: " << staPositions[i][0] 
        << ", y: " << staPositions[i][1] 
        << ", z: " << staPositions[i][2] << std::endl;
    }

    return positionAlloc;
}

std::vector<std::vector<double>> 
AssignFrequenciesToAps(uint16_t numBss, 
                    std::vector<uint16_t>& numApsPerBss,
                    uint32_t seed, 
                    std::vector<double> freqBands)
{
    std::vector<std::vector<double>> freqAssignment(numBss);

    // Set the random seed
    RngSeedManager::SetSeed(seed);
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();

    std::default_random_engine rng(static_cast<unsigned int>(rand->GetValue(0, UINT32_MAX)));

    for (uint16_t i = 0; i < numBss; i++) 
    {
        std::vector<double> randFreq = freqBands;
        std::shuffle(randFreq.begin(), randFreq.end(), rng);

        for (auto ap = 0; ap < numApsPerBss[i]; ap++) 
        {
            freqAssignment[i].push_back(randFreq[ap]);
        }
    }

    for (uint16_t i = 0; i < numBss; i++)
    {
        std::cout << "BSS " << i+1 << " frequency assignments:" << std::endl;
        for (uint32_t j = 0; j < numApsPerBss[i]; j++)
        {
            std::cout << "AP " << j+1 << ": " << freqAssignment[i][j] << " GHz" << std::endl;
        }
    }

    return freqAssignment;
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

/// True for verbose output.
static bool g_verbose = true;

/**
 * MAC-level TX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
DevTxTrace(std::string context, Ptr<const Packet> p)
{
    uint64_t packetUid = p->GetUid();
    if (g_verbose)
    {
        std::cout << Simulator::Now().As(Time::US) << " MAC TX p: " << packetUid << std::endl;
    }
    m_txPackets++;
}

/**
 * MAC-level RX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
DevRxTrace(std::string context, Ptr<const Packet> p)
{
    uint64_t packetUid = p->GetUid();
    if (g_verbose)
    {
        std::cout << Simulator::Now().As(Time::US) << " MAC RX p: " << packetUid << std::endl;
    }
    m_rxPackets++;
}

void
DevTxDequeueTrace(std::string context, Ptr<const Packet> p)
{
    uint64_t packetUid = p->GetUid();
    if (g_verbose)
    {
        std::cout << Simulator::Now().As(Time::US) << " Deqeueued MAC TX p: " << packetUid << std::endl;
    }
    m_txDequeue++;
}

/**
 * MAC-level TX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
DevTxDropTrace(std::string context, Ptr<const Packet> p)
{
    uint64_t packetUid = p->GetUid();
    if (g_verbose)
    {
        std::cout << Simulator::Now().As(Time::US) << " MAC TX drop p: " << packetUid << std::endl;
    }
}

/**
 * MAC-level RX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
DevRxDropTrace(std::string context, Ptr<const Packet> p)
{
    uint64_t packetUid = p->GetUid();
    if (g_verbose)
    {
        std::cout << Simulator::Now().As(Time::US) << " MAC RX drop p: " << packetUid << std::endl;
    }
}

// /**
//  * MAC-level TX trace.
//  *
//  * \param context The context.
//  * \param p The packet.
//  */
// void
// DevAckedMpduTrace(std::string context, Ptr<const WifiMpdu> mpdu)
// {
//     uint64_t mpduUid = mpdu->GetPacket()->GetUid();
//     if (g_verbose)
//     {
//         std::cout << Simulator::Now().As(Time::US) << " MAC ACK Packet : " << mpduUid << std::endl;
//     }
// }

// /**
//  * MAC-level RX trace.
//  *
//  * \param context The context.
//  * \param p The packet.
//  */
// void
// DevDroppedMpduTrace(std::string context, Ptr<const WifiMpdu> mpdu)
// {
//     uint64_t mpduUid = mpdu->GetPacket()->GetUid();
//     if (g_verbose)
//     {
//         std::cout << Simulator::Now().As(Time::US) << " MAC Dropped Packet : " << mpduUid << std::endl;
//     }
// }

/**
 * APP-level TX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
AppTxTrace(std::string context, Ptr<const Packet> p)
{
    uint64_t packetUid = p->GetUid();
    if (g_verbose)
    {
        std::cout << Simulator::Now().As(Time::US) << " APP TX p: " << packetUid << std::endl;
    }
}

/**
 * APP-level RX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
AppRxTrace(std::string context, Ptr<const Packet> p)
{
    if (g_verbose)
    {
        std::cout << " APP RX p: " << *p << std::endl;
    }
}

main(int argc, char* argv[])
{
    // General parameters
    std::string layoutDir = "./scratch/wifi-ehr-static-scenario/";
    std::string outputDir = "./scratch/wifi-ehr-static-scenario/";
    Time simulationTime{"1s"};
    uint16_t numBss = 4;
    std::vector<uint16_t> numApsPerBss = {3, 3, 3, 3}; 
    uint32_t seed = 5;

    // Wifi Simulation Parameters
    bool udp{true};
    bool downlink{true};
    bool useRts{false};
    bool frameAggregation{true};
    bool use80Plus80{false};
    uint16_t mpduBufferSize{512};
    std::vector<double> freqBands = {2.4,5,6};
    uint16_t paddingDelayUsec{32};
    uint16_t transitionDelayUsec{128};
    uint16_t channelSwitchDelayUsec{100};
    bool switchAuxPhy{true};
    uint16_t auxPhyChWidth{20};
    bool auxPhyTxCapable{true};
    dBm_u powSta{10.0};
    dBm_u powAp{21.0};
    dBm_u ccaEdTr{-62};
    dBm_u minimumRssi{-82};
    int channelWidth = 20;
    int gi = 3200;
    std::string dlAckSeqType{"ACK-SU-FORMAT"};//(NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    int mcs{13}; // -1 indicates an unset value
    uint32_t payloadSize =
        700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    double poissonLambda = 0;
    double minExpectedThroughput{0};
    double maxExpectedThroughput{0};
    Time accessReqInterval{0};
    std::vector<bool> enableMultiApCoordination(numBss); 
    for (uint16_t i = 0; i < numBss; i++)
    {
        enableMultiApCoordination[i] = true;
    }
    std::vector<bool> reliabilityMode(numBss);
    for (uint16_t i = 0; i < numBss; i++)
    {
        reliabilityMode[i] = false;
    }
    bool enablePoisson = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simulationTime", "Simulation time", simulationTime);
    // cmd.AddValue("emlsrLinks",
    //              "The comma separated list of IDs of EMLSR links (for MLDs only)",
    //              emlsrLinks);
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
    cmd.AddValue("numBss", "Number of BSS", numBss);
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
    // cmd.AddValue("enableMultiApCoordination",
    //              "Enable WiFi-8 Multi AP Coordination",
    //              enableMultiApCoordination);
    // cmd.AddValue("reliabilityMode",
    //              "Enable WiFi-8 reliability mode",
    //              reliabilityMode);
    cmd.Parse(argc, argv);
    
    LogComponentEnable("WifiEhrStaticScenario", LOG_LEVEL_DEBUG);

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
    
    // Create Wifi nodes
    NodeContainer wifiStaNodes; 
    wifiStaNodes.Create(numBss);
    NodeContainer wifiApNodes;
    uint16_t numAps = std::accumulate(numApsPerBss.begin(), numApsPerBss.end(), 0);
    wifiApNodes.Create(numAps);

    // Read APs and STAs from file
    Ptr<ListPositionAllocator> positionAlloc = ReadPositionAllocatorFromFiles(layoutDir, numBss, numApsPerBss);

    // Assign random frequencies to APs
    auto freqAssignment = AssignFrequenciesToAps(numBss, numApsPerBss, seed, freqBands);

    bool has24GHz = false;
    for (uint16_t i = 0; i < wifiStaNodes.GetN(); i++)
    {
        for (auto freq : freqAssignment[i])
        {
            if (freq == 2.4) {
                has24GHz = true;
                break;
            }
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

    NetDeviceContainer apDevices;
    NetDeviceContainer staDevices;

    WifiMacHelper mac;
    std::vector<WifiHelper> wifiStas(wifiStaNodes.GetN());
    std::vector<WifiHelper> wifiAps(wifiApNodes.GetN());
    std::vector<std::array<std::string, 3>> channelStrStas(wifiStaNodes.GetN());
    std::vector<std::array<std::string, 3>> channelStrAps(wifiApNodes.GetN());
    std::vector<std::array<FrequencyRange, 3>> freqRangesSta(wifiStaNodes.GetN());
    std::vector<std::array<FrequencyRange, 3>> freqRangesAps(wifiApNodes.GetN());
    std::vector<uint8_t> nLinksSta(wifiStaNodes.GetN());
    std::vector<uint8_t> nLinksAps(wifiApNodes.GetN());

    for (uint8_t i = 0; i < wifiStas.size(); i++)
    {
        wifiStas[i].SetStandard(WIFI_STANDARD_80211be);
    }
    for (uint8_t i = 0; i < wifiAps.size(); i++)
    {
        wifiAps[i].SetStandard(WIFI_STANDARD_80211be);
    }

    std::string ctrlRateStr;
    std::string dataModeStr;
    uint64_t nonHtRefRateMbps;
    if (mcs >= 0)
    {
        dataModeStr = "EhtMcs" + std::to_string(mcs);
        nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;  
    }

    for (uint16_t i = 0; i < wifiStaNodes.GetN(); i++)
    {
        for (auto freq : freqAssignment[i])
        {
            if (nLinksSta[i] > 0 && freq == 0)
            {
                break;
            }
            if (freq == 6)
            {
                channelStrStas[i][nLinksSta[i]] = "{1, " + segmentWidthStr + ", ";
                channelStrStas[i][nLinksSta[i]] += "BAND_6GHZ, 0}";
                freqRangesSta[i][nLinksSta[i]] = WIFI_SPECTRUM_6_GHZ;
                Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                    DoubleValue(48));
                if (mcs >= 0)
                {
                    wifiStas[i].SetRemoteStationManager(nLinksSta[i],
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue(dataModeStr));
                }
                else 
                {
                    wifiStas[i].SetRemoteStationManager(nLinksSta[i],
                                                "ns3::MinstrelHtWifiManager");
                }
            }
            else if (freq == 5)
            {
                channelStrStas[i][nLinksSta[i]] = "{36, " + segmentWidthStr + ", ";
                channelStrStas[i][nLinksSta[i]] += "BAND_5GHZ, 0}";
                freqRangesSta[i][nLinksSta[i]] = WIFI_SPECTRUM_5_GHZ;
                if (mcs >= 0)
                {
                    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                    wifiStas[i].SetRemoteStationManager(nLinksSta[i],
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue(dataModeStr));
                }
                else 
                {
                    wifiStas[i].SetRemoteStationManager(nLinksSta[i],
                                                "ns3::MinstrelHtWifiManager");
                }
            }
            else if (freq == 2.4)
            {
                channelStrStas[i][nLinksSta[i]] = "{1, " + segmentWidthStr + ", ";
                channelStrStas[i][nLinksSta[i]] += "BAND_2_4GHZ, 0}";
                freqRangesSta[i][nLinksSta[i]] = WIFI_SPECTRUM_2_4_GHZ;
                Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                    DoubleValue(40));
                ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                if (mcs >= 0)
                {
                    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                    wifiStas[i].SetRemoteStationManager(nLinksSta[i],
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue(dataModeStr));
                }
                else 
                {
                    wifiStas[i].SetRemoteStationManager(nLinksSta[i],
                                                "ns3::MinstrelHtWifiManager");
                }
            }
            else
            {
                NS_FATAL_ERROR("Wrong frequency value!");
            }

            if (is80Plus80)
            {
                channelStrStas[i][nLinksSta[i]] += std::string(";") + channelStrStas[i][nLinksSta[i]];
            }

            nLinksSta[i]++;
        }
    }

    uint16_t it = 0;
    for (uint16_t i = 0; i < numBss; i++)
    {
        for (uint16_t j = 0; j < freqAssignment[i].size(); j++)
        {
            if (nLinksAps[it] > 0 && freqAssignment[i][j] == 0)
            {
                break;
            }
            if (freqAssignment[i][j] == 6)
            {
                channelStrAps[it][nLinksAps[it]] = "{1, " + segmentWidthStr + ", ";
                channelStrAps[it][nLinksAps[it]] += "BAND_6GHZ, 0}";
                freqRangesAps[it][nLinksAps[it]] = WIFI_SPECTRUM_6_GHZ;
                Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                    DoubleValue(48));
                if (mcs >= 0)
                {
                    wifiAps[it].SetRemoteStationManager(nLinksAps[it],
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue(dataModeStr));
                }
                else 
                {
                    wifiAps[it].SetRemoteStationManager(nLinksAps[it],
                                                "ns3::MinstrelHtWifiManager");
                }
            }
            else if (freqAssignment[i][j] == 5)
            {
                channelStrAps[it][nLinksAps[it]] = "{36, " + segmentWidthStr + ", ";
                channelStrAps[it][nLinksAps[it]] += "BAND_5GHZ, 0}";
                freqRangesAps[it][nLinksAps[it]] = WIFI_SPECTRUM_5_GHZ;
                if (mcs >= 0)
                {
                    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                    wifiAps[it].SetRemoteStationManager(nLinksAps[it],
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue(dataModeStr));
                }
                else 
                {
                    wifiAps[it].SetRemoteStationManager(nLinksAps[it],
                                                "ns3::MinstrelHtWifiManager");
                }
            }
            else if (freqAssignment[i][j] == 2.4)
            {
                channelStrAps[it][nLinksAps[it]] = "{1, " + segmentWidthStr + ", ";
                channelStrAps[it][nLinksAps[it]] += "BAND_2_4GHZ, 0}";
                freqRangesAps[it][nLinksAps[it]] = WIFI_SPECTRUM_2_4_GHZ;
                Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                    DoubleValue(40));
                ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                if (mcs >= 0)
                {
                    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                    wifiAps[it].SetRemoteStationManager(nLinksAps[it],
                                                "ns3::ConstantRateWifiManager",
                                                "DataMode",
                                                StringValue(dataModeStr),
                                                "ControlMode",
                                                StringValue(dataModeStr));
                }
                else 
                {
                    wifiAps[it].SetRemoteStationManager(nLinksAps[it],
                                                "ns3::MinstrelHtWifiManager");
                }
            }
            else
            {
                NS_FATAL_ERROR("Wrong frequency value!");
            }

            if (is80Plus80)
            {
                channelStrAps[it][nLinksAps[it]] += std::string(";") + channelStrAps[it][nLinksAps[it]];
            }

            nLinksAps[it]++;
            it++;
        }
    }

    it = 0;
    for (uint16_t i = 0; i < numBss; i++)
    {
        if (nLinksSta[i] > 1 && enableMultiApCoordination[i])
        {
            wifiStas[i].ConfigEhtOptions("EnableMultiApCoordination", BooleanValue(true));
            for (uint16_t j = 0; j < freqAssignment[i].size(); j++)
            {
                wifiAps[it].ConfigEhtOptions("EnableMultiApCoordination", BooleanValue(true));
                it++;
            }
        }
    }

    for (uint16_t i = 0; i < numBss; i++)
    {
        if(reliabilityMode[i])
        {
            wifiStas[i].ConfigEhtOptions("ReliabilityMode", BooleanValue(true));
        }
    }

    auto spectrumChannel5 = CreateObject<MultiModelSpectrumChannel>();
    auto spectrumChannel6 = CreateObject<MultiModelSpectrumChannel>();
    auto spectrumChannel2_4 = CreateObject<MultiModelSpectrumChannel>();
    auto lossModel = CreateObject<FriisPropagationLossModel>();
    spectrumChannel5->AddPropagationLossModel(lossModel);
    spectrumChannel6->AddPropagationLossModel(lossModel);
    spectrumChannel2_4->AddPropagationLossModel(lossModel);

    std::vector<SpectrumWifiPhyHelper> phyStas;
    for (uint16_t i = 0; i < numBss; i++)
    {
        phyStas.emplace_back(nLinksSta[i]);
    }
    std::vector<SpectrumWifiPhyHelper> phyAps(wifiApNodes.GetN());

    it = 0;
    for (uint16_t i = 0; i < numBss; i++)
    {
        NS_LOG_INFO("Configuring BSS " << i+1 << " with " << numApsPerBss[i] << " APs");
        
        Ssid ssid = Ssid("ns3-80211bn-" + std::to_string(i+1));

        phyStas[i].SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

        // Sets up multiple link in the physical layer
        for (uint8_t linkId = 0; linkId < nLinksSta[i]; linkId++)
        {
            phyStas[i].Set(linkId, "ChannelSettings", StringValue(channelStrStas[i][linkId]));
            if (findSubstring(channelStrStas[i][linkId],"5GHZ"))
            {
                phyStas[i].AddChannel(spectrumChannel5, freqRangesSta[i][linkId]);
            }
            else if (findSubstring(channelStrStas[i][linkId],"6GHZ"))
            {
                phyStas[i].AddChannel(spectrumChannel6, freqRangesSta[i][linkId]);
            }
            else if (findSubstring(channelStrStas[i][linkId],"2_4GHZ"))
            {
                phyStas[i].AddChannel(spectrumChannel2_4, freqRangesSta[i][linkId]);
            }
        }

        phyStas[i].Set ("TxPowerStart", DoubleValue (powSta));
        phyStas[i].Set ("TxPowerEnd", DoubleValue (powSta));
        phyStas[i].Set ("TxPowerLevels", UintegerValue (1));
        phyStas[i].Set("CcaEdThreshold", DoubleValue(ccaEdTr));
        phyStas[i].Set("RxSensitivity", DoubleValue(-92.0));

        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));

        staDevices.Add(wifiStas[i].Install(phyStas[i], mac, wifiStaNodes.Get(i)));

        for (uint8_t linkId = 0; linkId < nLinksSta[i]; linkId++)
        {
            phyAps[it].SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
            for (uint8_t apLinkId = 0; apLinkId < nLinksAps[it]; apLinkId++)
            {
                phyAps[it].Set(apLinkId, "ChannelSettings", StringValue(channelStrAps[it][apLinkId]));
                if (findSubstring(channelStrAps[it][apLinkId],"5GHZ"))
                {
                    phyAps[it].AddChannel(spectrumChannel5, freqRangesAps[it][apLinkId]);
                }
                else if (findSubstring(channelStrAps[it][apLinkId],"6GHZ"))
                {
                    phyAps[it].AddChannel(spectrumChannel6, freqRangesAps[it][apLinkId]);
                }
                else if (findSubstring(channelStrAps[it][apLinkId],"2_4GHZ"))
                {
                    phyAps[it].AddChannel(spectrumChannel2_4, freqRangesAps[it][apLinkId]);
                }
            }

            phyAps[i].Set ("TxPowerStart", DoubleValue (powAp));
            phyAps[i].Set ("TxPowerEnd", DoubleValue (powAp));
            phyAps[i].Set ("TxPowerLevels", UintegerValue (1));
            phyAps[i].Set("CcaEdThreshold", DoubleValue(ccaEdTr));
            phyAps[i].Set("RxSensitivity", DoubleValue(-92.0));

            if(!frameAggregation)
            {
                mac.SetType("ns3::ApWifiMac",
                    "Ssid", SsidValue(ssid),
                    "BE_MaxAmpduSize",UintegerValue(0), // Enable A-MPDU with the highest maximum size allowed by the standard);
                    "BE_MaxAmsduSize",UintegerValue(0)); // Enable A-MSDU with the highest maximum size (in Bytes) allowed
            }
            else 
            {
                mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
            }
            
            
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

            apDevices.Add(wifiAps[it].Install(phyAps[it], mac, wifiApNodes.Get(it)));
            it++;
        }
    }

    // Install mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiStaNodes);
    mobility.Install(wifiApNodes);
    
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
    std::vector<ApplicationContainer> serverApps(numBss);
    std::vector<ApplicationContainer> clientApps(numBss);
    
    std::vector<uint64_t> maxLoad(numBss);
    it = 0;
    // Calculate maxLoad as
    for (uint16_t i = 0; i < numBss; i++)
    {
        if (mcs >= 0)
        {
            maxLoad[i] = nLinksSta[i] * EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1);
            std::cout << "Data rate BSS " << i+1 << " : " << EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1)/1e6 << " Mbit/s" << std::endl;
        }
        else
        {
            maxLoad[i] = nLinksSta[i] * EhtPhy::GetDataRate(13, channelWidth, NanoSeconds(gi), 1);
        }

        if (udp)
        {
            if(reliabilityMode[i])
            {
                clientApps[i].EnableReliabilityMode();
            }
            if(enablePoisson)
            {
                clientApps[i].EnablePoissonTraffic();
            }
        }

        // UDP flow
        uint16_t port = 9;
        UdpServerHelper server(port);
        serverApps[i].Add(server.Install(wifiStaNodes.Get(i)));

        serverApps[i].Start(Seconds(0.0));
        serverApps[i].Stop(simulationTime + Seconds(1.0));
        const auto packetInterval = payloadSize * 8.0 / (maxLoad[i]);

        if (!poissonLambda)
        {
            poissonLambda = 1/packetInterval;
        }
        std::cout << "poissonLambda for BSS " << i+1 << " : " << poissonLambda << std::endl;

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
        
        for (uint8_t linkId = 0; linkId < nLinksSta[i]; linkId++)
        {   
            clientApps[i].Add(client.Install(wifiApNodes.Get(it)));
            it++;
        }

        clientApps[i].Start(Seconds(1.0));
        clientApps[i].Stop(simulationTime + Seconds(1.0));
    }
    
    std::vector<std::vector<uint64_t>> allRxBytes;
    for (uint16_t i = 0; i < numBss; i++)
    {
        allRxBytes.emplace_back(1,0);
    }

    // Set guard interval and MPDU buffer size
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
        TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                UintegerValue(mpduBufferSize));

    // Enable tracing
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpClient/Tx", MakeCallback(&AppTxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&DevTxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&DevTxDropTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&DevRxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&DevRxDropTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback(&DevAckedMpduTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/DroppedMpdu", MakeCallback(&DevDroppedMpduTrace));
    // Config::Connect("/NodeList/[i]/DeviceList/[i]/$ns3::WifiNetDevice/Mac/Txop/Queue/Dequeue", MakeCallback(&DevTxDequeueTrace));

    Simulator::Schedule (MicroSeconds (100), &TimePasses);

    if (tputInterval.IsStrictlyPositive())
    {
        for (uint16_t i = 0; i < numBss; i++)
        {
            Simulator::Schedule(Seconds(1) + tputInterval,
                            &PrintIntermediateTput,
                            allRxBytes[i],
                            udp,
                            serverApps[i],
                            payloadSize,
                            tputInterval,
                            simulationTime + Seconds(1.0));
        }
    }

    Simulator::Stop(simulationTime + Seconds(1.0));
    Simulator::Run();

    for (uint16_t i = 0; i < numBss; i++)
    {
        allRxBytes[i] = GetRxBytes(udp, serverApps[i], payloadSize);
        auto rxBytes = std::accumulate(allRxBytes[i].cbegin(), allRxBytes[i].cend(), 0.0);
        auto throughput = (rxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s   

        std::cout <<  i+1 <<  "\t\t\t" << +mcs << "\t\t\t" << widthStr << " MHz\t\t"
                          << (widthStr.size() > 3 ? "" : "\t") << gi << " ns\t\t\t" << throughput
                          << " Mbit/s" << std::endl;
    }

    Simulator::Destroy();

    return 0;
}