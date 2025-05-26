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
#include "ns3/wifi-mac.h"
#include "ns3/frame-exchange-manager.h"

#include <array>
#include <functional>
#include <numeric>
#include <random>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiEhrRoamingScenario");

bool 
fileExists(const std::string& filename) 
{
    std::ifstream file(filename);
    return file.good();
}

bool findSubstring(const std::string& text, const std::string& substring) 
{
    return text.find(substring) != std::string::npos;
}

static bool g_verbose = false;
 
bool m_reliabilityMode = false;
std::map<Mac48Address,Mac48Address>  m_apStaAddressMap;
NetDeviceContainer m_staDevices;

typedef struct Analysis
{
    std::map<uint64_t,Time> phyTxMap;
    std::map<std::pair<uint64_t, Mac48Address>, Time> phyTxMapReliability;
    std::map<uint64_t,Time> phyRxMap;
    std::map<uint64_t,Time> macTxMap;
    std::map<std::pair<uint64_t, Mac48Address>, Time> macTxMapReliability;
    std::map<uint64_t,Time> macRxMap;
    std::map<uint64_t,uint64_t> retransmitTxMap;
    std::map<std::pair<uint64_t, Mac48Address>, uint64_t> retransmitTxMapReliability;
    std::map<uint64_t,uint64_t> dropTxMap;
    std::map<std::pair<uint64_t, Mac48Address>, uint64_t> dropTxMapReliability;
    std::map<uint64_t,Time> chReqTxMap;
    std::map<std::pair<uint64_t, Mac48Address>, Time> chReqTxMapReliability;
    std::map<uint64_t,Time> chGrantedTxMap;
    std::map<std::pair<uint64_t, Mac48Address>, Time> chGrantedTxMapReliability;
    uint64_t rxPackets{0};
    uint64_t txPackets{0};
    uint64_t phyRxPackets{0};
    uint64_t phyTxPackets{0};
    uint64_t dropPackets{0};
    uint64_t retransmitPackets{0};
    uint64_t chAccessCount{0};
    Time sumDelay{"0s"};
    Time avgDelay{"0s"};
    Time sumChAccessDelay{"0s"};
    Time avgChAccessDelay{"0s"};
} Analysis;
 
std::map<Mac48Address,Analysis> analysisMap;

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

        // Print walls of the building
        uint32_t wallIndex = 0;
        for (auto wallIt = (*it)->GetIntWalls().begin(); wallIt != (*it)->GetIntWalls().end(); ++wallIt)
        {
            ++wallIndex;
            Box wallBox = (*wallIt)->GetBoundaries();
            outFile << "set object " << index << "_" << wallIndex << " rect from " << wallBox.xMin << "," << wallBox.yMin
                    << " to " << wallBox.xMax << "," << wallBox.yMax << std::endl;
        }
    }
}

void 
MonitorPathLoss(uint32_t apIndex, Ptr<IndoorBuildingsPropagationLossModel> lossModel, NodeContainer wifiAps, NodeContainer wifiStas) 
{
    for (uint32_t i = 0; i < wifiAps.GetN(); i++)
    {
        double lossAp = lossModel->GetLoss(wifiAps.Get(i)->GetObject<MobilityModel>(), wifiStas.Get(0)->GetObject<MobilityModel>());
        NS_LOG_INFO("Loss AP " << apIndex << " using frequency " << lossModel->GetFrequency() << " : " << lossAp << " dB");
    }

    // Schedule the next callback
    Simulator::Schedule(MilliSeconds(100), &MonitorPathLoss, apIndex, lossModel, wifiAps, wifiStas);
}

std::vector<Vector> ReadWaypointsFromFile(const std::string& filename)
{
    std::vector<Vector> waypoints;
    std::ifstream infile(filename.c_str());
    
    if (!infile.is_open())
    {
        NS_LOG_ERROR("Cannot open waypoints file: " << filename);
        return waypoints;
    }

    double x, y, z;
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> x >> y >> z)
        {
            waypoints.emplace_back(x, y, z);
        }
    }
    return waypoints;
}

void AddWallsFromFile(Ptr<Factory> factory, const std::string& filename)
{
    std::ifstream infile(filename.c_str());
    if (!infile.is_open())
    {
        NS_LOG_ERROR("Cannot open walls assignment file: " << filename);
        return;
    }

    uint32_t roomX, roomY;
    double xMin, xMax, yMin, yMax, zMin, zMax;
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> roomX >> roomY >> xMin >> xMax >> yMin >> yMax >> zMin >> zMax)
        {
            Ptr<Wall> wall = CreateObject<Wall>();
            wall->SetBoundaries(Box(xMin, xMax, yMin, yMax, zMin, zMax));
            factory->AddWallToRoom(roomX, roomY, wall);
        }
    }
}

Ptr<ListPositionAllocator> ReadApPositionsFromFile(const std::string& filename)
{
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    std::ifstream infile(filename.c_str());
    if (!infile.is_open())
    {
        NS_LOG_ERROR("Cannot open AP positions file: " << filename);
        return allocator;
    }

    double x, y, z;
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> x >> y >> z)
        {
            allocator->Add(Vector(x, y, z));
        }
    }
    return allocator;
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
            double freq;
            if (ap < randFreq.size())
            {
                freq = randFreq[ap];
            }
            else
            {
                Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
                freq = randFreq[rand->GetInteger(0, randFreq.size() - 1)];
            }
            freqAssignment[i].push_back(freq);
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

void
AssocTrace(std::string context, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA is associated" << std::endl;
}

void
LinkAssocTrace(std::string context, uint8_t linkId, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA link " << (int)linkId << " is associated with " << apAdress << std::endl;
}

void
DeAssocTrace(std::string context, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA is deassociated" << std::endl;
}

void
LinkDeAssocTrace(std::string context, uint8_t linkId, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA link " << (int)linkId << " is deassociated with " << apAdress << std::endl;
}

int main(int argc, char *argv[])
{
    // Simulation parameters
    uint32_t numNodes = 1; // Number of nodes
    uint32_t seed = 75;    // Random seed
    Time simulationTime{"20s"};
    uint16_t numBss = 1;
    std::vector<uint16_t> numApsPerBss = {4};
    std::string currentDir = "./scratch/wifi-ehr-roaming-scenario/";
    std::string roomLayoutDir = currentDir + "walls-assignment.txt";
    std::string apLayoutDir = currentDir + "ap-positions.txt";
    std::string waypointDir = currentDir + "random-waypoints.txt";

    // create a grid of buildings
    double factorySizeX = 20; // m
    double factorySizeY = 20;  // m
    double factoryHeight = 3; // m

    // // Wi-Fi parameters
    // double frequency = 5.945e9; //Hz option : 5.945e9 5.975e9

    // Wifi Simulation Parameters
    bool udp{true};
    bool downlink{true};
    bool useRts{false};
    bool frameAggregation{false};
    bool use80Plus80{false};
    uint16_t mpduBufferSize{512};
    std::vector<double> freqBands = {2.4,5,6};//{2.4,5,6};
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
    int mcs{0}; // -1 indicates an unset value
    uint32_t payloadSize =
        1474; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    double poissonLambda = 0;
    double minExpectedThroughput{0};
    double maxExpectedThroughput{0};
    Time accessReqInterval{0};
    bool enableMultiApCoordinationMaster = true;
    bool reliabilityModeMaster = false;
    bool enablePoisson = true;
    bool printOutput = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simulationTime", "Simulation time", simulationTime);
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
    cmd.AddValue("enableMultiApCoordination",
                 "Enable WiFi-8 Multi AP Coordination",
                 enableMultiApCoordinationMaster);
    cmd.AddValue("reliabilityMode",
                 "Enable WiFi-8 reliability mode",
                 reliabilityModeMaster);
    cmd.AddValue("frameAggregation",
                "Enable frame aggregation",
                frameAggregation);
    cmd.AddValue("printOutput",
                "Print results to file",
                printOutput);
    cmd.AddValue("poissonLambda",
                "Arrival rate of the poisson traffic",
                poissonLambda);
    cmd.AddValue("seed",
                "rng seed number",
                seed);
    cmd.Parse(argc, argv);

    LogComponentEnable("WifiEhrRoamingScenario", LOG_LEVEL_FUNCTION);

    RngSeedManager::SetSeed(seed);

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
    
    std::vector<bool> enableMultiApCoordination(numBss); 
    for (uint16_t i = 0; i < numBss; i++)
    {
        enableMultiApCoordination[i] = enableMultiApCoordinationMaster;
    }

    std::vector<bool> reliabilityMode(numBss);
    for (uint16_t i = 0; i < numBss; i++)
    {
        reliabilityMode[i] = reliabilityModeMaster;
    }
    m_reliabilityMode = reliabilityModeMaster;

    Ptr<Factory> factory = CreateObject<Factory>();
    factory->SetBuildingType(Building::Office);
    factory->SetBoundaries(Box(0, factorySizeX, 0, factorySizeY, 0, factoryHeight));
    factory->SetIntWallsType(Building::ConcreteWithoutWindows);
    factory->SetNFloors(1);
    factory->SetNRoomsX(4);
    factory->SetNRoomsY(4);

    AddWallsFromFile(factory, roomLayoutDir);

    // print the list of the office and walls to file
    PrintGnuplottableBuildingListToFile("indoor-sequential-layout.txt");

    // create one STA node
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(numNodes);
    NodeContainer wifiApNodes;
    uint16_t numAps = std::accumulate(numApsPerBss.begin(), numApsPerBss.end(), 0);
    wifiApNodes.Create(numAps);

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
            if ((nLinksSta[i] > 0 && freq == 0) || (nLinksSta[i] >=  freqBands.size()))
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
    auto lossModel5 = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel5->SetBuildingType(Building::Office);
    auto lossModel6 = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel6->SetBuildingType(Building::Office);
    auto lossModel2_4 = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel2_4->SetBuildingType(Building::Office);
    spectrumChannel5->AddPropagationLossModel(lossModel5);
    spectrumChannel6->AddPropagationLossModel(lossModel6);
    spectrumChannel2_4->AddPropagationLossModel(lossModel2_4);

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

        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                    "EnableMultiApCoordination",BooleanValue(enableMultiApCoordination[i]));

        staDevices.Add(wifiStas[i].Install(phyStas[i], mac, wifiStaNodes.Get(i)));

        for (uint8_t linkId = 0; linkId < freqAssignment[i].size(); linkId++)
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

    Ptr<WifiNetDevice> staNetDev = staDevices.Get(0)->GetObject<WifiNetDevice>();
    for (uint8_t linkId = 0; linkId < nLinksSta[0]; linkId++)
    {
        Ptr<WifiPhy> phy = staNetDev->GetPhy(linkId);
        double operatingFrequency = phy->GetFrequency()*1e6;
        if (phy)
        {
            std::cout << "STA Link " << unsigned(linkId) << " frequency: " << operatingFrequency << " Hz" << std::endl;
        }

        if (findSubstring(channelStrStas[0][linkId],"5GHZ"))
        {
            lossModel5->SetFrequency(operatingFrequency);
        }
        else if (findSubstring(channelStrStas[0][linkId],"6GHZ"))
        {
            lossModel6->SetFrequency(operatingFrequency);
        }
        else if (findSubstring(channelStrStas[0][linkId],"2_4GHZ"))
        {
            lossModel2_4->SetFrequency(operatingFrequency);
        }
    }

    // Install mobility
    MobilityHelper mobility;
    MobilityHelper mobilitySta;
    BuildingsHelper building;

    mobilitySta.SetMobilityModel(
        "ns3::SequentialWalk2dIndoorMobilityModel",
        "TimeInterval", StringValue("1s"),
        "WaypointIndex", UintegerValue(0),
        "Bounds", RectangleValue(Rectangle(0, factorySizeX, 0.0, factorySizeY)),
        "Factory", PointerValue(factory));
    Ptr<IndoorPositionAllocator> position = CreateObject<IndoorPositionAllocator>();
    Ptr<UniformRandomVariable> xPos = CreateObject<UniformRandomVariable>();
    xPos->SetAttribute("Min", DoubleValue(0.25));
    xPos->SetAttribute("Max", DoubleValue(0.25));
    Ptr<UniformRandomVariable> yPos = CreateObject<UniformRandomVariable>();
    yPos->SetAttribute("Min", DoubleValue(0.25));
    yPos->SetAttribute("Max", DoubleValue(0.25));
    Ptr<UniformRandomVariable> zPos = CreateObject<UniformRandomVariable>();
    zPos->SetAttribute("Min", DoubleValue(1.5));
    zPos->SetAttribute("Max", DoubleValue(1.5));
    position->SetAttribute("X", PointerValue(xPos));
    position->SetAttribute("Y", PointerValue(yPos));
    position->SetAttribute("Z", PointerValue(zPos));
    mobilitySta.SetPositionAllocator(position);
    mobilitySta.Install(wifiStaNodes);
    building.Install(wifiStaNodes);

    Ptr<ListPositionAllocator> apPosition = ReadApPositionsFromFile(apLayoutDir);
    if (apPosition->GetSize() == 0)
    {
        NS_LOG_ERROR("No AP positions found in file: " << apLayoutDir);
        return 1;
    }
    mobility.SetPositionAllocator(apPosition);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes);
    building.Install(wifiApNodes);

    // Set waypoints programmatically
    auto mob = wifiStaNodes.Get(0)->GetObject<SequentialWalk2dIndoorMobilityModel>();
    std::vector<Vector> waypoints;
    waypoints = ReadWaypointsFromFile(waypointDir);
    if (waypoints.empty())
    {
        NS_LOG_ERROR("No waypoints found in file: " << waypointDir);
        return 1;
    }
    mob->SetWaypoints(waypoints);
    mob->SetCurrentWaypointIndex(0);

    // Internet stack
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staNodeInterfaces;
    Ipv4InterfaceContainer apNodeInterface;

    staNodeInterfaces = address.Assign(staDevices);
    apNodeInterface = address.Assign(apDevices);

    // enable the traces for the mobility model
    AsciiTraceHelper ascii;
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream("indoor-sequential-mobility-trace-example.mob"));

    // Simulation clock
    Simulator::Schedule (MicroSeconds (100), &TimePasses);

    // Schedule the pathloss calculation callback
    for (uint i = 0; i < wifiApNodes.GetN(); i++)
    {
        if (findSubstring(channelStrAps[i][0],"5GHZ"))
        {
            Simulator::Schedule(Seconds(0), &MonitorPathLoss, i, lossModel5, wifiApNodes.Get(i), wifiStaNodes);
        }
        else if (findSubstring(channelStrAps[i][0],"6GHZ"))
        {
            
            Simulator::Schedule(Seconds(0), &MonitorPathLoss, i, lossModel6, wifiApNodes.Get(i), wifiStaNodes);
        }
        else if (findSubstring(channelStrAps[i][0],"2_4GHZ"))
        {
            Simulator::Schedule(Seconds(0), &MonitorPathLoss, i, lossModel2_4, wifiApNodes.Get(i), wifiStaNodes);
        }
    }

    // Enable the traces
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Assoc", MakeCallback(&AssocTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/DeAssoc", MakeCallback(&DeAssocTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/LinkAssoc", MakeCallback(&LinkAssocTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/LinkDeAssoc", MakeCallback(&LinkDeAssocTrace));

    Simulator::Stop(simulationTime);

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}