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

NS_LOG_COMPONENT_DEFINE("WifiEhtRoamingScenario");

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
    uint64_t assocCount{0};
    uint64_t deAssocCount{0};
    Time sumDelay{"0s"};
    Time avgDelay{"0s"};
    Time sumChAccessDelay{"0s"};
    Time avgChAccessDelay{"0s"};
    Time sumAssocTime{"0s"};
    Time sumDeAssocTime{"0s"};
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
MonitorPathLoss(uint32_t linkId, Ptr<IndoorBuildingsPropagationLossModel> lossModel, NodeContainer wifiAps, NodeContainer wifiStas) 
{
    for (uint32_t i = 0; i < wifiAps.GetN(); i++)
    {
        double lossAp = lossModel->GetLoss(wifiAps.Get(i)->GetObject<MobilityModel>(), wifiStas.Get(0)->GetObject<MobilityModel>());
        NS_LOG_INFO("Loss AP " << i << " with linkId " << linkId << " using frequency " << lossModel->GetFrequency() << " : " << lossAp << " dB");
    }

    // Schedule the next callback
    Simulator::Schedule(MilliSeconds(100), &MonitorPathLoss, linkId, lossModel, wifiAps, wifiStas);
}

std::vector<Vector> 
ReadWaypointsFromFile(const std::string& filename)
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

void 
AddWallsFromFile(Ptr<Factory> factory, const std::string& filename)
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

Ptr<ListPositionAllocator> 
ReadApPositionsFromFile(const std::string& filename)
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
AssocTrace(std::string context, Mac48Address staAddress, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA is associated with " << apAdress << std::endl;

    Analysis& staAnalysis = analysisMap[staAddress];
    
    staAnalysis.sumAssocTime += Simulator::Now();
    staAnalysis.assocCount++;
}

void
LinkAssocTrace(std::string context, uint8_t linkId, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA link " << (int)linkId << " is associated with " << apAdress << std::endl;
}

void
DeAssocTrace(std::string context, Mac48Address staAddress, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA is deassociated" << std::endl;

    Analysis& staAnalysis = analysisMap[staAddress];

    staAnalysis.sumDeAssocTime += Simulator::Now();
    staAnalysis.deAssocCount++;
}

void
LinkDeAssocTrace(std::string context, uint8_t linkId, Mac48Address apAdress)
{
    std::cout << Simulator::Now().As(Time::S) << " STA link " << (int)linkId << " is deassociated with " << apAdress << std::endl;
}

/**
 * PHY-level TX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
PhyTxTrace(std::string context, Ptr<const Packet> p, double txPowerW)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if(hdr.IsQosData() || hdr.IsData())
    {
        for (uint16_t i = 0; i < m_staDevices.GetN(); i++)
        {
            Ptr<WifiNetDevice> wifiStaDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(i));
            for (uint8_t linkId = 0; linkId < wifiStaDev->GetMac()->GetNLinks(); linkId++)
            {
                if(hdr.GetAddr1() == wifiStaDev->GetMac()->GetFrameExchangeManager(linkId)->GetAddress() || (hdr.GetAddr1() == wifiStaDev->GetMac()->GetAddress()))
                {
                    uint64_t packetUid = p->GetUid();
                    if (g_verbose)
                    {
                        std::cout << Simulator::Now().As(Time::US) << " PHY TX p: " << packetUid << std::endl;
                    }

                    Analysis& staAnalysis = analysisMap[wifiStaDev->GetMac()->GetAddress()];

                    if (!m_reliabilityMode || staAnalysis.phyTxMap.find(packetUid) == staAnalysis.phyTxMap.end()) 
                    {
                        staAnalysis.phyTxMap[packetUid] = Simulator::Now();
                    }
                    staAnalysis.phyTxPackets++;

                    if(m_reliabilityMode)
                    {
                        staAnalysis.phyTxMapReliability[std::make_pair(packetUid,hdr.GetAddr2())] = Simulator::Now();
                    }                    
                }
            }
        }
    }
}

void
PhyRxTrace(std::string context, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if(hdr.IsQosData() || hdr.IsData())
    {
        for (uint16_t i = 0; i < m_staDevices.GetN(); i++)
        {
            Ptr<WifiNetDevice> wifiStaDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(i));
            for (uint8_t linkId = 0; linkId < wifiStaDev->GetMac()->GetNLinks(); linkId++)
            {
                if(hdr.GetAddr1() == wifiStaDev->GetMac()->GetFrameExchangeManager(linkId)->GetAddress() || (hdr.GetAddr1() == wifiStaDev->GetMac()->GetAddress()))
                {
                    uint64_t packetUid = p->GetUid();
                    if (g_verbose)
                    {
                        std::cout << Simulator::Now().As(Time::US) << " PHY TX p: " << packetUid << std::endl;
                    }

                    Analysis& staAnalysis = analysisMap[wifiStaDev->GetMac()->GetAddress()];

                    if (!m_reliabilityMode || staAnalysis.phyRxMap.find(packetUid) == staAnalysis.phyRxMap.end()) 
                    {
                        staAnalysis.phyRxMap[packetUid] = Simulator::Now();
                    }
                    staAnalysis.phyRxPackets++;            
                }
            }
        }
    }
}

/**
  * MAC-level TX trace.
  *
  * \param context The context.
  * \param p The packet.
  */
void
MacTxWithAddressTrace(std::string context, Mac48Address txMacAddress, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if((hdr.IsQosData() || hdr.IsData()))
    {
        uint64_t packetUid = p->GetUid();
        if (g_verbose)
        {
            std::cout << Simulator::Now().As(Time::US) << " MAC TX p: " << packetUid << std::endl;
        }

        if(m_apStaAddressMap.find(txMacAddress) != m_apStaAddressMap.end())
        {
            Analysis& staAnalysis = analysisMap[m_apStaAddressMap.find(txMacAddress)->second];
                    
            if (!m_reliabilityMode || staAnalysis.macTxMap.find(packetUid) == staAnalysis.macTxMap.end()) 
            {
                staAnalysis.macTxMap[packetUid] = Simulator::Now();
            }
            staAnalysis.txPackets++;

            if(m_reliabilityMode)
            {
                staAnalysis.macTxMapReliability[std::make_pair(packetUid,txMacAddress)] = Simulator::Now();
            }  
        }
    }
}

/**
 * MAC-level RX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
MacRxWithAddressTrace(std::string context, Mac48Address rxMacAddress,Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if((hdr.IsQosData() || hdr.IsData()))
    {
        for (uint16_t i = 0; i < m_staDevices.GetN(); i++)
        {
            Ptr<WifiNetDevice> wifiStaDev = DynamicCast<WifiNetDevice>(m_staDevices.Get(i));
            if (rxMacAddress == wifiStaDev->GetAddress())
            {
                uint64_t packetUid = p->GetUid();
                if (g_verbose)
                {
                    std::cout << Simulator::Now().As(Time::US) << " MAC RX p: " << packetUid << std::endl;
                }

                Analysis& staAnalysis = analysisMap[rxMacAddress];
                    
                if (!m_reliabilityMode || staAnalysis.macRxMap.find(packetUid) == staAnalysis.macRxMap.end()) 
                {
                    staAnalysis.macRxMap[packetUid] = Simulator::Now();
                }
                staAnalysis.rxPackets++;
            }
        }
    }
}

void
DroppedMpduTrace(std::string context, Mac48Address txMacAddress, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if((hdr.IsQosData() || hdr.IsData()))
    {
        uint64_t packetUid = p->GetUid();
        if (g_verbose)
        {
            std::cout << Simulator::Now().As(Time::US) << " FEM TX dropped p: " << packetUid << std::endl;
        }

        if(m_apStaAddressMap.find(txMacAddress) != m_apStaAddressMap.end())
        {
            Analysis& staAnalysis = analysisMap[m_apStaAddressMap.find(txMacAddress)->second];
                    
            if(staAnalysis.dropTxMap.find(packetUid) != staAnalysis.dropTxMap.end())
            {
                staAnalysis.dropTxMap[packetUid] = staAnalysis.dropTxMap[packetUid] + 1;
            }
            else
            {
                staAnalysis.dropTxMap[packetUid] = 1;
            }

            if(m_reliabilityMode)
            {
                if(staAnalysis.dropTxMapReliability.find(std::make_pair(packetUid,txMacAddress)) != staAnalysis.dropTxMapReliability.end())
                {
                    staAnalysis.dropTxMapReliability[std::make_pair(packetUid,txMacAddress)] = staAnalysis.dropTxMapReliability[std::make_pair(packetUid,txMacAddress)] + 1;
                }
                else
                {
                    staAnalysis.dropTxMapReliability[std::make_pair(packetUid,txMacAddress)] = 1;
                }
            }          
        } 
    }
}

void
RetransmitMpduTrace(std::string context, Mac48Address txMacAddress, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if((hdr.IsQosData() || hdr.IsData()))
    {
        uint64_t packetUid = p->GetUid();
        if (g_verbose)
        {
            std::cout << Simulator::Now().As(Time::US) << " FEM TX retransmit p: " << packetUid << std::endl;
        }

        if(m_apStaAddressMap.find(txMacAddress) != m_apStaAddressMap.end())
        {
            Analysis& staAnalysis = analysisMap[m_apStaAddressMap.find(txMacAddress)->second];
                    
            if(staAnalysis.retransmitTxMap.find(packetUid) != staAnalysis.retransmitTxMap.end())
            {
                staAnalysis.retransmitTxMap[packetUid] = staAnalysis.retransmitTxMap[packetUid] + 1;
            }
            else
            {
                staAnalysis.retransmitTxMap[packetUid] = 1;
            }
            
            if(m_reliabilityMode)
            {
                if(staAnalysis.retransmitTxMapReliability.find(std::make_pair(packetUid,txMacAddress)) != staAnalysis.retransmitTxMapReliability.end())
                {
                    staAnalysis.retransmitTxMapReliability[std::make_pair(packetUid,txMacAddress)] = staAnalysis.retransmitTxMapReliability[std::make_pair(packetUid,txMacAddress)] + 1;
                }
                else
                {
                    staAnalysis.retransmitTxMapReliability[std::make_pair(packetUid,txMacAddress)] = 1;
                }
            }           
        }
    }
}

void
ChAccessRequestTrace(std::string context, Mac48Address txMacAddress, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if((hdr.IsQosData() || hdr.IsData()))
    {
        uint64_t packetUid = p->GetUid();
        if (g_verbose)
        {
            std::cout << Simulator::Now().As(Time::US) << " Channel access TX requested p: " << packetUid << std::endl;
        }

        if(m_apStaAddressMap.find(txMacAddress) != m_apStaAddressMap.end())
        {
            Analysis& staAnalysis = analysisMap[m_apStaAddressMap.find(txMacAddress)->second];
            
            staAnalysis.chReqTxMap[packetUid] = Simulator::Now();

            if(m_reliabilityMode)
            {
                staAnalysis.chReqTxMapReliability[std::make_pair(packetUid,txMacAddress)] = Simulator::Now();
            }
        } 
    }
}

void
ChAccessGrantedTrace(std::string context, Mac48Address txMacAddress, Ptr<const Packet> p)
{
    WifiMacHeader hdr;
    p->PeekHeader(hdr);
    if((hdr.IsQosData() || hdr.IsData()))
    {
        uint64_t packetUid = p->GetUid();
        if (g_verbose)
        {
            std::cout << Simulator::Now().As(Time::US) << " Channel access TX granted p: " << packetUid << std::endl;
        }

        if(m_apStaAddressMap.find(txMacAddress) != m_apStaAddressMap.end())
        {
            Analysis& staAnalysis = analysisMap[m_apStaAddressMap.find(txMacAddress)->second];
                    
            staAnalysis.chGrantedTxMap[packetUid] = Simulator::Now();

            if(m_reliabilityMode)
            {
                staAnalysis.chGrantedTxMapReliability[std::make_pair(packetUid,txMacAddress)] = Simulator::Now();
            }
        } 
    }
}

void
CalculateE2EDelay(Time& sumDelay, const std::map<uint64_t, Time>& txMap, const std::map<uint64_t, Time>& rxMap)
{
    for (const auto& rxPackets : rxMap)
    {
        uint64_t packetUid = rxPackets.first;
        auto txPackets = txMap.find(packetUid);

        if (txPackets != txMap.end())
        {
            Time e2eDelay = rxPackets.second - txPackets->second;
            sumDelay += e2eDelay;
        }
    }
}

void
CalculateChAccessDelay(uint64_t& chAccessCount, Time& sumChAccessDelay, const std::map<uint64_t, Time>& txMap, const std::map<uint64_t, Time>& rxMap, const std::map<uint64_t, Time>& chReqMap, const std::map<uint64_t, Time>& chGrantedMap)
{
    for (const auto& rxPackets : rxMap)
    {
        uint64_t packetUid = rxPackets.first;
        auto txPackets = txMap.find(packetUid);
        auto chReqPackets = chReqMap.find(packetUid);
        auto chGrantedPackets = chGrantedMap.find(packetUid);

        if (chReqPackets == chReqMap.end())
        {
            chAccessCount++;
        }

        if (txPackets != txMap.end())// && chReqPackets != chReqMap.end())
        {
            Time chAccessTime = chGrantedPackets->second - chReqPackets->second;
            Time chAcessDelay;
            if (chReqPackets == chReqMap.end())
            {
                chAccessCount++;
                // std::cout << "Debug chAccessTime: " << chAccessTime.As(Time::S) << std::endl;
                chAcessDelay = rxPackets.second - txPackets->second;
            }
            else
            {
                chAccessTime = std::max(chAccessTime, Time("0s"));
                chAcessDelay = rxPackets.second - txPackets->second + chAccessTime;
            }
            sumChAccessDelay += chAcessDelay;
        }
    }
}

void
CalculateReliabilityE2EDelay(Time& sumDelay, const std::map<std::pair<uint64_t,Mac48Address>, Time>& txMap, const std::map<uint64_t, Time>& rxMap)
{
    for (const auto& rxPackets : rxMap)
    {
        uint64_t packetUid = rxPackets.first;
        Time rxTime = rxPackets.second;
        Time maxValidTxTime = Time::Min();
        Time sumTxTime{"0s"};
        uint16_t txCount = 0;
        
        for (const auto& txPackets : txMap)
        {
            if (txPackets.first.first == packetUid && txPackets.second < rxTime)
            {
                if (txPackets.second > maxValidTxTime)
                {
                    maxValidTxTime = txPackets.second;
                    sumTxTime += txPackets.second;
                    txCount++;
                }
            }
        }

        if (maxValidTxTime != Time::Min())
        {
            // Time chAccessDelay = rxTime - maxValidTxTime;
            Time e2eDelay = rxTime - sumTxTime/txCount;
            sumDelay += e2eDelay;
        }
    }
}

void
CalculateReliabilityChAccessDelay(uint64_t& chAccessCount, Time& sumChAccessDelay, const std::map<std::pair<uint64_t,Mac48Address>, Time>& txMap, const std::map<uint64_t, Time>& rxMap, const std::map<std::pair<uint64_t,Mac48Address>, Time>& chReqMap, const std::map<std::pair<uint64_t,Mac48Address>, Time>& chGrantedMap)
{
    for (const auto& rxPackets : rxMap)
    {
        uint64_t packetUid = rxPackets.first;
        Time rxTime = rxPackets.second;
        Time maxValidTxTime = Time::Min();
        Time minChAccessTxTime = Time::Max();
        Time sumTxTime{"0s"};
        Time sumChAccessTime{"0s"};
        uint16_t txCount = 0;

        for (const auto& txPackets : txMap)
        {
            if (txPackets.first.first == packetUid && txPackets.second < rxTime)
            {
                if (txPackets.second > maxValidTxTime)
                {
                    maxValidTxTime = txPackets.second;
                    sumTxTime += txPackets.second;
                    txCount++;
                }
            }
        }

        for (const auto& chReqPackets : chReqMap)
        {
            if (chReqPackets.first.first == packetUid)
            {
                auto it = chGrantedMap.find(std::make_pair(packetUid, chReqPackets.first.second));
                if (it != chGrantedMap.end())
                {
                    auto chReqTime = it->second - chReqPackets.second;
                    if (chReqTime < minChAccessTxTime)
                    {
                        minChAccessTxTime = chReqTime;
                        sumChAccessTime += minChAccessTxTime;
                    }
                }
            }
        }

        if (maxValidTxTime != Time::Min() && minChAccessTxTime != Time::Max())
        {
            Time chAccessDelay = rxTime - maxValidTxTime + minChAccessTxTime;
            // Time chAccessDelay = rxTime - sumTxTime/txCount;
            sumChAccessDelay += chAccessDelay;
        }
    }
}

uint64_t
CalculateFirstTransmissionReliability(uint8_t numLinks, const std::map<std::pair<uint64_t,Mac48Address>, uint64_t>& txMap)
{
    std::map<uint64_t, uint64_t> retryPacketMap;
    uint64_t initTx = 0;

    for (const auto& txPackets : txMap)
    {
        uint64_t packetUid = txPackets.first.first;
        if(retryPacketMap.find(packetUid) != retryPacketMap.end())
        {
            retryPacketMap[packetUid] = retryPacketMap[packetUid] + 1;
        }
        else
        {
            retryPacketMap[packetUid] = 1;
        }   
    }

    for (const auto& [packetId, count] : retryPacketMap)
    {
        // std::cout << count << std::endl;
        if (count < numLinks)
        {
            initTx++;
        }
    }
    return initTx;
}

uint64_t
CalculateAllDropReliability(uint8_t numLinks, const std::map<std::pair<uint64_t,Mac48Address>, uint64_t>& txMap)
{
    std::map<uint64_t, uint64_t> dropPacketMap;
    uint64_t numDrop = 0;

    for (const auto& txPackets : txMap)
    {
        uint64_t packetUid = txPackets.first.first;
        if(dropPacketMap.find(packetUid) != dropPacketMap.end())
        {
            dropPacketMap[packetUid] = dropPacketMap[packetUid] + 1;
        }
        else
        {
            dropPacketMap[packetUid] = 1;
        }   
    }

    for (const auto& [packetId, count] : dropPacketMap)
    {
        // std::cout << count << std::endl;
        if (count >= numLinks)
        {
            numDrop++;
        }
    }
    return numDrop;
}

int main(int argc, char *argv[])
{
    // Simulation parameters
    uint32_t numNodes = 1; // Number of nodes
    uint32_t seed = 75;    // Random seed
    Time simulationTime{"100s"};//{"100s"};
    uint16_t numBss = 1;
    std::vector<uint16_t> numApsPerBss = {4};
    std::string currentDir = "./scratch/wifi-eht-roaming-scenario/";
    std::string roomLayoutDir = currentDir + "walls-assignment.txt";
    std::string apLayoutDir = currentDir + "ap-positions.txt";
    std::string waypointDir = currentDir + "random-human-waypoints.txt";//"random-human-waypoints.txt";

    // create a grid of buildings
    double factorySizeX = 20; // m
    double factorySizeY = 20;  // m
    double factoryHeight = 3; // m
    double wallLoss = 9; // dB

    // Wifi Simulation Parameters
    bool udp{true};
    bool downlink{true};
    bool useRts{false};
    bool frameAggregation{false};
    bool use80Plus80{false};
    uint16_t mpduBufferSize{512};
    std::string emlsrLinks;//="0,1,2";
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
    std::string dlAckSeqType{"NO-OFDMA"};//(NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    int mcs{-1}; // -1 indicates an unset value
    uint32_t payloadSize =
        1474;//1474; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    double poissonLambda = 0;
    double minExpectedThroughput{0};
    double maxExpectedThroughput{0};
    Time accessReqInterval{0};
    uint32_t maxMissedBeacons = 2;
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
    cmd.AddValue("frameAggregation",
                "Enable frame aggregation",
                frameAggregation);
    cmd.AddValue("printOutput",
                "Print results to file",
                printOutput);
    cmd.AddValue("poissonLambda",
                "Arrival rate of the poisson traffic",
                poissonLambda);
    cmd.AddValue("maxMissedBeacons",
                "Maximum missed beacons before disassociation",
                maxMissedBeacons);
    cmd.AddValue("seed",
                "rng seed number",
                seed);
    cmd.Parse(argc, argv);

    LogComponentEnable("WifiEhtRoamingScenario", LOG_LEVEL_FUNCTION);

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

    bool has24GHz = false;
    for (uint16_t i = 0; i < wifiStaNodes.GetN(); i++)
    {
        for (auto freq : freqBands)
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

    for (auto freq : freqBands)
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
    auto lossModel5 = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel5->SetBuildingType(Building::Office);
    lossModel5->SetWallLoss(wallLoss);
    auto lossModel6 = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel6->SetBuildingType(Building::Office);
    lossModel6->SetWallLoss(wallLoss);
    auto lossModel2_4 = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel2_4->SetBuildingType(Building::Office);
    lossModel2_4->SetWallLoss(wallLoss);
    spectrumChannel5->AddPropagationLossModel(lossModel5);
    spectrumChannel6->AddPropagationLossModel(lossModel6);
    spectrumChannel2_4->AddPropagationLossModel(lossModel2_4);

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

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                "MaxMissedBeacons", UintegerValue(maxMissedBeacons));
    staDevices.Add(wifi.Install(phy, mac, wifiStaNodes));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));

    phy.Set ("TxPowerStart", DoubleValue (powAp));
    phy.Set ("TxPowerEnd", DoubleValue (powAp));
    phy.Set ("TxPowerLevels", UintegerValue (1));
    phy.Set("CcaEdThreshold", DoubleValue(ccaEdTr));
    phy.Set("RxSensitivity", DoubleValue(-92.0));

    apDevices.Add(wifi.Install(phy, mac, wifiApNodes));

    Ptr<WifiNetDevice> staNetDev = staDevices.Get(0)->GetObject<WifiNetDevice>();
    for (uint8_t linkId = 0; linkId < nLinks; linkId++)
    {
        Ptr<WifiPhy> phy = staNetDev->GetPhy(linkId);
        double operatingFrequency = phy->GetFrequency()*1e6;
        if (phy)
        {
            std::cout << "STA Link " << unsigned(linkId) << " frequency: " << operatingFrequency << " Hz" << std::endl;
        }

        if (findSubstring(channelStr[linkId],"5GHZ"))
        {
            lossModel5->SetFrequency(operatingFrequency);
        }
        else if (findSubstring(channelStr[linkId],"6GHZ"))
        {
            lossModel6->SetFrequency(operatingFrequency);
        }
        else if (findSubstring(channelStr[linkId],"2_4GHZ"))
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

    /* Setting applications */
    std::vector<ApplicationContainer> serverApps(numBss);
    std::vector<ApplicationContainer> clientApps(numBss);
    
    std::vector<uint64_t> maxLoad(numBss);
    // Calculate maxLoad as
    for (uint16_t i = 0; i < numBss; i++)
    {
        if (mcs >= 0)
        {
            // maxLoad[i] = nLinks * EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1);
            maxLoad[i] = EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1);
            std::cout << "Data rate BSS " << i+1 << " : " << EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1)/1e6 << " Mbit/s" << std::endl;
        }
        else
        {
            // maxLoad[i] = nLinks * EhtPhy::GetDataRate(11, channelWidth, NanoSeconds(gi), 1);
            maxLoad[i] = EhtPhy::GetDataRate(11, channelWidth, NanoSeconds(gi), 1);
        }

        if (udp)
        {
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
        
        clientApps[i].Add(client.Install(wifiApNodes.Get(i)));

        clientApps[i].Start(Seconds(1.0));
        clientApps[i].Stop(simulationTime + Seconds(1.0));
    }
    
    std::vector<std::vector<uint64_t>> allRxBytes;
    for (uint16_t i = 0; i < numBss; i++)
    {
        allRxBytes.emplace_back(1,0);
    }

    m_staDevices = staDevices;
    for (uint32_t i = 0; i < numBss; i++) 
    {
        Ptr<WifiNetDevice> wifiStaDev = DynamicCast<WifiNetDevice>(staDevices.Get(i));
        Ptr<WifiNetDevice> wifiApDev = DynamicCast<WifiNetDevice>(apDevices.Get(i));
        m_apStaAddressMap[wifiApDev->GetMac()->GetAddress()] = wifiStaDev->GetMac()->GetAddress();
    }

    // Set guard interval and MPDU buffer size
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
        TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                UintegerValue(mpduBufferSize));

    // Simulation clock
    Simulator::Schedule (MicroSeconds (100), &TimePasses);

    // // Schedule the pathloss calculation callback
    // for (uint32_t linkId = 0; linkId < nLinks; linkId++)
    // {
    //     if (findSubstring(channelStr[linkId],"5GHZ"))
    //     {
    //         Simulator::Schedule(Seconds(0), &MonitorPathLoss, linkId, lossModel5, wifiApNodes, wifiStaNodes);
    //     }
    //     else if (findSubstring(channelStr[linkId],"6GHZ"))
    //     {
            
    //         Simulator::Schedule(Seconds(0), &MonitorPathLoss, linkId, lossModel6, wifiApNodes, wifiStaNodes);
    //     }
    //     else if (findSubstring(channelStr[linkId],"2_4GHZ"))
    //     {
    //         Simulator::Schedule(Seconds(0), &MonitorPathLoss, linkId, lossModel2_4, wifiApNodes, wifiStaNodes);
    //     }
    // }

    // Enable the traces
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/NewAssoc", MakeCallback(&AssocTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/NewDeAssoc", MakeCallback(&DeAssocTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/LinkAssoc", MakeCallback(&LinkAssocTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/LinkDeAssoc", MakeCallback(&LinkDeAssocTrace));

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxWithAddress", MakeCallback(&MacTxWithAddressTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxWithAddress", MakeCallback(&MacRxWithAddressTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phys/*/PhyTxBegin",MakeCallback(&PhyTxTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phys/*/PhyRxEnd",MakeCallback(&PhyRxTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/FrameExchangeManagers/*/FemMpduDropped",MakeCallback(&DroppedMpduTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/FrameExchangeManagers/*/FemMpduRetransmitted",MakeCallback(&RetransmitMpduTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/StartChRequestAccess",MakeCallback(&ChAccessRequestTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/AccesRequestGranted",MakeCallback(&ChAccessGrantedTrace));


    Simulator::Stop(simulationTime);

    Simulator::Run();

    double sumThroughput = 0;
    double sumReliability = 0;
    double sumDropReliabilty = 0;
    double sumTxReliability = 0;
    Time sumDelay{"0s"};
    Time sumChAccessDelay{"0s"};
    Time sumAssocDelay{"0s"};

    for (auto& [macAddresss, analysis] : analysisMap) 
    {
        std::cout << "\n=== STA: " << macAddresss << " ===" << std::endl;
        
        std::cout << "Tx Packets : " << analysis.txPackets << std::endl;
        std::cout << "Phy Tx Packets : " << analysis.phyTxPackets << std::endl;
        std::cout << "Rx Packets : " << analysis.rxPackets << std::endl;
        std::cout << "Phy Rx Packets : " << analysis.phyRxPackets << std::endl;
        std::cout << "Drop Packets : " << analysis.dropTxMap.size() << std::endl;
        std::cout << "Retransmitted Packets : " <<  analysis.retransmitTxMap.size()  << std::endl;
        
        // Throughput
        auto throughput = (double) (analysis.rxPackets * 8 * payloadSize) / simulationTime.GetMicroSeconds();
        sumThroughput += throughput;
        
        //Reliability
        double reliability;
        double dropReliability;
        double txReliability;
        txReliability = 1 - (double) analysis.retransmitTxMap.size()/std::min(analysis.phyTxMap.size(),analysis.macTxMap.size());
        dropReliability = 1 - (double) analysis.dropTxMap.size()/std::min(analysis.phyTxPackets,analysis.txPackets);
        reliability = (double) analysis.phyRxMap.size()/std::max(analysis.phyTxMap.size(),analysis.macTxMap.size()); //analysis.rxPackets/std::min(analysis.phyTxPackets,analysis.txPackets);
        
        sumReliability += reliability;
        sumDropReliabilty += dropReliability;
        sumTxReliability += txReliability;

        //Delay
        CalculateChAccessDelay(analysis.chAccessCount, analysis.sumChAccessDelay, analysis.phyTxMap, analysis.phyRxMap, analysis.chReqTxMap, analysis.chGrantedTxMap);
        CalculateE2EDelay(analysis.sumDelay, analysis.macTxMap, analysis.macRxMap);

        analysis.avgChAccessDelay = analysis.sumChAccessDelay/(analysis.phyRxMap.size()-analysis.chAccessCount);
        sumChAccessDelay += analysis.avgChAccessDelay;
        analysis.avgDelay = analysis.sumDelay/analysis.macRxMap.size();
        sumDelay += analysis.avgDelay;

        //Association and Deassociation Delay
        Time assocDelay{"0s"};
        assocDelay = analysis.sumAssocTime - analysis.sumDeAssocTime;
        sumAssocDelay += assocDelay;

        // Performance Summary
        std::cout << "\nPerformance Evaluation" << std::endl;
        std::cout << "Throughput : " << throughput << " Mbit/s" << std::endl;
        std::cout << "Packets Dropped Reliability : " << dropReliability*100 << "%" << std::endl;
        std::cout << "Packets Received Reliability : " << reliability*100 << "%" << std::endl;
        std::cout << "1st Transmission Reliability : " << txReliability*100 << "%" << std::endl;
        std::cout << "E2E Delay: " << analysis.avgDelay.As(Time::MS) << std::endl;
        std::cout << "Ch Access Delay: " << analysis.avgChAccessDelay.As(Time::MS) << std::endl;
        std::cout << "Association Delay: " << assocDelay.As(Time::MS) << std::endl;
    }

    sumThroughput = sumThroughput/numBss;
    sumReliability = sumReliability/numBss;
    sumDropReliabilty = sumDropReliabilty/numBss;
    sumTxReliability = sumTxReliability/numBss;
    sumDelay = sumDelay/numBss;
    sumChAccessDelay = sumChAccessDelay/numBss;
    sumAssocDelay = sumAssocDelay/numBss;

    std::cout << "\n=== Evaluation Summary ===" << std::endl;
    std::cout << "Seed : " << seed << std::endl;
    std::cout << "Average Throughput : " << sumThroughput << " Mbit/s" << std::endl;
    std::cout << "Average Packets Dropped Reliability : " << sumDropReliabilty*100 << "%" << std::endl;
    std::cout << "Average Packets Received Reliability : " << sumReliability*100 << "%" << std::endl;
    std::cout << "Average 1st Transmission Reliability : " << sumTxReliability*100 << "%" << std::endl;
    std::cout << "Average E2E Delay: " << sumDelay.As(Time::MS) << std::endl;
    std::cout << "Average Ch Acess Delay: " << sumChAccessDelay.As(Time::MS) << std::endl;
    std::cout << "Average Assoc Delay: " << sumAssocDelay.As(Time::MS) << std::endl;

    if(printOutput)
    {
        std::ofstream outputFile;
        std::string outputFileName;
        
        if (poissonLambda > 0)
        {
        outputFileName = currentDir + "/wifi-eht-roaming-results-unsaturated";
        }
        else
        {
            outputFileName = currentDir + "/wifi-ehr-roaming-results";
        }

        if (!(fileExists(outputFileName)))
        {
            outputFile.open(outputFileName);
        }
        else
        {
            outputFile.open(outputFileName, std::ios::in | std::ios::out);
            outputFile.seekp(0, std::ios::end);
        }

        outputFile << seed << "," << numBss << ","  << sumThroughput << "," << sumDropReliabilty*100 << "," << sumReliability*100 << "," << sumDelay.As(Time::MS) << "," << sumChAccessDelay.As(Time::MS) << "\n";
        outputFile.flush();
        outputFile.close();
    }

    Simulator::Destroy();

    return 0;
}