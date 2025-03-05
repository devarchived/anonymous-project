/*
 * Copyright (c) 2022
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 */

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

// This is a simple example in order to show how to configure an IEEE 802.11be Wi-Fi network.
//
// It outputs the UDP or TCP goodput for every EHT MCS value, which depends on the MCS value (0 to
// 13), the channel width (20, 40, 80 or 160 MHz) and the guard interval (800ns, 1600ns or 3200ns).
// The PHY bitrate is constant over all the simulation run. The user can also specify the distance
// between the access point and the station: the larger the distance the smaller the goodput.
//
// The simulation assumes a configurable number of stations in an infrastructure network:
//
//  STA     AP
//    *     *
//    |     |
//   n1     n2
//
// Packets in this simulation belong to BestEffort Access Class (AC_BE).
// By selecting an acknowledgment sequence for DL MU PPDUs, it is possible to aggregate a
// Round Robin scheduler to the AP, so that DL MU PPDUs are sent by the AP via DL OFDMA.

using namespace ns3;

// NS_LOG_COMPONENT_DEFINE("ehr-dualband-network");

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

int
main(int argc, char* argv[])
{
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
    Time simulationTime{"0.2s"};
    meter_u distance{2.0};
    double frequencySta{5};  // whether the first link operates in the 2.4, 5 or 6 GHz
    double frequencySta_2{6}; // whether the second link operates in the 2.4, 5 or 6 GHz (0 means no
                          // second link exists)
    double frequencySta_3{0}; // whether the third link operates in the 2.4, 5 or 6 GHz (0 means no third link exists)
    double frequencyApA{5};
    double frequencyApA_2{0};
    double frequencyApA_3{0};
    double frequencyApB{6};
    double frequencyApB_2{0};
    double frequencyApB_3{0};
    dBm_u powSta{10.0};
    dBm_u powAp1{21.0};
    dBm_u powAp2{21.0};
    dBm_u ccaEdTrSta{-62};
    dBm_u ccaEdTrAp1{-62};
    dBm_u ccaEdTrAp2{-62};
    dBm_u minimumRssi{-82};
    int channelWidth = 20;
    int gi = 3200;
    std::size_t nStations{1};
    std::size_t nAPs{2};
    std::string dlAckSeqType{"MU-BAR"};//(NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    int mcs{0}; // -1 indicates an unset value
    uint32_t payloadSize =
        700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    double minExpectedThroughput{0};
    double maxExpectedThroughput{0};
    Time accessReqInterval{0};
    bool enableMultiApCoordination = true;
    bool reliabilityMode = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue(
        "frequency",
        "Whether the first link operates in the 2.4, 5 or 6 GHz band (other values gets rejected)",
        frequencySta);
    cmd.AddValue(
        "frequency2",
        "Whether the second link operates in the 2.4, 5 or 6 GHz band (0 means the device has one "
        "link, otherwise the band must be different than first link and third link)",
        frequencySta_2);
    cmd.AddValue(
        "frequency3",
        "Whether the third link operates in the 2.4, 5 or 6 GHz band (0 means the device has up to "
        "two links, otherwise the band must be different than first link and second link)",
        frequencySta_3);
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
    cmd.AddValue("distance",
                 "Distance in meters between the station and the access point",
                 distance);
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
    cmd.Parse(argc, argv);

    // LogComponentEnable("ehr-wifi-network-dualband", LOG_LEVEL_FUNCTION);

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
    uint16_t maxChannelWidth = (frequencySta != 2.4 && frequencySta_2 != 2.4 && frequencySta_3 != 2.4) ? 160 : 40;
    int minGi = enableUlOfdma ? 1600 : 800;
    const auto is80Plus80 = (use80Plus80 && (channelWidth == 160));
    const std::string widthStr = is80Plus80 ? "80+80" : std::to_string(channelWidth);
    const auto segmentWidthStr = is80Plus80 ? "80" : widthStr;

    if (!udp)
    {
        Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
    }

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nStations);
    NodeContainer wifiApNodes;
    wifiApNodes.Create(nAPs);

    NetDeviceContainer apDevices;
    NetDeviceContainer staDevices;

    WifiMacHelper mac;
    WifiHelper wifiSta;
    WifiHelper wifiApA;
    WifiHelper wifiApB;

    // Sets the Wi-Fi standard to 802.11be
    wifiSta.SetStandard(WIFI_STANDARD_80211be);
    wifiApA.SetStandard(WIFI_STANDARD_80211be);
    wifiApB.SetStandard(WIFI_STANDARD_80211be);
    std::array<std::string, 3> channelStrSta;
    std::array<std::string, 3> channelStrApA;
    std::array<std::string, 3> channelStrApB;
    std::array<FrequencyRange, 3> freqRangesSta;
    std::array<FrequencyRange, 3> freqRangesApA;
    std::array<FrequencyRange, 3> freqRangesApB;
    uint8_t nLinksSta = 0;
    uint8_t nLinksApA = 0;
    uint8_t nLinksApB = 0;

    std::string ctrlRateStr;
    std::string dataModeStr;
    uint64_t nonHtRefRateMbps;
    if (mcs >= 0)
    {
        dataModeStr = "EhtMcs" + std::to_string(mcs);
        nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;   
    }
    
    // Check if there are duplicate frequencies used
    if (frequencySta_2 == frequencySta || frequencySta_3 == frequencySta ||
        (frequencySta_3 != 0 && frequencySta_3 == frequencySta_2))
    {
        NS_FATAL_ERROR("Frequency values must be unique!");
    }

    // Check if there are duplicate frequencies used
    if (frequencyApA_2 == frequencyApA || frequencyApA_3 == frequencyApA ||
        (frequencyApA_3 != 0 && frequencyApA_3 == frequencyApA_2))
    {
        NS_FATAL_ERROR("Frequency values must be unique!");
    }

    // Check if there are duplicate frequencies used
    if (frequencyApB_2 == frequencyApB || frequencyApB_3 == frequencyApB ||
        (frequencyApB_3 != 0 && frequencyApB_3 == frequencyApB_2))
    {
        NS_FATAL_ERROR("Frequency values must be unique!");
    }

    for (auto freq : {frequencySta, frequencySta_2, frequencySta_3})
    {
        if (nLinksSta > 0 && freq == 0)
        {
            break;
        }
        if (freq == 6)
        {
            channelStrSta[nLinksSta] = "{1, " + segmentWidthStr + ", ";
            channelStrSta[nLinksSta] += "BAND_6GHZ, 0}";
            freqRangesSta[nLinksSta] = WIFI_SPECTRUM_6_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(48));
            if (mcs >= 0)
            {
                wifiSta.SetRemoteStationManager(nLinksSta,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiSta.SetRemoteStationManager(nLinksSta,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 5)
        {
            channelStrSta[nLinksSta] = "{36, " + segmentWidthStr + ", ";
            channelStrSta[nLinksSta] += "BAND_5GHZ, 0}";
            freqRangesSta[nLinksSta] = WIFI_SPECTRUM_5_GHZ;
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifiSta.SetRemoteStationManager(nLinksSta,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiSta.SetRemoteStationManager(nLinksSta,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 2.4)
        {
            channelStrSta[nLinksSta] = "{1, " + segmentWidthStr + ", ";
            channelStrSta[nLinksSta] += "BAND_2_4GHZ, 0}";
            freqRangesSta[nLinksSta] = WIFI_SPECTRUM_2_4_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(40));
            ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifiSta.SetRemoteStationManager(nLinksSta,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiSta.SetRemoteStationManager(nLinksSta,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else
        {
            NS_FATAL_ERROR("Wrong frequency value!");
        }

        if (is80Plus80)
        {
            channelStrSta[nLinksSta] += std::string(";") + channelStrSta[nLinksSta];
        }

        nLinksSta++;
    }

    for (auto freq : {frequencyApA, frequencyApA_2, frequencyApA_3})
    {
        if (nLinksApA > 0 && freq == 0)
        {
            break;
        }
        if (freq == 6)
        {
            channelStrApA[nLinksApA] = "{1, " + segmentWidthStr + ", ";
            channelStrApA[nLinksApA] += "BAND_6GHZ, 0}";
            freqRangesApA[nLinksApA] = WIFI_SPECTRUM_6_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(48));
            if (mcs >= 0)
            {
                wifiApA.SetRemoteStationManager(nLinksApA,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiApA.SetRemoteStationManager(nLinksApA,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 5)
        {
            channelStrApA[nLinksApA] = "{36, " + segmentWidthStr + ", ";
            channelStrApA[nLinksApA] += "BAND_5GHZ, 0}";
            freqRangesApA[nLinksApA] = WIFI_SPECTRUM_5_GHZ;
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifiApA.SetRemoteStationManager(nLinksApA,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiApA.SetRemoteStationManager(nLinksApA,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 2.4)
        {
            channelStrApA[nLinksApA] = "{1, " + segmentWidthStr + ", ";
            channelStrApA[nLinksApA] += "BAND_2_4GHZ, 0}";
            freqRangesApA[nLinksApA] = WIFI_SPECTRUM_2_4_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(40));
            ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifiApA.SetRemoteStationManager(nLinksApA,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiApA.SetRemoteStationManager(nLinksApA,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else
        {
            NS_FATAL_ERROR("Wrong frequency value!");
        }

        if (is80Plus80)
        {
            channelStrApA[nLinksApA] += std::string(";") + channelStrApA[nLinksApA];
        }

        nLinksApA++;
    }

    for (auto freq : {frequencyApB, frequencyApB_2, frequencyApB_3})
    {
        if (nLinksApB > 0 && freq == 0)
        {
            break;
        }
        if (freq == 6)
        {
            channelStrApB[nLinksApB] = "{1, " + segmentWidthStr + ", ";
            channelStrApB[nLinksApB] += "BAND_6GHZ, 0}";
            freqRangesApB[nLinksApB] = WIFI_SPECTRUM_6_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(48));
            if (mcs >= 0)
            {
                wifiApB.SetRemoteStationManager(nLinksApB,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiApB.SetRemoteStationManager(nLinksApB,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 5)
        {
            channelStrApB[nLinksApB] = "{36, " + segmentWidthStr + ", ";
            channelStrApB[nLinksApB] += "BAND_5GHZ, 0}";
            freqRangesApB[nLinksApB] = WIFI_SPECTRUM_5_GHZ;
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifiApB.SetRemoteStationManager(nLinksApB,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiApB.SetRemoteStationManager(nLinksApB,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else if (freq == 2.4)
        {
            channelStrApB[nLinksApB] = "{1, " + segmentWidthStr + ", ";
            channelStrApB[nLinksApB] += "BAND_2_4GHZ, 0}";
            freqRangesApB[nLinksApB] = WIFI_SPECTRUM_2_4_GHZ;
            Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                                DoubleValue(40));
            ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            if (mcs >= 0)
            {
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                wifiApB.SetRemoteStationManager(nLinksApB,
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue(dataModeStr),
                                            "ControlMode",
                                            StringValue(dataModeStr));
            }
            else 
            {
                wifiApB.SetRemoteStationManager(nLinksApB,
                                            "ns3::MinstrelHtWifiManager");
            }
        }
        else
        {
            NS_FATAL_ERROR("Wrong frequency value!");
        }

        if (is80Plus80)
        {
            channelStrApB[nLinksApB] += std::string(";") + channelStrApB[nLinksApB];
        }

        nLinksApB++;
    }
        
    // Check if multi-link is used!!
    if (nLinksSta > 1 && !emlsrLinks.empty())
    {
        wifiSta.ConfigEhtOptions("EmlsrActivated", BooleanValue(true));
    }

    if (nLinksSta > 1 && enableMultiApCoordination)
    {
        wifiSta.ConfigEhtOptions("EnableMultiApCoordination", BooleanValue(true));
        wifiApA.ConfigEhtOptions("EnableMultiApCoordination", BooleanValue(true));
        wifiApB.ConfigEhtOptions("EnableMultiApCoordination", BooleanValue(true));
    }

    if(reliabilityMode)
    {
        wifiSta.ConfigEhtOptions("ReliabilityMode", BooleanValue(true));
    }

    std::cout << "nLinksSta : " << (int)nLinksSta << std::endl;
    std::cout << "nLinksApA : " << (int)nLinksApA << std::endl;
    std::cout << "nLinksApB : " << (int)nLinksApB << std::endl;
    
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

    Ssid ssid = Ssid("ns3-80211bn");

    SpectrumWifiPhyHelper phySta(nLinksSta);
    phySta.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySta.Set("ChannelSwitchDelay", TimeValue(MicroSeconds(channelSwitchDelayUsec)));

    // Sets up multiple link in the physical layer
    for (uint8_t linkId = 0; linkId < nLinksSta; linkId++)
    {
        phySta.Set(linkId, "ChannelSettings", StringValue(channelStrSta[linkId]));
        if (findSubstring(channelStrSta[linkId],"5GHZ"))
        {
            phySta.AddChannel(spectrumChannel5, freqRangesSta[linkId]);
        }
        else if (findSubstring(channelStrSta[linkId],"6GHZ"))
        {
            phySta.AddChannel(spectrumChannel6, freqRangesSta[linkId]);
        }
        else if (findSubstring(channelStrSta[linkId],"2_4GHZ"))
        {
            phySta.AddChannel(spectrumChannel2_4, freqRangesSta[linkId]);
        }
    }

    phySta.Set ("TxPowerStart", DoubleValue (powSta));
    phySta.Set ("TxPowerEnd", DoubleValue (powSta));
    phySta.Set ("TxPowerLevels", UintegerValue (1));
    phySta.Set("CcaEdThreshold", DoubleValue(ccaEdTrSta));
    phySta.Set("RxSensitivity", DoubleValue(-92.0));

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));\
    staDevices.Add(wifiSta.Install(phySta, mac, wifiStaNodes.Get(0)));

    SpectrumWifiPhyHelper phyApA(nLinksApA);
    phyApA.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phyApA.Set("ChannelSwitchDelay", TimeValue(MicroSeconds(channelSwitchDelayUsec)));

    // Sets up multiple link in the physical layer
    for (uint8_t linkId = 0; linkId < nLinksApA; linkId++)
    {
        phyApA.Set(linkId, "ChannelSettings", StringValue(channelStrApA[linkId]));
        if (findSubstring(channelStrApA[linkId],"5GHZ"))
        {
            phyApA.AddChannel(spectrumChannel5, freqRangesApA[linkId]);
        }
        else if (findSubstring(channelStrApA[linkId],"6GHZ"))
        {
            phyApA.AddChannel(spectrumChannel6, freqRangesApA[linkId]);
        }
        else if (findSubstring(channelStrApA[linkId],"2_4GHZ"))
        {
            std::cout << "Debug 2.4" << std::endl;
            phyApA.AddChannel(spectrumChannel2_4, freqRangesApA[linkId]);
        }
    }

    phyApA.Set ("TxPowerStart", DoubleValue (powAp1));
    phyApA.Set ("TxPowerEnd", DoubleValue (powAp1));
    phyApA.Set ("TxPowerLevels", UintegerValue (1));
    phyApA.Set("CcaEdThreshold", DoubleValue(ccaEdTrAp1));
    phyApA.Set("RxSensitivity", DoubleValue(-92.0));

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

    apDevices.Add(wifiApA.Install(phyApA, mac, wifiApNodes.Get(0)));
    
    SpectrumWifiPhyHelper phyApB(nLinksApB);

    phyApB.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phyApB.Set("ChannelSwitchDelay", TimeValue(MicroSeconds(channelSwitchDelayUsec)));

    // Sets up multiple link in the physical layer
    for (uint8_t linkId = 0; linkId < nLinksApB; linkId++)
    {
        phyApB.Set(linkId, "ChannelSettings", StringValue(channelStrApB[linkId]));
        if (findSubstring(channelStrApB[linkId],"5GHZ"))
        {
            phyApB.AddChannel(spectrumChannel5, freqRangesApB[linkId]);
        }
        else if (findSubstring(channelStrApB[linkId],"6GHZ"))
        {
            phyApB.AddChannel(spectrumChannel6, freqRangesApB[linkId]);
        }
        else if (findSubstring(channelStrApB[linkId],"2_4GHZ"))
        {
            phyApB.AddChannel(spectrumChannel2_4, freqRangesApB[linkId]);
        }
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

    phyApB.Set ("TxPowerStart", DoubleValue (powAp2));
    phyApB.Set ("TxPowerEnd", DoubleValue (powAp2));
    phyApB.Set ("TxPowerLevels", UintegerValue (1));
    phyApB.Set("CcaEdThreshold", DoubleValue(ccaEdTrAp2));
    phyApB.Set("RxSensitivity", DoubleValue(-92.0));

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

    apDevices.Add(wifiApB.Install(phyApB, mac, wifiApNodes.Get(1)));  

    // Set guard interval and MPDU buffer size
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
        TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                UintegerValue(mpduBufferSize));

    // mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(distance, 0.0, 0.0));
    positionAlloc->Add(Vector(0.0, distance, 0.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
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
    ApplicationContainer serverApp;
    ApplicationContainer clientApp;

    uint64_t maxLoad;
    // Calculate maxLoad as
    if (mcs >= 0)
    {
        maxLoad = nLinksSta * EhtPhy::GetDataRate(mcs, channelWidth, NanoSeconds(gi), 1) / nStations;
    }
    else
    {
        maxLoad = nLinksSta * EhtPhy::GetDataRate(13, channelWidth, NanoSeconds(gi), 1) / nStations;
    }

    if (udp)
    {
        // UDP flow
        uint16_t port = 9;
        UdpServerHelper server(port);
        for (std::size_t i = 0; i < nStations; i++)
        {   
            serverApp.Add(server.Install(wifiStaNodes.Get(i)));
        }

        serverApp.Start(Seconds(0.0));
        serverApp.Stop(simulationTime + Seconds(1.0));
        const auto packetInterval = payloadSize * 8.0 / maxLoad;

        for (std::size_t i = 0; i < nStations; i++)
        {
            UdpClientHelper client(staNodeInterfaces.GetAddress(i), port);
            client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
            client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
            client.SetAttribute("PacketSize", UintegerValue(payloadSize));
            
            if (reliabilityMode)
            {
                clientApp.Add(client.Install(wifiApNodes.Get(0)));
            }
            else
            {
                for (std::size_t i = 0; i < nAPs; i++)
                {   
                    clientApp.Add(client.Install(wifiApNodes.Get(i)));
                }
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

    Simulator::Stop(simulationTime + Seconds(1.0));
    Simulator::Run();

    // When multiple stations are used, there are chances that association requests
    // collide and hence the throughput may be lower than expected. Therefore, we relax
    // the check that the throughput cannot decrease by introducing a scaling factor (or
    // tolerance)
    auto tolerance = 0.10;
    cumulRxBytes = GetRxBytes(udp, serverApp, payloadSize);
    auto rxBytes = std::accumulate(cumulRxBytes.cbegin(), cumulRxBytes.cend(), 0.0);
    auto throughput = (rxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s

    std::cout << "RxBytes : " << cumulRxBytes[0] << std::endl;

    std::cout << +mcs << "\t\t\t" << widthStr << " MHz\t\t"
                          << (widthStr.size() > 3 ? "" : "\t") << gi << " ns\t\t\t" << throughput
                          << " Mbit/s" << std::endl;

    Simulator::Destroy();

    return 0;
}
