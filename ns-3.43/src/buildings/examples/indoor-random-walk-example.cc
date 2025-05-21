#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("IndoorRandomWalkExample");

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
MonitorPathLoss(Ptr<IndoorBuildingsPropagationLossModel> lossModel, NodeContainer wifiAps, NodeContainer wifiStas) 
{
    double lossAp1 = lossModel->GetLoss(wifiAps.Get(0)->GetObject<MobilityModel>(), wifiStas.Get(0)->GetObject<MobilityModel>());
    double lossAp2 = lossModel->GetLoss(wifiAps.Get(1)->GetObject<MobilityModel>(), wifiStas.Get(0)->GetObject<MobilityModel>());
    std::cout << "Loss AP1: " << lossAp1 << " dB" << std::endl;
    std::cout << "Loss AP2: " << lossAp2 << " dB" << std::endl;

    // Schedule the next callback
    Simulator::Schedule(Seconds(1), &MonitorPathLoss, lossModel, wifiAps, wifiStas);
}

/**
 * This is an example on how to use the RandomWalk2dIndoorMobilityModel class.
 * The script indoor-random-walk-example.sh can be used to visualize the
 * positions visited by the random walk.
 */
int
main(int argc, char* argv[])
{
    LogComponentEnable("RandomWalk2dIndoor", LOG_LEVEL_LOGIC);

    // Simulation parameters
    uint32_t numNodes = 1; // Number of nodes
    uint32_t seed = 56;    // Random seed
    Time simulationTime{"100s"};

    // Wi-Fi parameters
    double frequency = 5.945e9; //Hz option : 5.945e9 5.975e9

    // create a grid of buildings
    double factorySizeX = 40; // m
    double factorySizeY = 20;  // m
    double factoryHeight = 25; // m

    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    Ptr<Factory> factory = CreateObject<Factory>();
    factory->SetBuildingType(Building::Factory);
    factory->SetBoundaries(Box(0, factorySizeX, 0, factorySizeY, 0, factoryHeight));
    factory->SetIntWallsType(Building::ConcreteWithoutWindows);
    factory->SetNFloors(1);
    factory->SetNRoomsX(3);
    factory->SetNRoomsY(1);

    Ptr<Wall> wall1 = CreateObject<Wall>();
    wall1->SetBoundaries(Box(5.0, 5.0, 5.0, 15.0, 0.0, factoryHeight));
    Ptr<Wall> wall2 = CreateObject<Wall>();
    wall2->SetBoundaries(Box(5.0, 15.0, 15.0, 15.0, 0.0, factoryHeight));
    Ptr<Wall> wall3 = CreateObject<Wall>();
    wall3->SetBoundaries(Box(15.0, 15.0, 5.0, 15.0, 0.0, factoryHeight));
    Ptr<Wall> wall4 = CreateObject<Wall>();
    wall4->SetBoundaries(Box(15.0, 25.0, 5.0, 5.0, 0.0, factoryHeight));
    Ptr<Wall> wall5 = CreateObject<Wall>();
    wall5->SetBoundaries(Box(25.0, 25.0, 5.0, 15.0, 0.0, factoryHeight));
    Ptr<Wall> wall6 = CreateObject<Wall>();
    wall6->SetBoundaries(Box(25.0, 35.0, 15.0, 15.0, 0.0, factoryHeight));
    Ptr<Wall> wall7 = CreateObject<Wall>();
    wall7->SetBoundaries(Box(35.0, 35.0, 5.0, 15.0, 0.0, factoryHeight));

    // Add Wall 8 - 10 to make node move in a hallway
    // Ptr<Wall> wall8 = CreateObject<Wall>();
    // wall8->SetBoundaries(Box(5.0, 15.0, 5.0, 5.0, 0.0, factoryHeight));
    // Ptr<Wall> wall9 = CreateObject<Wall>();
    // wall9->SetBoundaries(Box(15.0, 25.0, 15.0, 15.0, 0.0, factoryHeight));
    // Ptr<Wall> wall10 = CreateObject<Wall>();
    // wall10->SetBoundaries(Box(25.0, 35.0, 5.0, 5.0, 0.0, factoryHeight));
            
    // Add walls to the factory
    factory->AddWallToRoom(1, 1, wall1);
    factory->AddWallToRoom(1, 1, wall2);
    factory->AddWallToRoom(1, 1, wall3);
    factory->AddWallToRoom(2, 1, wall3);
    factory->AddWallToRoom(2, 1, wall4);
    factory->AddWallToRoom(2, 1, wall5);
    factory->AddWallToRoom(3, 1, wall5);
    factory->AddWallToRoom(3, 1, wall6);
    factory->AddWallToRoom(3, 1, wall7);
    // factory->AddWallToRoom(1, 1, wall8);
    // factory->AddWallToRoom(2, 1, wall9);
    // factory->AddWallToRoom(3, 1, wall10);

    // // print the list of the office and walls to file
    PrintGnuplottableBuildingListToFile("indoor-layout.txt");

    // create one STA node
    NodeContainer wifiStas;
    wifiStas.Create(numNodes);
    NodeContainer wifiAps;
    wifiAps.Create(2);

    RngSeedManager::SetSeed(seed);

    MobilityHelper mobility;
    MobilityHelper mobilitySta;
    BuildingsHelper building;

    mobilitySta.SetMobilityModel(
        "ns3::RandomWalk2dIndoorMobilityModel",
        "Mode", StringValue("Time"),
        "Time", StringValue("1s"),
        "Speed", StringValue("ns3::NormalRandomVariable[Mean=5|Variance=0.20401]"),
        "Bounds", RectangleValue(Rectangle(0, factorySizeX, 0.0, factorySizeY)),
        "Factory", PointerValue(factory));
    Ptr<IndoorPositionAllocator> position = CreateObject<IndoorPositionAllocator>();
    Ptr<UniformRandomVariable> xPos = CreateObject<UniformRandomVariable>();
    xPos->SetAttribute("Min", DoubleValue(0.0));
    xPos->SetAttribute("Max", DoubleValue(factorySizeX));
    Ptr<UniformRandomVariable> yPos = CreateObject<UniformRandomVariable>();
    yPos->SetAttribute("Min", DoubleValue(0.0));
    yPos->SetAttribute("Max", DoubleValue(factorySizeY));
    Ptr<UniformRandomVariable> zPos = CreateObject<UniformRandomVariable>();
    zPos->SetAttribute("Min", DoubleValue(5.0));
    zPos->SetAttribute("Max", DoubleValue(5.0));
    position->SetAttribute("X", PointerValue(xPos));
    position->SetAttribute("Y", PointerValue(yPos));
    position->SetAttribute("Z", PointerValue(zPos));
    mobilitySta.SetPositionAllocator(position);
    mobilitySta.Install(wifiStas);
    building.Install(wifiStas);

    Ptr<ListPositionAllocator> apPosition = CreateObject<ListPositionAllocator>();
    apPosition->Add(Vector(10.0, 10.0, 10.0));
    apPosition->Add(Vector(30.0, 10.0, 10.0));
    mobility.SetPositionAllocator(apPosition);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiAps);
    building.Install(wifiAps);

    auto lossModel = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel->SetBuildingType(Building::Factory);
    lossModel->SetFrequency(frequency);
    lossModel->SetFactoryType("InF-SL");

    // auto spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    // spectrumChannel->AddPropagationLossModel(lossModel);

    // // enable the traces for the mobility model
    AsciiTraceHelper ascii;
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream("indoor-mobility-trace-example.mob"));
    
    double lossAp1 = lossModel->GetLoss(wifiAps.Get(0)->GetObject<MobilityModel>(), wifiStas.Get(0)->GetObject<MobilityModel>());
    double lossAp2 = lossModel->GetLoss(wifiAps.Get(1)->GetObject<MobilityModel>(), wifiStas.Get(0)->GetObject<MobilityModel>());
    std::cout << "Loss AP1: " << lossAp1 << " dB" << std::endl;
    std::cout << "Loss AP2: " << lossAp2 << " dB" << std::endl;

    // Simulation clock
    Simulator::Schedule (MicroSeconds (100), &TimePasses);
    Simulator::Schedule(Seconds(1), &MonitorPathLoss, lossModel, wifiAps, wifiStas);

    Simulator::Stop(simulationTime);

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
