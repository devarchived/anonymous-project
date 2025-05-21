#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("IndoorSequentialWalkExample");

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
        NS_LOG_ERROR("Cannot open walls file: " << filename);
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

int main(int argc, char *argv[])
{
    LogComponentEnable("IndoorSequentialWalkExample", LOG_LEVEL_LOGIC);

    // Simulation parameters
    uint32_t numNodes = 1; // Number of nodes
    uint32_t seed = 56;    // Random seed
    Time simulationTime{"20s"};

    // create a grid of buildings
    double factorySizeX = 20; // m
    double factorySizeY = 20;  // m
    double factoryHeight = 3; // m

    // Wi-Fi parameters
    double frequency = 5.945e9; //Hz option : 5.945e9 5.975e9

    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    Ptr<Factory> factory = CreateObject<Factory>();
    factory->SetBuildingType(Building::Office);
    factory->SetBoundaries(Box(0, factorySizeX, 0, factorySizeY, 0, factoryHeight));
    factory->SetIntWallsType(Building::ConcreteWithoutWindows);
    factory->SetNFloors(1);
    factory->SetNRoomsX(1);
    factory->SetNRoomsY(1);

    Ptr<Wall> wall1 = CreateObject<Wall>();
    wall1->SetBoundaries(Box(0.5, 4.5, 0.5, 0.5, 0.0, factoryHeight));
    Ptr<Wall> wall2 = CreateObject<Wall>();
    wall2->SetBoundaries(Box(0.5, 0.5, 0.5, 4.5, 0.0, factoryHeight));
    Ptr<Wall> wall3 = CreateObject<Wall>();
    wall3->SetBoundaries(Box(0.5, 4.5, 4.5, 4.5, 0.0, factoryHeight));
    Ptr<Wall> wall4 = CreateObject<Wall>();
    wall4->SetBoundaries(Box(4.5, 4.5, 0.5, 4.5, 0.0, factoryHeight));

    // Add walls to the factory
    factory->AddWallToRoom(1, 1, wall1);
    factory->AddWallToRoom(1, 1, wall2);
    factory->AddWallToRoom(1, 1, wall3);
    factory->AddWallToRoom(1, 1, wall3);


    // AddWallsFromFile(factory, "walls.txt");

    // // print the list of the office and walls to file
    PrintGnuplottableBuildingListToFile("indoor-sequential-layout.txt");

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
    mobilitySta.Install(wifiStas);
    building.Install(wifiStas);

    Ptr<ListPositionAllocator> apPosition = CreateObject<ListPositionAllocator>();
    apPosition->Add(Vector(5.0, 5.0, factoryHeight));
    apPosition->Add(Vector(15.0, 5.0, factoryHeight));
    mobility.SetPositionAllocator(apPosition);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiAps);
    building.Install(wifiAps);

    // Set waypoints programmatically
    auto mob = wifiStas.Get(0)->GetObject<SequentialWalk2dIndoorMobilityModel>();
    std::vector<Vector> waypoints;
    waypoints = ReadWaypointsFromFile("waypoints.txt");
    if (waypoints.empty())
    {
        waypoints = {
            Vector(0.25, 0.25, 1.5),
            Vector(5.0, 0.25, 1.5),
            Vector(10.0, 0.25, 1.5),
            Vector(15.0, 0.25, 1.5),
            Vector(19.75, 0.25, 1.5)
        };
    }
    mob->SetWaypoints(waypoints);
    mob->SetCurrentWaypointIndex(0);

    auto lossModel = CreateObject<IndoorBuildingsPropagationLossModel>();
    lossModel->SetBuildingType(Building::Office);
    lossModel->SetFrequency(frequency);
    lossModel->SetFactoryType("InF-SL");

    // // enable the traces for the mobility model
    AsciiTraceHelper ascii;
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream("indoor-sequential-mobility-trace-example.mob"));
    
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