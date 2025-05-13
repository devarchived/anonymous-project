
 #include "factory.h"

 #include "building-list.h"
 
 #include <ns3/assert.h>
 #include <ns3/enum.h>
 #include <ns3/log.h>
 #include <ns3/uinteger.h>
 
 #include <cmath>
 #include <set>     
 
 namespace ns3
 {
 
 NS_LOG_COMPONENT_DEFINE("Factory");
 
 NS_OBJECT_ENSURE_REGISTERED(Factory);
 
TypeId
Factory::GetTypeId()
{
    static TypeId tid = TypeId("ns3::Factory")
        .SetParent<Building>() // Inherit from Building
        .SetGroupName("Buildings")
        .AddConstructor<Factory>()
        .AddAttribute("InternalWallsType",
            "The type of material of which the internal walls are made",
            EnumValue(Building::ConcreteWithoutWindows),
            MakeEnumAccessor<ExtWallsType_t>(&Building::GetExtWallsType,
                                            &Building::SetExtWallsType),
            MakeEnumChecker(Building::Wood,
                            "Wood",
                            Building::ConcreteWithWindows,
                            "ConcreteWithWindows",
                            Building::ConcreteWithoutWindows,
                            "ConcreteWithoutWindows",
                            Building::StoneBlocks,
                            "StoneBlocks"));
    return tid;
}

Factory::Factory()
        : Building(),
          m_interiorWallType(Building::ConcreteWithoutWindows)
{
    NS_LOG_INFO("Default Factory created.");
}

Factory::~Factory()
{
    NS_LOG_FUNCTION(this);
}

void
Factory::SetIntWallsType(Building::ExtWallsType_t t)
{
    NS_LOG_FUNCTION(this << t);
    m_interiorWallType = t;
}

void 
Factory::AddIntWall(Ptr<Wall> wall)
{
    NS_LOG_FUNCTION(this << wall);

    auto it = std::find(m_interiorWalls.begin(), m_interiorWalls.end(), wall);
    if (it == m_interiorWalls.end())
    {
        // Add the wall only if it does not already exist
        m_interiorWalls.push_back(wall);
        NS_LOG_LOGIC("Wall added to m_interiorWalls: " << wall->GetBoundaries());
    }
    else
    {
        NS_LOG_LOGIC("Wall already exists in m_interiorWalls: " << wall->GetBoundaries());
    }
}

void
Factory::AddWallToRoom(uint16_t roomX, uint16_t roomY, Ptr<Wall> wall)
{
    std::pair<uint16_t, uint16_t> roomKey = std::make_pair(roomX, roomY);

    AddIntWall(wall);
    m_roomWalls[roomKey].push_back(wall);

    NS_LOG_LOGIC("Added wall to room (" << roomX << ", " << roomY << ")"
                 << " with boundaries: " << wall->GetBoundaries());
}

uint16_t
Factory::CountWallsBetweenPoints(Vector pointA, Vector pointB) const
{
    NS_LOG_FUNCTION(this << pointA << pointB);

    uint16_t wallCount = 0;
    std::set<Ptr<Wall>> countedWalls; // Track unique walls

    // Iterate through all rooms
    for (const auto& [roomKey, walls] : m_roomWalls)
    {
        for (const auto& wall : walls)
        {
            // Check if the wall intersects the line between pointA and pointB
            if (countedWalls.find(wall) == countedWalls.end() && wall->IsIntersect(pointA, pointB))
            {
                wallCount++;
                countedWalls.insert(wall); // Mark wall as processed
            }
        }
    }

    NS_LOG_LOGIC("Number of walls between points: " << wallCount);
    return wallCount;
}

Building::ExtWallsType_t
Factory::GetIntWallsType() const
{
    return (m_interiorWallType);
}

std::vector<Ptr<Wall>>& 
Factory::GetIntWalls()
{
    return (m_interiorWalls);
}

uint16_t
Factory::GetRoomX(Vector position) const
{
    NS_ASSERT(IsInside(position));

    uint16_t roomX = 0;

    for (const auto& [roomKey, walls] : m_roomWalls)
    {
        bool leftWallFound = false;
        bool rightWallFound = false;

        for (const auto& wall : walls)
        {
            if (position.x > wall->GetBoundaries().xMax) // Left wall
            {
                leftWallFound = true;
            }
            if (position.x < wall->GetBoundaries().xMin) // Right wall
            {
                rightWallFound = true;
            }
        }

        // Prioritize finding two walls that enclose the position
        if (leftWallFound && rightWallFound)
        {
            roomX = roomKey.first;
            break;
        }
    }

    // If no enclosing walls are found, find the closest single wall
    if (roomX == 0)
    {
        double minDistance = std::numeric_limits<double>::max();

        for (const auto& [roomKey, walls] : m_roomWalls)
        {
            for (const auto& wall : walls)
            {
                double distance = std::abs(position.x - wall->GetBoundaries().xMin);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    roomX = roomKey.first;
                }
            }
        }
    }

    NS_LOG_LOGIC("RoomX: " << roomX);
    return roomX;
}

uint16_t
Factory::GetRoomY(Vector position) const
{
    NS_ASSERT(IsInside(position));

    uint16_t roomY = 0;

    for (const auto& [roomKey, walls] : m_roomWalls)
    {
        bool bottomWallFound = false;
        bool topWallFound = false;

        for (const auto& wall : walls)
        {
            if (position.y > wall->GetBoundaries().yMax) // Bottom wall
            {
                bottomWallFound = true;
            }
            if (position.y < wall->GetBoundaries().yMin) // Top wall
            {
                topWallFound = true;
            }
        }

        // Prioritize finding two walls that enclose the position
        if (bottomWallFound && topWallFound)
        {
            roomY = roomKey.second;
            break;
        }
    }

    // If no enclosing walls are found, find the closest single wall
    if (roomY == 0)
    {
        double minDistance = std::numeric_limits<double>::max();

        for (const auto& [roomKey, walls] : m_roomWalls)
        {
            for (const auto& wall : walls)
            {
                double distance = std::abs(position.y - wall->GetBoundaries().yMin);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    roomY = roomKey.second;
                }
            }
        }
    }

    NS_LOG_LOGIC("RoomY: " << roomY);
    return roomY;
}
 
} // namespace ns3
 