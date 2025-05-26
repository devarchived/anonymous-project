#include "sequential-walk-2d-indoor-mobility-model.h"

#include "building-list.h"
#include "building.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SequentialWalk2dIndoorMobilityModel");

NS_OBJECT_ENSURE_REGISTERED(SequentialWalk2dIndoorMobilityModel);

TypeId
SequentialWalk2dIndoorMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SequentialWalk2dIndoorMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<SequentialWalk2dIndoorMobilityModel>()
            .AddAttribute("Bounds",
                          "Bounds of the area to cruise.",
                          RectangleValue(Rectangle(0.0, 100.0, 0.0, 100.0)),
                          MakeRectangleAccessor(&SequentialWalk2dIndoorMobilityModel::m_bounds),
                          MakeRectangleChecker())
            .AddAttribute("Factory",
                          "Pointer to the factory object for indoor mobility.",
                          PointerValue(),
                          MakePointerAccessor(&SequentialWalk2dIndoorMobilityModel::SetFactory),
                          MakePointerChecker<Factory>())
            .AddAttribute("WaypointIndex",
                          "The current waypoint index.",
                          UintegerValue(),
                          MakeUintegerAccessor(&SequentialWalk2dIndoorMobilityModel::SetCurrentWaypointIndex),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("TimeInterval",
                          "Time interval for each waypoint.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&SequentialWalk2dIndoorMobilityModel::m_timeInterval),
                          MakeTimeChecker());
    return tid;
}

void
SequentialWalk2dIndoorMobilityModel::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    DoInitializePrivate();
    MobilityModel::DoInitialize();
}

void
SequentialWalk2dIndoorMobilityModel::DoInitializePrivate()
{
    NS_LOG_FUNCTION(this);

    m_helper.Update();
    m_helper.Unpause();

    if (!m_waypoints.empty())
    {
        // SetPosition(m_waypoints[m_currentWaypointIndex]);
        ScheduleNextWaypoint();
    }
}

void
SequentialWalk2dIndoorMobilityModel::ScheduleNextWaypoint()
{
    NS_LOG_FUNCTION(this);

    if (m_currentWaypointIndex < m_waypoints.size())
    {
        NS_LOG_INFO("Moving to waypoint: " << m_currentWaypointIndex);
        Time delay = m_timeInterval;
        m_event = Simulator::Schedule(delay, &SequentialWalk2dIndoorMobilityModel::MoveToNextWaypoint, this);
    }
}

void
SequentialWalk2dIndoorMobilityModel::MoveToNextWaypoint()
{
    NS_LOG_FUNCTION(this);

    if (m_waypoints.empty()) 
    {
        return;
    }
    
    m_currentWaypointIndex = (m_currentWaypointIndex + 1) % m_waypoints.size();
    DoSetPosition(GetWaypoints()[m_currentWaypointIndex]);
}

void
SequentialWalk2dIndoorMobilityModel::SetWaypoints(const std::vector<Vector>& waypoints)
{
    NS_LOG_FUNCTION(this);

    m_waypoints = waypoints;
}

void
SequentialWalk2dIndoorMobilityModel::AddWaypoint(const Vector& waypoint)
{
    NS_LOG_FUNCTION(this);

    m_waypoints.push_back(waypoint);
}

void 
SequentialWalk2dIndoorMobilityModel::SetCurrentWaypointIndex(uint32_t index)
{
    NS_LOG_FUNCTION(this << index);

    if (index < m_waypoints.size())
    {
        m_currentWaypointIndex = index;
        SetPosition(m_waypoints[m_currentWaypointIndex]);
    }
    else
    {
        NS_LOG_WARN("Invalid waypoint index");
    }
}

std::vector<Vector>
SequentialWalk2dIndoorMobilityModel::GetWaypoints() const
{
    NS_LOG_FUNCTION(this);

    return m_waypoints;
}

void
SequentialWalk2dIndoorMobilityModel::DoDispose()
{
    NS_LOG_FUNCTION(this);

    MobilityModel::DoDispose();
}

Vector
SequentialWalk2dIndoorMobilityModel::DoGetPosition() const
{
    NS_LOG_FUNCTION(this);

    m_helper.UpdateWithBounds(m_bounds);
    return m_helper.GetCurrentPosition();
}

void
SequentialWalk2dIndoorMobilityModel::DoSetPosition(const Vector& position)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("New position: " << position);
    std::cout << "New position: " << position << std::endl;

    NS_ASSERT(m_bounds.IsInside(position));
    m_helper.SetPosition(position);
    Simulator::Remove(m_event);
    m_event = Simulator::ScheduleNow(&SequentialWalk2dIndoorMobilityModel::DoInitializePrivate, this);
}

void 
SequentialWalk2dIndoorMobilityModel::SetFactory(Ptr<Factory> factory)
{
    NS_LOG_FUNCTION(this);

    m_factory = factory;
}

Vector
SequentialWalk2dIndoorMobilityModel::DoGetVelocity() const
{
    NS_LOG_FUNCTION(this);
    
    return m_helper.GetVelocity();
}

} // namespace ns3