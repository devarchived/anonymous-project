// filepath: /sequential-waypoint-mobility/model/sequential-waypoint-mobility-model.h
#ifndef SEQUENTIAL_WALK_3D_INDOOR_MOBILITY_MODEL_H
#define SEQUENTIAL_WALK_3D_INDOOR_MOBILITY_MODEL_H

#include "building.h"
#include "factory.h"
#include "wall.h"

#include "ns3/constant-velocity-helper.h"
#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rectangle.h"

namespace ns3 {

class SequentialWalk2dIndoorMobilityModel : public MobilityModel
{
public:
    static TypeId GetTypeId(void);

    void SetWaypoints(const std::vector<Vector>& waypoints);
    std::vector<Vector> GetWaypoints() const;

    void AddWaypoint(const Vector& waypoint);

    void DoWalk();

    void SetFactory(Ptr<Factory> factory);
    void ScheduleNextWaypoint();

    void SetCurrentWaypointIndex(uint32_t index);
    

protected:
    void DoDispose() override;
    void DoInitialize() override;
    Vector DoGetPosition(void) const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity(void) const override;

private:
    void DoInitializePrivate();
    void MoveToNextWaypoint();
    
    std::vector<Vector> m_waypoints;
    Time m_timeInterval;
    uint32_t m_currentWaypointIndex{0};
    Vector m_currentPosition;
    Vector m_currentVelocity;

    ConstantVelocityHelper m_helper;       //!< helper for this object
    EventId m_event;                       //!< stored event ID
    Rectangle m_bounds;                    //!< Bounds of the area to cruise
    Vector m_prevPosition; //!< Store the previous position in case a step back is needed

    Ptr<Factory> m_factory;
};

} // namespace ns3

#endif // SEQUENTIAL_WAYPOINT_MOBILITY_MODEL_H