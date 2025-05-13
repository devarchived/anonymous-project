
#include "random-walk-2d-indoor-mobility-model.h"

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

NS_LOG_COMPONENT_DEFINE("RandomWalk2dIndoor");

NS_OBJECT_ENSURE_REGISTERED(RandomWalk2dIndoorMobilityModel);

TypeId
RandomWalk2dIndoorMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::RandomWalk2dIndoorMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<RandomWalk2dIndoorMobilityModel>()
            .AddAttribute("Bounds",
                          "Bounds of the area to cruise.",
                          RectangleValue(Rectangle(0.0, 100.0, 0.0, 100.0)),
                          MakeRectangleAccessor(&RandomWalk2dIndoorMobilityModel::m_bounds),
                          MakeRectangleChecker())
            .AddAttribute("Time",
                          "Change current direction and speed after moving for this delay.",
                          TimeValue(Seconds(20.0)),
                          MakeTimeAccessor(&RandomWalk2dIndoorMobilityModel::m_modeTime),
                          MakeTimeChecker())
            .AddAttribute("Distance",
                          "Change current direction and speed after moving for this distance.",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&RandomWalk2dIndoorMobilityModel::m_modeDistance),
                          MakeDoubleChecker<double>())
            .AddAttribute("Mode",
                          "The mode indicates the condition used to "
                          "change the current speed and direction",
                          EnumValue(RandomWalk2dIndoorMobilityModel::MODE_DISTANCE),
                          MakeEnumAccessor<Mode>(&RandomWalk2dIndoorMobilityModel::m_mode),
                          MakeEnumChecker(RandomWalk2dIndoorMobilityModel::MODE_DISTANCE,
                                          "Distance",
                                          RandomWalk2dIndoorMobilityModel::MODE_TIME,
                                          "Time"))
            .AddAttribute("Direction",
                          "A random variable used to pick the direction (radians).",
                          StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                          MakePointerAccessor(&RandomWalk2dIndoorMobilityModel::m_direction),
                          MakePointerChecker<RandomVariableStream>())
            .AddAttribute(
                "Speed",
                "A random variable used to pick the speed (m/s)."
                "The default value is taken from Figure 1 of the paper"
                "Henderson, L.F., 1971. The statistics of crowd fluids. nature, 229(5284), p.381.",
                StringValue("ns3::NormalRandomVariable[Mean=1.53|Variance=0.040401]"),
                MakePointerAccessor(&RandomWalk2dIndoorMobilityModel::m_speed),
                MakePointerChecker<RandomVariableStream>())
            .AddAttribute("Tolerance",
                          "Tolerance for the intersection point with walls (m)."
                          "It represents a small distance from where the walls limit"
                          "is actually placed, for example to represent a sidewalk.",
                          DoubleValue(1e-6),
                          MakeDoubleAccessor(&RandomWalk2dIndoorMobilityModel::m_epsilon),
                          MakeDoubleChecker<double>())
            .AddAttribute("MaxIterations",
                          "Maximum number of attempts to find an alternative next position"
                          "if the original one is inside a wall.",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&RandomWalk2dIndoorMobilityModel::m_maxIter),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("Factory",
                          "Pointer to the factory object for indoor mobility.",
                          PointerValue(),
                          MakePointerAccessor(&RandomWalk2dIndoorMobilityModel::SetFactory),
                          MakePointerChecker<Factory>());
    return tid;
}

void
RandomWalk2dIndoorMobilityModel::DoInitialize()
{
    DoInitializePrivate();
    MobilityModel::DoInitialize();
}

void
RandomWalk2dIndoorMobilityModel::DoInitializePrivate()
{
    m_helper.Update();
    double speed = m_speed->GetValue();
    double direction = m_direction->GetValue();
    Vector velocity(std::cos(direction) * speed, std::sin(direction) * speed, 0.0);
    m_helper.SetVelocity(velocity);
    m_helper.Unpause();

    Time delayLeft;
    if (m_mode == RandomWalk2dIndoorMobilityModel::MODE_TIME)
    {
        delayLeft = m_modeTime;
    }
    else
    {
        delayLeft = Seconds(m_modeDistance / speed);
    }
    DoWalk(delayLeft);
}

void
RandomWalk2dIndoorMobilityModel::DoWalk(Time delayLeft)
{
    if (delayLeft.IsNegative())
    {
        NS_LOG_INFO(this << " Ran out of time");
        return;
    }
    NS_LOG_FUNCTION(this << delayLeft.GetSeconds());

    Vector position = m_helper.GetCurrentPosition();
    Vector velocity = m_helper.GetVelocity();
    Vector nextPosition = position;
    nextPosition.x += velocity.x * delayLeft.GetSeconds();
    nextPosition.y += velocity.y * delayLeft.GetSeconds();
    m_event.Cancel();

    // check if the nextPosition is inside a wall, or if the line
    // from position to the next position intersects a wall
    auto indoorWall = IsLineClearOfWalls(position, nextPosition);
    bool isClear = std::get<0>(indoorWall);
    Ptr<Wall> intersectingWall = std::get<1>(indoorWall);

    if (m_bounds.IsInside(nextPosition))
    {
        if (isClear)
        {
            m_event = Simulator::Schedule(delayLeft,
                                          &RandomWalk2dIndoorMobilityModel::DoInitializePrivate,
                                          this);
        }
        else
        {
            NS_LOG_LOGIC("NextPosition would lead into a wall");
            nextPosition =
                CalculateIntersectionFromOutside(position, nextPosition, intersectingWall->GetBoundaries());

            // double delaySecondsX = std::numeric_limits<double>::max();
            // double delaySecondsY = std::numeric_limits<double>::max();
            // if (velocity.x != 0)
            // {
            //     delaySecondsX = std::abs((nextPosition.x - position.x) / velocity.x);
            // }
            // if (velocity.y != 0)
            // {
            //     delaySecondsY = std::abs((nextPosition.y - position.y) / velocity.y);
            // }
            // Time delay = Seconds(std::min(delaySecondsX, delaySecondsY));
            // m_event = Simulator::Schedule(delay,
            //                               &RandomWalk2dIndoorMobilityModel::AvoidWall,
            //                               this,
            //                               delayLeft - delay,
            //                               nextPosition);
            m_event = Simulator::ScheduleNow(&RandomWalk2dIndoorMobilityModel::AvoidWall,
                                            this,
                                            delayLeft,
                                            nextPosition);
        }
    }
    else
    {
        NS_LOG_LOGIC("Out of bounding box");
        nextPosition = m_bounds.CalculateIntersection(position, velocity);
        // check that this nextPosition is clear of walls
        auto wallIntersection = IsLineClearOfWalls(position, nextPosition);
        bool isClear = std::get<0>(wallIntersection);
        Ptr<Wall> intersectingWall = std::get<1>(wallIntersection);

        if (isClear)
        {
            double delaySeconds = std::numeric_limits<double>::max();
            if (velocity.x != 0)
            {
                delaySeconds =
                    std::min(delaySeconds, std::abs((nextPosition.x - position.x) / velocity.x));
            }
            else if (velocity.y != 0)
            {
                delaySeconds =
                    std::min(delaySeconds, std::abs((nextPosition.y - position.y) / velocity.y));
            }
            else
            {
                NS_ABORT_MSG("RandomWalk2dIndoorMobilityModel::DoWalk: unable to calculate the "
                             "rebound time "
                             "(the node is stationary).");
            }
            Time delay = Seconds(delaySeconds);
            m_event = Simulator::Schedule(delay,
                                          &RandomWalk2dIndoorMobilityModel::Rebound,
                                          this,
                                          delayLeft - delay);
        }
        else
        {
            NS_LOG_LOGIC("NextPosition would lead into a wall");
            if (intersectingWall != nullptr)
            {
                nextPosition = CalculateIntersectionFromOutside(position, nextPosition, intersectingWall->GetBoundaries());
            }
            else
            {
                NS_LOG_ERROR("Intersecting wall is null.");
            }

            double delaySecondsX = std::numeric_limits<double>::max();
            double delaySecondsY = std::numeric_limits<double>::max();
            if (velocity.x != 0)
            {
                delaySecondsX =
                    std::min(delaySecondsX, std::abs((nextPosition.x - position.x) / velocity.x));
            }
            if (velocity.y != 0)
            {
                delaySecondsY =
                    std::min(delaySecondsY, std::abs((nextPosition.y - position.y) / velocity.y));
            }
            if (delaySecondsX == std::numeric_limits<double>::max() &&
                delaySecondsY == std::numeric_limits<double>::max())
            {
                NS_ABORT_MSG("RandomWalk2dIndoorMobilityModel::DoWalk: unable to calculate the "
                             "rebound time "
                             "(the node is stationary).");
            }

            Time delay = Seconds(std::min(delaySecondsX, delaySecondsY));
            m_event = Simulator::Schedule(delay,
                                          &RandomWalk2dIndoorMobilityModel::AvoidWall,
                                          this,
                                          delayLeft - delay,
                                          nextPosition);
        }
    }
    NS_LOG_LOGIC("Position " << position << " NextPosition " << nextPosition);

    // store the previous position
    m_prevPosition = position;
    NotifyCourseChange();
}

std::pair<bool, Ptr<Wall>>
RandomWalk2dIndoorMobilityModel::IsLineClearOfWalls(Vector currentPosition,
                                                         Vector nextPosition) const
{
    NS_LOG_FUNCTION(this << currentPosition << nextPosition);

    bool intersectWall = false;
    double minIntersectionDistance = std::numeric_limits<double>::max();
    Ptr<Wall> minIntersectionDistanceWall;

    for (const auto& wall : m_factory->GetIntWalls())
    {
        // check if this wall intersects the line between the current and next positions
        // this checks also if the next position is clear of walls
        if (wall->IsIntersect(currentPosition, nextPosition))
        {
            NS_LOG_LOGIC("Wall " << wall << " intersects the line between "
                                 << currentPosition << " and " << nextPosition);
            auto intersection = CalculateIntersectionFromOutside(currentPosition,
                                                                 nextPosition,
                                                                 wall->GetBoundaries());
            double distance = CalculateDistance(intersection, currentPosition);
            intersectWall = true;
            if (distance < minIntersectionDistance)
            {
                minIntersectionDistance = distance;
                minIntersectionDistanceWall = wall;
            }
        }
    }

    return std::make_pair(!intersectWall, minIntersectionDistanceWall);
}

Vector
RandomWalk2dIndoorMobilityModel::CalculateIntersectionFromOutside(const Vector& current,
                                                                   const Vector& next,
                                                                   Box boundaries) const
{
    NS_LOG_FUNCTION(this << " current " << current << " next " << next);
    bool inside = boundaries.IsInside(current);
    NS_ASSERT(!inside);

    // get the closest side
    Rectangle rect = Rectangle(boundaries.xMin, boundaries.xMax, boundaries.yMin, boundaries.yMax);
    NS_LOG_DEBUG("rect " << rect);
    Rectangle::Side closestSide = rect.GetClosestSideOrCorner(current);

    double xIntersect = 0;
    double yIntersect = 0;

    switch (closestSide)
    {
    case Rectangle::RIGHTSIDE:
        NS_LOG_DEBUG("The closest side is RIGHT");
        xIntersect = boundaries.xMax + m_epsilon;
        NS_ABORT_MSG_IF(next.x - current.x == 0, "x position not updated");
        yIntersect =
            (next.y - current.y) / (next.x - current.x) * (xIntersect - current.x) + current.y;
        break;
    case Rectangle::LEFTSIDE:
        NS_LOG_DEBUG("The closest side is LEFT");
        xIntersect = boundaries.xMin - m_epsilon;
        NS_ABORT_MSG_IF(next.x - current.x == 0, "x position not updated");
        yIntersect =
            (next.y - current.y) / (next.x - current.x) * (xIntersect - current.x) + current.y;
        break;
    case Rectangle::TOPSIDE:
        NS_LOG_DEBUG("The closest side is TOP");
        yIntersect = boundaries.yMax + m_epsilon;
        NS_ABORT_MSG_IF(next.y - current.y == 0, "y position not updated");
        xIntersect =
            (next.x - current.x) / (next.y - current.y) * (yIntersect - current.y) + current.x;
        break;
    case Rectangle::BOTTOMSIDE:
        NS_LOG_DEBUG("The closest side is BOTTOM");
        yIntersect = boundaries.yMin - m_epsilon;
        NS_ABORT_MSG_IF(next.y - current.y == 0, "y position not updated");
        xIntersect =
            (next.x - current.x) / (next.y - current.y) * (yIntersect - current.y) + current.x;
        break;
    case Rectangle::TOPRIGHTCORNER:
        NS_LOG_DEBUG("The closest side is TOPRIGHT");
        xIntersect = boundaries.xMax + m_epsilon;
        NS_ABORT_MSG_IF(next.x - current.x == 0, "x position not updated");
        yIntersect = boundaries.yMax + m_epsilon;
        NS_ABORT_MSG_IF(next.y - current.y == 0, "y position not updated");
        break;
    case Rectangle::TOPLEFTCORNER:
        NS_LOG_DEBUG("The closest side is TOPLEFT");
        xIntersect = boundaries.xMin - m_epsilon;
        NS_ABORT_MSG_IF(next.x - current.x == 0, "x position not updated");
        yIntersect = boundaries.yMax + m_epsilon;
        NS_ABORT_MSG_IF(next.y - current.y == 0, "y position not updated");
        break;
    case Rectangle::BOTTOMRIGHTCORNER:
        NS_LOG_DEBUG("The closest side is BOTTOMRIGHT");
        xIntersect = boundaries.xMax + m_epsilon;
        NS_ABORT_MSG_IF(next.x - current.x == 0, "x position not updated");
        yIntersect = boundaries.yMin - m_epsilon;
        NS_ABORT_MSG_IF(next.y - current.y == 0, "y position not updated");
        break;
    case Rectangle::BOTTOMLEFTCORNER:
        NS_LOG_DEBUG("The closest side is BOTTOMLEFT");
        xIntersect = boundaries.xMin - m_epsilon;
        NS_ABORT_MSG_IF(next.x - current.x == 0, "x position not updated");
        yIntersect = boundaries.yMin - m_epsilon;
        NS_ABORT_MSG_IF(next.y - current.y == 0, "y position not updated");
        break;
    }
    NS_LOG_DEBUG("xIntersect " << xIntersect << " yIntersect " << yIntersect);
    return Vector(xIntersect, yIntersect, 0);
}

void
RandomWalk2dIndoorMobilityModel::Rebound(Time delayLeft)
{
    NS_LOG_FUNCTION(this << delayLeft.GetSeconds());
    m_helper.UpdateWithBounds(m_bounds);
    Vector position = m_helper.GetCurrentPosition();
    Vector velocity = m_helper.GetVelocity();
    switch (m_bounds.GetClosestSideOrCorner(position))
    {
    case Rectangle::RIGHTSIDE:
    case Rectangle::LEFTSIDE:
        NS_LOG_DEBUG("The closest side is RIGHT or LEFT");
        velocity.x = -velocity.x;
        break;
    case Rectangle::TOPSIDE:
    case Rectangle::BOTTOMSIDE:
        NS_LOG_DEBUG("The closest side is TOP or BOTTOM");
        velocity.y = -velocity.y;
        break;
    case Rectangle::TOPRIGHTCORNER:
    case Rectangle::BOTTOMRIGHTCORNER:
    case Rectangle::TOPLEFTCORNER:
    case Rectangle::BOTTOMLEFTCORNER:
        NS_LOG_DEBUG("The closest side is a corner");
        velocity.x = -velocity.x;
        velocity.y = -velocity.y;
        break;
    }
    m_helper.SetVelocity(velocity);
    m_helper.Unpause();
    DoWalk(delayLeft);
}

void
RandomWalk2dIndoorMobilityModel::AvoidWall(Time delayLeft, Vector intersectPosition)
{
    NS_LOG_FUNCTION(this << delayLeft.GetSeconds());
    m_helper.Update();

    bool nextWouldBeInside = true;
    uint32_t iter = 0;

    while (nextWouldBeInside && iter < m_maxIter)
    {
        NS_LOG_INFO("The next position would be inside a wall, compute an alternative");
        iter++;
        
        // Check if we have reached a quarter of the maximum iterations
        if (iter >= m_maxIter / 4)
        {
            NS_LOG_INFO("Reached a quarter of max iterations, invoking WallRebound.");
            Vector position = m_helper.GetCurrentPosition();
            auto wallIntersection = IsLineClearOfWalls(position, intersectPosition);
            Ptr<Wall> intersectingWall = std::get<1>(wallIntersection);

            if (intersectingWall != nullptr)
            {
                NS_LOG_INFO("Invoking WallRebound with the interctingWall");
                WallRebound(delayLeft, intersectingWall);
                return; // Exit the function after invoking WallRebound
            }
            else
            {
                NS_LOG_ERROR("No intersecting wall found, unable to perform WallRebound.");
                break; // Exit the loop if no wall is found
            }
        }

        double speed = m_speed->GetValue();
        double direction = m_direction->GetValue();
        Vector velocityVector(std::cos(direction) * speed, std::sin(direction) * speed, 0.0);
        m_helper.SetVelocity(velocityVector);

        Vector nextPosition = intersectPosition;
        nextPosition.x += velocityVector.x * delayLeft.GetSeconds();
        nextPosition.y += velocityVector.y * delayLeft.GetSeconds();

        // check if this is inside the current wall
        auto wallIntersection = IsLineClearOfWalls(intersectPosition, nextPosition);
        bool isClear = std::get<0>(wallIntersection);
        Ptr<Wall> intersectingWall = std::get<1>(wallIntersection);

        
        if (!isClear)
        {
            NS_LOG_LOGIC("inside loop intersect " << intersectPosition << " nextPosition "
                                                  << nextPosition << " intersects wall "
                                                  << std::get<1>(wallIntersection));
        }
        else
        {
            NS_LOG_LOGIC("inside loop intersect " << intersectPosition << " nextPosition "
                                                  << nextPosition << " is clear of walls");
        }

        if (isClear && m_bounds.IsInside(nextPosition))
        {
            nextWouldBeInside = false;
        }
    }

    // after m_maxIter iterations, the positions tested are all inside walls
    // to avoid increasing m_maxIter too much, it is possible to perform a step back
    // to the previous position and continue from there
    if (iter >= m_maxIter)
    {
        NS_LOG_INFO("Move back to the previous position");

        // compute the difference between the previous position and the intersection
        Vector posDiff = m_prevPosition - intersectPosition;
        // compute the distance
        double distance = CalculateDistance(m_prevPosition, intersectPosition);
        double speed = distance / delayLeft.GetSeconds(); // compute the speed

        NS_LOG_LOGIC("prev " << m_prevPosition << " intersectPosition " << intersectPosition
                             << " diff " << posDiff << " dist " << distance);

        Vector velocityVector(posDiff.x / distance * speed, posDiff.y / distance * speed, 0.0);
        m_helper.SetVelocity(velocityVector);

        Vector nextPosition = intersectPosition;
        nextPosition.x += velocityVector.x * delayLeft.GetSeconds();
        nextPosition.y += velocityVector.y * delayLeft.GetSeconds();

        // check if the path is clear
        auto wallIntersection = IsLineClearOfWalls(intersectPosition, nextPosition);
        bool isClear = std::get<0>(wallIntersection);
        Ptr<Wall> intersectingWall = std::get<1>(wallIntersection);

        if (!isClear)
        {
            NS_LOG_LOGIC("The position is still inside after "
                         << m_maxIter + 1 << " iterations, loop intersect " << intersectPosition
                         << " nextPosition " << nextPosition << " intersects wall "
                         << std::get<1>(wallIntersection));
            // This error may be due to walls being attached to one another, or to the boundary
            // of the scenario.
            NS_FATAL_ERROR(
                "Not able to find a clear position. Try to increase the attribute MaxIterations "
                "and check the position of the walls in the scenario.");
        }
        else
        {
            NS_LOG_LOGIC("inside loop intersect " << intersectPosition << " nextPosition "
                                                  << nextPosition << " is clear of walls");
        }
    }

    m_helper.Unpause();

    DoWalk(delayLeft);
}

// void
// RandomWalk2dIndoorMobilityModel::DoWalk(Time delayLeft)
// {
//     if (delayLeft.IsNegative())
//     {
//         NS_LOG_INFO(this << " Ran out of time");
//         return;
//     }
//     NS_LOG_FUNCTION(this << delayLeft.GetSeconds());

//     Vector position = m_helper.GetCurrentPosition();
//     Vector velocity = m_helper.GetVelocity();
//     Vector nextPosition = position;
//     nextPosition.x += velocity.x * delayLeft.GetSeconds();
//     nextPosition.y += velocity.y * delayLeft.GetSeconds();
//     m_event.Cancel();

//     // check if the nextPosition is inside a wall, or if the line
//     // from position to the next position intersects a wall
//     auto indoorWall = IsLineClearOfWalls(position, nextPosition);
//     bool isClear = std::get<0>(indoorWall);
//     Ptr<Wall> intersectingWall = std::get<1>(indoorWall);

//     if (m_bounds.IsInside(nextPosition))
//     {
//         // if (isClear)
//         // {
//         //     m_event = Simulator::Schedule(delayLeft,
//         //                                   &RandomWalk2dIndoorMobilityModel::DoInitializePrivate,
//         //                                   this);
//         // }
//         // else
//         // {
//         //     NS_LOG_LOGIC("NextPosition would lead into a wall");
//         //     nextPosition =
//         //         CalculateIntersectionFromOutside(position, nextPosition, intersectingWall->GetBoundaries());

//         //     double delaySecondsX = std::numeric_limits<double>::max();
//         //     double delaySecondsY = std::numeric_limits<double>::max();
//         //     if (velocity.x != 0)
//         //     {
//         //         delaySecondsX = std::abs((nextPosition.x - position.x) / velocity.x);
//         //     }
//         //     if (velocity.y != 0)
//         //     {
//         //         delaySecondsY = std::abs((nextPosition.y - position.y) / velocity.y);
//         //     }
//         //     Time delay = Seconds(std::min(delaySecondsX, delaySecondsY));
//         //     m_event = Simulator::Schedule(delay,
//         //                                   &RandomWalk2dIndoorMobilityModel::AvoidWall,
//         //                                   this,
//         //                                   delayLeft - delay,
//         //                                   nextPosition);
//         // }
//         m_event = Simulator::Schedule(delayLeft,
//                                               &RandomWalk2dIndoorMobilityModel::DoInitializePrivate,
//                                               this);
//     }
//     else
//     {
//         NS_LOG_LOGIC("Out of bounding box");
//         nextPosition = m_bounds.CalculateIntersection(position, velocity);
//         // check that this nextPosition is clear of walls
//         auto wallIntersection = IsLineClearOfWalls(position, nextPosition);
//         bool isClear = std::get<0>(wallIntersection);
//         Ptr<Wall> intersectingWall = std::get<1>(wallIntersection);

//         // if (isClear)
//         // {
//         //     double delaySeconds = std::numeric_limits<double>::max();
//         //     if (velocity.x != 0)
//         //     {
//         //         delaySeconds =
//         //             std::min(delaySeconds, std::abs((nextPosition.x - position.x) / velocity.x));
//         //     }
//         //     else if (velocity.y != 0)
//         //     {
//         //         delaySeconds =
//         //             std::min(delaySeconds, std::abs((nextPosition.y - position.y) / velocity.y));
//         //     }
//         //     else
//         //     {
//         //         NS_ABORT_MSG("RandomWalk2dIndoorMobilityModel::DoWalk: unable to calculate the "
//         //                      "rebound time "
//         //                      "(the node is stationary).");
//         //     }
//         //     Time delay = Seconds(delaySeconds);
//         //     m_event = Simulator::Schedule(delay,
//         //                                   &RandomWalk2dIndoorMobilityModel::Rebound,
//         //                                   this,
//         //                                   delayLeft - delay);
//         // }
//         // else
//         // {
//         //     NS_LOG_LOGIC("NextPosition would lead into a wall");
//         //     if (intersectingWall != nullptr)
//         //     {
//         //         nextPosition = CalculateIntersectionFromOutside(position, nextPosition, intersectingWall->GetBoundaries());
//         //     }
//         //     else
//         //     {
//         //         NS_LOG_ERROR("Intersecting wall is null.");
//         //     }

//         //     double delaySecondsX = std::numeric_limits<double>::max();
//         //     double delaySecondsY = std::numeric_limits<double>::max();
//         //     if (velocity.x != 0)
//         //     {
//         //         delaySecondsX =
//         //             std::min(delaySecondsX, std::abs((nextPosition.x - position.x) / velocity.x));
//         //     }
//         //     if (velocity.y != 0)
//         //     {
//         //         delaySecondsY =
//         //             std::min(delaySecondsY, std::abs((nextPosition.y - position.y) / velocity.y));
//         //     }
//         //     if (delaySecondsX == std::numeric_limits<double>::max() &&
//         //         delaySecondsY == std::numeric_limits<double>::max())
//         //     {
//         //         NS_ABORT_MSG("RandomWalk2dIndoorMobilityModel::DoWalk: unable to calculate the "
//         //                      "rebound time "
//         //                      "(the node is stationary).");
//         //     }

//         //     Time delay = Seconds(std::min(delaySecondsX, delaySecondsY));
//         //     m_event = Simulator::Schedule(delay,
//         //                                   &RandomWalk2dIndoorMobilityModel::AvoidWall,
//         //                                   this,
//         //                                   delayLeft - delay,
//         //                                   nextPosition);
//         // }
//         double delaySeconds = std::numeric_limits<double>::max();
//         if (velocity.x != 0)
//         {
//             delaySeconds =
//                 std::min(delaySeconds, std::abs((nextPosition.x - position.x) / velocity.x));
//         }
//         else if (velocity.y != 0)
//         {
//             delaySeconds =
//                 std::min(delaySeconds, std::abs((nextPosition.y - position.y) / velocity.y));
//         }
//         else
//         {
//             NS_ABORT_MSG("RandomWalk2dIndoorMobilityModel::DoWalk: unable to calculate the "
//                             "rebound time "
//                             "(the node is stationary).");
//         }
//         Time delay = Seconds(delaySeconds);
//         m_event = Simulator::Schedule(delay,
//                                         &RandomWalk2dIndoorMobilityModel::Rebound,
//                                         this,
//                                         delayLeft - delay);
//     }
//     NS_LOG_LOGIC("Position " << position << " NextPosition " << nextPosition);

//     // store the previous position
//     m_prevPosition = position;
//     NotifyCourseChange();
// }

void
RandomWalk2dIndoorMobilityModel::WallRebound(Time delayLeft, Ptr<Wall> wall)
{
    NS_LOG_FUNCTION(this << delayLeft.GetSeconds());
    m_helper.Update();
    
    Vector position = m_helper.GetCurrentPosition();
    Vector velocity = m_helper.GetVelocity();
    Box wallBoundaries = wall->GetBoundaries();
    
    // Determine which side of the wall was hit
    Rectangle wallRect(wallBoundaries.xMin, wallBoundaries.xMax, 
                      wallBoundaries.yMin, wallBoundaries.yMax);
    Rectangle::Side hitSide = wallRect.GetClosestSideOrCorner(position);
    
    // Adjust velocity based on which wall side was hit
    switch (hitSide)
    {
    case Rectangle::RIGHTSIDE:
    case Rectangle::LEFTSIDE:
        // Reverse x velocity for vertical walls
        velocity.x = -velocity.x;
        break;
    case Rectangle::TOPSIDE:
    case Rectangle::BOTTOMSIDE:
        // Reverse y velocity for horizontal walls
        velocity.y = -velocity.y;
        break;
    case Rectangle::TOPRIGHTCORNER:
    case Rectangle::TOPLEFTCORNER:
    case Rectangle::BOTTOMRIGHTCORNER:
    case Rectangle::BOTTOMLEFTCORNER:
        // For corners, reverse both components
        velocity.x = -velocity.x;
        velocity.y = -velocity.y;
        break;
    }
    
    // Add small random perturbation to avoid getting stuck
    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
    velocity.x += (rand->GetValue() - 0.5) * 0.1 * velocity.x;
    velocity.y += (rand->GetValue() - 0.5) * 0.1 * velocity.y;
    
    m_helper.SetVelocity(velocity);
    m_helper.Unpause();
    DoWalk(delayLeft);
}

void 
RandomWalk2dIndoorMobilityModel::SetFactory(Ptr<Factory> factory)
{
    m_factory = factory;
}

void
RandomWalk2dIndoorMobilityModel::DoDispose()
{
    // chain up
    MobilityModel::DoDispose();
}

Vector
RandomWalk2dIndoorMobilityModel::DoGetPosition() const
{
    m_helper.UpdateWithBounds(m_bounds);
    return m_helper.GetCurrentPosition();
}

void
RandomWalk2dIndoorMobilityModel::DoSetPosition(const Vector& position)
{
    NS_ASSERT(m_bounds.IsInside(position));
    m_helper.SetPosition(position);
    Simulator::Remove(m_event);
    m_event = Simulator::ScheduleNow(&RandomWalk2dIndoorMobilityModel::DoInitializePrivate, this);
}

Vector
RandomWalk2dIndoorMobilityModel::DoGetVelocity() const
{
    return m_helper.GetVelocity();
}

int64_t
RandomWalk2dIndoorMobilityModel::DoAssignStreams(int64_t stream)
{
    m_speed->SetStream(stream);
    m_direction->SetStream(stream + 1);
    return 2;
}

} // namespace ns3
