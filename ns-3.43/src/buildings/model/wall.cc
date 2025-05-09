#include "wall.h"

#include <ns3/assert.h>
#include <ns3/enum.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("Wall");

NS_OBJECT_ENSURE_REGISTERED(Wall);

TypeId
Wall::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::Wall")
            .SetParent<Object>()
            .AddConstructor<Wall>()
            .SetGroupName("Buildings")
            .AddAttribute("Boundaries",
                        "The boundaries of this Building as a value of type ns3::Box",
                        BoxValue(Box()),
                        MakeBoxAccessor(&Wall::GetBoundaries, &Wall::SetBoundaries),
                        MakeBoxChecker());
    return tid;
}

Wall::Wall()
{
    NS_LOG_FUNCTION(this);
}

Wall::~Wall()
{
    NS_LOG_FUNCTION(this);
}

void
Wall::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

void
Wall::SetBoundaries(Box boundaries)
{
    NS_LOG_FUNCTION(this << boundaries);
    m_wallBounds = boundaries;
}

Box
Wall::GetBoundaries() const
{
    NS_LOG_FUNCTION(this);
    return m_wallBounds;
}

bool
Wall::IsInside(Vector position) const
{
    return m_wallBounds.IsInside(position);
}

bool
Wall::IsIntersect(const Vector& l1, const Vector& l2) const
{
    return m_wallBounds.IsIntersect(l1, l2);
}

} // namespace ns3
