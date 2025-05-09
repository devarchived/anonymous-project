#ifndef WALL_H
#define WALL_H

#include <ns3/attribute-helper.h>
#include <ns3/attribute.h>
#include <ns3/box.h>
#include <ns3/object.h>
#include <ns3/simple-ref-count.h>
#include <ns3/vector.h>

namespace ns3
{

/**
 * \defgroup walls Walls
 *
 * The models to define 3d walls
 */

/**
 * \ingroup walls
 * \brief a 3d wall block
 */
class Wall : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     * Create a zero-sized wall located at coordinates (0.0,0.0,0.0)
     */
    Wall();

    /**
     * Destructor
     *
     */
    ~Wall() override;

    /**
     * Set the boundaries of the wall
     *
     * \param box the Box defining the boundaries of the wall
     */
    void SetBoundaries(Box box);

    /**
     *
     * \return the boundaries of the walls
     */
    Box GetBoundaries() const;

    /**
     *
     *
     * \param position some position
     *
     * \return true if the position fall inside the wall, false otherwise
     */
    bool IsInside(Vector position) const;

    /**
     * \brief Checks if a line-segment between position l1 and position l2
     *        intersects a wall.
     *
     * \param l1 position
     * \param l2 position
     * \return true if there is a intersection, false otherwise
     */
    bool IsIntersect(const Vector& l1, const Vector& l2) const;

  private:
    Box m_wallBounds; //!< Wall boundaries

    uint32_t m_wallId;          //!< Wall ID number
};

} // namespace ns3

#endif /* WALL_H */
