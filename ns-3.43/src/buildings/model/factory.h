/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Marco Miozzo  <marco.miozzo@cttc.es>
 *
 */
#ifndef FACTORY_H
#define FACTORY_H

#include "building.h"
#include "wall.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \defgroup buildings Buildings
 *
 * The models to define 3d buildings, associated channel models, and mobility.
 */

/**
 * \ingroup buildings
 * \brief a 3d building block
 */
class Factory : public Building
{
  public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();

    /**
     * Construct a simple building with 1 room and 1 floor
     *
     * \param xMin x coordinates of left boundary.
     * \param xMax x coordinates of right boundary.
     * \param yMin y coordinates of bottom boundary.
     * \param yMax y coordinates of top boundary.
     * \param zMin z coordinates of down boundary.
     * \param zMax z coordinates of up boundary.
     *
     */
    Factory(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);

    /**
     * Create a zero-sized building located at coordinates (0.0,0.0,0.0)
     * and with 1 floors and 1 room.
     */
    Factory();

    /**
     * Destructor
     *
     */
    ~Factory() override;

    /**
     * Set the type of interior walls
     *
     * \param t the type of interior walls (e.g., Wood, ConcreteWithWindows, etc.)
     */
    void SetIntWallsType(Building::ExtWallsType_t t);

    void AddIntWall(Ptr<Wall> wall);

    // /**
    //  * \param nfloors the number of floors in the building
    //  *
    //  * This method allows to set the number of floors in the building
    //  * (default is 1)
    //  */
    // void SetNFloors(uint16_t nfloors);

    // /**
    //  * \param nroomx the number of rooms along the x axis
    //  *
    //  * This method allows to set the number of rooms along the x-axis
    //  */
    // void SetNRoomsX(uint16_t nroomx);

    // /**
    //  * \param nroomy the number of floors in the building
    //  *
    //  * This method allows to set the number of rooms along the y-axis
    //  */
    // void SetNRoomsY(uint16_t nroomy);

    /**
     * Get the type of interior walls
     *
     * \return the type of interior walls
     */
    Building::ExtWallsType_t GetIntWallsType() const;

    virtual std::vector<Ptr<Wall>>&  GetIntWalls() override;

    /**
     * \return the number of floors of the building
     */
    uint16_t GetNFloors() const;

    /**
     * \return the number of rooms along the x-axis of the building
     */
    uint16_t GetNRoomsX() const;

    /**
     * \return the number of rooms along the y-axis
     */
    uint16_t GetNRoomsY() const;

    void AddWallToRoom(uint16_t roomX, uint16_t roomY, Ptr<Wall> wall);

    virtual uint16_t CountWallsBetweenPoints(Vector pointA, Vector pointB) const override;

    /**
     *
     *
     * \param position a position inside the building
     *
     * \return the number of the room along the X axis where the
     * position falls
     */
    virtual uint16_t GetRoomX(Vector position) const override;

    /**
     *
     *
     * \param position a position inside the building
     *
     * \return  the number of the room along the Y axis where the
     * position falls
     */
    virtual uint16_t GetRoomY(Vector position) const override;

  private:
    uint16_t m_roomsX; //!< X Room coordinate
    uint16_t m_roomsY; //!< Y Room coordinate

    ExtWallsType_t m_interiorWallType; //!< Internal factory wall type

  protected:
    std::map<std::pair<uint16_t, uint16_t>, std::vector<Ptr<Wall>>> m_roomWalls; //!< Internal factory walls
    std::vector<Ptr<Wall>> m_interiorWalls; //!< Internal factory walls
};

} // namespace ns3

#endif /* FACTORY_H */
