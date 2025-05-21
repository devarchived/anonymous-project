/*
* Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*
* SPDX-License-Identifier: GPL-2.0-only
*
* Author: Marco Miozzo  <marco.miozzo@cttc.es>
*
*/

#include "indoor-buildings-propagation-loss-model.h"

#include "mobility-building-info.h"

#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/pointer.h"
#include "ns3/propagation-loss-model.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("IndoorBuildingsPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(IndoorBuildingsPropagationLossModel);

IndoorBuildingsPropagationLossModel::IndoorBuildingsPropagationLossModel()
{
}

IndoorBuildingsPropagationLossModel::~IndoorBuildingsPropagationLossModel()
{
}

TypeId
IndoorBuildingsPropagationLossModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::IndoorBuildingsPropagationLossModel")

                            .SetParent<BuildingsPropagationLossModel>()
                            .SetGroupName("Buildings")
                            .AddConstructor<IndoorBuildingsPropagationLossModel>()
                            .AddAttribute("FactoryType",
                                          "Type of 3GPP indoor factory model",
                                          StringValue("InF-SL"),
                                          MakeStringAccessor(&IndoorBuildingsPropagationLossModel::GetFactoryType,
                                                             &IndoorBuildingsPropagationLossModel::SetFactoryType),
                                          MakeStringChecker());

    return tid;
}

void
IndoorBuildingsPropagationLossModel::SetBuildingType(Building::BuildingType_t t)
{
    NS_LOG_FUNCTION(this << t);
    m_buildingType = t;

    if (m_buildingType == Building::Factory)
    {
        m_lossModel = CreateObject<ThreeGppIndoorFactoryPropagationLossModel>();
    }
    else
    {
        m_lossModel = CreateObject<ThreeGppIndoorOfficePropagationLossModel>();
    }
}

void 
IndoorBuildingsPropagationLossModel::SetFactoryType(std::string factoryType)
{
    NS_LOG_FUNCTION(this);
    m_factoryType = factoryType;
}

void
IndoorBuildingsPropagationLossModel::SetFrequency(double frequency)
{
    NS_LOG_FUNCTION(this << frequency);
    m_frequency = frequency;
    m_lossModel->SetAttribute("Frequency", DoubleValue(m_frequency));
}

Building::BuildingType_t
IndoorBuildingsPropagationLossModel::GetBuildingType() const
{
    NS_LOG_FUNCTION(this);
    return m_buildingType;
}

std::string
IndoorBuildingsPropagationLossModel::GetFactoryType() const
{
    NS_LOG_FUNCTION(this);
    return m_factoryType;
}

double
IndoorBuildingsPropagationLossModel::GetFrequency() const
{
    NS_LOG_FUNCTION(this);
    return m_frequency;
}

double
IndoorBuildingsPropagationLossModel::GetLoss(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    NS_LOG_FUNCTION(this << a << b);

    // get the MobilityBuildingInfo pointers
    Ptr<MobilityBuildingInfo> a1 = a->GetObject<MobilityBuildingInfo>();
    Ptr<MobilityBuildingInfo> b1 = b->GetObject<MobilityBuildingInfo>();
    NS_ASSERT_MSG(a1 && b1, "IndoorBuildingsPropagationLossModel only works with MobilityBuildingInfo");

    double loss = 0.0;

    bool isAIndoor = a1->IsIndoor();
    bool isBIndoor = b1->IsIndoor();

    if(isAIndoor && isBIndoor)
    {
        if (a1->GetBuilding() == b1->GetBuilding())
        {
            // nodes are in same building -> indoor communication ITU-R P.1238
            loss = m_lossModel->GetLoss(m_lossModel->GetChannelConditionModel()->GetChannelCondition(a, b), a, b) + InternalWallsLoss(a1, b1);
            NS_LOG_INFO(this << " I-I (same building)" << loss);
        }
    }
    else
    {
        NS_ABORT_MSG("This propagation loss model only works for indoor nodes in the same building");
    }

    loss = std::max(0.0, loss);
    return loss;
}

double
IndoorBuildingsPropagationLossModel::InternalWallsLoss(Ptr<MobilityBuildingInfo> a,
                                                 Ptr<MobilityBuildingInfo> b) const
{
    NS_LOG_FUNCTION(this << a << b);

    if(m_buildingType != Building::Factory)
    {
        // approximate the number of internal walls with the Manhattan distance in "rooms" units
        // double dx = std::abs(a->GetRoomNumberX() - b->GetRoomNumberX());
        // double dy = std::abs(a->GetRoomNumberY() - b->GetRoomNumberY());
        // return m_lossInternalWall * (dx + dy);
        auto wallCount = a->GetBuilding()->CountWallsBetweenPoints(a->GetCachedPosition(), b->GetCachedPosition());
        return m_lossInternalWall * wallCount;
    }
    else
    {
        auto wallCount = a->GetBuilding()->CountWallsBetweenPoints(a->GetCachedPosition(), b->GetCachedPosition());
        return m_lossInternalWall * wallCount;
    }   
}

} // namespace ns3
