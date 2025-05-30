
#ifndef INDOOR_BUILDINGS_PROPAGATION_LOSS_MODEL_H_
#define INDOOR_BUILDINGS_PROPAGATION_LOSS_MODEL_H_

#include "buildings-propagation-loss-model.h"

namespace ns3
{

class ThreeGppPropagationLossModel;

/**
 * \ingroup buildings
 * \ingroup propagation
 *
 *  this model modifies the BuildingsPropagationLossModel for indoor scenarios
 *
 *  \warning This model works with MobilityBuildingInfo only
 *
 */
class IndoorBuildingsPropagationLossModel : public BuildingsPropagationLossModel
{
public:
    /**
     * \brief Get the type ID.
     * \return The object TypeId.
     */
    static TypeId GetTypeId();
    IndoorBuildingsPropagationLossModel();
    ~IndoorBuildingsPropagationLossModel() override;
    
    /**
     * \param a the mobility model of the source
     * \param b the mobility model of the destination
     * \returns the propagation loss (in dBm)
     */
    double GetLoss(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override;

    Building::BuildingType_t GetBuildingType() const;

    std::string GetFactoryType() const;

    double GetFrequency() const;

    void SetBuildingType(Building::BuildingType_t buildingType);

    void SetFactoryType(std::string factoryType);

    void SetFrequency(double frequency);

    void SetWallLoss(double loss);

    double InternalWallsLoss(Ptr<MobilityBuildingInfo> a, Ptr<MobilityBuildingInfo> b) const;

protected:
    
    Ptr<Building> m_building;
    Building::BuildingType_t m_buildingType; //!< type of building (e.g., Factory, Office, etc.)
    std::string m_factoryType; //!< type of factory (e.g., InF-SL, InF-DL, etc.)
    Ptr<ThreeGppPropagationLossModel> m_lossModel; 
    double m_frequency; //!< frequency in Hz

};

} // namespace ns3

#endif /* OH_BUILDINGS_PROPAGATION_LOSS_MODEL_H_ */
