/**
 * Utilities for Valkyrie
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <map>
#include <memory>
#include <utility>

#include <Utils/wrap_eigen.hpp>
#include "Valkyrie_Definition.h"
#include "Valkyrie_Model.hpp"

namespace ValUtils {

    void constructJointGroupMap(std::map<int, std::vector<std::pair<int, std::string>>>& joint_group_map);

    void getStandingConfiguration(std::shared_ptr<Valkyrie_Model> val_model, dynacore::Vector& q, dynacore::Vector& qdot);

} // end namespace ValUtils