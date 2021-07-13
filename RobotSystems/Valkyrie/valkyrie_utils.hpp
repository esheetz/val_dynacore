/**
 * Utilities for Valkyrie
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <map>
#include <utility>

#include <Utils/wrap_eigen.hpp>
#include "Valkyrie_Model.hpp"
#include "Valkyrie_Definition.h"

namespace ValUtils {

    void constructJointGroupMap(std::map<int, std::vector<std::pair<int, std::string>>>& joint_group_map);
    void getStandingConfiguration(Valkyrie_Model val_model, dynacore::Vector& q, dynacore::Vector& qdot);
    void setIKTasksForGroup(); // TODO

} // end namespace ValUtils