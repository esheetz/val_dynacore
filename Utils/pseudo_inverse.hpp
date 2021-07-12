#ifndef JSPACE_PSEUDO_INVERSE_HPP
#define JSPACE_PSEUDO_INVERSE_HPP

#include "wrap_eigen.hpp"

namespace dynacore {
  void pseudoInverse(Matrix const & matrix,
		     double sigmaThreshold,
		     Matrix & invMatrix,
		     Vector * opt_sigmaOut = 0);

  /*
   * pseudo-inverse function based on TRACLabs kinematics utilities
   */
  bool pInv(const dynacore::Matrix& _J, dynacore::Matrix& _Jinv, double _eps = std::numeric_limits<double>::epsilon());
  
}

#endif
