#include "pseudo_inverse.hpp"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <stdio.h>
using namespace std;

namespace dynacore {

    void pseudoInverse(Matrix const & matrix,
                       double sigmaThreshold,
                       Matrix & invMatrix,
                       Vector * opt_sigmaOut)    {
        
        if ((1 == matrix.rows()) && (1 == matrix.cols())) {
            // workaround for Eigen2
            invMatrix.resize(1, 1);
            if (matrix.coeff(0, 0) > sigmaThreshold) {
                invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
            }
            else {
                invMatrix.coeffRef(0, 0) = 0.0;
            }
            if (opt_sigmaOut) {
                opt_sigmaOut->resize(1);
                opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
            }
            return;
        }
      
        Eigen::JacobiSVD<Matrix> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // not sure if we need to svd.sort()... probably not
        int const nrows(svd.singularValues().rows());
        Matrix invS;
        invS = Matrix::Zero(nrows, nrows);
        for (int ii(0); ii < nrows; ++ii) {
            if (svd.singularValues().coeff(ii) > sigmaThreshold) {
                invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
            }
            else{
                // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
                // printf("sigular value is too small: %f\n", svd.singularValues().coeff(ii));
            }
        }
        invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
        if (opt_sigmaOut) {
            *opt_sigmaOut = svd.singularValues();
        }
    }

    bool pInv(const dynacore::Matrix& _J, dynacore::Matrix& _Jinv, double _eps) {
        if (_J.rows() < _J.cols()) {
            bool b;
            dynacore::Matrix Jit;
            b = pInv(_J.transpose(), Jit, _eps);
            _Jinv = Jit.transpose();
            return b;
        }

        Eigen::JacobiSVD<dynacore::Matrix> svd = _J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        dynacore::Matrix::Scalar tolerance = _eps * std::max(_J.cols(), _J.rows()) * svd.singularValues().array().abs().maxCoeff();

        _Jinv = svd.matrixV() * dynacore::Matrix(dynacore::Matrix((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal()) * svd.matrixU().adjoint();

        return true;
    }
  
}
