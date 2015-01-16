#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

/**
Adapted from: http://wiki.ros.org/constrained_ik
*/
namespace hqp_controllers {
//------------------------------------------------------------------
 void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform)
 {
  transform.setIdentity();

   // translation
  for (size_t i=0; i<3; ++i)
    transform(i,3) = frame.p[i];

  // rotation matrix
  for (size_t i=0; i<9; ++i)
     transform(i/3, i%3) = frame.M.data[i];
}
//------------------------------------------------------------------
void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix)
{
  matrix.resize(jacobian.rows(), jacobian.columns());

  for (size_t i=0; i<jacobian.rows(); ++i)
    for (size_t j=0; j<jacobian.columns(); ++j)
      matrix(i,j) = jacobian(i,j);
 }
//------------------------------------------------------------------
void EigenToKDL(const Eigen::VectorXd &vec, KDL::JntArray &joints)
{
    joints.data = vec;
}
//------------------------------------------------------------------
} //end namespace hqp_controllers

#endif // CONVERSIONS_H
