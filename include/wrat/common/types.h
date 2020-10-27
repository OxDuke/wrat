#ifndef WRAT_COMMON_TYPES_H_
#define WRAT_COMMON_TYPES_H_

#include "Eigen/Core"

namespace wrat
{
	
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;


template <typename Scalar>
using Translation3 = Eigen::Translation<Scalar, 3>;

template <typename Scalar>
using Quaternion = Eigen::Quaternion<Scalar>;

template <typename Scalar>
using Rotation3 = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar>
using Transform3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
}
#endif //WRATR_COMMON_TYPES_H_