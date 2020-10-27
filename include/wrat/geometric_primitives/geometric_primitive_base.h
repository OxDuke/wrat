
#ifndef WRAT_GEOMETRIC_PRIMITIVES_GEOMETRIC_PRIMITIVE_BASE_H_
#define WRAT_GEOMETRIC_PRIMITIVES_GEOMETRIC_PRIMITIVE_BASE_H_ 

#include "Eigen/Geometry"

// Headers for collision checking using FCL 
#include "fcl/narrowphase/collision_object.h"

// Headers for visualization using RViz


template <typename Scalar>
using FCLCollisionObjectSharedPtr = std::shared_ptr<fcl::CollisionObject<Scalar>>;

//#include "wrat/common/types.h"

namespace wrat
{

template <typename Scalar, template<typename> class Transform>
class GeometricPrimitiveBase
{
public:
    GeometricPrimitiveBase() {}
	
	GeometricPrimitiveBase(const Transform<Scalar>& transform): tf_{transform} {}
	
	GeometricPrimitiveBase(const Vector3<Scalar>& translation, const Quaternion<Scalar>& rotation)
	{
		tf_.translation() = translation;
		tf_.linear() = rotation.toRotationMatrix();
	}

	GeometricPrimitiveBase(const Vector3<Scalar>& translation, const Matrix3<Scalar>& rotation)
	{
		tf_.translation() = translation;
		tf_.linear() = rotation;
	}
	
	~GeometricPrimitiveBase() {}

	virtual const std::shared_ptr<fcl::CollisionObject<Scalar>>& getFCLObject() = 0;
	// virtual const std::shared_ptr<visualization_msgs::Marker>& getVisualizationMarker() = 0;

	Transform3<Scalar> getTransform() const {return tf_;}

	void setTranslation(const Vector3<Scalar>& translation)
	{
		tf_.translation() = translation;
	}

	void setRotation(const Quaternion<Scalar>& rotation)
	{
		tf_.linear() = rotation.toRotationMatrix();
	}

	void setRotation(const Matrix3<Scalar>& rotation)
	{
		tf_.linear() = rotation;
	}

	void setTransform(const Transform<Scalar>& transform)
	{
		this -> tf_ = transform;
		this -> update_fcl_collision_object_transform = true;
	}

	void setTransform(const Vector3<Scalar>& translation, const Quaternion<Scalar>& rotation)
	{
		setTranslation(translation);
		setRotation(rotation);
	}

	void setTransform(const Vector3<Scalar>& translation, const Matrix3<Scalar>& rotation)
	{
		setTranslation(translation);
		setRotation(rotation);
	}
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Transform<Scalar> tf_{typename Transform<Scalar>::Identity()};

	std::shared_ptr<fcl::CollisionObject<Scalar>> fcl_collision_object_ptr{nullptr};
	bool update_fcl_collision_object_transform{false};

};


} // namespace wrat

#endif // WRAT_GEOMETRIC_PRIMITIVES_GEOMETRIC_PRIMITIVE_BASE_H_