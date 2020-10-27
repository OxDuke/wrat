
#ifndef WRAT_GEOMETRIC_PRIMITIVES_GEOMETRIC_PRIMITIVE_BASE_H_
#define WRAT_GEOMETRIC_PRIMITIVES_GEOMETRIC_PRIMITIVE_BASE_H_ 

#include "Eigen/Geometry"

// Headers for collision checking using FCL 
#include "fcl/narrowphase/collision_object.h"

// Headers for visualization using RViz

#include "wrat/common/types.h"

using CollisionObjectPtr = std::shared_ptr<fcl::CollisionObject<wrat::WRAT_REAL>>;
using VisualObjectPtr = std::shared_ptr<wrat::VisualObject<wrat::WRAT_REAL>>;



namespace wrat
{


class GeometricPrimitiveBase
{
public:
    GeometricPrimitiveBase() {}
	
	GeometricPrimitiveBase(const SE3& transform): tf_{transform} {}
	
	// GeometricPrimitiveBase(const Vector3<Scalar>& translation, const Quaternion<Scalar>& rotation)
	// {
	// 	tf_.translation() = translation;
	// 	tf_.linear() = rotation.toRotationMatrix();
	// }

	// GeometricPrimitiveBase(const Vector3<Scalar>& translation, const Matrix3<Scalar>& rotation)
	// {
	// 	tf_.translation() = translation;
	// 	tf_.linear() = rotation;
	// }
	
	~GeometricPrimitiveBase() {}

	virtual const CollisionObjectPtr& getCollisionObject() = 0;
	virtual const VisualObjectPtr& getVisualObject() = 0;

	Transform3<Scalar> getTransform() const {return tf_;}

	// void setTranslation(const Vector3<Scalar>& translation)
	// {
	// 	tf_.translation() = translation;
	// }

	// void setRotation(const Quaternion<Scalar>& rotation)
	// {
	// 	tf_.linear() = rotation.toRotationMatrix();
	// }

	// void setRotation(const Matrix3<Scalar>& rotation)
	// {
	// 	tf_.linear() = rotation;
	// }

	void setTransform(const SE3& transform)
	{
		this -> tf_ = transform;
		this -> setCollisionTransform(transform);
		this -> setVisualTransform(transform);
	}
    
    // make this inline
	void setCollisionTransform(const SE3& transform)
	{
		this -> collision_object_ -> setTransform(transform);
	}

	void setVisualTransform(const SE3& transform)
	{
		this -> visual_object_ -> setTransform(transform);
	}

	// void setTransform(const Vector3<Scalar>& translation, const Quaternion<Scalar>& rotation)
	// {
	// 	setTranslation(translation);
	// 	setRotation(rotation);
	// }

	// void setTransform(const Vector3<Scalar>& translation, const Matrix3<Scalar>& rotation)
	// {
	// 	setTranslation(translation);
	// 	setRotation(rotation);
	// }
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// SE3 tf_{typename SE3::Identity()};
	SE3 tf_{SE3::Identity()};
    
    // Collision-related stuff
	CollisionObjectPtr collision_object_{nullptr};

	// visual-related stuff
	VisualObjectPtr visual_object_{nullptr};

};


} // namespace wrat

#endif // WRAT_GEOMETRIC_PRIMITIVES_GEOMETRIC_PRIMITIVE_BASE_H_