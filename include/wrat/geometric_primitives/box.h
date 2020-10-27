#ifndef WRAT_GEOMETRIC_PRIMITIVES_BOX_H_
#define WRAT_GEOMETRIC_PRIMITIVES_BOX_H_

#include <iostream>

#include "Eigen/Geometry"

#include "fcl/narrowphase/collision_object.h"
#include "fcl/geometry/shape/box.h"

#include "wrat/common/types.h"
#include "wrat/geometric_primitives/geometric_primitive_base.h"

namespace wrat
{

template <typename Scalar, template<typename> class Transform>
class Box: public GeometricPrimitiveBase<Scalar, Transform>
{
public:

	Box():
		GeometricPrimitiveBase<Scalar, Transform>()
	{}

	Box(Vector3<Scalar> extents, Transform3<Scalar> transform):
		GeometricPrimitiveBase<Scalar, Transform>(transform),
		extents_{extents}
	{}

	Box(Scalar x_extent, Scalar y_extent, Scalar z_extent, Transform3<Scalar> transform):
		GeometricPrimitiveBase<Scalar, Transform>(transform),
		extents_{x_extent, y_extent, z_extent}
	{}

	~Box() {}

	const Vector3<Scalar>& getExtents() const {return extents_;}


	virtual const std::shared_ptr<fcl::CollisionObject<Scalar>>& getFCLObject() override
	{
		// If the CollisionObject has never been created, we create one.
		if (this -> fcl_collision_object_ptr == nullptr)
		{
			// Construct the box geometry
			std::shared_ptr<fcl::Box<Scalar>> geom_ptr =
			                                   std::make_shared<fcl::Box<Scalar>>(fcl::Box<Scalar>(extents_[0], extents_[1], extents_[2]));

			// Set the box's transform
			fcl::Transform3<Scalar> fcl_box_transform;

			fcl_box_transform.setIdentity();
			fcl_box_transform.linear() = this->tf_.linear().matrix();
			fcl_box_transform.translation() = this->tf_.translation().matrix();

			// Instantiate a fcl::Box CollisionObject
			this->fcl_collision_object_ptr = std::make_shared<fcl::CollisionObject<Scalar>>(fcl::CollisionObject<Scalar>(geom_ptr, fcl_box_transform));
		}

		// if (this->update_fcl_collision_object_translation)
		// {
		// 	int j;
		// 	// do nothing
		// 	for (int i = 0; i < 100; ++i)
		// 	{
		// 		j = 1;
		// 	}

		// 	this -> update_fcl_collision_object_transform = false;

		// }

		return this->fcl_collision_object_ptr;
	}


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	friend
	std::ostream& operator<<(std::ostream& out, const Box& box) {
		out << "Box:" << "\nExtents: " << box.extents_.transpose()
		    << "\nTransform: \n" << box.getTransform().matrix() << "\n";

		return out;
	}

private:
	Vector3<Scalar> extents_{0, 0, 0};
};

} // namespace wrat

#endif // WRAT_GEOMETRIC_PRIMITIVES_BOX_H_