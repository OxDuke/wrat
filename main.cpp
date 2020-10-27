
#include <iostream>
#include <chrono>

#include "Eigen/Core"
//#include "Eigen/Geometry"

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_result.h"

#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_result.h"

#include "wrat/geometric_primitives/box.h"

using namespace std;
using namespace Eigen;

using myScalar = double;

void transformTooo(Vector3d& v1, Quaterniond& q1, Vector3d& v2, Quaterniond& q2, Vector3d& v3, Quaterniond& q3)
{
	v3 = q1*v2 + v1;
	q3 = q1 * q2;
}

int main()
{
	cout << "hi" << endl;
	cout << " Eigen version : " << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << endl;
	
	int N = 1;


	Vector3d v3;
	Quaterniond q3;

	chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
    
    for (int i = 0; i < N; ++i)
    {
    	Quaterniond q1(1, 0, 0, 0), q2(1, 0, 0, 0);
		Vector3d v1(1,1,1), v2(0, 1, 0);

    	transformTooo(v1, q1, v2, q2, v3, q3);
    }

	chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
	chrono::duration<double> time_span = chrono::duration_cast< chrono::duration<double> >(t2 - t1);

	cout << v3 << ", " << endl; 
	cout << "Duration " << time_span.count() / N * 1000 << "ms" << endl;
	

    fcl::Box<myScalar> box1(2,2,2);
    cout << "box type: " << box1.getNodeType() << endl;
    cout << "COM: " << box1.computeCOM() << endl;
    cout << "fcl box: " << box1.side << "\n" << "volume: " << box1.computeVolume() << "\n" << box1.computeMomentofInertia() << endl;

	fcl::Transform3<myScalar> box1_tf(fcl::Translation3<myScalar>{0, 0, 0});

	fcl::Box<myScalar> box2(2,2,2);
	fcl::Transform3<myScalar> box2_tf(fcl::Translation3<myScalar>{3, 0, 0});

	fcl::CollisionRequest<myScalar> crequest;
	// result will be returned via the collision result structure
	fcl::CollisionResult<myScalar> cresult;

	int M = 10000;

	t1 = chrono::high_resolution_clock::now();

	for (int i = 0; i < M; ++i)
	{
		//fcl::collide(&box1, box1_tf, &box2, box2_tf, crequest, cresult);
    	cresult.clear();
	}

    t2 = chrono::high_resolution_clock::now();
    time_span = chrono::duration_cast< chrono::duration<double> >(t2 - t1);
	cout << "Duration " << time_span.count() / M * 1000 << "ms" << endl;

	cout << "Collided?: " << cresult.isCollision() << endl;

	fcl::DistanceRequest<myScalar> drequest;
	fcl::DistanceResult<myScalar> dresult;

	drequest.enable_nearest_points = true;
	fcl::distance(&box1, box1_tf, &box2, box2_tf, drequest, dresult);

	cout << "distance: " << dresult.min_distance << endl;
	cout << "NP1: " << dresult.nearest_points[0].transpose() << endl;
	cout << "NP2: " << dresult.nearest_points[1].transpose() << endl;

    
    wrat::Transform3<float> tff = wrat::Transform3<float>{wrat::Transform3<float>::Identity()};
    wrat::Transform3<double> tfd = wrat::Transform3<double>{wrat::Transform3<double>::Identity()};
	wrat::Box<double, wrat::Transform3> gpd(1,2,3, tfd);
	wrat::Box<float, wrat::Transform3> gpf(4,5,6, tff);

	gpd.getFCLObject();
	gpf.getFCLObject();

	cout << gpf << endl;




	return 0;
}
