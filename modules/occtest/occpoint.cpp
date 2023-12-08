// occ_test.cpp
#include "occpoint.h"
#include <gp_Pnt.hxx>


Vector3 OccPoint::getPoint() {
    gp_Pnt point(1.0, 2.0, 3.0);

	return Vector3(point.X(), point.Y(), point.Z());

    // return Vector3(0.1, 0.2, 0.3);

}

void OccPoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("getPoint"), &OccPoint::getPoint);
}
