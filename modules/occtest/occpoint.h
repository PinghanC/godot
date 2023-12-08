// occ_test.h
#ifndef OCCPOINT_H
#define OCCPOINT_H

#include "core/object/ref_counted.h"

#include "core/math/vector3.h"

class OccPoint : public RefCounted {
    GDCLASS(OccPoint, RefCounted);

protected:
	static void _bind_methods();

public:
    Vector3 getPoint();
};

#endif // OCC_TEST_H

