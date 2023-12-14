// occ_test.h
#ifndef OCCPOINT_H
#define OCCPOINT_H

#include "core/object/ref_counted.h"

// godot
#include "scene/resources/mesh.h"
#include "core/math/vector3.h"

// occ
#include <Standard_TypeDef.hxx>

class OccPoint : public RefCounted {
    GDCLASS(OccPoint, RefCounted);

protected:
	static void _bind_methods();

public:
    Vector3 getPoint();
	Ref<Mesh> getOccTorusMesh(Standard_Real majorRadius, Standard_Real minorRadius, Standard_Real deflection);
    Ref<Mesh> getOccSphereMesh(Standard_Real radius, Standard_Real deflection);
};

#endif // OCC_TEST_H

