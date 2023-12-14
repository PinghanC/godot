// occ_test.cpp
#include "occpoint.h"
#include <gp_Pnt.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>

#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Precision.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>

// godot
#include "scene/resources/surface_tool.h"

Vector3 OccPoint::getPoint() {
    gp_Pnt point(1.0, 2.0, 3.0);

	return Vector3(point.X(), point.Y(), point.Z());

    // return Vector3(0.1, 0.2, 0.3);

}

Ref<Mesh> BuildShapeMesh(const TopoDS_Shape shape, double deflection) {
    Ref<SurfaceTool> surfaceTool;
	surfaceTool.instantiate();
	surfaceTool->begin(Mesh::PRIMITIVE_TRIANGLES);

    BRepMesh_IncrementalMesh(shape, deflection);

    TopExp_Explorer faceExplorer;
    for (faceExplorer.Init(shape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
        TopLoc_Location theLocation;
		TopoDS_Face theFace = TopoDS::Face(faceExplorer.Current());
	
		if (theFace.IsNull())
			continue;

		const Handle_Poly_Triangulation &theTriangulation = BRep_Tool::Triangulation(theFace, theLocation);
		BRepLProp_SLProps theProp(BRepAdaptor_Surface(theFace), 1, Precision::Confusion());
	
		Standard_Integer nTriangles = theTriangulation->NbTriangles();

        for (Standard_Integer i = 1; i <= nTriangles; i++) {
			const Poly_Triangle &theTriangle = theTriangulation->Triangle(i);
			gp_Pnt theVertex1 = theTriangulation->Node((theTriangle.Value(1)));
			gp_Pnt theVertex2 = theTriangulation->Node((theTriangle.Value(2)));
			gp_Pnt theVertex3 = theTriangulation->Node((theTriangle.Value(3)));

			const gp_Pnt2d &theUV1 = theTriangulation->UVNode(theTriangle.Value(1));
			const gp_Pnt2d &theUV2 = theTriangulation->UVNode(theTriangle.Value(2));
			const gp_Pnt2d &theUV3 = theTriangulation->UVNode(theTriangle.Value(3));

			theVertex1.Transform(theLocation.Transformation());
			theVertex2.Transform(theLocation.Transformation());
			theVertex3.Transform(theLocation.Transformation());

			// find the normal for the triangle mesh.
			gp_Vec V12(theVertex1, theVertex2);
			gp_Vec V13(theVertex1, theVertex3);
			gp_Vec theNormal = V12 ^ V13;
			gp_Vec theNormal1 = theNormal;
			gp_Vec theNormal2 = theNormal;
			gp_Vec theNormal3 = theNormal;

			if (theNormal.Magnitude() > Precision::Confusion())
			{
				theNormal.Normalize();
				theNormal1.Normalize();
				theNormal2.Normalize();
				theNormal3.Normalize();
			}

			theProp.SetParameters(theUV1.X(), theUV1.Y());
			if (theProp.IsNormalDefined())
			{
				theNormal1 = theProp.Normal();
			}

			theProp.SetParameters(theUV2.X(), theUV2.Y());
			if (theProp.IsNormalDefined())
			{
				theNormal2 = theProp.Normal();
			}

			theProp.SetParameters(theUV3.X(), theUV3.Y());
			if (theProp.IsNormalDefined())
			{
				theNormal3 = theProp.Normal();
			}

			if (theFace.Orientation() == TopAbs_REVERSED)
			{
				theNormal.Reverse();
				theNormal1.Reverse();
				theNormal2.Reverse();
				theNormal3.Reverse();
			}

            // Add vertices and normals to the SurfaceTool
            surfaceTool->set_normal(Vector3(theNormal1.X(), theNormal1.Y(), theNormal1.Z()));
            surfaceTool->add_vertex(Vector3(theVertex1.X(), theVertex1.Y(), theVertex1.Z()));

            surfaceTool->set_normal(Vector3(theNormal2.X(), theNormal2.Y(), theNormal2.Z()));
            surfaceTool->add_vertex(Vector3(theVertex2.X(), theVertex2.Y(), theVertex2.Z()));

            surfaceTool->set_normal(Vector3(theNormal3.X(), theNormal3.Y(), theNormal3.Z()));
            surfaceTool->add_vertex(Vector3(theVertex3.X(), theVertex3.Y(), theVertex3.Z()));
        }
    }

    // Commit the surfaces
    surfaceTool->commit();

    // Create an ArrayMesh and add the surface
    Ref<ArrayMesh> arrayMesh;
    arrayMesh.instantiate();
    arrayMesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surfaceTool->commit_to_arrays());
    
    return arrayMesh;
}

Ref<Mesh> OccPoint::getOccTorusMesh(Standard_Real majorRadius, Standard_Real minorRadius, Standard_Real deflection) {
    BRepPrimAPI_MakeTorus aTorus(majorRadius, minorRadius);
    TopoDS_Shape aShape = aTorus.Shape();

    return BuildShapeMesh(aShape, deflection);
}

Ref<Mesh> OccPoint::getOccSphereMesh(Standard_Real radius, Standard_Real deflection) {
	BRepPrimAPI_MakeSphere aSphere(radius);
	TopoDS_Shape aShape = aSphere.Shape();

	return BuildShapeMesh(aShape, deflection);
}



void OccPoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("getPoint"), &OccPoint::getPoint);
	ClassDB::bind_method(D_METHOD("getOccTorusMesh", "majorRadius", "minorRadius", "deflection"), &OccPoint::getOccTorusMesh);
	ClassDB::bind_method(D_METHOD("getOccSphereMesh", "radius", "deflection"), &OccPoint::getOccSphereMesh);
}
