
#include "teeth_operators.h"
#include "stdio.h"
#include "stdlib.h"
#include "function.h"

using namespace dentist;


void *function_new()
{
	return new Function;
}

void function_delete(void *function)
{
	delete (Function *)function;
}

int function_init_upper_teeth_mesh(void *f, const float *vertices, size_t vertices_size, const int *triangles, size_t triangles_size)
{
	Function *function = (Function *)f;
	if (!function)
		return 0;
	function->init_upper_teeth_mesh(vertices, vertices_size, triangles, triangles_size);
}

int function_init_lower_teeth_mesh(void *f, const float *vertices, size_t vertices_size, const int *triangles, size_t triangles_size)
{
	Function *function = (Function *)f;
	if (!function)
		return 0;
	function->init_lower_teeth_mesh(vertices, vertices_size, triangles, triangles_size);
}




// struct V3
// {
// 	float x;
// 	float y;
// 	float z;

// 	inline float operator[](size_t index)
// 	{
// 		return ((float *)this)[index];
// 	}
// };

// struct TriangleIndex
// {
// 	int v0;
// 	int v1;
// 	int v2;

// 	inline int operator[](size_t index)
// 	{
// 		return ((int *)this)[index];
// 	}
// };

// void *teeth_create_polyhedron(float *vertices_floats, size_t vertices_size, int *triangles, size_t triangles_size)
// {
// 	PointList poly_points;
// 	PolyList poly_faces;

// 	V3 *vertices = (V3 *)vertices_floats;

// 	for (size_t vi = 0; vi < vertices_size; ++vi)
// 	{
// 		V3 p = vertices[vi];
// 		poly_points.push_back(p);
// 	}

// 	IntList face_inds;
// 	face_inds.resize(3);
// 	for (size_t ti = 0; ti < triangles_size; ++ti)
// 	{
// 		TriangleIndex tri = triangles[ti];
// 		face_inds[0] = tri[0];
// 		face_inds[1] = tri[1];
// 		face_inds[2] = tri[2];
// 		poly_faces.push_back(face_inds);
// 	}

// 	Polyhedron *polygon = new Polyhedron;
// 	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, *polygon);

// 	return polygon;
// }

// void teeth_delete_polyhedron(void *polygon)
// {
// 	delete (Polyhedron *)polygon;
// }


// void *teeth_load(const char *path)
// {
// 	printf("Loading teeth from : %s\n", path);
// 	Teeth *teeth = new Teeth;
// 	teeth->load_teeth_mesh(path);
// 	Polyhedron &poly = teeth->get_teeth_poly();
// 	printf("Loaded teeth %p f:%lld v:%lld\n", teeth, poly.size_of_facets(), poly.size_of_vertices());
// 	return teeth;
// }

// void *teeth_create(float *vertices_floats, size_t vertices_size, int *triangles, size_t triangles_size)
// {
// 	PointList poly_points;
// 	PolyList poly_faces;

// 	V3 *vertices = (V3 *)vertices_floats;

// 	for (size_t vi = 0; vi < vertices_size; ++vi)
// 	{
// 		V3 p = vertices[vi];
// 		poly_points.push_back(p);
// 	}

// 	IntList face_inds;
// 	face_inds.resize(3);
// 	for (size_t ti = 0; ti < triangles_size; ++ti)
// 	{
// 		TriangleIndex tri = triangles[ti];
// 		face_inds[0] = tri[0];
// 		face_inds[1] = tri[1];
// 		face_inds[2] = tri[2];
// 		poly_faces.push_back(face_inds);
// 	}

// 	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, teeth->get_teeth_poly());

// 	return polygon;
// }

// void teeth_free(void *ptr)
// {
// 	Teeth *teeth = (Teeth *)ptr;
// 	printf("Freeing teeth %p\n", teeth);
// 	delete teeth;
// }

// size_t teeth_vertices_size(void *ptr)
// {
// 	Teeth *teeth = (Teeth *)ptr;
// 	if (!teeth)
// 		return 0;
// 	int size = teeth->size_of_vertices();
// 	if (size < 0)
// 		size = 0;
// 	return (size_t)size;
// }

// void teeth_vertices(void *ptr, float *vert, size_t vertices_size)
// {
// 	Teeth *teeth = (Teeth *)ptr;
// 	if (!teeth)
// 		return;
// 	int size = teeth->size_of_vertices();
// 	if (size < 0)
// 		size = 0;
// 	size_t vi = 0;
// 	for (Vertex_handle vd : vertices(teeth->get_teeth_poly()))
// 	{
// 		Point &p = vd->point();
// 		vert[vi++] = p[0];
// 		vert[vi++] = p[1];
// 		vert[vi++] = p[2];
// 	}
// }
