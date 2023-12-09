// cgalAABB.h
#ifndef CGALAABB_H
#define CGALAABB_H

#include "core/object/ref_counted.h"
 
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_segment_primitive.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Surface_mesh_shortest_path.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>

// kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
typedef CGAL::Polyhedron_3<Kernel>                                     Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>           AABBPrimitive;
typedef CGAL::AABB_traits<Kernel, AABBPrimitive>                       AABBTraits;
typedef CGAL::AABB_tree<AABBTraits>                                    AABBTree;

class cgalAABB : public RefCounted {
    GDCLASS(cgalAABB, RefCounted);

protected:
	static void _bind_methods();

public:
	Vector3 getPoint(); 

    AABBTree        m_inside_tree;
};

#endif // CGALAABB_H

