#ifndef TYPES_H
#define TYPES_H

// CGAL
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

// Boost
#include <boost/property_map/property_map.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>

// std
#include <map>
#include <set>
#include <queue>
#include <vector>
#include <unordered_set>

// OpenMP
#include <omp.h>

namespace dentist {

// kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel     Kernel;

// Simple geometric types
typedef Kernel::FT                  FT;
typedef Kernel::Point_2             Point_2;
typedef Kernel::Point_3             Point;
typedef Kernel::Vector_3            Vector;
typedef Kernel::Triangle_3          Triangle;
typedef Kernel::Segment_3           Segment;
typedef CGAL::Bbox_3                Bbox;
typedef Kernel::Ray_3               Ray;
typedef Kernel::Line_3              Line;
typedef Kernel::Plane_3             Plane;

// Polyhedron
typedef CGAL::Polyhedron_3<Kernel>                              Polyhedron;
typedef Polyhedron::Vertex_handle                               Vertex_handle;
typedef Polyhedron::Edge_iterator                               Edge_iterator;
typedef Polyhedron::Halfedge_handle                             Halfedge_handle;
typedef Polyhedron::Facet_handle                                Facet_handle;
typedef Polyhedron::Facet_iterator                              Facet_iterator;
typedef Polyhedron::Halfedge_iterator                           Halfedge_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator            Halfedge_facet_circulator;
typedef Polyhedron::Halfedge_around_vertex_circulator           Halfedge_vertex_circulator;

// Polyhedron related structures
typedef std::map<Facet_handle, bool>                     FaceBoolMap;
typedef std::map<Vertex_handle, bool>                    VertBoolMap;
typedef std::map<Vertex_handle, int>                     VertIntMap;
typedef std::map<Halfedge_iterator, int>                 HEdgeIntMap;
typedef std::vector<Halfedge_handle>                     HEdgeList;
typedef std::set<Halfedge_handle>                        HEdgeSet;
typedef std::set<Facet_handle>                           FacetSet;
typedef std::set<Vertex_handle>                          VertexSet;
typedef std::vector<Facet_handle>                        FacetList;
typedef std::vector<Vertex_handle>                       VertexList;
typedef std::pair<Facet_handle, bool>                    FacetPair;
typedef std::queue<FacetPair>                            FacetQueue;
typedef std::queue<Vertex_handle>                        VertexQueue;
typedef std::map<Vertex_handle, double>                  VertDoubleMap;
typedef boost::associative_property_map<VertDoubleMap>   VertDouble_property_map;
typedef std::map<Vertex_handle, Vector>                  VertVecMap;
typedef boost::associative_property_map< VertVecMap>     VertVec_property_map;
typedef std::map<Facet_handle, Vector>                   FaceVecMap;
typedef boost::associative_property_map< FaceVecMap >    FaceVec_property_map;
typedef std::map<int, double>                            IntDoubleMap;

// UV mapping
typedef CGAL::Unique_hash_map<Vertex_handle, Point_2>           UV_uhm;
typedef boost::associative_property_map<UV_uhm>                 UV_pmap;
typedef CGAL::Unique_hash_map<Vertex_handle, Point>             UV_uhm_3;
typedef boost::associative_property_map<UV_uhm_3>               UV_pmap_3;
typedef CGAL::Aff_transformation_3<Kernel>                      Aff_transformation_3;

// Data Structure
typedef std::pair<Point, Vector>            Point_with_normal;
typedef std::vector<Point_with_normal>      PwnList;
typedef std::vector<Point>                  PointList;
typedef std::vector<float>                  DataList;
typedef std::vector<int>                    IntList;
typedef std::vector<double>                 DoubleList;
typedef std::vector<IntList>                PolyList;
typedef std::vector<Triangle>               TriangleList;
typedef std::vector<bool>                   BoolList;
typedef std::vector<Segment>                SegmentList;
typedef std::vector<Vector>                 VectorList;


// AABBTree
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>           AABBPrimitive;
typedef CGAL::AABB_traits<Kernel, AABBPrimitive>                       AABBTraits;
typedef CGAL::AABB_tree<AABBTraits>                                    AABBTree;
typedef AABBTree::Point_and_primitive_id                               Point_and_primitive_id;

// KDTree
typedef CGAL::Search_traits_3<Kernel>                           KDTraits;
typedef CGAL::Fuzzy_sphere<KDTraits>                            Fuzzy_circle;
typedef CGAL::Kd_tree<KDTraits>                                 KDTree;

// Eigen
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>          EMatrix;
typedef Eigen::VectorXd                                                EVector;
typedef Eigen::SparseMatrix<double>                                    ESMatrix;
typedef std::vector<Eigen::Triplet<double> >                           ESTripleList;
typedef Eigen::SimplicialLDLT<ESMatrix>                                ESolver;

// Geodesic distance
typedef typename CGAL::Surface_mesh_shortest_path_traits<Kernel, Polyhedron>                    Geodesic_traits;
typedef typename Geodesic_traits::Barycentric_coordinates                                       Geodesic_coordinates;
typedef typename boost::property_map<Polyhedron, boost::vertex_external_index_t>::const_type    Vertex_index_map;
typedef typename boost::property_map<Polyhedron, CGAL::halfedge_external_index_t>::const_type   Halfedge_index_map;
typedef typename boost::property_map<Polyhedron, CGAL::face_external_index_t>::const_type       Face_index_map;
typedef typename CGAL::Surface_mesh_shortest_path<  Geodesic_traits, 
                                                    Vertex_index_map,
                                                    Halfedge_index_map,
                                                    Face_index_map >                            Geodesic_tree;
typedef typename Geodesic_tree::Face_location                                                   Face_location;
} // namespace dentist

#endif
