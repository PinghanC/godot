#ifndef MY_TEETH_H
#define MY_TEETH_H

#include "types.h"
#include "tooth.h"
#include "wire.h"

namespace dentist {

class Teeth
{
public:
    typedef typename std::vector<Tooth> ToothList;
    typedef typename std::vector<Wire>  WireList;

private:
    Polyhedron      m_teeth_poly;
    VertIntMap      m_vert_ind_map;
    VertDoubleMap   m_vert_ray_map;
    VertDoubleMap   m_vert_smooth_ray_map;
    VertDoubleMap   m_vert_inside_func_map;
    
    Polyhedron      m_teeth_inside;
    VertIntMap      m_inside_vert_ind_map;
    VertDoubleMap   m_inside_vert_convex_map;
    VertDoubleMap   m_inside_vert_smooth_convex_map;
    VertDoubleMap   m_inside_vert_convex_func_map;
    AABBTree        m_inside_tree;
    KDTree          m_inside_boundary_tree;

    ToothList       m_tooth;
    WireList        m_wires;

    
public:

    //----------------------------- Initialization -----------------------------//

    Teeth();

    ~Teeth();

    void reset();

    void reset_inside();

    void reset_convexity();

    bool load_teeth_mesh(std::string filename);

    bool init_teeth_mesh(const float* vertices, size_t vertices_size, const int* triangles, size_t triangles_size);

    Polyhedron& get_teeth_poly();
    VertDoubleMap& get_smoothed_ray_map();
    VertDoubleMap& get_vertex_inside_function_map();
    Polyhedron& get_inside_teeth_poly();
    VertDoubleMap& get_smoothed_convex_map();
    VertDoubleMap& get_vertex_convex_function_map();

    int size_of_facets();
    int size_of_vertices();
    int size_of_inside_facets();
    int size_of_inside_vertices();

    void read_polygon_vertices(PointList& points);

    void initialize_vertex_indices();

    double get_zmax();

    //----------------------------- Inside Function -----------------------------//

    bool compute_laplacian_based_inside(AABBTree& teeth_tree, Point& center, int smooth_range, double lambda);

    void compute_intersection_ray(AABBTree& teeth_tree, Point& center);

    double compute_distance_to_inside_teeth(Point& point);

    void extract_inside_from_function(double value);

    void initialize_inside_vertex_index();

    void initialize_inside_boundary_kdtree();

    void fill_holes_in_vertmap(Polyhedron& poly, VertDoubleMap& vert_map, double isovalue, VertDoubleMap& new_vert_map);

    //----------------------------- Convexity Function -----------------------------//

    bool compute_laplacian_based_convexity(double radius_ratio, double bounding_radius, int smooth_range, double lambda);

    void compute_convexity_from_sampling(double radius_ratio, double bounding_radius);

    //----------------------------- Segmentation -----------------------------//

    void compute_selected_segmentation(PointList& selected_points, double isovalue, Polyhedron& segment_poly);

    bool validate_selected_segmentation(Polyhedron& segment_poly);

    //----------------------------- Tooth Function -----------------------------//

    Polyhedron& get_tooth_poly(int index);
    VertDoubleMap& get_geodesic_distance_map(int index);
    Polyhedron& get_pad_outline_poly(int index);

    HEdgeSet& get_upper_bound_set(int index);
    PointList& get_cut_points(int index);
    Plane& get_cut_plane(int index);

    Wire& get_wire(int index);

    Tooth& get_tooth(int index);

    void compute_geodesic_pad_outlines(int index, int discretize_step, double isovalue);

    void recompute_geodesic_pad_outlines(int index, double isovalue);

    void compute_tooth_cut_plane(int index);

    void compute_pad_shapes(int index, double pad_height, double wire_height);

    //----------------------------- Wire Function -----------------------------//

    void compute_paired_wires(int index, double distance);

    void copy_faces_to_triangles(Polyhedron& tooth_poly, TriangleList& triangles);

    void copy_boundary_edges_to_segments(HEdgeSet& bound_hedge_set, PointList& points, SegmentList& segments);

    void project_points_on_line(PointList& candidate_points, Line& fitting_line, Point& left_point, Point& right_point);

    void compute_pair_plane(double distance, Point& plane_center, Point& left_left_point, Point& right_right_point, Point& pair_center, Plane& fitting_plane, Plane& pair_plane);

    void compute_plane_mesh(double scale, Plane& plane, Point& center, Point& right, Polyhedron& plane_mesh);

    void compute_wire_shapes(int index, int discretize_step, double width);

    void clip_mesh_by_plane(Polyhedron& mesh, Plane& plane, Point& center);

    bool extract_wire_shape_for_halftooth(  double width, Plane& cut_plane, Geodesic_tree& geodesic_tree, AABBTree& geodesic_aabb_tree, Polyhedron& halftooth,  // input
                                            Polyhedron& halfwire, Point& upper_bound, Point& lower_bound);

    void find_vnormal_from_adjacent_mesh(Polyhedron& halfwire, VertexList& vert_handles, VectorList& vert_normals, double threshold = 1e-6);

    

    //----------------------------- Inside / Convexity Help Function -----------------------------//

    void compute_smoothed_field(int smooth_range, VertDoubleMap& init_map, VertDoubleMap& value_map);

    double locate_and_evaluate_function(Point& query, Facet_handle face, VertDoubleMap& vert_func_map);

    void barycentric_coordinates(Point& p, Point& pa, Point& pb, Point& pc, double& a, double& b, double& c);
    
}; // end of class Teeth

} // namespace dentist

#endif
