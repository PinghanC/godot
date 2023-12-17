#ifndef MY_FUNCTION_H
#define MY_FUNCTION_H

#include "types.h"
#include "teeth.h"

namespace dentist {

class Function
{
private:
    bool        m_upper_init;
    bool        m_lower_init;
    Teeth       m_upper_teeth;
    Teeth       m_lower_teeth;

    Point       m_center;
    double      m_radius;
    AABBTree    m_teeth_tree;

    Polyhedron  m_segment;
    int         m_segment_status; // 0 if non available segment, 1 if upper, -1 if lower

public:

    //----------------------------- Initialization -----------------------------//

    Function();

    ~Function();

    void reset();

    void clear_segmentation();

    bool load_upper_teeth_mesh(std::string filename);

    bool load_lower_teeth_mesh(std::string filename);

    bool init_upper_teeth_mesh(const float* vertices, size_t vertices_size, const int* triangles, size_t triangles_size);

    bool init_lower_teeth_mesh(const float* vertices, size_t vertices_size, const int* triangles, size_t triangles_size);

    void compute_bounding_box();

    void update_teeth_tree();

    const double get_radius();
    const Point& get_center();
    int number_of_upper_teeth_faces();
    int number_of_upper_teeth_vertices();
    int number_of_lower_teeth_faces();
    int number_of_lower_teeth_vertices();

    //----------------------------- Selection -----------------------------//

    bool update_selection(Point camera_pos, Vector camera_dir, PointList& selects, DataList& selected_points);

    //----------------------------- Algorithm -----------------------------//

    bool compute_laplacian_based_inside(int smooth_range, double lambda, double value);

    bool compute_laplacian_based_convexity(double radius_ratio, int smooth_range, double lambda);

    bool compute_selected_segmentation(PointList& selected_points, double isovalue);

    int validate_selected_segmentation();

    void compute_geodesic_pad_outlines(int index, bool flag_lower, int discretize_step, double isovalue);

    void recompute_geodesic_pad_outlines(int index, bool flag_lower, double isovalue);

    void compute_tooth_cut_plane(int index, bool flag_lower);

    void compute_paired_wires(int index, bool flag_lower, double distance);

    void compute_wire_shapes(int index, bool flag_lower, int geodesic_step, double width);

    void compute_pad_shapes(int index, bool flag_lower, double pad_height, double wire_height);


    //----------------------------- Visualization -----------------------------//

    void update_teeth_mesh(DataList& teeth_facets, DataList& teeth_normals, bool flag_lower);

    void update_inside_function(DataList& inside_ray_facets,
                                DataList& inside_ray_normals,
                                DataList& inside_ray_colors,
                                DataList& inside_func_colors,
                                DataList& inside_facets,
                                DataList& inside_normals,
                                bool flag_lower);

    void update_convex_function(DataList& local_convex_facets,
                                DataList& local_convex_normals,
                                DataList& local_convex_colors,
                                DataList& convex_func_colors,
                                bool flag_lower);

    void update_convexity_isovalue(DataList& convex_isoedges, double value, bool flag_lower);

    void update_segmentation(DataList& seg_facets, DataList& seg_normals);

    void update_validated_segmentation(int index, bool flag_lower, DataList& seg_facets, DataList& seg_normals);

    void update_tooth_geodesic_function(int index, bool flag_lower, DataList& poly_colors);

    void update_pad_outline(int index, bool flag_lower, DataList& poly_facets, DataList& poly_normals);

    void update_tooth_cut_plane(int index, bool flag_lower, DataList& upper_bound, DataList& upper_points, DataList& cut_plane_points, DataList& cut_plane_normals);

    void update_pair_cut_plane( int index, bool flag_lower, 
                                DataList& fitting_plane_points, 
                                DataList& fitting_plane_normals,
                                DataList& fitting_lines,
                                DataList& fitting_points,
                                DataList& cut_plane_points,
                                DataList& cut_plane_normals,
                                DataList& cut_polylines);

    void compute_plane_corners(Plane& plane, Point& center, Point& right, PointList& corners, Vector& normal);

    void update_pair_wire_shape( int index, bool flag_lower, 
                                DataList& wire_shape_points, 
                                DataList& wire_shape_normals,
                                DataList& wire_extrude_shape_points, 
                                DataList& wire_extrude_shape_normals);

    void update_pad_shape(  int index, bool flag_lower, 
                            DataList& pad_shape_points, 
                            DataList& pad_shape_normals);

}; // end of class Function

} // namespace dentist

#endif
