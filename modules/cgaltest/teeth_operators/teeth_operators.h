#include "stdlib.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void *function_new();
    void function_delete(void *function);
    int function_init_upper_teeth_mesh(void* function, const float *vertices, size_t vertices_size, const int *triangles, size_t triangles_size);
    int function_init_lower_teeth_mesh(void* function, const float *vertices, size_t vertices_size, const int *triangles, size_t triangles_size);

    int compute_laplacian_based_inside(int smooth_range, double lambda, double value);
    int compute_laplacian_based_convexity(double radius_ratio, int smooth_range, double lambda);
    int compute_selected_segmentation(float *selected_points, size_t selected_points_size, double isovalue);
    int validate_selected_segmentation();
    void compute_geodesic_pad_outlines(int index, int flag_lower, int discretize_step, double isovalue);
    void recompute_geodesic_pad_outlines(int index, int flag_lower, double isovalue);
    void compute_tooth_cut_plane(int index, int flag_lower);
    // void compute_paired_attributes(int flag_lower, int *tooth_flags, size_t tooth_flags_size,
    //                                float *paired_fitting_plane_facets, size_t paired_fitting_plane_facets_size,
    //                                float *paired_fitting_plane_normals, size_t paired_fitting_plane_normals_size,
    //                                float *paired_fitting_lines, size_t paired_fitting_lines_size,
    //                                float *paired_fitting_points, size_t paired_fitting_points_size,
    //                                float *paired_plane_facets, size_t paired_plane_facets_size,
    //                                float *paired_plane_normals, size_t paired_plane_normals_size);

    // void update_teeth_mesh(DataList &teeth_facets, DataList &teeth_normals, int flag_lower);
    // void update_inside_function(DataList &inside_ray_facets,
    //                             DataList &inside_ray_normals,
    //                             DataList &inside_ray_colors,
    //                             DataList &inside_func_colors,
    //                             DataList &inside_facets,
    //                             DataList &inside_normals,
    //                             int flag_lower);
    // void update_convex_function(DataList &local_convex_facets,
    //                             DataList &local_convex_normals,
    //                             DataList &local_convex_colors,
    //                             DataList &convex_func_colors,
    //                             int flag_lower);
    // void update_convexity_isovalue(DataList &convex_isoedges, double value, int flag_lower);
    // void update_segmentation(DataList &seg_facets, DataList &seg_normals);
    // void update_validated_segmentation(int index, int flag_lower, DataList &seg_facets, DataList &seg_normals);
    // void update_tooth_geodesic_function(int index, int flag_lower, DataList &poly_colors);
    // void update_pad_outline(int index, int flag_lower, DataList &poly_facets, DataList &poly_normals);
    // void update_tooth_cut_plane(int index, int flag_lower, DataList &upper_bound, DataList &upper_points, DataList &cut_plane_points, DataList &cut_plane_normals);
    // void compute_plane_corners(Plane &plane, Point &center, Point &right, PointList &corners, Vector &normal);

    void *teeth_create_polyhedron(float *vertices, size_t vertices_size, int *triangles, size_t triangles_size);
    void teeth_delete_polyhedron(void *polyhedron);

    void *teeth_load(const char *path);
    void *teeth_create(float *vertices, size_t vertices_size, int *triangles, size_t triangles_size);
    void teeth_free(void *teeth);
    size_t teeth_vertices_size(void *teeth);
    void teeth_vertices(void *teeth, float *vertices, size_t vertices_size);

#ifdef __cplusplus
}
#endif
