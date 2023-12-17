#ifndef MY_TOOTH_H
#define MY_TOOTH_H

#include "types.h"

namespace dentist {

class Tooth
{
private:
    Polyhedron      m_tooth_poly;
    Point           m_center;
    VertIntMap      m_vert_ind_map;
    VertDoubleMap   m_vert_geodesic_map;

    HEdgeSet        m_upper_bound;
    VertexSet       m_upper_points;
    PointList       m_cut_points;
    Plane           m_cut_plane;

    Polyhedron      m_pad_outlines;
    Polyhedron      m_pad_shape;
    bool            m_pad_split_flag;
    Polyhedron      m_pad_left;
    Polyhedron      m_pad_right;

public:

    Tooth();

    ~Tooth();

    void reset();

    void reset_cut_plane();

    void reset_pad();

    void set_tooth_mesh(Polyhedron& seg_poly, Point centroid);

    double compute_length_for_hole(Halfedge_handle he, Polyhedron& mesh);

    void initialize_vertex_indices();

    bool get_init_flag();
    const Point& get_center();
    Polyhedron& get_tooth_poly();
    VertDoubleMap& get_geodesic_distance_map();
    Polyhedron& get_pad_outline_poly();
    HEdgeSet& get_upper_bound_set();
    PointList& get_cut_points();
    Plane& get_cut_plane();
    bool get_split_flag();
    Polyhedron& get_pad_shape_poly();
    Polyhedron& get_left_pad_shape_poly();
    Polyhedron& get_right_pad_shape_poly();

    bool compute_geodedic_pad_outlines(int discretize_step, double isovalue);

    void compute_geodesic_distance(int discretize_step);

    bool update_geodesic_pad_outlines(double isovalue);

    bool compute_tooth_cut_plane(KDTree& teeth_boundary_tree, AABBTree& teeth_tree);

    void compute_geodesic_center(Point& geodesic_center);

    void compute_boundary_halfedges(KDTree& boundary_tree, SegmentList& upper_bound_segs);

    void compute_boundary_midpoint(Line& fitting_line, Point& upper_bound_center);

    void compute_pad_shape(Polyhedron& wire_outlines, Polyhedron& wire_shape, double pad_height, double wire_height);

    void compute_splited_pad_shape( Polyhedron& left_wire_outlines, Polyhedron& left_wire_shape, 
                                    Polyhedron& right_wire_outlines, Polyhedron& right_wire_shape, 
                                    double pad_height, double wire_height);

}; // end of class Tooth

} // namespace dentist

#endif
