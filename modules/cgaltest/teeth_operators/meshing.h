#ifndef MY_MESHING_H
#define MY_MESHING_H

#include "types.h"


namespace dentist {

void extract_inside_vertices_from_function(Polyhedron& poly, PointList& poly_points);

bool find_level_set_point(Point& point_a, double value_a, Point point_b, double value_b, double value, Point& p);

int extract_isovertices_from_function(Polyhedron& poly,
    VertDoubleMap& vert_value_map,
    double isovalue,
    PointList& poly_points,
    HEdgeIntMap& he_point_map);

// Extract a selected positive part from an implicit function
void extract_positive_faces_from_function(VertDoubleMap& vert_value_map,
    double value,
    VertIntMap& vert_ind_map,
    HEdgeIntMap& he_point_map,
    FacetList& candidate_facets,
    PolyList& poly_faces);

void copy_polyehdral_surface(Polyhedron& poly_in, Polyhedron& poly_out);

void merge_polyehdral_surface(Polyhedron& poly_in_1, Polyhedron& poly_in_2, Polyhedron& poly_out);

} // namespace dentist

#endif
