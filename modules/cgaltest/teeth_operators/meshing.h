#ifndef MY_MESHING_H
#define MY_MESHING_H

#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include "types.h"


namespace dentist {

inline void extract_inside_vertices_from_function(  Polyhedron& poly, 
                                                    PointList& poly_points)
{
    for(Vertex_handle vd: vertices(poly))
        poly_points.push_back(vd->point());
}

inline bool find_level_set_point(Point& point_a, double value_a, Point point_b, double value_b, double value, Point& p)
{
    double f1 = value_a - value;
    double f2 = value_b - value;

    // v1 and v2 have opposite function values while at least one is not 0.
    if((f1 * f2 <= 0) && !((std::abs(f1) < 1e-8) && (std::abs(f2) < 1e-8)))
    {
        if(std::abs(f1) < 1e-8) 
            p = point_a;
        else if(std::abs(f2) < 1e-8) 
            p = point_b;
        else if(f1 > 0 && f2 < 0) {
            double ratio = (0. - f1) / (f2 - f1);
            p = point_a + ratio * (point_b - point_a);
        }
        else {
            double ratio = (0. - f2) / (f1 - f2);
            p = point_b + ratio * (point_a - point_b);
        }
        return true;
    }
    return false;
}

inline int extract_isovertices_from_function(   Polyhedron& poly, 
                                                VertDoubleMap& vert_value_map, 
                                                double isovalue, 
                                                PointList& poly_points, 
                                                HEdgeIntMap& he_point_map)
{
    int index = (int)poly_points.size();

    for(Halfedge_iterator hedge = poly.halfedges_begin(); hedge != poly.halfedges_end(); ++hedge) 
    {
        Vertex_handle vert_a = hedge->vertex();
        Vertex_handle vert_b = hedge->opposite()->vertex();
        double value_a = vert_value_map[vert_a];
        double value_b = vert_value_map[vert_b];

        if(value_a > isovalue && value_b < isovalue)
        {
            Point point;
            bool flag = find_level_set_point(   vert_a->point(), 
                                                value_a, 
                                                vert_b->point(), 
                                                value_b, 
                                                isovalue, 
                                                point);
            if(flag)
            {
                poly_points.push_back(point);
                he_point_map.insert({hedge, index});
                index++;
            }
        }
    }

    return index;
}

// Extract a selected positive part from an implicit function
inline void extract_positive_faces_from_function(   VertDoubleMap& vert_value_map, 
                                                    double value, 
                                                    VertIntMap& vert_ind_map, 
                                                    HEdgeIntMap& he_point_map, 
                                                    FacetList& candidate_facets, 
                                                    PolyList& poly_faces)
{
    for(Facet_handle fd: candidate_facets)
    {
        // Count positive and negative number
        HEdgeList hedges;
        int pos_count = 0, neg_count = 0;
        Halfedge_facet_circulator he = fd->facet_begin();
        do {
            hedges.push_back(he);
            if(vert_value_map[he->vertex()] >= value)
                pos_count++;
            else
                neg_count++;
        } while (++he != fd->facet_begin());

        // case ---
        if(neg_count == 3)
            continue;

        // case +++
        if(pos_count == 3)   
        {
            IntList face;
            for(int i = 0; i < 3; i++)
                face.push_back(vert_ind_map[hedges[i]->vertex()]);
            poly_faces.push_back(face);
        }
        
        // case +--
        if(pos_count == 1 && neg_count == 2)   
        {
            IntList face;
            for(int i = 0; i < 3; i++)
            {
                Halfedge_handle hedge = hedges[i];
                double va = vert_value_map[hedge->vertex()];
                double vb = vert_value_map[hedge->opposite()->vertex()];
                if(va >= value && vb < value)
                    face.push_back(vert_ind_map[hedge->vertex()]);
                else if(va < value && vb >= value)
                    face.push_back(he_point_map[hedge->opposite()]);
                else
                    face.push_back(he_point_map[hedge->next()]);
            }
            poly_faces.push_back(face);
        }
        
        // case ++-
        if(pos_count == 2 && neg_count == 1)   
        {
            IntList face_1(3, 0), face_2(3, 0);
            for(int i = 0; i < 3; i++)
            {
                Halfedge_handle hedge = hedges[i];
                double va = vert_value_map[hedge->vertex()];
                double vb = vert_value_map[hedge->opposite()->vertex()];
                if(va >= value && vb >= value)
                {
                    int index = vert_ind_map[hedge->vertex()];
                    face_1[2] = index;
                    face_2[2] = index;
                }
                else if(va >= value && vb < value)
                {
                    int index = he_point_map[hedge];
                    face_1[1] = index;
                    face_2[0] = index;
                    face_2[1] = vert_ind_map[hedge->vertex()];
                }
                else
                    face_1[0] = he_point_map[hedge->opposite()];
            }
            poly_faces.push_back(face_1);
            poly_faces.push_back(face_2);
        }
    }
}

inline void copy_polyehdral_surface(Polyhedron& poly_in, Polyhedron& poly_out)
{
    PointList poly_points;
    PolyList  poly_faces;
    VertIntMap vert_index_map;
    int count = 0;

    for(Vertex_handle vd: vertices(poly_in))
    {
        Point p = vd->point();
        poly_points.push_back(p);
        vert_index_map.insert({vd, count});
        count++;
    }

    for(Facet_iterator face = poly_in.facets_begin(); face != poly_in.facets_end(); ++face) 
    {
        Halfedge_facet_circulator he = face->facet_begin();
        IntList face_inds;
        do {
            Vertex_handle vd = he->vertex();
            face_inds.push_back(vert_index_map[vd]);
        } while (++he != face->facet_begin());
        poly_faces.push_back(face_inds);
    }

    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, poly_out);
}

inline void merge_polyehdral_surface(Polyhedron& poly_in_1, Polyhedron& poly_in_2, Polyhedron& poly_out)
{
    PointList poly_points;
    PolyList  poly_faces;
    VertIntMap vert_index_map;
    int count = 0;

    for(Vertex_handle vd: vertices(poly_in_1))
    {
        Point p = vd->point();
        poly_points.push_back(p);
        vert_index_map.insert({vd, count});
        count++;
    }

    for(Vertex_handle vd: vertices(poly_in_2))
    {
        Point p = vd->point();
        poly_points.push_back(p);
        vert_index_map.insert({vd, count});
        count++;
    }

    for(Facet_iterator face = poly_in_1.facets_begin(); face != poly_in_1.facets_end(); ++face) 
    {
        Halfedge_facet_circulator he = face->facet_begin();
        IntList face_inds;
        do {
            Vertex_handle vd = he->vertex();
            face_inds.push_back(vert_index_map[vd]);
        } while (++he != face->facet_begin());
        poly_faces.push_back(face_inds);
    }

    for(Facet_iterator face = poly_in_2.facets_begin(); face != poly_in_2.facets_end(); ++face) 
    {
        Halfedge_facet_circulator he = face->facet_begin();
        IntList face_inds;
        do {
            Vertex_handle vd = he->vertex();
            face_inds.push_back(vert_index_map[vd]);
        } while (++he != face->facet_begin());
        poly_faces.push_back(face_inds);
    }

    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, poly_out);
}

} // namespace dentist

#endif
