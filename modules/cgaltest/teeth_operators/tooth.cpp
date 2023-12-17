#include "tooth.h"

#include <boost/property_map/property_map.hpp>
#include <CGAL/bounding_box.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "meshing.h"
#include "extrusion.h"

namespace dentist {

Tooth::Tooth() {}

Tooth::~Tooth()
{
    reset();
}

void Tooth::reset()
{
    m_tooth_poly.clear();
    m_vert_ind_map.clear();
    m_vert_geodesic_map.clear();
    m_center = Point(0., 0., 0.);
    reset_cut_plane();
    reset_pad();
}

void Tooth::reset_cut_plane()
{
    m_upper_bound.clear();
    m_upper_points.clear();
    m_cut_points.clear();
    m_cut_plane = Plane(0., 0., 0., 1.);
}

void Tooth::reset_pad()
{
    m_pad_outlines.clear();
    m_pad_shape.clear();
    m_pad_left.clear();
    m_pad_right.clear();
    m_pad_split_flag = false;
}

void Tooth::set_tooth_mesh(Polyhedron& seg_poly, Point centroid)
{
    reset();

    m_center = centroid;
    copy_polyehdral_surface(seg_poly, m_tooth_poly);
    std::cout << "    Load tooth mesh with " << m_tooth_poly.size_of_vertices() << " vertices and " << m_tooth_poly.size_of_facets() << " faces." << std::endl;

    // hole filling
    HEdgeList border_cycles;
    CGAL::Polygon_mesh_processing::extract_boundary_cycles(m_tooth_poly, std::back_inserter(border_cycles));

    if (border_cycles.size() > 1) // contain holes
    {
        DoubleList border_lengths;
        for (int i = 0; i < border_cycles.size(); i++)
        {
            double length = compute_length_for_hole(border_cycles[i], m_tooth_poly);
            border_lengths.push_back(length);
        }

        unsigned int nb_holes = 0;
        std::vector<int> indices(border_lengths.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&](int A, int B) -> bool { return border_lengths[A] > border_lengths[B]; });

        for (int i = 1; i < indices.size(); i++)
        {
            FacetList  patch_facets;
            VertexList patch_vertices;
            bool success = std::get<0>(CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(m_tooth_poly,
                border_cycles[indices[i]],
                CGAL::parameters::face_output_iterator(std::back_inserter(patch_facets))
                .vertex_output_iterator(std::back_inserter(patch_vertices))
                .vertex_point_map(get(CGAL::vertex_point, m_tooth_poly))
                .geom_traits(Kernel())));
            ++nb_holes;
        }

        std::cout << "    Filled " << nb_holes << "holes!" << std::endl;
    }

    initialize_vertex_indices();
}

double Tooth::compute_length_for_hole(Halfedge_handle he, Polyhedron& mesh)
{
    double length = 0.;

    for (Halfedge_handle hc : CGAL::halfedges_around_face(he, mesh))
        length += std::sqrt(CGAL::squared_distance(hc->vertex()->point(), hc->opposite()->vertex()->point()));

    return length;
}

void Tooth::initialize_vertex_indices()
{
    m_vert_ind_map.clear();
    int index = 0;

    for (Vertex_handle vd : vertices(m_tooth_poly))
    {
        m_vert_ind_map.insert({ vd, index });
        index++;
    }
}

bool Tooth::get_init_flag() { return (m_tooth_poly.size_of_vertices() > 0); }
const Point& Tooth::get_center() { return m_center; }
Polyhedron& Tooth::get_tooth_poly() { return m_tooth_poly; }
VertDoubleMap& Tooth::get_geodesic_distance_map() { return m_vert_geodesic_map; }
Polyhedron& Tooth::get_pad_outline_poly() { return m_pad_outlines; }
HEdgeSet& Tooth::get_upper_bound_set() { return m_upper_bound; }
PointList& Tooth::get_cut_points() { return m_cut_points; }
Plane& Tooth::get_cut_plane() { return m_cut_plane; }
bool Tooth::get_split_flag() { return m_pad_split_flag; }
Polyhedron& Tooth::get_pad_shape_poly() { return m_pad_shape; }
Polyhedron& Tooth::get_left_pad_shape_poly() { return m_pad_left; }
Polyhedron& Tooth::get_right_pad_shape_poly() { return m_pad_right; }

bool Tooth::compute_geodedic_pad_outlines(int discretize_step, double isovalue)
{
    if (m_tooth_poly.size_of_vertices() == 0)
    {
        std::cout << "    No available tooth polyhedral surface!" << std::endl;
        return false;
    }

    compute_geodesic_distance(discretize_step);
    update_geodesic_pad_outlines(isovalue);

    return true;
}

void Tooth::compute_geodesic_distance(int discretize_step)
{
    // clear map
    m_vert_geodesic_map.clear();

    Geodesic_tree geodesic_tree(m_tooth_poly,
        get(boost::vertex_external_index, m_tooth_poly),
        get(CGAL::halfedge_external_index, m_tooth_poly),
        get(CGAL::face_external_index, m_tooth_poly),
        get(CGAL::vertex_point, m_tooth_poly));

    // discretize tooth outline
    double offset = 1. / discretize_step;
    std::vector<Face_location> faceLocations;

    for (Halfedge_iterator hedge = m_tooth_poly.halfedges_begin(); hedge != m_tooth_poly.halfedges_end(); ++hedge)
    {
        if (hedge->is_border()) // infinite face
        {
            Facet_handle face = hedge->opposite()->facet(); // opposite face
            Vertex_handle va = hedge->vertex();
            Vertex_handle vb = hedge->opposite()->vertex();
            Halfedge_handle face_hedge = face->halfedge()->prev();
            int index_a = -1, index_b = -1;

            for (int i = 0; i < 3; i++)
            {
                if (face_hedge->vertex() == va)
                    index_a = i;
                if (face_hedge->vertex() == vb)
                    index_b = i;
                face_hedge = face_hedge->next();
            }

            for (int i = 0; i < discretize_step; i++)  // discretize edge
            {
                DoubleList weights(3, 0);
                weights[index_a] = offset * i;
                weights[index_b] = 1 - weights[index_a];

                Geodesic_coordinates face_location = { {weights[0], weights[1], weights[2]} };
                faceLocations.push_back(Face_location(face, face_location));
            }
        }
    }

    geodesic_tree.add_source_points(faceLocations.begin(), faceLocations.end());

    for (Vertex_handle vd : vertices(m_tooth_poly))
    {
        double dist = geodesic_tree.shortest_distance_to_source_points(vd).first;
        m_vert_geodesic_map.insert({ vd, dist });
    }
}

bool Tooth::update_geodesic_pad_outlines(double isovalue)
{
    if (m_vert_geodesic_map.empty())
    {
        std::cout << "    No available geodesic distance map!" << std::endl;
        return false;
    }

    reset_pad();

    PointList poly_points;
    extract_inside_vertices_from_function(m_tooth_poly, poly_points);

    HEdgeIntMap he_point_map;
    int count = extract_isovertices_from_function(m_tooth_poly, m_vert_geodesic_map, isovalue, poly_points, he_point_map);

    FacetList candidate_facets;
    for (Facet_handle fd : faces(m_tooth_poly))
        candidate_facets.push_back(fd);

    PolyList  poly_faces;
    extract_positive_faces_from_function(m_vert_geodesic_map, isovalue, m_vert_ind_map, he_point_map, candidate_facets, poly_faces);

    CGAL::Polygon_mesh_processing::remove_isolated_points_in_polygon_soup(poly_points, poly_faces);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, m_pad_outlines);
    CGAL::Polygon_mesh_processing::keep_largest_connected_components(m_pad_outlines, 1);
    double area = CGAL::Polygon_mesh_processing::area(m_pad_outlines);

    std::cout << "    Compute pad mesh with " << m_pad_outlines.size_of_vertices() << " vertices and " << m_pad_outlines.size_of_facets() << " faces with area " << area << "." << std::endl;
    return true;
}

bool Tooth::compute_tooth_cut_plane(KDTree& teeth_boundary_tree, AABBTree& teeth_tree)
{
    if (m_vert_geodesic_map.empty())
    {
        std::cout << "    No available geodesic distance map!" << std::endl;
        return false;
    }

    reset_cut_plane();

    // Find three points to fix the cut plane
    //Point geodesic_center;
    //compute_geodesic_center(geodesic_center);
    //m_cut_points.push_back(geodesic_center);
    m_cut_points.push_back(m_center);

    SegmentList upper_bound_segs;
    compute_boundary_halfedges(teeth_boundary_tree, upper_bound_segs);
    Line fitting_line;
    CGAL::linear_least_squares_fitting_3(upper_bound_segs.begin(), upper_bound_segs.end(), fitting_line, CGAL::Dimension_tag<1>());
    Point upper_bound_center;
    compute_boundary_midpoint(fitting_line, upper_bound_center);
    m_cut_points.push_back(upper_bound_center);

    Point mid = CGAL::midpoint(m_cut_points[0], m_cut_points[1]);
    Point mid_proj = teeth_tree.closest_point(mid);
    m_cut_points.push_back(mid_proj);

    // Fit the plane
    m_cut_plane = Plane(m_cut_points[0], m_cut_points[1], m_cut_points[2]);
    std::cout << m_cut_points.size() << " " << upper_bound_segs.size() << std::endl;
    return true;
}

void Tooth::compute_geodesic_center(Point& geodesic_center)
{
    double value = -1e10;

    for (Vertex_handle vd : vertices(m_tooth_poly))
    {
        double dist = m_vert_geodesic_map[vd];
        if (dist > value)
        {
            value = dist;
            geodesic_center = vd->point();
        }
    }
}

void Tooth::compute_boundary_halfedges(KDTree& boundary_tree, SegmentList& upper_bound_segs)
{
    for (Halfedge_iterator hedge = m_tooth_poly.halfedges_begin(); hedge != m_tooth_poly.halfedges_end(); ++hedge)
    {
        if (hedge->is_border()) // infinite face
        {
            PointList neighbor_points;
            Fuzzy_circle target_query(hedge->vertex()->point(), 1e-5);
            boundary_tree.search(std::back_inserter(neighbor_points), target_query);
            bool flag_target = (neighbor_points.size() > 0);

            neighbor_points.clear();
            Fuzzy_circle source_query(hedge->opposite()->vertex()->point(), 1e-5);
            boundary_tree.search(std::back_inserter(neighbor_points), source_query);
            bool flag_source = (neighbor_points.size() > 0);

            if (flag_target && flag_source)
            {
                m_upper_bound.insert(hedge);
                upper_bound_segs.push_back(Segment(hedge->vertex()->point(), hedge->opposite()->vertex()->point()));
                m_upper_points.insert(hedge->vertex());
                m_upper_points.insert(hedge->opposite()->vertex());
            }

        }
    }
}

void Tooth::compute_boundary_midpoint(Line& fitting_line, Point& upper_bound_center)
{
    Point center = fitting_line.point(0);
    Vector direction = fitting_line.to_vector();
    direction = direction / std::sqrt(direction.squared_length());
    double min_scale = 1e10, max_scale = -1e10;

    for (auto& elem : m_upper_points)
    {
        Point proj = fitting_line.projection(elem->point());
        double value = std::sqrt(CGAL::squared_distance(proj, center));
        if ((proj - center) * direction < 0)
            value = -value;
        min_scale = std::min(min_scale, value);
        max_scale = std::max(max_scale, value);
    }

    // Cut with plane
    double avg_scale = (min_scale + max_scale) / 2.;
    upper_bound_center = center + avg_scale * direction;
}

void Tooth::compute_pad_shape(Polyhedron& wire_outlines, Polyhedron& wire_shape, double pad_height, double wire_height)
{
    if (m_pad_outlines.empty())
        return;

    if (wire_height <= pad_height)
    {
        std::cout << "Bad parameter!" << std::endl;
        return;
    }

    m_pad_shape.clear();
    m_pad_left.clear();
    m_pad_right.clear();
    m_pad_split_flag = false;

    Polyhedron wire_extrusion;

    compute_extrusion(m_pad_outlines, m_pad_shape, pad_height);
    compute_extrusion(wire_outlines, wire_extrusion, wire_height);

    CGAL::Polygon_mesh_processing::corefine_and_compute_union(m_pad_shape, wire_extrusion, m_pad_shape);
    CGAL::Polygon_mesh_processing::corefine_and_compute_difference(m_pad_shape, wire_shape, m_pad_shape);

    std::cout << "Computed pad shape with " << m_pad_shape.size_of_vertices() << " vertices and " << m_pad_shape.size_of_facets() << " faces!" << std::endl;
}

void Tooth::compute_splited_pad_shape(Polyhedron& left_wire_outlines, Polyhedron& left_wire_shape,
    Polyhedron& right_wire_outlines, Polyhedron& right_wire_shape,
    double pad_height, double wire_height)
{
    if (m_pad_outlines.empty())
        return;

    if (wire_height <= pad_height)
    {
        std::cout << "Bad parameter!" << std::endl;
        return;
    }

    m_pad_shape.clear();
    m_pad_left.clear();
    m_pad_right.clear();
    m_pad_split_flag = true;

    Polyhedron left_wire_extrusion, right_wire_extrusion;

    compute_extrusion(m_pad_outlines, m_pad_shape, pad_height);
    compute_extrusion(left_wire_outlines, left_wire_extrusion, wire_height);
    compute_extrusion(right_wire_outlines, right_wire_extrusion, wire_height);

    CGAL::Polygon_mesh_processing::corefine_and_compute_union(m_pad_shape, left_wire_extrusion, m_pad_shape);
    CGAL::Polygon_mesh_processing::corefine_and_compute_union(m_pad_shape, right_wire_extrusion, m_pad_shape);
    CGAL::Polygon_mesh_processing::corefine_and_compute_difference(m_pad_shape, left_wire_shape, m_pad_shape);
    CGAL::Polygon_mesh_processing::corefine_and_compute_difference(m_pad_shape, right_wire_shape, m_pad_shape);

    Point left_center = CGAL::ORIGIN;
    double ratio = 1. / left_wire_outlines.size_of_vertices();
    for (Vertex_handle vd : vertices(left_wire_outlines))
        left_center = left_center + ratio * (vd->point() - CGAL::ORIGIN);

    copy_polyehdral_surface(m_pad_shape, m_pad_left);
    copy_polyehdral_surface(m_pad_shape, m_pad_right);

    if ((left_center - m_cut_plane.projection(left_center)) * m_cut_plane.orthogonal_vector() < 0.)
    {
        CGAL::Polygon_mesh_processing::clip(m_pad_left, m_cut_plane);
        CGAL::Polygon_mesh_processing::clip(m_pad_right, m_cut_plane.opposite());
    }
    else
    {
        CGAL::Polygon_mesh_processing::clip(m_pad_right, m_cut_plane);
        CGAL::Polygon_mesh_processing::clip(m_pad_left, m_cut_plane.opposite());
    }

    std::cout << "Computed left pad shape with " << m_pad_left.size_of_vertices() << " vertices and " << m_pad_left.size_of_facets() << " faces!" << std::endl;
    std::cout << "Computed right pad shape with " << m_pad_right.size_of_vertices() << " vertices and " << m_pad_right.size_of_facets() << " faces!" << std::endl;
}

} // namespace dentist
