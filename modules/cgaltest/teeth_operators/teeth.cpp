#include "teeth.h"

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/centroid.h>

#include "types.h"
#include "ramp.h"
#include "solver.h"
#include "meshing.h"
#include "tooth.h"
#include "wire.h"
#include "extrusion.h"

namespace dentist {

//----------------------------- Initialization -----------------------------//

Teeth::Teeth()
{
    for (int i = 0; i < 6; i++)
        m_tooth.push_back(Tooth());

    for (int i = 0; i < 5; i++)
        m_wires.push_back(Wire());
}

Teeth::~Teeth()
{
    reset();
}

void Teeth::reset()
{
    m_teeth_poly.clear();
    m_vert_ray_map.clear();
    m_vert_ind_map.clear();
    m_vert_smooth_ray_map.clear();
    m_vert_inside_func_map.clear();

    reset_inside();

    for (int i = 0; i < m_tooth.size(); i++)
        m_tooth[i].reset();

    for (int i = 0; i < m_wires.size(); i++)
        m_wires[i].reset();
}

void Teeth::reset_inside()
{
    m_teeth_inside.clear();
    m_inside_vert_ind_map.clear();
    m_inside_vert_convex_map.clear();
    m_inside_vert_smooth_convex_map.clear();
    m_inside_vert_convex_func_map.clear();
    m_inside_tree.clear();
    m_inside_boundary_tree.clear();
}

void Teeth::reset_convexity()
{
    m_inside_vert_convex_map.clear();
    m_inside_vert_smooth_convex_map.clear();
    m_inside_vert_convex_func_map.clear();
}

bool Teeth::load_teeth_mesh(std::string filename)
{
    reset();

    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(filename, m_teeth_poly))
    {
        std::cout << "  Invalid teeth: " << filename << "." << std::endl;
        return false;
    }

    std::cout << "  Load teeth mesh with " << m_teeth_poly.size_of_vertices() << " vertices and " << m_teeth_poly.size_of_facets() << " faces." << std::endl;

    initialize_vertex_indices();

    return true;
}

bool Teeth::init_teeth_mesh(const float* vertices, size_t vertices_size, const int* triangles, size_t triangles_size)
{
    reset();

    PointList poly_points;
    PolyList poly_faces;

    for (size_t vi = 0; vi < vertices_size; ++vi)
    {
        const float* v = vertices + (vi * 3);
        Point p(v[0], v[2], v[3]);
        poly_points.push_back(p);
    }

    IntList face_inds;
    face_inds.resize(3);
    for (size_t ti = 0; ti < triangles_size; ++ti)
    {
        const int* tri = triangles + ti * 3;
        face_inds[0] = tri[0];
        face_inds[1] = tri[1];
        face_inds[2] = tri[2];
        poly_faces.push_back(face_inds);
    }

    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, m_teeth_poly);

    std::cout << "  Init teeth mesh with " << m_teeth_poly.size_of_vertices() << " vertices and " << m_teeth_poly.size_of_facets() << " faces." << std::endl;

    initialize_vertex_indices();

    return true;
}

Polyhedron& Teeth::get_teeth_poly() { return m_teeth_poly; }
VertDoubleMap& Teeth::get_smoothed_ray_map() { return m_vert_smooth_ray_map; }
VertDoubleMap& Teeth::get_vertex_inside_function_map() { return m_vert_inside_func_map; }
Polyhedron& Teeth::get_inside_teeth_poly() { return m_teeth_inside; }
VertDoubleMap& Teeth::get_smoothed_convex_map() { return m_inside_vert_smooth_convex_map; }
VertDoubleMap& Teeth::get_vertex_convex_function_map() { return m_inside_vert_convex_func_map; }

int Teeth::size_of_facets() { return static_cast<int>(m_teeth_poly.size_of_facets()); }
int Teeth::size_of_vertices() { return static_cast<int>(m_teeth_poly.size_of_vertices()); }
int Teeth::size_of_inside_facets() { return static_cast<int>(m_teeth_inside.size_of_facets()); }
int Teeth::size_of_inside_vertices() { return static_cast<int>(m_teeth_inside.size_of_vertices()); }

void Teeth::read_polygon_vertices(PointList& points)
{
    for (Vertex_handle vd : vertices(m_teeth_poly))
        points.push_back(vd->point());
}

void Teeth::initialize_vertex_indices()
{
    m_vert_ind_map.clear();
    int index = 0;

    for (Vertex_handle vd : vertices(m_teeth_poly))
    {
        m_vert_ind_map.insert({ vd, index });
        index++;
    }
}

double Teeth::get_zmax()
{
    double zmax = -1e10;

    for (Vertex_handle vd : vertices(m_teeth_poly))
        zmax = std::max(zmax, (vd->point()).z());

    return zmax;
}

//----------------------------- Inside Function -----------------------------//

bool Teeth::compute_laplacian_based_inside(AABBTree& teeth_tree, Point& center, int smooth_range, double lambda)
{
    // Initialize intersection field
    compute_intersection_ray(teeth_tree, center);

    // Smoothed intersection field
    compute_smoothed_field(smooth_range, m_vert_ray_map, m_vert_smooth_ray_map);

    // Initialize linear system
    int nb_variables = size_of_vertices();
    ESMatrix L(nb_variables * 2, nb_variables);
    EVector B(nb_variables * 2), X(nb_variables);

    ESTripleList LT;
    LT.reserve(12 * nb_variables);

    // Assemble system
    IntDoubleMap smoothed_ray_map;
    for (Vertex_handle vd : vertices(m_teeth_poly))
        smoothed_ray_map.insert({ m_vert_ind_map[vd], m_vert_smooth_ray_map[vd] });

    assemble_laplacian_matrix(m_teeth_poly, LT, B, m_vert_ind_map);
    assemble_constraint_matrix(lambda, LT, B, smoothed_ray_map, nb_variables);

    L.setFromTriplets(LT.begin(), LT.end());
    LT.clear();
    bool flag_solver = solve_laplacian(L, B, X);

    if (!flag_solver)
        return false;

    // Assign value
    m_vert_inside_func_map.clear();
    for (Vertex_handle vd : vertices(m_teeth_poly))
        m_vert_inside_func_map.insert({ vd, X[m_vert_ind_map[vd]] });

    return true;
}

void Teeth::compute_intersection_ray(AABBTree& teeth_tree, Point& center)
{
    if (!m_vert_ray_map.empty())
        return;

    for (Vertex_handle vd : vertices(m_teeth_poly))
    {
        Vector direction = vd->point() - center;
        Segment segment(center, center + direction * 1.00001);
        int number = (int)teeth_tree.number_of_intersected_primitives(segment);

        if (number == 1)
            m_vert_ray_map[vd] = 1.;
        else
            m_vert_ray_map[vd] = 0.;
    }
}

double Teeth::compute_distance_to_inside_teeth(Point& point)
{
    if (m_inside_tree.empty())
        return -1.;

    double dist = std::sqrt(m_inside_tree.squared_distance(point));
    return dist;
}

void Teeth::extract_inside_from_function(double value)
{
    if (m_vert_inside_func_map.empty())
        return;

    reset_inside();

    // Fill holes
    VertDoubleMap vert_inside_func_fill_map;
    fill_holes_in_vertmap(m_teeth_poly, m_vert_inside_func_map, value, vert_inside_func_fill_map);

    PointList poly_points;
    extract_inside_vertices_from_function(m_teeth_poly, poly_points);

    HEdgeIntMap he_point_map;
    int count = extract_isovertices_from_function(m_teeth_poly, vert_inside_func_fill_map, value, poly_points, he_point_map);

    FacetList candidate_facets;
    for (Facet_handle fd : faces(m_teeth_poly))
        candidate_facets.push_back(fd);

    PolyList  poly_faces;
    extract_positive_faces_from_function(vert_inside_func_fill_map, value, m_vert_ind_map, he_point_map, candidate_facets, poly_faces);

    CGAL::Polygon_mesh_processing::remove_isolated_points_in_polygon_soup(poly_points, poly_faces);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, m_teeth_inside);
    CGAL::Polygon_mesh_processing::keep_largest_connected_components(m_teeth_inside, 1);
    std::cout << "  Compute inside mesh with " << m_teeth_inside.size_of_vertices() << " vertices and " << m_teeth_inside.size_of_facets() << " faces." << std::endl;

    // initialize vertex index map for inside teeth
    initialize_inside_vertex_index();
    // initialize aabbtree
    m_inside_tree.insert(faces(m_teeth_inside).first, faces(m_teeth_inside).second, m_teeth_inside);
    m_inside_tree.accelerate_distance_queries();
    // initialize boundary kdtree
    initialize_inside_boundary_kdtree();
}

void Teeth::initialize_inside_vertex_index()
{
    if (m_teeth_inside.size_of_vertices() > 0)
    {
        int index = 0;
        for (Vertex_handle vd : vertices(m_teeth_inside))
        {
            m_inside_vert_ind_map.insert({ vd, index });
            index++;
        }
    }
}

void Teeth::initialize_inside_boundary_kdtree()
{
    PointList boundary_verts;

    for (Halfedge_iterator hedge = m_teeth_inside.halfedges_begin(); hedge != m_teeth_inside.halfedges_end(); ++hedge)
    {
        if (hedge->is_border()) // infinite face
        {
            Point source = hedge->opposite()->vertex()->point();
            Vector offset = hedge->vertex()->point() - source;

            for (int i = 0; i < 20; i++)
                boundary_verts.push_back(source + i * 0.05 * offset);
        }
    }

    m_inside_boundary_tree.insert(boundary_verts.begin(), boundary_verts.end());
}

void Teeth::fill_holes_in_vertmap(Polyhedron& poly, VertDoubleMap& vert_map, double isovalue, VertDoubleMap& new_vert_map)
{
    VertexSet finished_set;
    std::vector<VertexList> connected_components;

    for (Vertex_handle vd : vertices(poly))
    {
        new_vert_map.insert({ vd, vert_map[vd] });

        if (finished_set.find(vd) != finished_set.end())
            continue;

        if (vert_map[vd] >= isovalue)
        {
            finished_set.insert(vd);
            continue;
        }

        VertexList component;
        VertexQueue to_finish_queue;
        to_finish_queue.push(vd);

        while (!to_finish_queue.empty())
        {
            Vertex_handle curr_vert = to_finish_queue.front();
            to_finish_queue.pop();

            if (finished_set.find(curr_vert) == finished_set.end())
            {
                finished_set.insert(curr_vert);
                component.push_back(curr_vert);

                Halfedge_vertex_circulator vd_begin = curr_vert->vertex_begin();
                do {
                    Vertex_handle nb_vert = vd_begin->opposite()->vertex();
                    if ((finished_set.find(nb_vert) == finished_set.end()) && vert_map[nb_vert] < isovalue)
                        to_finish_queue.push(nb_vert);
                } while (++vd_begin != curr_vert->vertex_begin());
            }
        }

        if (component.size() > 0)
            connected_components.push_back(component);
    }

    if (connected_components.size() <= 1)
        return;

    std::sort(connected_components.begin(), connected_components.end(), [&](VertexList A, VertexList B) -> bool { return A.size() > B.size(); });

    for (int i = 1; i < connected_components.size(); i++)
    {
        for (int j = 0; j < connected_components[i].size(); j++)
            new_vert_map[connected_components[i][j]] = isovalue + 1e-3;
    }
}

//----------------------------- Convexity Function -----------------------------//

bool Teeth::compute_laplacian_based_convexity(double radius_ratio, double bounding_radius, int smooth_range, double lambda)
{
    reset_convexity();

    // initialize convexity map
    compute_convexity_from_sampling(radius_ratio, bounding_radius);

    // smooth convexity map
    compute_smoothed_field(smooth_range, m_inside_vert_convex_map, m_inside_vert_smooth_convex_map);

    // Initialize linear system
    int nb_variables = size_of_inside_vertices();
    ESMatrix L(nb_variables * 2, nb_variables);
    EVector B(nb_variables * 2), X(nb_variables);

    ESTripleList LT;
    LT.reserve(12 * nb_variables);

    // Assemble system
    IntDoubleMap smoothed_convex_map;
    for (Vertex_handle vd : vertices(m_teeth_inside))
        smoothed_convex_map.insert({ m_inside_vert_ind_map[vd], m_inside_vert_smooth_convex_map[vd] });

    assemble_laplacian_matrix(m_teeth_inside, LT, B, m_inside_vert_ind_map);
    assemble_constraint_matrix(lambda, LT, B, smoothed_convex_map, nb_variables);

    L.setFromTriplets(LT.begin(), LT.end());
    LT.clear();
    bool flag_solver = solve_laplacian(L, B, X);

    if (!flag_solver)
        return false;

    // Assign value
    for (Vertex_handle vd : vertices(m_teeth_inside))
        m_inside_vert_convex_func_map.insert({ vd, X[m_inside_vert_ind_map[vd]] });

    return true;
}

void Teeth::compute_convexity_from_sampling(double radius_ratio, double bounding_radius)
{
    PointList samples;
    CGAL::Polygon_mesh_processing::sample_triangle_mesh(m_teeth_inside,
        std::back_inserter(samples),
        CGAL::Polygon_mesh_processing::parameters::use_random_uniform_sampling(true)
        .number_of_points_on_faces(10));
    std::cout << "  Sampled " << samples.size() << " points on the surface." << std::endl;

    KDTree tree(samples.begin(), samples.end());
    double nb_radius = bounding_radius * radius_ratio;

    for (Vertex_handle vd : vertices(m_teeth_inside))
    {
        PointList neighbor_points;
        Fuzzy_circle query(vd->point(), nb_radius);
        tree.search(std::back_inserter(neighbor_points), query);

        if (neighbor_points.size() == 0)
            continue;

        Vector mean_coord = CGAL::NULL_VECTOR;

        for (int i = 0; i < neighbor_points.size(); i++)
        {
            Point neighbor = neighbor_points[i];
            mean_coord += (vd->point() - neighbor);
        }

        mean_coord = mean_coord / (double)neighbor_points.size();
        Vector normal = CGAL::Polygon_mesh_processing::compute_vertex_normal(vd, m_teeth_inside);
        double convexity = mean_coord * normal;
        m_inside_vert_convex_map.insert({ vd, convexity });
    }
}

//----------------------------- Segmentation -----------------------------//

void Teeth::compute_selected_segmentation(PointList& selected_points, double isovalue, Polyhedron& segment_poly)
{
    if (m_inside_vert_convex_func_map.empty())
    {
        std::cout << "  No available convexity function map for extracting segmentation." << std::endl;
        return;
    }

    FacetSet finished_set;
    FacetList candidate_facets;
    FacetQueue to_finish_queue;

    for (int i = 0; i < selected_points.size(); i++)
    {
        Point_and_primitive_id pp = m_inside_tree.closest_point_and_primitive(selected_points[i]);
        Point point = pp.first;
        Facet_handle face = pp.second;
        double value = locate_and_evaluate_function(point, face, m_inside_vert_convex_func_map);
        if (value > isovalue)
            to_finish_queue.push(std::make_pair(face, true));
        else
            to_finish_queue.push(std::make_pair(face, false));
    }

    VertDoubleMap vert_value_map;

    while (!to_finish_queue.empty())
    {
        FacetPair elem = to_finish_queue.front();
        to_finish_queue.pop();
        Facet_handle curr_face = elem.first;
        bool flag_pos = elem.second;

        // skip if already processed
        if (finished_set.find(curr_face) != finished_set.end())
            continue;
        finished_set.insert(curr_face);

        // find pos and neg counts
        int pos_count = 0;
        HEdgeList hedges;
        Halfedge_facet_circulator he = curr_face->facet_begin();
        do {
            hedges.push_back(he);
            if (m_inside_vert_convex_func_map[he->vertex()] >= isovalue)
                pos_count++;
        } while (++he != curr_face->facet_begin());

        if (flag_pos && pos_count == 0)
            continue;
        if (!flag_pos && pos_count == 3)
            continue;

        // insert face to the candidate facet list
        candidate_facets.push_back(curr_face);

        // modify the isovalues for negative zones and keep the original isovalues for positive zones
        for (int i = 0; i < 3; i++)
        {
            Vertex_handle vd = hedges[i]->vertex();
            if (!flag_pos)
                vert_value_map[vd] = isovalue + 1.;
            else
            {
                if (vert_value_map.find(vd) == vert_value_map.end())
                    vert_value_map[vd] = m_inside_vert_convex_func_map[vd];
            }
        }

        // insert all neighbor faces into priority queue
        if (pos_count == 3 || pos_count == 0)
        {
            for (int i = 0; i < 3; i++)
            {
                Vertex_handle curr_vert = hedges[i]->vertex();
                Halfedge_vertex_circulator vd_begin = curr_vert->vertex_begin();
                do {
                    if (!vd_begin->is_border())
                    {
                        Facet_handle oppo_face = vd_begin->facet();
                        to_finish_queue.push(std::make_pair(oppo_face, flag_pos));
                    }
                } while (++vd_begin != curr_vert->vertex_begin());
            }
        }
    }

    PointList poly_points;
    extract_inside_vertices_from_function(m_teeth_inside, poly_points);

    HEdgeIntMap he_point_map;
    int count = extract_isovertices_from_function(m_teeth_inside, vert_value_map, isovalue, poly_points, he_point_map);

    PolyList  poly_faces;
    extract_positive_faces_from_function(vert_value_map, isovalue, m_inside_vert_ind_map, he_point_map, candidate_facets, poly_faces);

    segment_poly.clear();
    CGAL::Polygon_mesh_processing::remove_isolated_points_in_polygon_soup(poly_points, poly_faces);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, segment_poly);
    //CGAL::Polygon_mesh_processing::keep_largest_connected_components(segment_poly, 1);
    std::cout << "  Compute segment mesh with " << segment_poly.size_of_vertices() << " vertices and " << segment_poly.size_of_facets() << " faces." << std::endl;
}

bool Teeth::validate_selected_segmentation(Polyhedron& segment_poly)
{
    for (int i = 0; i < 6; i++)
        m_tooth[i].reset();

    Polyhedron orig_poly;
    copy_polyehdral_surface(segment_poly, orig_poly);

    std::vector<Polyhedron> split_meshes;
    CGAL::Polygon_mesh_processing::split_connected_components(segment_poly, split_meshes);
    std::cout << "  Found " << split_meshes.size() << " components!" << std::endl;

    if (split_meshes.size() > 6)
    {
        std::cout << "  Found too many components!" << std::endl;
        return false;
    }

    PointList centroids;
    for (int i = 0; i < split_meshes.size(); i++)
    {
        Bbox bounding_box = CGAL::Polygon_mesh_processing::bbox(split_meshes[i]);
        Point center = Point(0.5 * (bounding_box.xmin() + bounding_box.xmax()),
            0.5 * (bounding_box.ymin() + bounding_box.ymax()),
            0.5 * (bounding_box.zmin() + bounding_box.zmax()));
        centroids.push_back(center);
    }

    std::vector<int> indices(centroids.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int A, int B) -> bool { return centroids[A].x() > centroids[B].x(); });

    for (int i = 0; i < indices.size(); i++)
    {
        m_tooth[i].set_tooth_mesh(split_meshes[indices[i]], centroids[indices[i]]);
    }

    return true;
}

//----------------------------- Tooth Function -----------------------------//

Polyhedron& Teeth::get_tooth_poly(int index) { return m_tooth[index].get_tooth_poly(); }
VertDoubleMap& Teeth::get_geodesic_distance_map(int index) { return m_tooth[index].get_geodesic_distance_map(); }
Polyhedron& Teeth::get_pad_outline_poly(int index) { return m_tooth[index].get_pad_outline_poly(); }

HEdgeSet& Teeth::get_upper_bound_set(int index) { return m_tooth[index].get_upper_bound_set(); }
PointList& Teeth::get_cut_points(int index) { return m_tooth[index].get_cut_points(); }
Plane& Teeth::get_cut_plane(int index) { return m_tooth[index].get_cut_plane(); }

Wire& Teeth::get_wire(int index) { return m_wires[index]; }

Tooth& Teeth::get_tooth(int index) { return m_tooth[index]; }

void Teeth::compute_geodesic_pad_outlines(int index, int discretize_step, double isovalue)
{
    if (m_tooth[index].get_init_flag())
        m_tooth[index].compute_geodedic_pad_outlines(discretize_step, isovalue);
}

void Teeth::recompute_geodesic_pad_outlines(int index, double isovalue)
{
    if (m_tooth[index].get_init_flag())
        m_tooth[index].update_geodesic_pad_outlines(isovalue);
}

void Teeth::compute_tooth_cut_plane(int index)
{
    if (m_tooth[index].get_init_flag())
        m_tooth[index].compute_tooth_cut_plane(m_inside_boundary_tree, m_inside_tree);
}

void Teeth::compute_pad_shapes(int index, double pad_height, double wire_height)
{
    if (m_tooth[index].get_init_flag())
    {
        if (index == 0)
        {
            Polyhedron& wire_outlines = m_wires[0].get_wire_right_shape();
            Polyhedron& wire_shape = m_wires[0].get_wire_extrude_shape();
            m_tooth[index].compute_pad_shape(wire_outlines, wire_shape, pad_height, wire_height);
        }
        else if (index == 5)
        {
            Polyhedron& wire_outlines = m_wires[4].get_wire_left_shape();
            Polyhedron& wire_shape = m_wires[4].get_wire_extrude_shape();
            m_tooth[index].compute_pad_shape(wire_outlines, wire_shape, pad_height, wire_height);
        }
        else
        {
            Polyhedron& left_wire_outlines = m_wires[index - 1].get_wire_left_shape();
            Polyhedron& right_wire_outlines = m_wires[index].get_wire_right_shape();
            Polyhedron& left_wire_shape = m_wires[index - 1].get_wire_extrude_shape();
            Polyhedron& right_wire_shape = m_wires[index].get_wire_extrude_shape();
            m_tooth[index].compute_splited_pad_shape(left_wire_outlines, left_wire_shape, right_wire_outlines, right_wire_shape, pad_height, wire_height);
        }

    }
}

//----------------------------- Wire Function -----------------------------//

void Teeth::compute_paired_wires(int index, double distance)
{
    if (!m_tooth[index].get_init_flag() || !m_tooth[index + 1].get_init_flag())
        return;

    Polyhedron& right_tooth = get_tooth_poly(index);
    Polyhedron& left_tooth = get_tooth_poly(index + 1);

    // Pair fitting plane
    TriangleList candidate_faces;
    copy_faces_to_triangles(left_tooth, candidate_faces);
    copy_faces_to_triangles(right_tooth, candidate_faces);
    Plane fitting_plane;
    CGAL::linear_least_squares_fitting_3(candidate_faces.begin(), candidate_faces.end(), fitting_plane, CGAL::Dimension_tag<2>());
    Point pair_centroid = CGAL::centroid(candidate_faces.begin(), candidate_faces.end());

    // Pair fitting line
    PointList left_boundaries, right_boundaries;
    SegmentList segment_boundaries;
    HEdgeSet& right_bound_hedge_set = get_upper_bound_set(index);
    HEdgeSet& left_bound_hedge_set = get_upper_bound_set(index + 1);
    copy_boundary_edges_to_segments(left_bound_hedge_set, left_boundaries, segment_boundaries);
    copy_boundary_edges_to_segments(right_bound_hedge_set, right_boundaries, segment_boundaries);
    Line fitting_line;
    CGAL::linear_least_squares_fitting_3(segment_boundaries.begin(), segment_boundaries.end(), fitting_line, CGAL::Dimension_tag<1>());

    // Pair fitting point
    Point left_left_point, left_right_point;
    project_points_on_line(left_boundaries, fitting_line, left_left_point, left_right_point);
    Point right_left_point, right_right_point;
    project_points_on_line(right_boundaries, fitting_line, right_left_point, right_right_point);
    left_left_point = fitting_line.projection(left_left_point);
    right_right_point = fitting_line.projection(right_right_point);
    Point fitting_point = CGAL::midpoint(left_right_point, right_left_point);

    // Pair cut plane
    Plane pair_plane;
    compute_pair_plane(distance, fitting_point, left_left_point, right_right_point, pair_centroid, fitting_plane, pair_plane);
    Polyhedron pair_plane_mesh;
    compute_plane_mesh(10., pair_plane, fitting_point, right_right_point, pair_plane_mesh);

    // Compute wire line
    Plane right_plane = get_cut_plane(index);
    Plane left_plane = get_cut_plane(index + 1);
    clip_mesh_by_plane(pair_plane_mesh, left_plane, fitting_point);
    clip_mesh_by_plane(pair_plane_mesh, right_plane, fitting_point);

    std::vector<PointList> polylines;
    CGAL::Polygon_mesh_processing::surface_intersection(m_teeth_inside, pair_plane_mesh, std::back_inserter(polylines));

    // Init Wire class
    m_wires[index].reset();
    m_wires[index].set_fitting_plane(fitting_plane, fitting_point, right_right_point);
    Segment fitting_segment(left_left_point, right_right_point);
    m_wires[index].set_fitting_segment(fitting_segment);
    m_wires[index].set_fitting_point(fitting_point);
    m_wires[index].set_cut_plane(pair_plane, fitting_point, right_right_point);

    if (polylines.size() == 0)
    {
        std::cout << "Wrong intersection!" << std::endl;
        m_wires[index].set_flag(false);
        return;
    }
    else if (polylines.size() > 1)
    {
        size_t max_size = -1;
        int max_id = -1;
        for (int i = 0; i < polylines.size(); i++)
        {
            if (polylines[i].size() > max_size)
            {
                max_size = polylines[i].size();
                max_id = i;
            }
        }
        m_wires[index].set_polylines(polylines[max_id]);
        m_wires[index].set_flag(true);
    }
    else
    {
        m_wires[index].set_polylines(polylines[0]);
        m_wires[index].set_flag(true);
    }

    return;
}

void Teeth::copy_faces_to_triangles(Polyhedron& tooth_poly, TriangleList& triangles)
{
    for (Facet_iterator face = tooth_poly.facets_begin(); face != tooth_poly.facets_end(); ++face)
    {
        Halfedge_facet_circulator he = face->facet_begin();
        PointList triangle;
        do {
            triangle.push_back(he->vertex()->point());
        } while (++he != face->facet_begin());

        if (triangle.size() == 3)
        {
            Triangle tri(triangle[0], triangle[1], triangle[2]);
            triangles.push_back(tri);
        }
    }
}

void Teeth::copy_boundary_edges_to_segments(HEdgeSet& bound_hedge_set, PointList& points, SegmentList& segments)
{
    for (auto& hedge : bound_hedge_set)
    {
        points.push_back(hedge->vertex()->point());
        points.push_back(hedge->opposite()->vertex()->point());
        segments.push_back(Segment(hedge->vertex()->point(), hedge->opposite()->vertex()->point()));
    }
}

void Teeth::project_points_on_line(PointList& candidate_points, Line& fitting_line, Point& left_point, Point& right_point)
{
    Point center = fitting_line.projection(candidate_points[0]);
    Vector direction = fitting_line.to_vector();
    Vector xaxis(1., 0., 0.);
    if (direction * xaxis < 0.)
        direction = -direction;

    double fmin = 0;
    double fmax = 0;
    int min_index = 0;
    int max_index = 0;

    for (int i = 0; i < candidate_points.size(); i++)
    {
        Point proj = fitting_line.projection(candidate_points[i]);
        double value = CGAL::squared_distance(proj, center);
        if ((proj - center) * direction < 0)
            value = -value;

        if (value < fmin)
        {
            fmin = value;
            min_index = i;
        }
        if (value > fmax)
        {
            fmax = value;
            max_index = i;
        }
    }

    left_point = candidate_points[min_index];
    right_point = candidate_points[max_index];
}

void Teeth::compute_pair_plane(double distance, Point& plane_center, Point& left_left_point, Point& right_right_point, Point& pair_center, Plane& fitting_plane, Plane& pair_plane)
{
    Vector horizontal = right_right_point - left_left_point;
    Vector vertical = fitting_plane.orthogonal_vector();

    Vector normal = CGAL::cross_product(horizontal, vertical);
    normal = normal / std::sqrt(normal.squared_length());
    if (normal * (pair_center - plane_center) < 0.)
        normal = -normal;

    Point moved_center = plane_center + distance * normal;
    pair_plane = Plane(moved_center, normal);
}

void Teeth::compute_plane_mesh(double scale, Plane& plane, Point& center, Point& right, Polyhedron& plane_mesh)
{
    Point plane_center = plane.projection(center);
    Point plane_right = plane.projection(right);
    Vector horizontal = plane_right - plane_center;
    horizontal = horizontal / std::sqrt(horizontal.squared_length());
    Vector normal = plane.orthogonal_vector();
    Vector vertical = CGAL::cross_product(horizontal, normal);
    vertical = vertical / std::sqrt(vertical.squared_length());

    PointList corners;
    Point left_down = plane_center - scale * horizontal - scale * vertical;
    Point left_up = plane_center - scale * horizontal + scale * vertical;
    Point right_up = plane_center + scale * horizontal + scale * vertical;
    Point right_down = plane_center + scale * horizontal - scale * vertical;
    corners.push_back(left_down);
    corners.push_back(left_up);
    corners.push_back(right_up);
    corners.push_back(right_down);

    PolyList faces;
    faces.push_back(IntList({ 0, 1, 3 }));
    faces.push_back(IntList({ 1, 2, 3 }));

    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(corners, faces, plane_mesh);
}

void Teeth::compute_wire_shapes(int index, int discretize_step, double width)
{
    if (m_teeth_inside.size_of_vertices() == 0 || !m_wires[index].get_flag())
        return;

    Polyhedron& right_tooth = get_pad_outline_poly(index);
    Polyhedron& left_tooth = get_pad_outline_poly(index + 1);
    Plane right_plane = get_cut_plane(index);
    Plane left_plane = get_cut_plane(index + 1);
    Point fitting_point = m_wires[index].get_fitting_point();

    Polyhedron inter_teeth;
    copy_polyehdral_surface(m_teeth_inside, inter_teeth);
    clip_mesh_by_plane(inter_teeth, left_plane, fitting_point);
    clip_mesh_by_plane(inter_teeth, right_plane, fitting_point);

    Geodesic_tree geodesic_tree(inter_teeth,
        get(boost::vertex_external_index, inter_teeth),
        get(CGAL::halfedge_external_index, inter_teeth),
        get(CGAL::face_external_index, inter_teeth),
        get(CGAL::vertex_point, inter_teeth));
    AABBTree geodesic_aabb_tree;
    geodesic_tree.build_aabb_tree(geodesic_aabb_tree);
    geodesic_aabb_tree.accelerate_distance_queries();

    // discretize segments
    double offset = 1. / discretize_step;
    std::vector<Face_location> faceLocations;

    SegmentList wire_outlines = m_wires[index].get_polylines();
    for (int j = 0; j < wire_outlines.size(); j++)
    {
        Segment& seg = wire_outlines[j];
        Point source = seg.source();
        Vector direction = seg.target() - seg.source();
        for (int i = 0; i < discretize_step; i++)  // discretize edge
        {
            Point p = source + i * offset * direction;
            Face_location source_loc = geodesic_tree.locate<AABBTraits>(p, geodesic_aabb_tree);
            if (source_loc.first != boost::graph_traits<Polyhedron>::null_face())
                faceLocations.push_back(source_loc);
        }
    }
    geodesic_tree.add_source_points(faceLocations.begin(), faceLocations.end());

    Polyhedron left_halftooth, right_halftooth;
    copy_polyehdral_surface(right_tooth, right_halftooth);
    clip_mesh_by_plane(right_halftooth, right_plane, fitting_point);
    copy_polyehdral_surface(left_tooth, left_halftooth);
    clip_mesh_by_plane(left_halftooth, left_plane, fitting_point);

    Polyhedron left_halfwire, right_halfwire;
    Point left_upper_point, right_upper_point, left_lower_point, right_lower_point;
    bool flag_right = extract_wire_shape_for_halftooth(width, right_plane, geodesic_tree, geodesic_aabb_tree, right_halftooth, right_halfwire, right_upper_point, right_lower_point);
    bool flag_left = extract_wire_shape_for_halftooth(width, left_plane, geodesic_tree, geodesic_aabb_tree, left_halftooth, left_halfwire, left_upper_point, left_lower_point);

    if (!flag_right || !flag_left)
        return;

    Polyhedron wire_shape;

    Polyhedron middle_wire;
    PointList bound_points;
    bound_points.push_back(right_upper_point);
    bound_points.push_back(right_lower_point);
    bound_points.push_back(left_lower_point);
    bound_points.push_back(left_upper_point);
    PolyList bound_polys;
    bound_polys.push_back(IntList({ 0, 1, 2 }));
    bound_polys.push_back(IntList({ 0, 2, 3 }));
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(bound_points, bound_polys, middle_wire);
    Polyhedron middle_result;
    merge_polyehdral_surface(left_halfwire, right_halfwire, middle_result);
    merge_polyehdral_surface(middle_result, middle_wire, wire_shape);

    m_wires[index].set_wire_left_shape(left_halfwire);
    m_wires[index].set_wire_middle_shape(middle_wire);
    m_wires[index].set_wire_right_shape(right_halfwire);
    m_wires[index].set_wire_shape(wire_shape);

    // Extrude
    Polyhedron extrude_mesh, extrude_leftwire, extrude_rightwire, extrude_middle;

    compute_extrusion(left_halfwire, extrude_leftwire, width * 2.);
    compute_extrusion(right_halfwire, extrude_rightwire, width * 2.);
    compute_extrusion(middle_wire, extrude_middle, width * 2.);

    CGAL::Polygon_mesh_processing::corefine_and_compute_union(extrude_leftwire, extrude_middle, extrude_mesh);
    CGAL::Polygon_mesh_processing::corefine_and_compute_union(extrude_mesh, extrude_rightwire, extrude_mesh);


    m_wires[index].set_wire_extrude_shape(extrude_mesh);

    std::cout << "Finish!" << std::endl;
}

void Teeth::clip_mesh_by_plane(Polyhedron& mesh, Plane& plane, Point& center)
{
    if ((center - plane.point()) * plane.orthogonal_vector() > 0.)
        CGAL::Polygon_mesh_processing::clip(mesh, plane.opposite());
    else
        CGAL::Polygon_mesh_processing::clip(mesh, plane);
}

bool Teeth::extract_wire_shape_for_halftooth(double width, Plane& cut_plane, Geodesic_tree& geodesic_tree, AABBTree& geodesic_aabb_tree, Polyhedron& halftooth,  // input
    Polyhedron& halfwire, Point& upper_bound, Point& lower_bound)   // output
{
    // isotropic remeshing
    std::cout << "  Before isotropic remeshing: " << halftooth.size_of_vertices() << " vertices and " << halftooth.size_of_facets() << std::endl;
    CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(halftooth), 0.8 * width, halftooth);
    std::cout << "  After isotropic remeshing: " << halftooth.size_of_vertices() << " vertices and " << halftooth.size_of_facets() << std::endl;

    VertDoubleMap vert_geodesic_map;
    VertIntMap vert_ind_map;
    int count = 0;
    for (Vertex_handle vd : vertices(halftooth))
    {
        Point proj = geodesic_aabb_tree.closest_point(vd->point());
        Face_location proj_loc = geodesic_tree.locate<AABBTraits>(proj, geodesic_aabb_tree);
        double dist = geodesic_tree.shortest_distance_to_source_points(proj_loc.first, proj_loc.second).first;
        if (dist < 0.)
            vert_geodesic_map.insert({ vd, -width - 1. });
        else
            vert_geodesic_map.insert({ vd, -dist });
        vert_ind_map.insert({ vd, count });
        count++;
    }

    // Extract surface
    PointList poly_points;
    extract_inside_vertices_from_function(halftooth, poly_points);

    HEdgeIntMap he_point_map;
    extract_isovertices_from_function(halftooth, vert_geodesic_map, -width, poly_points, he_point_map);

    PointList bounds;
    for (Halfedge_iterator hedge = halftooth.halfedges_begin(); hedge != halftooth.halfedges_end(); ++hedge)
    {
        if (hedge->is_border_edge() && he_point_map.find(hedge) != he_point_map.end())
        {
            Point p = poly_points[he_point_map[hedge]];
            if (CGAL::squared_distance(p, cut_plane.projection(p)) > 1.)
                bounds.push_back(p);
        }
    }

    FacetList candidate_facets;
    for (Facet_handle fd : faces(halftooth))
        candidate_facets.push_back(fd);

    PolyList  poly_faces;
    extract_positive_faces_from_function(vert_geodesic_map, -width, vert_ind_map, he_point_map, candidate_facets, poly_faces);
    CGAL::Polygon_mesh_processing::remove_isolated_points_in_polygon_soup(poly_points, poly_faces);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(poly_points, poly_faces, halfwire);

    if (bounds.size() != 2)
    {
        std::cout << "Wrong boundary points!" << std::endl;
        return false;
    }

    if (bounds[0].z() > bounds[1].z())
    {
        upper_bound = bounds[0];
        lower_bound = bounds[1];
    }
    else
    {
        upper_bound = bounds[1];
        lower_bound = bounds[0];
    }

    return true;
}

void Teeth::find_vnormal_from_adjacent_mesh(Polyhedron& halfwire, VertexList& vert_handles, VectorList& vert_normals, double threshold)
{
    for (Vertex_handle vd : vertices(halfwire))
    {
        for (int i = 0; i < vert_handles.size(); i++)
        {
            if (CGAL::squared_distance(vd->point(), vert_handles[i]->point()) < threshold)
                vert_normals[i] = CGAL::Polygon_mesh_processing::compute_vertex_normal(vd, halfwire);
        }
    }
}



//----------------------------- Inside / Convexity Help Function -----------------------------//

void Teeth::compute_smoothed_field(int smooth_range, VertDoubleMap& init_map, VertDoubleMap& value_map)
{
    // initialize value map
    for (auto& elem : init_map)
        value_map.insert({ elem.first, elem.second });

    // Compute smoothed intersection map
    for (int i = 0; i < smooth_range; i++)
    {
        VertDoubleMap new_value_map;

        for (auto& elem : value_map)
        {
            Vertex_handle vd = elem.first;
            double count = 1., value = elem.second;
            Halfedge_vertex_circulator he = vd->vertex_begin();
            do {
                count = count + 1.;
                value = value + value_map[he->opposite()->vertex()];
            } while (++he != vd->vertex_begin());
            double new_value = value / count;
            new_value_map.insert({ vd, new_value });
        }

        value_map.swap(new_value_map);
        new_value_map.clear();
    }
}

double Teeth::locate_and_evaluate_function(Point& query, Facet_handle face, VertDoubleMap& vert_func_map)
{
    Halfedge_facet_circulator he = face->facet_begin();
    Vertex_handle va = he->vertex();
    Vertex_handle vb = he->next()->vertex();
    Vertex_handle vc = he->next()->next()->vertex();

    double a, b, c;
    barycentric_coordinates(query, va->point(), vb->point(), vc->point(), a, b, c);
    double value = a * vert_func_map[va] + b * vert_func_map[vb] + c * vert_func_map[vc];

    return value;
}

void Teeth::barycentric_coordinates(Point& p, Point& pa, Point& pb, Point& pc, double& a, double& b, double& c)
{
    double area_abc = std::sqrt(CGAL::squared_area(pa, pb, pc));
    a = std::sqrt(CGAL::squared_area(p, pb, pc)) / area_abc;
    b = std::sqrt(CGAL::squared_area(p, pa, pc)) / area_abc;
    c = std::sqrt(CGAL::squared_area(p, pa, pb)) / area_abc;
}

} // namespace dentist
