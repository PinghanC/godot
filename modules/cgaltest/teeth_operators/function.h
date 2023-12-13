#ifndef MY_FUNCTION_H
#define MY_FUNCTION_H

#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include "types.h"
#include "teeth.h"
#include "wire.h"

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

    Function(): m_upper_init(false), m_lower_init(false), m_radius(1.), m_center(Point(0., 0., 0.)) {}

    ~Function() 
	{
		reset();
	}

    void reset()
    {
        m_upper_init = false;
        m_lower_init = false;
        m_center = Point(0., 0., 0.);
        m_radius = 1.;
        m_upper_teeth.reset();
        m_lower_teeth.reset();
        m_teeth_tree.clear();
        clear_segmentation();
    }

    void clear_segmentation()
    {
        m_segment.clear();
        m_segment_status = 0;
    }

    bool load_upper_teeth_mesh(std::string filename)
    {
        std::cout << "Loading upper teeth!" << std::endl;
        m_upper_teeth.reset();
        m_upper_init = m_upper_teeth.load_teeth_mesh(filename);

        if(m_upper_init)
        {
            compute_bounding_box();
            update_teeth_tree();
        }
        
        return m_upper_init;
    }

    bool load_lower_teeth_mesh(std::string filename)
    {
        std::cout << "Loading lower teeth!" << std::endl;
        m_lower_teeth.reset();
        m_lower_init = m_lower_teeth.load_teeth_mesh(filename);

        if(m_lower_init)
        {
            compute_bounding_box();
            update_teeth_tree();
        }

        return m_lower_init;
    }

    bool init_upper_teeth_mesh(const float* vertices, size_t vertices_size, const int* triangles, size_t triangles_size)
    {
        std::cout << "Init upper teeth!" << std::endl;
        m_upper_teeth.reset();
        m_upper_init = m_upper_teeth.init_teeth_mesh(vertices, vertices_size, triangles, triangles_size);

        if (m_upper_init)
        {
            compute_bounding_box();
            update_teeth_tree();
        }

        return m_upper_init;
    }

    bool init_lower_teeth_mesh(const float* vertices, size_t vertices_size, const int* triangles, size_t triangles_size)
    {
        std::cout << "Init lower teeth!" << std::endl;
        m_lower_teeth.reset();
        m_lower_init = m_lower_teeth.init_teeth_mesh(vertices, vertices_size, triangles, triangles_size);

        if (m_lower_init)
        {
            compute_bounding_box();
            update_teeth_tree();
        }

        return m_lower_init;
    }

    void compute_bounding_box()
    {
        PointList points;

        if(m_upper_init)
        {
            m_upper_teeth.read_polygon_vertices(points);
        }

        if(m_lower_init)
        {
            m_lower_teeth.read_polygon_vertices(points);
        }

        if(points.size() > 0)
        {
            Bbox bounding_box = CGAL::bbox_3(points.begin(), points.end());
            m_center = Point(   0.5 * (bounding_box.xmin() + bounding_box.xmax()), 
                                0.5 * (bounding_box.ymin() + bounding_box.ymax()), 
                                0.5 * (bounding_box.zmin() + bounding_box.zmax()));
            Point corner(bounding_box.xmin(), bounding_box.ymin(), bounding_box.zmin());
            m_radius = std::sqrt(CGAL::squared_distance(m_center, corner));
        }
    }

    void update_teeth_tree()
    {
        m_teeth_tree.clear();

        if(m_upper_init)
        {
            Polyhedron& teeth_poly = m_upper_teeth.get_teeth_poly();
            m_teeth_tree.insert(faces(teeth_poly).first, faces(teeth_poly).second, teeth_poly);
        }

        if(m_lower_init)
        {
            Polyhedron& teeth_poly = m_lower_teeth.get_teeth_poly();
            m_teeth_tree.insert(faces(teeth_poly).first, faces(teeth_poly).second, teeth_poly);
        }

        if(!m_teeth_tree.empty())
            m_teeth_tree.accelerate_distance_queries();
    }

    const double get_radius() { return m_radius; }
    const Point& get_center() { return m_center; }
    int number_of_upper_teeth_faces() {     return m_upper_teeth.size_of_facets(); }
    int number_of_upper_teeth_vertices() {  return m_upper_teeth.size_of_vertices(); }
    int number_of_lower_teeth_faces() {     return m_lower_teeth.size_of_facets(); }
    int number_of_lower_teeth_vertices() {  return m_lower_teeth.size_of_vertices(); }

    //----------------------------- Selection -----------------------------//

    bool update_selection(Point camera_pos, Vector camera_dir, PointList& selects, DataList& selected_points)
    {
        if(m_teeth_tree.empty())
            return false;

        Ray ray(camera_pos, camera_dir);
        auto intersection = m_teeth_tree.first_intersection(ray);
        if(intersection)
        {
            if(boost::get<Point>(&(intersection->first))){
                Point select =  *(boost::get<Point>(&(intersection->first)));
                selects.push_back(select);
                selected_points.push_back((float)select.x());
                selected_points.push_back((float)select.y());
                selected_points.push_back((float)select.z());
                return true;
            }
        }

        return false;
    }

    //----------------------------- Algorithm -----------------------------//

    bool compute_laplacian_based_inside(int smooth_range, double lambda, double value)
    {
        if(!m_upper_init || !m_lower_init)
        {
            std::cout << "Please load both upper teeth and lower teeth!";
            return false;
        }

        // Find camera position for teeth
        PointList lower_verts;
        m_lower_teeth.read_polygon_vertices(lower_verts);
        Bbox bounding_box = CGAL::bbox_3(lower_verts.begin(), lower_verts.end());
        double x = 0.5 * (bounding_box.xmin() + bounding_box.xmax());
        double y = 0.5 * (bounding_box.ymin() + bounding_box.ymax());
        double z = 0.5 * (bounding_box.zmin() + bounding_box.zmax());

        Point upper_center(x, y, bounding_box.zmax());
        Point lower_center(x, y, z);

        // Solve constrained laplacian for inside segmentation
        std::cout << "Solving laplacian-based inside for upper teeth..." << std::endl;
        bool flag_upper = m_upper_teeth.compute_laplacian_based_inside(m_teeth_tree, upper_center, smooth_range, lambda);
        std::cout << "Solving laplacian-based inside for lower teeth..." << std::endl;
        bool flag_lower = m_lower_teeth.compute_laplacian_based_inside(m_teeth_tree, lower_center, smooth_range, lambda);

        if(!flag_upper || !flag_lower)
        {
            std::cout << "Solver failed!" << std::endl;
            return false;
        }
            
        // Extract isosurface for inside segmentation
        std::cout << "Extracting inside surface for upper teeth..." << std::endl;
        m_upper_teeth.extract_inside_from_function(value);
        std::cout << "Extracting inside surface for lower teeth..." << std::endl;
        m_lower_teeth.extract_inside_from_function(value);

        return true;
    }

    bool compute_laplacian_based_convexity(double radius_ratio, int smooth_range, double lambda)
    {
        if(!m_upper_init || !m_lower_init)
        {
            std::cout << "Please load both upper teeth and lower teeth!";
            return false;
        }

        // Solve constrained laplacian for convexity field
        std::cout << "Solving laplacian-based convexity field for upper teeth..." << std::endl;
        bool flag_upper = m_upper_teeth.compute_laplacian_based_convexity(radius_ratio, m_radius, smooth_range, lambda);
        std::cout << "Solving laplacian-based convexity field for lower teeth..." << std::endl;
        bool flag_lower = m_lower_teeth.compute_laplacian_based_convexity(radius_ratio, m_radius, smooth_range, lambda);

        if(!flag_upper || !flag_lower)
        {
            std::cout << "Solver failed!" << std::endl;
            return false;
        }

        return true;
    }

    bool compute_selected_segmentation(PointList& selected_points, double isovalue)
    {
        if(selected_points.size() == 0)
        {
            std::cout << "No available selection!" << std::endl;
            return false;
        }

        double upper_dist = m_upper_teeth.compute_distance_to_inside_teeth(selected_points[0]);
        double lower_dist = m_lower_teeth.compute_distance_to_inside_teeth(selected_points[0]);

        if(upper_dist < 0 || lower_dist < 0)
        {
            std::cout << "The inside surfaces are not available, please compute the inside surface!" << std::endl;
            return false;
        }

        clear_segmentation();

        std::cout << "Begin extracting segmentation!" << std::endl;
        bool flag_lower = (upper_dist < lower_dist)? false: true;
        if(!flag_lower)
            m_upper_teeth.compute_selected_segmentation(selected_points, isovalue, m_segment);
        else
            m_lower_teeth.compute_selected_segmentation(selected_points, isovalue, m_segment);

        if(m_segment.size_of_vertices() == 0)
        {
            std::cout << "Segmentation failed!" << std::endl;
            return false;
        }

        m_segment_status = (flag_lower) ? -1: 1;

        return true;
    }

    int validate_selected_segmentation()
    {
        if(m_segment_status == 0)
        {
            std::cout << "No available segmentation!" << std::endl;
            return 0;
        }

        int curr_stat = m_segment_status;
        if(curr_stat == -1)
            m_lower_teeth.validate_selected_segmentation(m_segment);
        else
            m_upper_teeth.validate_selected_segmentation(m_segment);

        clear_segmentation();

        return curr_stat;
    }

    void compute_geodesic_pad_outlines(int index, bool flag_lower, int discretize_step, double isovalue)
    {
        if(flag_lower)
            m_lower_teeth.compute_geodesic_pad_outlines(index, discretize_step, isovalue);
        else
            m_upper_teeth.compute_geodesic_pad_outlines(index, discretize_step, isovalue);
    }

    void recompute_geodesic_pad_outlines(int index, bool flag_lower, double isovalue)
    {
        if(flag_lower)
            m_lower_teeth.recompute_geodesic_pad_outlines(index, isovalue);
        else
            m_upper_teeth.recompute_geodesic_pad_outlines(index, isovalue);
    }

    void compute_tooth_cut_plane(int index, bool flag_lower)
    {
        if(flag_lower)
            m_lower_teeth.compute_tooth_cut_plane(index);
        else
            m_upper_teeth.compute_tooth_cut_plane(index);
    }

    void compute_paired_wires(int index, bool flag_lower, double distance)
    {
        if(flag_lower)
            m_lower_teeth.compute_paired_wires(index, distance);
        else
            m_upper_teeth.compute_paired_wires(index, distance);
    }

    void compute_wire_shapes(int index, bool flag_lower, int geodesic_step, double width)
    {
        if(flag_lower)
            m_lower_teeth.compute_wire_shapes(index, geodesic_step, width);
        else
            m_upper_teeth.compute_wire_shapes(index, geodesic_step, width);
    }


    //----------------------------- Visualization -----------------------------//

    void update_teeth_mesh(DataList& teeth_facets, DataList& teeth_normals, bool flag_lower)
    {
        Polyhedron& teeth_poly = (flag_lower) ? m_lower_teeth.get_teeth_poly() : m_upper_teeth.get_teeth_poly();

        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(teeth_poly, boost::make_assoc_property_map(fnormals));

        teeth_facets.reserve(teeth_poly.size_of_facets() * 9);
        teeth_normals.reserve(teeth_poly.size_of_facets() * 9);

        for(Facet_iterator face = teeth_poly.facets_begin(); face != teeth_poly.facets_end(); ++face) {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                teeth_facets.push_back((float)p.x());
                teeth_facets.push_back((float)p.y());
                teeth_facets.push_back((float)p.z());
                teeth_normals.push_back((float)face_normal.x());
                teeth_normals.push_back((float)face_normal.y());
                teeth_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }
    }

    void update_inside_function(DataList& inside_ray_facets,
                                DataList& inside_ray_normals,
                                DataList& inside_ray_colors,
                                DataList& inside_func_colors,
                                DataList& inside_facets,
                                DataList& inside_normals,
                                bool flag_lower)
    {
        Polyhedron& teeth_poly = (flag_lower) ? m_lower_teeth.get_teeth_poly() : m_upper_teeth.get_teeth_poly();
        VertDoubleMap& smoothed_ray_map = (flag_lower) ? m_lower_teeth.get_smoothed_ray_map() : m_upper_teeth.get_smoothed_ray_map();
        VertDoubleMap& inside_func_map = (flag_lower) ? m_lower_teeth.get_vertex_inside_function_map() : m_upper_teeth.get_vertex_inside_function_map();

        // Find min and max value
        double fmin = 0., fmax = 1.;
        for(Vertex_handle vd: vertices(teeth_poly))
        {
            fmin = std::min(inside_func_map[vd], fmin);
            fmax = std::max(inside_func_map[vd], fmax);
        }

        if(flag_lower)
            std::cout << "  Lower teeth fmin: " << fmin << ", lower teeth fmax: " << fmax << std::endl;
        else
            std::cout << "  Upper teeth fmin: " << fmin << ", Upper teeth fmax: " << fmax << std::endl;

        // Fill Datalists
        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(teeth_poly, boost::make_assoc_property_map(fnormals));
        inside_ray_facets.reserve(teeth_poly.size_of_facets() * 9);
        inside_ray_normals.reserve(teeth_poly.size_of_facets() * 9);
        inside_ray_colors.reserve(teeth_poly.size_of_facets() * 9);
        inside_func_colors.reserve(teeth_poly.size_of_facets() * 9);
        Ramp ramp;
        ramp.set_range(fmin, fmax);

        for(Facet_iterator face = teeth_poly.facets_begin(); face != teeth_poly.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                // smoothed ray field
                Point p = he->vertex()->point();
                inside_ray_facets.push_back((float)p.x());
                inside_ray_facets.push_back((float)p.y());
                inside_ray_facets.push_back((float)p.z());
                inside_ray_normals.push_back((float)face_normal.x());
                inside_ray_normals.push_back((float)face_normal.y());
                inside_ray_normals.push_back((float)face_normal.z());
                double r, g, b;
                ramp.get_color(smoothed_ray_map[he->vertex()], r, g ,b);
                inside_ray_colors.push_back((float)r);
                inside_ray_colors.push_back((float)g);
                inside_ray_colors.push_back((float)b);
                ramp.get_color(inside_func_map[he->vertex()], r, g ,b);
                inside_func_colors.push_back((float)r);
                inside_func_colors.push_back((float)g);
                inside_func_colors.push_back((float)b);
            } while (++he != face->facet_begin());
        }

        Polyhedron& inside_teeth_poly = (flag_lower) ? m_lower_teeth.get_inside_teeth_poly() : m_upper_teeth.get_inside_teeth_poly();

        fnormals.clear();
        CGAL::Polygon_mesh_processing::compute_face_normals(inside_teeth_poly, boost::make_assoc_property_map(fnormals));
        inside_facets.reserve(inside_teeth_poly.size_of_facets() * 9);
        inside_normals.reserve(inside_teeth_poly.size_of_facets() * 9);
        for(Facet_iterator face = inside_teeth_poly.facets_begin(); face != inside_teeth_poly.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                inside_facets.push_back((float)p.x());
                inside_facets.push_back((float)p.y());
                inside_facets.push_back((float)p.z());
                inside_normals.push_back((float)face_normal.x());
                inside_normals.push_back((float)face_normal.y());
                inside_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }
    }

    void update_convex_function(DataList& local_convex_facets,
                                DataList& local_convex_normals,
                                DataList& local_convex_colors,
                                DataList& convex_func_colors,
                                bool flag_lower)
    {
        Polyhedron& inside_teeth_poly = (flag_lower) ? m_lower_teeth.get_inside_teeth_poly() : m_upper_teeth.get_inside_teeth_poly();
        VertDoubleMap& smoothed_convex_map = (flag_lower) ? m_lower_teeth.get_smoothed_convex_map() : m_upper_teeth.get_smoothed_convex_map();
        VertDoubleMap& convex_func_map = (flag_lower) ? m_lower_teeth.get_vertex_convex_function_map() : m_upper_teeth.get_vertex_convex_function_map();

        // Find min and max value
        double fmin = 1e10, fmax = -1e10;
        for(Vertex_handle vd: vertices(inside_teeth_poly))
        {
            fmin = std::min(smoothed_convex_map[vd], fmin);
            fmin = std::min(convex_func_map[vd], fmin);
            fmax = std::max(smoothed_convex_map[vd], fmax);
            fmax = std::max(convex_func_map[vd], fmax);
        }

        if(flag_lower)
            std::cout << "  Lower teeth fmin: " << fmin << ", lower teeth fmax: " << fmax << std::endl;
        else
            std::cout << "  Upper teeth fmin: " << fmin << ", Upper teeth fmax: " << fmax << std::endl;

        // Fill Datalists
        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(inside_teeth_poly, boost::make_assoc_property_map(fnormals));
        local_convex_facets.reserve(inside_teeth_poly.size_of_facets() * 9);
        local_convex_normals.reserve(inside_teeth_poly.size_of_facets() * 9);
        local_convex_colors.reserve(inside_teeth_poly.size_of_facets() * 9);
        convex_func_colors.reserve(inside_teeth_poly.size_of_facets() * 9);
        Ramp ramp;
        ramp.set_range(fmin, fmax);

        for(Facet_iterator face = inside_teeth_poly.facets_begin(); face != inside_teeth_poly.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                // smoothed ray field
                Point p = he->vertex()->point();
                local_convex_facets.push_back((float)p.x());
                local_convex_facets.push_back((float)p.y());
                local_convex_facets.push_back((float)p.z());
                local_convex_normals.push_back((float)face_normal.x());
                local_convex_normals.push_back((float)face_normal.y());
                local_convex_normals.push_back((float)face_normal.z());
                double r, g, b;
                ramp.get_color(smoothed_convex_map[he->vertex()], r, g ,b);
                local_convex_colors.push_back((float)r);
                local_convex_colors.push_back((float)g);
                local_convex_colors.push_back((float)b);
                ramp.get_color(convex_func_map[he->vertex()], r, g ,b);
                convex_func_colors.push_back((float)r);
                convex_func_colors.push_back((float)g);
                convex_func_colors.push_back((float)b);
            } while (++he != face->facet_begin());
        }
    }

    void update_convexity_isovalue(DataList& convex_isoedges, double value, bool flag_lower)
    {
        Polyhedron& inside_teeth_poly = (flag_lower) ? m_lower_teeth.get_inside_teeth_poly() : m_upper_teeth.get_inside_teeth_poly();
        VertDoubleMap& convex_func_map = (flag_lower) ? m_lower_teeth.get_vertex_convex_function_map() : m_upper_teeth.get_vertex_convex_function_map();

        for(Facet_iterator face = inside_teeth_poly.facets_begin(); face != inside_teeth_poly.facets_end(); ++face) 
        {
            Halfedge_facet_circulator he = face->facet_begin();
            PointList iso_points;
            do {
                double a = convex_func_map[he->vertex()] - value;
                double b = convex_func_map[he->opposite()->vertex()] - value; 
                if(a * b < 0)
                {
                    Point pa = he->vertex()->point();
                    Point pb = he->opposite()->vertex()->point();
                    Point iso;
                    bool flag = find_level_set_point(pa, a, pb, b, 0., iso);
                    if(flag)
                        iso_points.push_back(iso);
                }
            } while (++he != face->facet_begin());

            if(iso_points.size() == 2)
            {
                convex_isoedges.push_back((float)iso_points[0].x());
                convex_isoedges.push_back((float)iso_points[0].y());
                convex_isoedges.push_back((float)iso_points[0].z());
                convex_isoedges.push_back((float)iso_points[1].x());
                convex_isoedges.push_back((float)iso_points[1].y());
                convex_isoedges.push_back((float)iso_points[1].z());
            }
        }
    }

    void update_segmentation(DataList& seg_facets, DataList& seg_normals)
    {
        if(m_segment.size_of_vertices() == 0)
            return;

        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(m_segment, boost::make_assoc_property_map(fnormals));

        for(Facet_iterator face = m_segment.facets_begin(); face != m_segment.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                seg_facets.push_back((float)p.x());
                seg_facets.push_back((float)p.y());
                seg_facets.push_back((float)p.z());
                seg_normals.push_back((float)face_normal.x());
                seg_normals.push_back((float)face_normal.y());
                seg_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }
    }

    void update_validated_segmentation(int index, bool flag_lower, DataList& seg_facets, DataList& seg_normals)
    {
        Polyhedron& tooth_poly = (flag_lower) ? m_lower_teeth.get_tooth_poly(index): m_upper_teeth.get_tooth_poly(index);
        if(tooth_poly.empty())
            return;

        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(tooth_poly, boost::make_assoc_property_map(fnormals));

        for(Facet_iterator face = tooth_poly.facets_begin(); face != tooth_poly.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                seg_facets.push_back((float)p.x());
                seg_facets.push_back((float)p.y());
                seg_facets.push_back((float)p.z());
                seg_normals.push_back((float)face_normal.x());
                seg_normals.push_back((float)face_normal.y());
                seg_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }
    }

    void update_tooth_geodesic_function(int index, bool flag_lower, DataList& poly_colors)
    {
        Polyhedron& tooth_poly = (flag_lower) ? m_lower_teeth.get_tooth_poly(index): m_upper_teeth.get_tooth_poly(index);
        if(tooth_poly.empty())
            return;

        VertDoubleMap& tooth_func = (flag_lower) ? m_lower_teeth.get_geodesic_distance_map(index): m_upper_teeth.get_geodesic_distance_map(index);
        if(tooth_func.empty())
            return;

        // Find min and max value
        double fmin = 1e10, fmax = -1e10;
        for(Vertex_handle vd: vertices(tooth_poly))
        {
            fmin = std::min(tooth_func[vd], fmin);
            fmax = std::max(tooth_func[vd], fmax);
        }

        std::cout << "  Geodesic distance fmin: " << fmin << ", geodesic distance fmax: " << fmax << std::endl;


        poly_colors.reserve(tooth_poly.size_of_facets() * 9);
        Ramp ramp;
        ramp.set_range(fmin, fmax);

        for(Facet_iterator face = tooth_poly.facets_begin(); face != tooth_poly.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            do {
                double r, g, b;
                ramp.get_color(tooth_func[he->vertex()], r, g ,b);
                poly_colors.push_back((float)r);
                poly_colors.push_back((float)g);
                poly_colors.push_back((float)b);
            } while (++he != face->facet_begin());
        }
    }

    void update_pad_outline(int index, bool flag_lower, DataList& poly_facets, DataList& poly_normals)
    {
        Polyhedron& pad_outline_poly = (flag_lower) ? m_lower_teeth.get_pad_outline_poly(index): m_upper_teeth.get_pad_outline_poly(index);
        if(pad_outline_poly.empty())
            return;

        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(pad_outline_poly, boost::make_assoc_property_map(fnormals));

        for(Facet_iterator face = pad_outline_poly.facets_begin(); face != pad_outline_poly.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                poly_facets.push_back((float)p.x());
                poly_facets.push_back((float)p.y());
                poly_facets.push_back((float)p.z());
                poly_normals.push_back((float)face_normal.x());
                poly_normals.push_back((float)face_normal.y());
                poly_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }
    }

    void update_tooth_cut_plane(int index, bool flag_lower, DataList& upper_bound, DataList& upper_points, DataList& cut_plane_points, DataList& cut_plane_normals)
    {
        HEdgeSet& upper_bound_set = (flag_lower) ? m_lower_teeth.get_upper_bound_set(index): m_upper_teeth.get_upper_bound_set(index);
        PointList& cut_points = (flag_lower) ? m_lower_teeth.get_cut_points(index): m_upper_teeth.get_cut_points(index);
        Plane& cut_plane = (flag_lower) ? m_lower_teeth.get_cut_plane(index): m_upper_teeth.get_cut_plane(index);

        if(upper_bound_set.size() == 0 || cut_points.size() == 0)
            return;

        for(int i = 0; i < cut_points.size(); i++)
        {
            Point p = cut_points[i];
            upper_points.push_back((float)p.x());
            upper_points.push_back((float)p.y());
            upper_points.push_back((float)p.z());
        }

        for(auto& elem: upper_bound_set)
        {
            Point source = elem->vertex()->point();
            upper_bound.push_back((float)source.x());
            upper_bound.push_back((float)source.y());
            upper_bound.push_back((float)source.z());
            Point target = elem->opposite()->vertex()->point();
            upper_bound.push_back((float)target.x());
            upper_bound.push_back((float)target.y());
            upper_bound.push_back((float)target.z());
        }

        Vector normal;
        PointList corners;
        compute_plane_corners(cut_plane, cut_points[0], cut_points[1], corners, normal);

        for(int i = 0; i < corners.size(); i++)
        {
            Point p = corners[i];
            cut_plane_points.push_back((float)p.x());
            cut_plane_points.push_back((float)p.y());
            cut_plane_points.push_back((float)p.z());
            cut_plane_normals.push_back((float)normal.x());
            cut_plane_normals.push_back((float)normal.y());
            cut_plane_normals.push_back((float)normal.z());
        }
    }

    void update_pair_cut_plane( int index, bool flag_lower, 
                                DataList& fitting_plane_points, 
                                DataList& fitting_plane_normals,
                                DataList& fitting_lines,
                                DataList& fitting_points,
                                DataList& cut_plane_points,
                                DataList& cut_plane_normals,
                                DataList& cut_polylines)
    {
        Wire& wire = (flag_lower) ? m_lower_teeth.get_wire(index): m_upper_teeth.get_wire(index);
        MyPlane& wire_fitting_plane = wire.get_fitting_plane();
        Segment& wire_fitting_seg = wire.get_fitting_segment();
        Point& wire_fitting_point = wire.get_fitting_point();
        MyPlane& wire_cut_plane = wire.get_cut_plane();
        SegmentList& wire_polylines = wire.get_polylines();

        fitting_points.push_back((float)wire_fitting_point.x());
        fitting_points.push_back((float)wire_fitting_point.y());
        fitting_points.push_back((float)wire_fitting_point.z());

        fitting_lines.push_back((float)wire_fitting_seg.source().x());
        fitting_lines.push_back((float)wire_fitting_seg.source().y());
        fitting_lines.push_back((float)wire_fitting_seg.source().z());
        fitting_lines.push_back((float)wire_fitting_seg.target().x());
        fitting_lines.push_back((float)wire_fitting_seg.target().y());
        fitting_lines.push_back((float)wire_fitting_seg.target().z());

        for(int i = 0; i < wire_polylines.size(); i++)
        {
            Segment seg = wire_polylines[i];
            Point source = seg.source();
            Point target = seg.target();
            cut_polylines.push_back((float)source.x());
            cut_polylines.push_back((float)source.y());
            cut_polylines.push_back((float)source.z());
            cut_polylines.push_back((float)target.x());
            cut_polylines.push_back((float)target.y());
            cut_polylines.push_back((float)target.z());
        }

        Vector normal;
        PointList corners;
        compute_plane_corners(wire_fitting_plane.m_plane, wire_fitting_plane.m_center, wire_fitting_plane.m_right, corners, normal);
        for(int i = 0; i < corners.size(); i++)
        {
            Point p = corners[i];
            fitting_plane_points.push_back((float)p.x());
            fitting_plane_points.push_back((float)p.y());
            fitting_plane_points.push_back((float)p.z());
            fitting_plane_normals.push_back((float)normal.x());
            fitting_plane_normals.push_back((float)normal.y());
            fitting_plane_normals.push_back((float)normal.z());
        }

        corners.clear();
        compute_plane_corners(wire_cut_plane.m_plane, wire_cut_plane.m_center, wire_cut_plane.m_right, corners, normal);
        for(int i = 0; i < corners.size(); i++)
        {
            Point p = corners[i];
            cut_plane_points.push_back((float)p.x());
            cut_plane_points.push_back((float)p.y());
            cut_plane_points.push_back((float)p.z());
            cut_plane_normals.push_back((float)normal.x());
            cut_plane_normals.push_back((float)normal.y());
            cut_plane_normals.push_back((float)normal.z());
        }
    }

    void compute_plane_corners(Plane& plane, Point& center, Point& right, PointList& corners, Vector& normal)
    {
        double scale = 3.;

        Point plane_center = plane.projection(center);
        Point plane_right = plane.projection(right);
        Vector horizontal = plane_right - plane_center;
        horizontal = horizontal / std::sqrt(horizontal.squared_length());
        normal = plane.orthogonal_vector();
        Vector vertical = CGAL::cross_product(horizontal, normal);
        vertical = vertical / std::sqrt(vertical.squared_length());

        Point left_down = plane_center - scale * horizontal - scale * vertical;
        Point left_up = plane_center - scale * horizontal + scale * vertical;
        Point right_up = plane_center + scale * horizontal + scale * vertical;
        Point right_down = plane_center + scale * horizontal - scale * vertical;

        corners.push_back(left_down);
        corners.push_back(left_up);
        corners.push_back(right_up);
        corners.push_back(left_down);
        corners.push_back(right_up);
        corners.push_back(right_down);
    }

    void update_pair_wire_shape( int index, bool flag_lower, 
                                DataList& wire_shape_points, 
                                DataList& wire_shape_normals,
                                DataList& wire_extrude_shape_points, 
                                DataList& wire_extrude_shape_normals)
    {
        Wire& wire = (flag_lower) ? m_lower_teeth.get_wire(index): m_upper_teeth.get_wire(index);
        Polyhedron& wire_shape = wire.get_wire_shape();

        FaceVecMap fnormals;
        CGAL::Polygon_mesh_processing::compute_face_normals(wire_shape, boost::make_assoc_property_map(fnormals));

        for(Facet_iterator face = wire_shape.facets_begin(); face != wire_shape.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                wire_shape_points.push_back((float)p.x());
                wire_shape_points.push_back((float)p.y());
                wire_shape_points.push_back((float)p.z());
                wire_shape_normals.push_back((float)face_normal.x());
                wire_shape_normals.push_back((float)face_normal.y());
                wire_shape_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }

        Polyhedron& wire_extrude_shape = wire.get_wire_extrude_shape();
        fnormals.clear();
        CGAL::Polygon_mesh_processing::compute_face_normals(wire_extrude_shape, boost::make_assoc_property_map(fnormals));

        for(Facet_iterator face = wire_extrude_shape.facets_begin(); face != wire_extrude_shape.facets_end(); ++face)
        {
            Halfedge_facet_circulator he = face->facet_begin();
            Vector face_normal = fnormals[face];
            do {
                Point p = he->vertex()->point();
                wire_extrude_shape_points.push_back((float)p.x());
                wire_extrude_shape_points.push_back((float)p.y());
                wire_extrude_shape_points.push_back((float)p.z());
                wire_extrude_shape_normals.push_back((float)face_normal.x());
                wire_extrude_shape_normals.push_back((float)face_normal.y());
                wire_extrude_shape_normals.push_back((float)face_normal.z());
            } while (++he != face->facet_begin());
        }

    }

}; // end of class Function

} // namespace dentist

#endif
