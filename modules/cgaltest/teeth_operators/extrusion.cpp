#include "extrusion.h"

#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>

#include "types.h"
#include "meshing.h"

namespace dentist {

Bottom::Bottom() {}
void Bottom::operator() (Vertex_handle vin, Vertex_handle vout) const {}

Top::Top(VertVec_property_map nmap, double vlen)
    : nmap(nmap), vlen(vlen)
{}
void Top::operator()(Vertex_handle vin, Vertex_handle vout) const
{
    Vector normal = nmap[vin];
    vout->point() = vin->point() + vlen * normal;
}

void compute_extrusion(Polyhedron& poly_in, Polyhedron& poly_out, double height)
{
    std::cout << "      Begin!" << std::endl;

    VertVecMap vnormals;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(poly_in, boost::make_assoc_property_map(vnormals));
    Bottom bottom;
    Top top(vnormals, height);
    CGAL::Polygon_mesh_processing::extrude_mesh(poly_in, poly_out, bottom, top);
    CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(poly_out);

    std::cout << "      Extrusion done!" << std::endl;

    if (CGAL::Polygon_mesh_processing::does_self_intersect(poly_out))
    {
        std::cout << "      Find self-intersection!" << std::endl;
        CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(poly_out);
        double diag_length = std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
            CGAL::square(bbox.ymax() - bbox.ymin()) +
            CGAL::square(bbox.zmax() - bbox.zmin()));

        double alpha = diag_length / 200;
        double offset = diag_length / 2000;

        Polyhedron poly_out_wrap;
        CGAL::alpha_wrap_3(poly_out, alpha, offset, poly_out_wrap);
        std::cout << "      Alpha wrapper done!" << std::endl;

        poly_out.clear();
        copy_polyehdral_surface(poly_out_wrap, poly_out);
    }
}

} // namespace dentist
