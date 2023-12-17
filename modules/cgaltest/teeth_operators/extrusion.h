#pragma once

#include "types.h"

namespace dentist {

struct Bottom
{
    Bottom();
    void operator() (Vertex_handle vin, Vertex_handle vout) const;
};

struct Top
{
    Top(VertVec_property_map nmap, double vlen);
    void operator()(Vertex_handle vin, Vertex_handle vout) const;
    VertVec_property_map nmap;
    double vlen;
};

void compute_extrusion(Polyhedron& poly_in, Polyhedron& poly_out, double height);

} // namespace dentist