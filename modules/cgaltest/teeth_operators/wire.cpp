#include "wire.h"

#include "types.h"

#include "meshing.h"

namespace dentist {

Wire::Wire() { m_init = false; }

Wire::~Wire()
{
    reset();
}

void Wire::reset()
{
    m_polyline.clear();
    m_wire_left_shape.clear();
    m_wire_middle_shape.clear();
    m_wire_right_shape.clear();
    m_wire_shape.clear();
    m_wire_extrude_shape.clear();
    m_init = false;
}

void Wire::set_fitting_plane(Plane& fitting_plane, Point& center, Point& right)
{
    m_fitting_plane.m_plane = fitting_plane;
    m_fitting_plane.m_center = center;
    m_fitting_plane.m_right = right;
}

void Wire::set_fitting_segment(Segment& fitting_segment) { m_fitting_segment = fitting_segment; }
void Wire::set_fitting_point(Point& fitting_point) { m_fitting_point = fitting_point; }

void Wire::set_cut_plane(Plane& cut_plane, Point& center, Point& right)
{
    m_cut_plane.m_plane = cut_plane;
    m_cut_plane.m_center = center;
    m_cut_plane.m_right = right;
}

void Wire::set_polylines(PointList& intersection)
{
    m_polyline.clear();
    for (int i = 0; i < intersection.size() - 1; i++)
        m_polyline.push_back(Segment(intersection[i], intersection[i + 1]));
}

void Wire::set_flag(bool flag) { m_init = flag; }

void Wire::set_wire_left_shape(Polyhedron& wire_left_shape)
{
    m_wire_left_shape.clear();
    copy_polyehdral_surface(wire_left_shape, m_wire_left_shape);
}

void Wire::set_wire_middle_shape(Polyhedron& wire_middle_shape)
{
    m_wire_middle_shape.clear();
    copy_polyehdral_surface(wire_middle_shape, m_wire_middle_shape);
}

void Wire::set_wire_right_shape(Polyhedron& wire_right_shape)
{
    m_wire_right_shape.clear();
    copy_polyehdral_surface(wire_right_shape, m_wire_right_shape);
}

void Wire::set_wire_shape(Polyhedron& wire_shape)
{
    m_wire_shape.clear();
    copy_polyehdral_surface(wire_shape, m_wire_shape);
}

void Wire::set_wire_extrude_shape(Polyhedron& wire_extrude_shape)
{
    m_wire_extrude_shape.clear();
    copy_polyehdral_surface(wire_extrude_shape, m_wire_extrude_shape);
}

MyPlane& Wire::get_fitting_plane() { return m_fitting_plane; }
Segment& Wire::get_fitting_segment() { return m_fitting_segment; }
Point& Wire::get_fitting_point() { return m_fitting_point; }
MyPlane& Wire::get_cut_plane() { return m_cut_plane; }
SegmentList& Wire::get_polylines() { return m_polyline; }
bool Wire::get_flag() { return m_init; }

Polyhedron& Wire::get_wire_left_shape() { return m_wire_left_shape; }
Polyhedron& Wire::get_wire_right_shape() { return m_wire_right_shape; }
Polyhedron& Wire::get_wire_shape() { return m_wire_shape; }

Polyhedron& Wire::get_wire_extrude_shape() { return m_wire_extrude_shape; }

} // namespace dentist
