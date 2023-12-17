#ifndef MY_WIRE_H
#define MY_WIRE_H

#include "types.h"

namespace dentist {

struct MyPlane
{
	Plane m_plane = Plane(0., 0., 0., 1.);
    Point m_center = CGAL::ORIGIN;
    Point m_right = CGAL::ORIGIN;
};

class Wire
{
private:
    bool        m_init;
    MyPlane     m_fitting_plane;
    Segment     m_fitting_segment;
    Point       m_fitting_point;
    MyPlane     m_cut_plane;
    SegmentList m_polyline;
    Polyhedron  m_wire_left_shape;
    Polyhedron  m_wire_middle_shape;
    Polyhedron  m_wire_right_shape;
    Polyhedron  m_wire_shape;
    Polyhedron  m_wire_extrude_shape;

public:

    Wire();

    ~Wire();

    void reset();

    void set_fitting_plane(Plane& fitting_plane, Point& center, Point& right);

    void set_fitting_segment(Segment& fitting_segment);
    void set_fitting_point(Point& fitting_point);

    void set_cut_plane(Plane& cut_plane, Point& center, Point& right);

    void set_polylines(PointList& intersection);

    void set_flag(bool flag);

    void set_wire_left_shape(Polyhedron& wire_left_shape);

    void set_wire_middle_shape(Polyhedron& wire_middle_shape);

    void set_wire_right_shape(Polyhedron& wire_right_shape);

    void set_wire_shape(Polyhedron& wire_shape);

    void set_wire_extrude_shape(Polyhedron& wire_extrude_shape);

    MyPlane& get_fitting_plane();
    Segment& get_fitting_segment();
    Point& get_fitting_point();
    MyPlane& get_cut_plane();
    SegmentList& get_polylines();
    bool get_flag();

    Polyhedron& get_wire_left_shape();
    Polyhedron& get_wire_right_shape();
    Polyhedron& get_wire_shape();

    Polyhedron& get_wire_extrude_shape();

}; // end of class Wire

} // namespace dentist

#endif