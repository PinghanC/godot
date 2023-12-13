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
    Polyhedron  m_wire_shape;
    Polyhedron  m_wire_extrude_shape;

public:

    Wire() { m_init = false; }

    ~Wire() 
    {
      reset();
    }

    void reset()
    {
        m_polyline.clear();
        m_wire_shape.clear();
        m_wire_extrude_shape.clear();
        m_init = false;
    }

    void set_fitting_plane(Plane& fitting_plane, Point& center, Point& right) 
    { 
        m_fitting_plane.m_plane = fitting_plane; 
        m_fitting_plane.m_center = center; 
        m_fitting_plane.m_right = right; 
    }

    void set_fitting_segment(Segment& fitting_segment) { m_fitting_segment = fitting_segment; }
    void set_fitting_point(Point& fitting_point) { m_fitting_point = fitting_point; }

    void set_cut_plane(Plane& cut_plane, Point& center, Point& right) 
    { 
        m_cut_plane.m_plane = cut_plane; 
        m_cut_plane.m_center = center; 
        m_cut_plane.m_right = right; 
    }

    void set_polylines(PointList& intersection)
    {
        m_polyline.clear();
        for(int i = 0; i < intersection.size() - 1; i++)
            m_polyline.push_back(Segment(intersection[i], intersection[i+1]));
    }

    void set_flag(bool flag) { m_init = flag; }

    void set_wire_shape(Polyhedron& wire_shape)
    {
        m_wire_shape.clear();
        copy_polyehdral_surface(wire_shape, m_wire_shape);
    }

    void set_wire_extrude_shape(Polyhedron& wire_extrude_shape)
    {
        m_wire_extrude_shape.clear();
        copy_polyehdral_surface(wire_extrude_shape, m_wire_extrude_shape);
    }

    MyPlane& get_fitting_plane() { return m_fitting_plane; }
    Segment& get_fitting_segment() { return m_fitting_segment; }
    Point& get_fitting_point() { return m_fitting_point; }
    MyPlane& get_cut_plane() { return m_cut_plane; }
    SegmentList& get_polylines() { return m_polyline; }
    bool get_flag() { return m_init; }
    Polyhedron& get_wire_shape() { return m_wire_shape; }
    Polyhedron& get_wire_extrude_shape() { return m_wire_extrude_shape; }


}; // end of class Wire

} // namespace dentist

#endif