/**********************************************************************
 *
 * GEOS - Geometry Engine Open Source
 * http://geos.osgeo.org
 *
 * Copyright (C) 2006 Refractions Research Inc.
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the GNU Lesser General Licence as published
 * by the Free Software Foundation. 
 * See the COPYING file for more information.
 *
 **********************************************************************
 *
 * Last port: triangulate/quadedge/Vertex.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_VERTEX_H
#define GEOS_TRIANGULATE_QUADEDGE_VERTEX_H

#include <math.h>

#include <geos/geom/Coordinate.h>
#include <geos/triangulate/quadedge/TrianglePredicate.h>
#include <geos/algorithm/HCoordinate.h>
#include <geos/algorithm/NotRepresentableException.h>


//fwd declarations
namespace geos {
	namespace triangulate {
		namespace quadedge {
			class QuadEdge;
		}
	}
}

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

using namespace algorithm;
using namespace geom;

class GEOS_DLL Vertex {
public:
	static const int LEFT		= 0;
	static const int RIGHT	   = 1;
	static const int BEYOND	  = 2;
	static const int BEHIND	  = 3;
	static const int BETWEEN	 = 4;
	static const int ORIGIN	  = 5;
	static const int DESTINATION = 6;
private:
	Coordinate	  p;

public:
	Vertex(double _x, double _y) : p(_x, _y) {
	}

	Vertex(double _x, double _y, double _z): p( _x, _y, _z) {
	}

	Vertex(const Coordinate &_p) : p(_p) {
	}

	Vertex() : p() {
	}

	virtual double getX() const {
		return p.x;
	}

	virtual double getY() const {
		return p.y;
	}

	virtual double getZ() const {
		return p.z;
	}

	virtual void setZ(double _z) {
		p.z = _z;
	}

	virtual const Coordinate& getCoordinate() const {
		return p;
	}

	virtual bool equals(const Vertex &_x) const {
		if (p.x == _x.getX() && p.y == _x.getY()) {
			return true;
		} else {
			return false;
		}
	}

	virtual bool equals(const Vertex &_x, double tolerance) const {
		if (p.distance(_x.getCoordinate()) < tolerance) {
			return true;
		} else {
			return false;
		}
	}

	virtual int classify(const Vertex &p0, const Vertex &p1) {
		Vertex &p2 = *this;
		Vertex *a = p1.sub(p0);
		Vertex *b = p2.sub(p0);
		double sa = a->crossProduct(*b);
		int ret;

		if (sa > 0.0)
			ret =  LEFT;
		if (sa < 0.0)
			ret =  RIGHT;
		if ((a->getX() * b->getX() < 0.0) || (a->getY() * b->getY() < 0.0))
			ret =  BEHIND;
		if (a->magn() < b->magn())
			ret =  BEYOND;
		if (p0.equals(p2))
			ret =  ORIGIN;
		if (p1.equals(p2))
			ret =  DESTINATION;
		else
			ret =  BETWEEN;

		delete a;
		delete b;

		return ret;
	}

	/**
	 * Computes the cross product k = u X v.
	 * 
	 * @param v a vertex
	 * @return returns the magnitude of u X v
	 */
	virtual double crossProduct(const Vertex &v) const {
		return (p.x * v.getY() - p.y * v.getX());
	}

	/**
	 * Computes the inner or dot product
	 * 
	 * @param v, a vertex
	 * @return returns the dot product u.v
	 */
	virtual double dot(Vertex v) const {
		return (p.x * v.getX() + p.y * v.getY());
	}

	/**
	 * Computes the scalar product c(v)
	 * 
	 * @param v, a vertex
	 * @return returns the scaled vector
	 */
	virtual Vertex* times(double c) const {
		return (new Vertex(c * p.x, c * p.y));
	}

	/* Vector addition */
	virtual Vertex* sum(Vertex v) const {
		return (new Vertex(p.x + v.getX(), p.y + v.getY()));
	}

	/* and subtraction */
	virtual Vertex* sub(const Vertex &v) const {
		return (new Vertex(p.x - v.getX(), p.y - v.getY()));
	}

	/* magnitude of vector */
	virtual double magn() const {
		return (sqrt(p.x * p.x + p.y * p.y));
	}

	/* returns k X v (cross product). this is a vector perpendicular to v */
	virtual Vertex* cross() const {
		return (new Vertex(p.y, -p.x));
	}

  /** ************************************************************* */
  /***********************************************************************************************
   * Geometric primitives /
   **********************************************************************************************/

	/**
	 * Tests if the vertex is inside the circle defined by 
	 * the triangle with vertices a, b, c (oriented counter-clockwise). 
	 * 
	 * @param a a vertex of the triangle
	 * @param b a vertex of the triangle
	 * @param c a vertex of the triangle
	 * @return true if this vertex is in the circumcircle of (a,b,c)
	 */
	virtual bool isInCircle(const Vertex &a, const Vertex &b, const Vertex &c) const
	{
		return TrianglePredicate::isInCircleRobust(a.p, b.p, c.p, this->p);
		// non-robust - best to not use
		//return TrianglePredicate.isInCircle(a.p, b.p, c.p, this->p);
	}

	/**
	 * Tests whether the triangle formed by this vertex and two
	 * other vertices is in CCW orientation.
	 * 
	 * @param b a vertex
	 * @param c a vertex
	 * @returns true if the triangle is oriented CCW
	 */
	virtual bool isCCW(const Vertex &b, const Vertex &c) 
	{
		// is equal to the signed area of the triangle

		return (b.p.x - p.x) * (c.p.y - p.y) 
			- (b.p.y - p.y) * (c.p.x - p.x) > 0;
	}

	//TODO:BLC: XXX Implement
	//bool rightOf(QuadEdge e) {
		//return isCCW(e.dest(), e.orig());
	//}

	//TODO:BLC: XXX Implement
	//final bool leftOf(QuadEdge e) {
		//return isCCW(e.orig(), e.dest());
	//}

private:
	static HCoordinate* bisector(const Vertex &a, const Vertex &b) {
		// returns the perpendicular bisector of the line segment ab
		double dx = b.getX() - a.getX();
		double dy = b.getY() - a.getY();
		HCoordinate l1 = HCoordinate(a.getX() + dx / 2.0, a.getY() + dy / 2.0, 1.0);
		HCoordinate l2 = HCoordinate(a.getX() - dy + dx / 2.0, a.getY() + dx + dy / 2.0, 1.0);

		return new HCoordinate(l1, l2);
	}

	static double distance(const Vertex &v1, const Vertex &v2) {
		return sqrt(pow(v2.getX() - v1.getX(), 2.0)
				+ pow(v2.getY() - v1.getY(), 2.0));
	}

	/**
	 * Computes the value of the ratio of the circumradius to shortest edge. If smaller than some
	 * given tolerance B, the associated triangle is considered skinny. For an equal lateral
	 * triangle this value is 0.57735. The ratio is related to the minimum triangle angle theta by:
	 * circumRadius/shortestEdge = 1/(2sin(theta)).
	 * 
	 * @param b second vertex of the triangle
	 * @param c third vertex of the triangle
	 * @return ratio of circumradius to shortest edge.
	 */
	virtual double circumRadiusRatio(const Vertex &b, const Vertex &c) {
		Vertex *x = circleCenter(b, c);
		double radius = distance(*x, b);
		double edgeLength = distance(*this, b);
		double el = distance(b, c);
		if (el < edgeLength) {
			edgeLength = el;
		}
		el = distance(c, *this);
		if (el < edgeLength) {
			edgeLength = el;
		}

		delete x;

		return radius / edgeLength;
	}

	/**
	 * returns a new vertex that is mid-way between this vertex and another end point.
	 * 
	 * @param a the other end point.
	 * @return the point mid-way between this and that.
	 */
	virtual Vertex* midPoint(const Vertex &a) {
		double xm = (p.x + a.getX()) / 2.0;
		double ym = (p.y + a.getY()) / 2.0;
		double zm = (p.z + a.getZ()) / 2.0;
		return new Vertex(xm, ym, zm);
	}

	/**
	 * Computes the centre of the circumcircle of this vertex and two others.
	 * 
	 * @param b
	 * @param c
	 * @return the Coordinate which is the circumcircle of the 3 points.
	 */
	virtual Vertex* circleCenter(const Vertex &b, const Vertex &c) const {
		Vertex *a = new Vertex(getX(), getY());
		// compute the perpendicular bisector of cord ab
		HCoordinate *cab = bisector(*a, b);
		// compute the perpendicular bisector of cord bc
		HCoordinate *cbc = bisector(b, c);
		// compute the intersection of the bisectors (circle radii)
		HCoordinate *hcc = new HCoordinate(*cab, *cbc);
		Vertex *cc = NULL;
		
		try{
			cc = new Vertex(hcc->getX(), hcc->getY());
		} catch (NotRepresentableException nre) {
		}

		delete a;
		delete cab;
		delete cbc;
		delete hcc;

		return cc;
	}

	/**
	 * For this vertex enclosed in a triangle defined by three verticies v0, v1 and v2, interpolate
	 * a z value from the surrounding vertices.
	 */
	virtual double interpolateZValue(const Vertex &v0, const Vertex &v1,
			const Vertex &v2) const {
		double x0 = v0.getX();
		double y0 = v0.getY();
		double a = v1.getX() - x0;
		double b = v2.getX() - x0;
		double c = v1.getY() - y0;
		double d = v2.getY() - y0;
		double det = a * d - b * c;
		double dx = this->getX() - x0;
		double dy = this->getY() - y0;
		double t = (d * dx - b * dy) / det;
		double u = (-c * dx + a * dy) / det;
		double z = v0.getZ() + t * (v1.getZ() - v0.getZ()) + u * (v2.getZ() - v0.getZ());
		return z;
	}

	/**
	 * Interpolates the Z value of a point enclosed in a 3D triangle.
	 */
	static double interpolateZ(const Coordinate &p, const Coordinate &v0, 
			const Coordinate &v1, const Coordinate &v2) {
		double x0 = v0.x;
		double y0 = v0.y;
		double a = v1.x - x0;
		double b = v2.x - x0;
		double c = v1.y - y0;
		double d = v2.y - y0;
		double det = a * d - b * c;
		double dx = p.x - x0;
		double dy = p.y - y0;
		double t = (d * dx - b * dy) / det;
		double u = (-c * dx + a * dy) / det;
		double z = v0.z + t * (v1.z - v0.z) + u * (v2.z - v0.z);
		return z;
	}

	/**
	 * Computes the interpolated Z-value for a point p lying on the segment p0-p1
	 * 
	 * @param p
	 * @param p0
	 * @param p1
	 * @return
	 */
	static double interpolateZ(const Coordinate &p, const Coordinate &p0, 
			const Coordinate &p1) {
		double segLen = p0.distance(p1);
		double ptLen = p.distance(p0);
		double dz = p1.z - p0.z;
		double pz = p0.z + dz * (ptLen / segLen);
		return pz;
	}
};

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace geos

#endif //GEOS_TRIANGULATE_QUADEDGE_VERTEX_H

