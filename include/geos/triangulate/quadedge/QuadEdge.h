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
 * Last port: triangulate/quadedge/QuadEdge.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_QUADEDGE_H
#define GEOS_TRIANGULATE_QUADEDGE_QUADEDGE_H

#include <memory>

//TODO: XXX: FIXME: BLC: QuadEdge Memory managment and valgrind!!
//TODO: XXX: FIXME: BLC: Return auto_ptr's when appropriate
//TODO: XXX: FIXME: BLC: Move function defs to .cpp files!!
//TODO: XXX: FIXME: BLC: Don't put 'using' in header files!!

#include <geos/triangulate/quadedge/Vertex.h>
#include <geos/geom/LineSegment.h>

using geos::geom::LineSegment;

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

/**
 * A class that represents the edge data structure which implements the quadedge algebra. 
 * The quadedge algebra was described in a well-known paper by Guibas and Stolfi,
 * "Primitives for the manipulation of general subdivisions and the computation of Voronoi diagrams", 
 * <i>ACM Transactions on Graphics</i>, 4(2), 1985, 75-123.
 * <p>
 * Each edge object is part of a quartet of 4 edges,
 * linked via their <tt>_rot</tt> references.
 * Any edge in the group may be accessed using a series of {@link #rot()} operations.
 * Quadedges in a subdivision are linked together via their <tt>next</tt> references.
 * The linkage between the quadedge quartets determines the topology
 * of the subdivision. 
 * <p>
 * The edge class does not contain separate information for vertice or faces; a vertex is implicitly
 * defined as a ring of edges (created using the <tt>next</tt> field).
 * 
 * @author JTS: David Skea
 * @author JTS: Martin Davis
 * @author Benjamin Campbell
 * */
class GEOS_DLL QuadEdge {
public:
	/**
	 * Creates a new QuadEdge quartet from {@link Vertex} o to {@link Vertex} d.
	 * 
	 * @param o
	 *		  the origin Vertex
	 * @param d
	 *		  the destination Vertex
	 * @return the new QuadEdge* The caller is reponsible for
	 * freeing the returned pointer
	 */
	static QuadEdge* makeEdge(const Vertex &o, const Vertex &d) {
		QuadEdge *q0 = new QuadEdge();
		//q1-q3 are free()'d by q0
		QuadEdge *q1 = new QuadEdge();
		QuadEdge *q2 = new QuadEdge();
		QuadEdge *q3 = new QuadEdge();

		q0->_rot = q1;
		q1->_rot = q2;
		q2->_rot = q3;
		q3->_rot = q0;

		q0->setNext(q0);
		q1->setNext(q3);
		q2->setNext(q2);
		q3->setNext(q1);

		QuadEdge *base = q0;
		base->setOrig(o);
		base->setDest(d);

		return base;
	}

	/**
	 * Creates a new QuadEdge connecting the destination of a to the origin of
	 * b, in such a way that all three have the same left face after the
	 * connection is complete. Additionally, the data pointers of the new edge
	 * are set.
	 *
	 * @return the new QuadEdge* The caller is reponsible for
	 * freeing the returned pointer
	 */
	static QuadEdge* connect(QuadEdge &a, QuadEdge &b) {
		QuadEdge *q0 = makeEdge(a.dest(), b.orig());
		splice(*q0, a.lNext());
		splice(q0->sym(), b);
		return q0;
	}

	/**
	 * Splices two edges together or apart.
	 * Splice affects the two edge rings around the origins of a and b, and, independently, the two
	 * edge rings around the left faces of <tt>a</tt> and <tt>b</tt>. 
	 * In each case, (i) if the two rings are distinct,
	 * Splice will combine them into one, or (ii) if the two are the same ring, Splice will break it
	 * into two separate pieces. Thus, Splice can be used both to attach the two edges together, and
	 * to break them apart.
	 * 
	 * @param a an edge to splice
	 * @param b an edge to splice
	 * 
	 */
	static void splice(QuadEdge &a, QuadEdge &b) {
		QuadEdge &alpha = a.oNext().rot();
		QuadEdge &beta = b.oNext().rot();

		QuadEdge &t1 = b.oNext();
		QuadEdge &t2 = a.oNext();
		QuadEdge &t3 = beta.oNext();
		QuadEdge &t4 = alpha.oNext();

		a.setNext(&t1);
		b.setNext(&t2);
		alpha.setNext(&t3);
		beta.setNext(&t4);
	}

	/**
	 * Turns an edge counterclockwise inside its enclosing quadrilateral.
	 * 
	 * @param e the quadedge to turn
	 */
	static void swap(QuadEdge &e) {
		QuadEdge &a = e.oPrev();
		QuadEdge &b = e.sym().oPrev();
		splice(e, a);
		splice(e.sym(), b);
		splice(e, a.lNext());
		splice(e.sym(), b.lNext());
		e.setOrig(a.dest());
		e.setDest(b.dest());
	}

private:
	//// the dual of this edge, directed from right to left
	QuadEdge *_rot;
	Vertex   vertex;			// The vertex that this edge represents
	QuadEdge *next;			  // A reference to a connected edge
	void*   data;
	bool isAlive;

	/**
	 * Quadedges must be made using {@link makeEdge}, 
	 * to ensure proper construction.
	 */
	QuadEdge() : _rot(NULL), vertex(), next(NULL), data(NULL), isAlive(true)
	{ }

public:
	~QuadEdge()
	{
	}

	/**
	 * Free the QuadEdge quartet associated with this QuadEdge by a connect()
	 * or makeEdge() call.
	 * DO NOT call this function on a QuadEdge that was not returned
	 * by connect() or makeEdge() as it will cause double free's.
	 */
	virtual void free()
	{
		if(_rot)
		{
			if(_rot->_rot)
			{
				if(_rot->_rot->_rot)
				{
					delete _rot->_rot->_rot;
					_rot->_rot->_rot = NULL;
				}
				delete _rot->_rot;
				_rot->_rot = NULL;
			}
			delete _rot;
			_rot = NULL;
		}
	}

	/**
	 * Gets the primary edge of this quadedge and its <tt>sym</tt>.
	 * The primary edge is the one for which the origin
	 * and destination coordinates are ordered
	 * according to the standard {@link Coordinate} ordering
	 * 
	 * @return the primary quadedge
	 */
	const QuadEdge& getPrimary() const
	{
		if (orig().getCoordinate().compareTo(dest().getCoordinate()) <= 0)
			return *this;
		else 
			return sym();
	}
	
	/**
	 * Sets the external data value for this edge.
	 * 
	 * @param data an object containing external data
	 */
	virtual void setData(void* data) {
		this->data = data;
	}
	
	/**
	 * Gets the external data value for this edge.
	 * 
	 * @return the data object
	 */
	virtual void* getData() {
		return data;
	}

	/**
	 * Marks this quadedge as being deleted.
	 * This does not free the memory used by
	 * this quadedge quartet, but indicates
	 * that this quadedge quartet no longer participates
	 * in a subdivision.
	 *
	 */
	virtual void remove() {
		rot().rot().rot().isAlive = false;
		rot().rot().isAlive = false;
		rot().isAlive = false;
		isAlive = false;
	}
	
	/**
	 * Tests whether this edge has been deleted.
	 * 
	 * @return true if this edge has not been deleted.
	 */
	virtual bool isLive() {
		return isAlive;
	}


	/**
	 * Sets the connected edge
	 * 
	 * @param nextEdge edge
	 */
	virtual void setNext(QuadEdge *next) {
		this->next = next;
	}
	
	/***************************************************************************
	 * QuadEdge Algebra 
	 ***************************************************************************
	 */

	/**
	 * Gets the dual of this edge, directed from its right to its left.
	 * 
	 * @return the rotated edge
	 */
	 QuadEdge& rot() const {
	  return *_rot;
	}

	/**
	 * Gets the dual of this edge, directed from its left to its right.
	 * 
	 * @return the inverse rotated edge.
	 */
	QuadEdge& invRot() const  {
	  return rot().sym();
	}

	/**
	 * Gets the edge from the destination to the origin of this edge.
	 * 
	 * @return the sym of the edge
	 */
	QuadEdge& sym() const {
	  return rot().rot();
	}

	/**
	 * Gets the next CCW edge around the origin of this edge.
	 * 
	 * @return the next linked edge.
	 */
	QuadEdge& oNext() const {
		return *next;
	}

	/**
	 * Gets the next CW edge around (from) the origin of this edge.
	 * 
	 * @return the previous edge.
	 */
	QuadEdge& oPrev() const {
		return rot().oNext().rot();
	}

	/**
	 * Gets the next CCW edge around (into) the destination of this edge.
	 * 
	 * @return the next destination edge.
	 */
	QuadEdge& dNext() const {
		return sym().oNext().sym();
	}

	/**
	 * Gets the next CW edge around (into) the destination of this edge.
	 * 
	 * @return the previous destination edge.
	 */
	QuadEdge& dPrev() const {
		return invRot().oNext().invRot();
	}

	/**
	 * Gets the CCW edge around the left face following this edge.
	 * 
	 * @return the next left face edge.
	 */
	QuadEdge& lNext() const {
		return invRot().oNext().rot();
	}

	/**
	 * Gets the CCW edge around the left face before this edge.
	 * 
	 * @return the previous left face edge.
	 */
	QuadEdge& lPrev() const {
		return oNext().sym();
	}

	/**
	 * Gets the edge around the right face ccw following this edge.
	 * 
	 * @return the next right face edge.
	 */
	QuadEdge& rNext() {
		return rot().oNext().invRot();
	}

	/**
	 * Gets the edge around the right face ccw before this edge.
	 * 
	 * @return the previous right face edge.
	 */
	QuadEdge& rPrev() {
		return sym().oNext();
	}

	/***********************************************************************************************
	 * Data Access
	 **********************************************************************************************/
	/**
	 * Sets the vertex for this edge's origin
	 * 
	 * @param o the origin vertex
	 */
	virtual void setOrig(const Vertex &o) {
		vertex = o;
	}

	/**
	 * Sets the vertex for this edge's destination
	 * 
	 * @param d the destination vertex
	 */
	virtual void setDest(const Vertex &d) {
		sym().setOrig(d);
	}

	/**
	 * Gets the vertex for the edge's origin
	 * 
	 * @return the origin vertex
	 */
	const Vertex& orig() const {
		return vertex;
	}

	/**
	 * Gets the vertex for the edge's destination
	 * 
	 * @return the destination vertex
	 */
	const Vertex& dest() const {
		return sym().orig();
	}

	/**
	 * Gets the length of the geometry of this quadedge.
	 * 
	 * @return the length of the quadedge
	 */
	double getLength() const {
		return orig().getCoordinate().distance(dest().getCoordinate());
	}

	/**
	 * Tests if this quadedge and another have the same line segment geometry, 
	 * regardless of orientation.
	 * 
	 * @param qe a quadege
	 * @return true if the quadedges are based on the same line segment regardless of orientation
	 */
	bool equalsNonOriented(const QuadEdge &qe) const {
		if (equalsOriented(qe))
			return true;
		if (equalsOriented(qe.sym()))
			return true;
		return false;
	}

	/**
	 * Tests if this quadedge and another have the same line segment geometry
	 * with the same orientation.
	 * 
	 * @param qe a quadege
	 * @return true if the quadedges are based on the same line segment
	 */
	bool equalsOriented(const QuadEdge &qe) const {
		if (orig().getCoordinate().equals2D(qe.orig().getCoordinate())
				&& dest().getCoordinate().equals2D(qe.dest().getCoordinate()))
			return true;
		return false;
	}

	/**
	 * Creates a {@link LineSegment} representing the
	 * geometry of this edge.
	 * 
	 * @return a LineSegment
	 */
	std::auto_ptr<LineSegment> toLineSegment() const
	{
		return std::auto_ptr<LineSegment>(
				new LineSegment(vertex.getCoordinate(), dest().getCoordinate()));
	}
};

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_QUADEDGE_H

