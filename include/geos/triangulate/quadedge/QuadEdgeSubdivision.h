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
 * Last port: triangulate/quadedge/QuadEdgeSubdivision.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_QUADEDGESUBDIVISION_H
#define GEOS_TRIANGULATE_QUADEDGE_QUADEDGESUBDIVISION_H

#include <memory>
#include <list>
#include <vector>
#include <stack>
#include <set>

#include <geos/geom/Envelope.h>
#include <geos/geom/LineSegment.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/CoordinateArraySequenceFactory.h>
#include <geos/geom/GeometryCollection.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/MultiLineString.h>
#include <geos/util/IllegalArgumentException.h>
#include <geos/triangulate/quadedge/QuadEdge.h>
#include <geos/triangulate/quadedge/QuadEdgeLocator.h>
#include <geos/triangulate/quadedge/LastFoundQuadEdgeLocator.h>
#include <geos/triangulate/quadedge/LocateFailureException.h>
#include <geos/triangulate/quadedge/TriangleVisitor.h>
#include <geos/util/GEOSException.h>

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

const double EDGE_COINCIDENCE_TOL_FACTOR = 1000;

/**
 * A class that contains the {@link QuadEdge}s representing a planar
 * subdivision that models a triangulation. 
 * The subdivision is constructed using the
 * quadedge algebra defined in the classs {@link QuadEdge}. 
 * All metric calculations
 * are done in the {@link Vertex} class.
 * In addition to a triangulation, subdivisions
 * support extraction of Voronoi diagrams.
 * This is easily accomplished, since the Voronoi diagram is the dual
 * of the Delaunay triangulation.
 * <p>
 * Subdivisions can be provided with a tolerance value. Inserted vertices which
 * are closer than this value to vertices already in the subdivision will be
 * ignored. Using a suitable tolerance value can prevent robustness failures
 * from happening during Delaunay triangulation.
 * <p>
 * Subdivisions maintain a <b>frame</b> triangle around the client-created
 * edges. The frame is used to provide a bounded "container" for all edges
 * within a TIN. Normally the frame edges, frame connecting edges, and frame
 * triangles are not included in client processing.
 * 
 * @author JTS: David Skea
 * @author JTS: Martin Davis
 * @author Benjamin Campbell
 */
class GEOS_DLL QuadEdgeSubdivision {
public:
	typedef std::list<QuadEdge*> QuadEdgeList;

	/**
	 * Gets the edges for the triangle to the left of the given {@link QuadEdge}.
	 * 
	 * @param startQE
	 * @param triEdge
	 * 
	 * @throws IllegalArgumentException
	 *           if the edges do not form a triangle
	 */
	static void getTriangleEdges(const QuadEdge &startQE,
			const QuadEdge* triEdge[3]) {
		triEdge[0] = &startQE;
		triEdge[1] = &triEdge[0]->lNext();
		triEdge[2] = &triEdge[1]->lNext();
		if (&triEdge[2]->lNext() != triEdge[0]) {
			throw new
				util::IllegalArgumentException("Edges do not form a triangle");
		}
	}
private:
	QuadEdgeList quadEdges;
	QuadEdgeList removedEdges;
	QuadEdge* startingEdges[3];
	double tolerance;
	double edgeCoincidenceTolerance;
	Vertex frameVertex[3];
	geom::Envelope frameEnv;
	std::auto_ptr<QuadEdgeLocator> locator;

public:
	/**
	 * Creates a new instance of a quad-edge subdivision based on a frame triangle
	 * that encloses a supplied bounding box. A new super-bounding box that
	 * contains the triangle is computed and stored.
	 * 
	 * @param env
	 *          the bouding box to surround
	 * @param tolerance
	 *          the tolerance value for determining if two sites are equal
	 */
	QuadEdgeSubdivision(const geom::Envelope &env, double tolerance) :
			tolerance(tolerance),
			locator(new LastFoundQuadEdgeLocator(this)) {

		edgeCoincidenceTolerance = tolerance / EDGE_COINCIDENCE_TOL_FACTOR;
		createFrame(env);
		initSubdiv(startingEdges);
		quadEdges.push_back(startingEdges[0]);
		quadEdges.push_back(startingEdges[1]);
		quadEdges.push_back(startingEdges[2]);
	}

	~QuadEdgeSubdivision() {
		for(QuadEdgeList::iterator iter=quadEdges.begin(); iter!=quadEdges.end(); ++iter)
		{
			(*iter)->free();
			delete *iter;
		}

		for(QuadEdgeList::iterator iter=removedEdges.begin(); iter!=removedEdges.end(); ++iter)
		{
			(*iter)->free();
			delete *iter;
		}
	}

private:
	virtual void createFrame(const geom::Envelope &env)
	{
		double deltaX = env.getWidth();
		double deltaY = env.getHeight();
		double offset = 0.0;
		if (deltaX > deltaY) {
			offset = deltaX * 10.0;
		} else {
			offset = deltaY * 10.0;
		}

		frameVertex[0] = Vertex((env.getMaxX() + env.getMinX()) / 2.0, env
				.getMaxY() + offset);
		frameVertex[1] = Vertex(env.getMinX() - offset, env.getMinY() - offset);
		frameVertex[2] = Vertex(env.getMaxX() + offset, env.getMinY() - offset);

		frameEnv = Envelope(frameVertex[0].getCoordinate(), frameVertex[1]
				.getCoordinate());
		frameEnv.expandToInclude(frameVertex[2].getCoordinate());
	}
	
	virtual void initSubdiv(QuadEdge* initEdges[3])
	{
		std::auto_ptr<QuadEdge> tmp_auto_ptr;
		// build initial subdivision from frame
		tmp_auto_ptr = QuadEdge::makeEdge(frameVertex[0], frameVertex[1]);
		initEdges[0] = tmp_auto_ptr.get();
		tmp_auto_ptr.release();


		tmp_auto_ptr = QuadEdge::makeEdge(frameVertex[1], frameVertex[2]);
		initEdges[1] = tmp_auto_ptr.get();
		tmp_auto_ptr.release();

		QuadEdge::splice(initEdges[0]->sym(), *initEdges[1]);

		tmp_auto_ptr = QuadEdge::makeEdge(frameVertex[2], frameVertex[0]);
		initEdges[2] = tmp_auto_ptr.get();
		tmp_auto_ptr.release();

		QuadEdge::splice(initEdges[1]->sym(), *initEdges[2]);
		QuadEdge::splice(initEdges[2]->sym(), *initEdges[0]);
	}
	
public:
	/**
	 * Gets the vertex-equality tolerance value
	 * used in this subdivision
	 * 
	 * @return the tolerance value
	 */
	virtual double getTolerance() const {
		return tolerance;
	}

	/**
	 * Gets the envelope of the Subdivision (including the frame).
	 * 
	 * @return the envelope
	 */
	virtual const geom::Envelope& getEnvelope() const {
		return frameEnv;
	}

	/**
	 * Gets the collection of base {@link QuadEdge}s (one for every pair of
	 * vertices which is connected).
	 * 
	 * @return a QuadEdgeList
	 */
	virtual const QuadEdgeList& getEdges() const {
		return quadEdges;
	}

	/**
	 * Sets the {@link QuadEdgeLocator} to use for locating containing triangles
	 * in this subdivision.
	 * 
	 * @param locator
	 *          a QuadEdgeLocator
	 */
	virtual void setLocator(std::auto_ptr<QuadEdgeLocator> locator) {
		this->locator = locator;
	}

	/**
	 * Creates a new quadedge, recording it in the edges list.
	 * 
	 * @param o
	 * @param d
	 * @return
	 */
	virtual QuadEdge& makeEdge(const Vertex &o, const Vertex &d) {
		std::auto_ptr<QuadEdge> q0 = QuadEdge::makeEdge(o, d);
		QuadEdge *q0_ptr = q0.get();
		q0.release();

		quadEdges.push_back(q0_ptr);
		return *q0_ptr;
	}

	/**
	 * Creates a new QuadEdge connecting the destination of a to the origin of b,
	 * in such a way that all three have the same left face after the connection
	 * is complete. The quadedge is recorded in the edges list.
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	virtual QuadEdge& connect(QuadEdge &a, QuadEdge &b) {
		std::auto_ptr<QuadEdge> q0 = QuadEdge::connect(a, b);
		QuadEdge *q0_ptr = q0.get();
		q0.release();

		quadEdges.push_back(q0_ptr);
		return *q0_ptr;
	}

	/**
	 * Deletes a quadedge from the subdivision. Linked quadedges are updated to
	 * reflect the deletion.
	 * 
	 * @param e
	 *          the quadedge to delete
	 */
	void remove(QuadEdge &e) {
		QuadEdge::splice(e, e.oPrev());
		QuadEdge::splice(e.sym(), e.sym().oPrev());

		// this is inefficient on an ArrayList, but this method should be called infrequently
		quadEdges.remove(&e);

		//mark these edges as removed
		e.remove();

		//keep a list of removed edges so that we can
		//properly free memory
		removedEdges.push_back(&e);
	}

	/**
	 * Locates an edge of a triangle which contains a location 
	 * specified by a Vertex v. 
	 * The edge returned has the
	 * property that either v is on e, or e is an edge of a triangle containing v.
	 * The search starts from startEdge amd proceeds on the general direction of v.
	 * <p>
	 * This locate algorithm relies on the subdivision being Delaunay. For
	 * non-Delaunay subdivisions, this may loop for ever.
	 * 
	 * @param v the location to search for
	 * @param startEdge an edge of the subdivision to start searching at
	 * @returns a QuadEdge which contains v, or is on the edge of a triangle containing v
	 * @throws LocateFailureException
	 *           if the location algorithm fails to converge in a reasonable
	 *           number of iterations
	 */
	QuadEdge* locateFromEdge(const Vertex &v,
			const QuadEdge &startEdge) const {
		int iter = 0;
		int maxIter = quadEdges.size();

		QuadEdge *e = startingEdges[0];

		while (true) {
			++iter;
			/**
			 * So far it has always been the case that failure to locate indicates an
			 * invalid subdivision. So just fail completely. (An alternative would be
			 * to perform an exhaustive search for the containing triangle, but this
			 * would mask errors in the subdivision topology)
			 * 
			 * This can also happen if two vertices are located very close together,
			 * since the orientation predicates may experience precision failures.
			 */
			if (iter > maxIter) {
				throw LocateFailureException("");
			}

			if ((v.equals(e->orig())) || (v.equals(e->dest()))) {
				break;
			} else if (v.rightOf(*e)) {
				e = &e->sym();
			} else if (!v.rightOf(e->oNext())) {
				e = &e->oNext();
			} else if (!v.rightOf(e->dPrev())) {
				e = &e->dPrev();
			} else {
				// on edge or in triangle containing edge
				break;
			}
		}
		return e;
	}

	/**
	 * Finds a quadedge of a triangle containing a location 
	 * specified by a {@link Vertex}, if one exists.
	 * 
	 * @param x the vertex to locate
	 * @return a quadedge on the edge of a triangle which touches or contains the location
	 * @return null if no such triangle exists
	 */
	QuadEdge* locate(const Vertex &v) const {
		return locator->locate(v);
	}

	/**
	 * Finds a quadedge of a triangle containing a location
	 * specified by a {@link Coordinate}, if one exists.
	 * 
	 * @param p the Coordinate to locate
	 * @return a quadedge on the edge of a triangle which touches or contains the location
	 * @return null if no such triangle exists
	 */
	QuadEdge* locate(const Coordinate &p) {
		return locator->locate(Vertex(p));
	}

	/**
	 * Locates the edge between the given vertices, if it exists in the
	 * subdivision.
	 * 
	 * @param p0 a coordinate
	 * @param p1 another coordinate
	 * @return the edge joining the coordinates, if present
	 * @return null if no such edge exists
	 * @return the caller _should not_ free the returned pointer
	 */
	QuadEdge* locate(const Coordinate &p0, const Coordinate &p1) {
		// find an edge containing one of the points
		QuadEdge *e = locator->locate(Vertex(p0));
		if (e == NULL)
			return NULL;

		// normalize so that p0 is origin of base edge
		QuadEdge *base = e;
		if (e->dest().getCoordinate().equals2D(p0))
			base = &e->sym();
		// check all edges around origin of base edge
		QuadEdge *locEdge = base;
		do {
			if (locEdge->dest().getCoordinate().equals2D(p1))
				return locEdge;
			locEdge = &locEdge->oNext();
		} while (locEdge != base);
		return NULL;
	}

	/**
	 * Inserts a new site into the Subdivision, connecting it to the vertices of
	 * the containing triangle (or quadrilateral, if the split point falls on an
	 * existing edge).
	 * <p>
	 * This method does NOT maintain the Delaunay condition. If desired, this must
	 * be checked and enforced by the caller.
	 * <p>
	 * This method does NOT check if the inserted vertex falls on an edge. This
	 * must be checked by the caller, since this situation may cause erroneous
	 * triangulation
	 * 
	 * @param v
	 *          the vertex to insert
	 * @return a new quad edge terminating in v
	 */
	QuadEdge& insertSite(const Vertex &v) {
		QuadEdge *e = locate(v);

		if ((v.equals(e->orig(), tolerance)) || (v.equals(e->dest(), tolerance))) {
			return *e; // point already in subdivision.
		}

		// Connect the new point to the vertices of the containing
		// triangle (or quadrilateral, if the new point fell on an
		// existing edge.)
		QuadEdge *base = &makeEdge(e->orig(), v);
		QuadEdge::splice(*base, *e);
		QuadEdge *startEdge = base;
		do {
			base = &connect(*e, base->sym());
			e = &base->oPrev();
		} while (&e->lNext() != startEdge);

		return *startEdge;
	}

	/**
	 * Tests whether a QuadEdge is an edge incident on a frame triangle vertex.
	 * 
	 * @param e
	 *          the edge to test
	 * @return true if the edge is connected to the frame triangle
	 */
	bool isFrameEdge(const QuadEdge &e) const {
		if (isFrameVertex(e.orig()) || isFrameVertex(e.dest()))
			return true;
		return false;
	}

	/**
	 * Tests whether a QuadEdge is an edge on the border of the frame facets and
	 * the internal facets. E.g. an edge which does not itself touch a frame
	 * vertex, but which touches an edge which does.
	 * 
	 * @param e
	 *          the edge to test
	 * @return true if the edge is on the border of the frame
	 */
	bool isFrameBorderEdge(const QuadEdge &e) const {
		// check other vertex of triangle to left of edge
		Vertex vLeftTriOther = e.lNext().dest();
		if (isFrameVertex(vLeftTriOther))
			return true;
		// check other vertex of triangle to right of edge
		Vertex vRightTriOther = e.sym().lNext().dest();
		if (isFrameVertex(vRightTriOther))
			return true;

		return false;
	}

	/**
	 * Tests whether a vertex is a vertex of the outer triangle.
	 * 
	 * @param v
	 *          the vertex to test
	 * @return true if the vertex is an outer triangle vertex
	 */
	bool isFrameVertex(const Vertex &v) const {
		if (v.equals(frameVertex[0]))
			return true;
		if (v.equals(frameVertex[1]))
			return true;
		if (v.equals(frameVertex[2]))
			return true;
		return false;
	}


	/**
	 * Tests whether a {@link Coordinate} lies on a {@link QuadEdge}, up to a
	 * tolerance determined by the subdivision tolerance.
	 * 
	 * @param e
	 *          a QuadEdge
	 * @param p
	 *          a point
	 * @return true if the vertex lies on the edge
	 */
	bool isOnEdge(const QuadEdge &e, const Coordinate &p) const {
		geom::LineSegment seg;
		seg.setCoordinates(e.orig().getCoordinate(), e.dest().getCoordinate());
		double dist = seg.distance(p);
		// heuristic (hack?)
		return dist < edgeCoincidenceTolerance;
	}

	/**
	 * Tests whether a {@link Vertex} is the start or end vertex of a
	 * {@link QuadEdge}, up to the subdivision tolerance distance.
	 * 
	 * @param e
	 * @param v
	 * @return true if the vertex is a endpoint of the edge
	 */
	bool isVertexOfEdge(const QuadEdge &e, const Vertex &v) const {
		if ((v.equals(e.orig(), tolerance)) || (v.equals(e.dest(), tolerance))) {
			return true;
		}
		return false;
	}

  /**
   * Gets the unique {@link Vertex}es in the subdivision,
   * including the frame vertices if desired.
   * 
	 * @param includeFrame
	 *          true if the frame vertices should be included
   * @return a collection of the subdivision vertices
   * 
   * @see #getVertexUniqueEdges
   */
  //public Collection getVertices(boolean includeFrame) 
  //{
	//Set vertices = new HashSet();
	//for (Iterator i = quadEdges.iterator(); i.hasNext();) {
	  //QuadEdge qe = (QuadEdge) i.next();
	  //Vertex v = qe.orig();
	  //System.out.println(v);
	  //if (includeFrame || ! isFrameVertex(v))
		//vertices.add(v);
	  
	  //*
	   //Inspect the sym edge as well, since it is
	   //possible that a vertex is only at the 
	   //dest of all tracked quadedges.
	  //Vertex vd = qe.dest();
	  //System.out.println(vd);
	  //if (includeFrame || ! isFrameVertex(vd))
		//vertices.add(vd);
	//}
	//return vertices;
  //}

  /**
   * Gets a collection of {@link QuadEdge}s whose origin
   * vertices are a unique set which includes
   * all vertices in the subdivision. 
   * The frame vertices can be included if required.
   * <p>
   * This is useful for algorithms which require traversing the 
   * subdivision starting at all vertices.
   * Returning a quadedge for each vertex
   * is more efficient than 
   * the alternative of finding the actual vertices
   * using {@link #getVertices) and then locating 
   * quadedges attached to them.
   * 
   * @param includeFrame true if the frame vertices should be included
   * @return a collection of QuadEdge with the vertices of the subdivision as their origins
   */
  //public List getVertexUniqueEdges(boolean includeFrame) 
  //{
      //List edges = new ArrayList();
    //Set visitedVertices = new HashSet();
    //for (Iterator i = quadEdges.iterator(); i.hasNext();) {
      //QuadEdge qe = (QuadEdge) i.next();
      //Vertex v = qe.orig();
      ////System.out.println(v);
      //if (! visitedVertices.contains(v)) {
          //visitedVertices.add(v);
        //if (includeFrame || ! isFrameVertex(v)) {
            //edges.add(qe);
        //}
      //}
      
      /**
       * Inspect the sym edge as well, since it is
       * possible that a vertex is only at the 
       * dest of all tracked quadedges.
       */
      //QuadEdge qd = qe.sym();
      //Vertex vd = qd.orig();
      ////System.out.println(vd);
      //if (! visitedVertices.contains(vd)) {
          //visitedVertices.add(vd);
        //if (includeFrame || ! isFrameVertex(vd)) {
            //edges.add(qd);
        //}
      //}
    //}
    //return edges;
  //}

	/**
	 * Gets all primary quadedges in the subdivision. 
	 * A primary edge is a {@link QuadEdge}
	 * which occupies the 0'th position in its array of associated quadedges. 
	 * These provide the unique geometric edges of the triangulation.
	 * 
	 * @param includeFrame true if the frame edges are to be included
	 * @return a List of QuadEdges. The caller takes ownership of the returned QuadEdgeList but not the
	 * items it contains.
	 */
	QuadEdgeList* getPrimaryEdges(bool includeFrame)
	{

		QuadEdgeList *edges = new QuadEdgeList();
		QuadEdgeStack edgeStack;
		QuadEdgeSet visitedEdges;

		edgeStack.push(startingEdges[0]);

		while (!edgeStack.empty())
		{
			QuadEdge *edge = edgeStack.top();
			edgeStack.pop();
			if (visitedEdges.find(edge) == visitedEdges.end())
			{
				QuadEdge* priQE = (QuadEdge*)&edge->getPrimary();

				if (includeFrame || ! isFrameEdge(*priQE))
					edges->push_back(priQE);

				edgeStack.push(&edge->oNext());
				edgeStack.push(&edge->sym().oNext());
				
				visitedEdges.insert(edge);
				visitedEdges.insert(&edge->sym());
			}
		}
		return edges;
	}
  
  /**
   * A TriangleVisitor which computes and sets the 
   * circumcentre as the origin of the dual 
   * edges originating in each triangle.
   * 
   * @author mbdavis
   *
   */
	//private static class TriangleCircumcentreVisitor implements TriangleVisitor 
	//{
		//public TriangleCircumcentreVisitor() {
		//}

		//public void visit(QuadEdge[] triEdges) 
		//{
			//Coordinate a = triEdges[0].orig().getCoordinate();
			//Coordinate b = triEdges[1].orig().getCoordinate();
			//Coordinate c = triEdges[2].orig().getCoordinate();
			
			//// TODO: choose the most accurate circumcentre based on the edges
      //Coordinate cc = Triangle.circumcentre(a, b, c);
			//Vertex ccVertex = new Vertex(cc);
			//// save the circumcentre as the origin for the dual edges originating in this triangle
			//for (int i = 0; i < 3; i++) {
				//triEdges[i].rot().setOrig(ccVertex);
			//}
		//}
	//}

	/*****************************************************************************
	 * Visitors
	 ****************************************************************************/

	void visitTriangles(TriangleVisitor *triVisitor,
			bool includeFrame) {

		QuadEdgeStack edgeStack;
		edgeStack.push(startingEdges[0]);

		QuadEdgeSet visitedEdges;
		
		while (!edgeStack.empty()) {
			QuadEdge *edge = edgeStack.top();
			edgeStack.pop();
			if (visitedEdges.find(edge) == visitedEdges.end()) {
				QuadEdge **triEdges = fetchTriangleToVisit(edge, edgeStack,
						includeFrame, visitedEdges);
				if (triEdges != NULL)
					triVisitor->visit(triEdges);
			}
		}
	}

private:
	typedef std::stack<QuadEdge*> QuadEdgeStack;
	typedef std::set<QuadEdge*> QuadEdgeSet;
	typedef std::list< geom::CoordinateSequence*> TriList;

	/**
	 * The quadedges forming a single triangle.
	 * Only one visitor is allowed to be active at a
	 * time, so this is safe.
	 */
	QuadEdge* triEdges[3];
	/**
	 * Stores the edges for a visited triangle. Also pushes sym (neighbour) edges
	 * on stack to visit later.
	 * 
	 * @param edge
	 * @param edgeStack
	 * @param includeFrame
	 * @return the visited triangle edges
	 * @return null if the triangle should not be visited (for instance, if it is
	 *         outer)
	 */
	//QuadEdge (*fetchTriangleToVisit(QuadEdge *edge,
			//QuadEdgeStack &edgeStack, bool includeFrame,
			//QuadEdgeSet &visitedEdges))[3] {
	QuadEdge** fetchTriangleToVisit(QuadEdge *edge,
			QuadEdgeStack &edgeStack, bool includeFrame,
			QuadEdgeSet &visitedEdges) {
		QuadEdge *curr = edge;
		int edgeCount = 0;
		bool isFrame = false;
		do {
			triEdges[edgeCount] = curr;

			if (isFrameEdge(*curr))
				isFrame = true;
			
			// push sym edges to visit next
			QuadEdge *sym = &curr->sym();
			if (visitedEdges.find(sym) == visitedEdges.end())
				edgeStack.push(sym);
			
			// mark this edge as visited
			visitedEdges.insert(curr);
			
			edgeCount++;
			curr = &curr->lNext();

		} while (curr != edge);

		if (isFrame && !includeFrame)
			return NULL;
		return triEdges;
	}

	/**
	 * Gets a list of the triangles
	 * in the subdivision, specified as
	 * an array of the primary quadedges around the triangle.
	 * 
	 * @param includeFrame
	 *          true if the frame triangles should be included
	 * @return a List of QuadEdge[3] arrays
	 */
	//public List getTriangleEdges(boolean includeFrame) {
		//TriangleEdgesListVisitor visitor = new TriangleEdgesListVisitor();
		//visitTriangles(visitor, includeFrame);
		//return visitor.getTriangleEdges();
	//}

	//private static class TriangleEdgesListVisitor implements TriangleVisitor {
		//private List triList = new ArrayList();

		//public void visit(QuadEdge[] triEdges) {
			//triList.add(triEdges.clone());
		//}

		//public List getTriangleEdges() {
			//return triList;
		//}
	//}

	/**
	 * Gets a list of the triangles in the subdivision,
	 * specified as an array of the triangle {@link Vertex}es.
	 * 
	 * @param includeFrame
	 *          true if the frame triangles should be included
	 * @return a List of Vertex[3] arrays
	 */
	//public List getTriangleVertices(boolean includeFrame) {
		//TriangleVertexListVisitor visitor = new TriangleVertexListVisitor();
		//visitTriangles(visitor, includeFrame);
		//return visitor.getTriangleVertices();
	//}

	//private static class TriangleVertexListVisitor implements TriangleVisitor {
		//private List triList = new ArrayList();

		//public void visit(QuadEdge[] triEdges) {
			//triList.add(new Vertex[] { triEdges[0].orig(), triEdges[1].orig(),
					//triEdges[2].orig() });
		//}

		//public List getTriangleVertices() {
			//return triList;
		//}
	//}

	/**
	 * Gets the coordinates for each triangle in the subdivision as an array.
	 * 
	 * @param includeFrame
	 *          true if the frame triangles should be included
	 * @param triList a list of Coordinate[4] representing each triangle
	 */
	void getTriangleCoordinates(TriList* triList, bool includeFrame) {
		TriangleCoordinatesVisitor visitor(triList);
		visitTriangles((TriangleVisitor*)&visitor, includeFrame);
	}
private:
	class TriangleCoordinatesVisitor : public TriangleVisitor {
	private:
		TriList *triCoords;
		CoordinateArraySequenceFactory coordSeqFact;

	public:
		TriangleCoordinatesVisitor(TriList *triCoords): triCoords(triCoords) {
		}

		virtual void visit(QuadEdge* triEdges[3]) {
			geom::CoordinateSequence *coordSeq = coordSeqFact.create(4,2);
			for (int i = 0; i < 3; i++) {
				Vertex v = triEdges[i]->orig();
				coordSeq->setAt(v.getCoordinate(), i);
			}
			coordSeq->setAt(triEdges[0]->orig().getCoordinate(), 3);
			triCoords->push_back(coordSeq);
		}
	} ; 

public:
	/**
	 * Gets the geometry for the edges in the subdivision as a {@link MultiLineString}
	 * containing 2-point lines.
	 * 
	 * @param geomFact the GeometryFactory to use
	 * @return a MultiLineString
	 */
	geom::MultiLineString* getEdges(const geom::GeometryFactory& geomFact)
	{
		QuadEdgeList *quadEdges = getPrimaryEdges(false);
		std::vector<Geometry *> edges(quadEdges->size());
		CoordinateArraySequenceFactory coordSeqFact;
		int i = 0;
		for (QuadEdgeList::iterator it = quadEdges->begin(); it != quadEdges->end(); ++it)
		{
			QuadEdge *qe = *it;
			CoordinateSequence *coordSeq = coordSeqFact.create((std::vector<geom::Coordinate>*)NULL, 2);;

			coordSeq->add(qe->orig().getCoordinate());
			coordSeq->add(qe->dest().getCoordinate());

			edges[i++] = geomFact.createLineString(*coordSeq);

			delete coordSeq;
		}
		delete quadEdges;

		geom::MultiLineString* result = geomFact.createMultiLineString(edges);

		for(std::vector<Geometry*>::iterator it=edges.begin(); it!=edges.end(); ++it)
			delete *it;

		return result;
	}

	/**
	 * Gets the geometry for the triangles in a triangulated subdivision as a {@link GeometryCollection}
	 * of triangular {@link Polygon}s.
	 * 
	 * @param geomFact the GeometryFactory to use
	 * @return a GeometryCollection of triangular Polygons
	 */
	geom::GeometryCollection* getTriangles(const GeometryFactory &geomFact);

	/**
	 * Gets the cells in the Voronoi diagram for this triangulation.
	 * The cells are returned as a {@link GeometryCollection} of {@link Polygon}s
   * <p>
   * The userData of each polygon is set to be the {@link Coordinate)
   * of the cell site.  This allows easily associating external 
   * data associated with the sites to the cells.
	 * 
	 * @param geomFact a geometry factory
	 * @return a GeometryCollection of Polygons
	 */
  //public Geometry getVoronoiDiagram(GeometryFactory geomFact)
  //{
    //List vorCells = getVoronoiCellPolygons(geomFact);
    //return geomFact.createGeometryCollection(GeometryFactory.toGeometryArray(vorCells));   
  //}
  
	/**
	 * Gets a List of {@link Polygon}s for the Voronoi cells 
	 * of this triangulation.
   * <p>
   * The userData of each polygon is set to be the {@link Coordinate)
   * of the cell site.  This allows easily associating external 
   * data associated with the sites to the cells.
	 * 
	 * @param geomFact a geometry factory
	 * @return a List of Polygons
	 */
  //public List getVoronoiCellPolygons(GeometryFactory geomFact)
  //{
      /*
       * Compute circumcentres of triangles as vertices for dual edges.
       * Precomputing the circumcentres is more efficient, 
       * and more importantly ensures that the computed centres
       * are consistent across the Voronoi cells.
       */ 
      //visitTriangles(new TriangleCircumcentreVisitor(), true);
      
    //List cells = new ArrayList();
    //Collection edges = getVertexUniqueEdges(false);
    //for (Iterator i = edges.iterator(); i.hasNext(); ) {
        //QuadEdge qe = (QuadEdge) i.next();
      //cells.add(getVoronoiCellPolygon(qe, geomFact));
    //}
    //return cells;
  //}
  
  /**
   * Gets the Voronoi cell around a site specified
   * by the origin of a QuadEdge.
   * <p>
   * The userData of the polygon is set to be the {@link Coordinate)
   * of the site.  This allows attaching external 
   * data associated with the site to this cell polygon.
   * 
   * @param qe a quadedge originating at the cell site
   * @param geomFact a factory for building the polygon
   * @return a polygon indicating the cell extent
   */
//public Polygon getVoronoiCellPolygon(QuadEdge qe, GeometryFactory geomFact)
//{
	//List cellPts = new ArrayList();
	//QuadEdge startQE = qe;
	//do {
		////    	Coordinate cc = circumcentre(qe);
		//// use previously computed circumcentre
		//Coordinate cc = qe.rot().orig().getCoordinate();
		//cellPts.add(cc);

		//// move to next triangle CW around vertex
		//qe = qe.oPrev();
	//} while (qe != startQE);

	//CoordinateList coordList = new CoordinateList();
	//coordList.addAll(cellPts, false);
	//coordList.closeRing();

	//if (coordList.size() < 4) {
		//System.out.println(coordList);
		//coordList.add(coordList.get(coordList.size()-1), true);
	//}

	//Coordinate[] pts = coordList.toCoordinateArray();
	//Polygon cellPoly = geomFact.createPolygon(geomFact.createLinearRing(pts), null);

	//Vertex v = startQE.orig();
	//cellPoly.setUserData(v.getCoordinate());
	//return cellPoly;
//}
  
};

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_QUADEDGESUBDIVISION_H
