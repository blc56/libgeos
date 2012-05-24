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
 * Last port: triangulate/IncrementalDelaunayTriangulator.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_INCREMENTALDELAUNAYTRIANGULATOR_H
#define GEOS_TRIANGULATE_INCREMENTALDELAUNAYTRIANGULATOR_H

#include <geos/triangulate/quadedge/QuadEdge.h>
#include <geos/triangulate/quadedge/QuadEdgeSubdivision.h>
#include <geos/triangulate/quadedge/Vertex.h>


using namespace geos::triangulate;
using namespace geos::triangulate::quadedge;

namespace geos {
namespace triangulate { //geos.triangulate

/**
 * Computes a Delauanay Triangulation of a set of {@link Vertex}es, using an
 * incrementatal insertion algorithm.
 * 
 * @author JTS: Martin Davis
 * @author Benjamin Campbell
 */
class GEOS_DLL IncrementalDelaunayTriangulator 
{
private:
	QuadEdgeSubdivision *subdiv;
	bool isUsingTolerance;

public:
	/**
	 * Creates a new triangulator using the given {@link QuadEdgeSubdivision}.
	 * The triangulator uses the tolerance of the supplied subdivision.
	 * 
	 * @param subdiv
	 *          a subdivision in which to build the TIN
	 */
	IncrementalDelaunayTriangulator(QuadEdgeSubdivision *subdiv) :
			subdiv(subdiv), isUsingTolerance(subdiv->getTolerance() > 0.0) { 
	}

	typedef std::list<Vertex*> VertexList;

	/**
	 * Inserts all sites in a collection. The inserted vertices <b>MUST</b> be
	 * unique up to the provided tolerance value. (i.e. no two vertices should be
	 * closer than the provided tolerance value). They do not have to be rounded
	 * to the tolerance grid, however.
	 * 
	 * @param vertices a Collection of Vertex
	 * 
   * @throws LocateFailureException if the location algorithm fails to converge in a reasonable number of iterations
	 */
	void insertSites(const VertexList& vertices) {
		for (VertexList::const_iterator x=vertices.begin(); 
				x != vertices.end(); ++x) {
			insertSite(**x);
		}
	}

	/**
	 * Inserts a new point into a subdivision representing a Delaunay
	 * triangulation, and fixes the affected edges so that the result is still a
	 * Delaunay triangulation.
	 * <p>
	 * 
	 * @return a quadedge containing the inserted vertex
	 */
	QuadEdge& insertSite(const Vertex &v) {
		/**
		 * This code is based on Guibas and Stolfi (1985), with minor modifications
		 * and a bug fix from Dani Lischinski (Graphic Gems 1993). (The modification
		 * I believe is the test for the inserted site falling exactly on an
		 * existing edge. Without this test zero-width triangles have been observed
		 * to be created)
		 */
		QuadEdge *e = subdiv->locate(v);

		if(!e) {
			throw LocateFailureException("");
		}

		if (subdiv->isVertexOfEdge(*e, v)) {
			// point is already in subdivision.
			return *e; 
		} 
		else if (subdiv->isOnEdge(*e, v.getCoordinate())) {
			// the point lies exactly on an edge, so delete the edge 
			// (it will be replaced by a pair of edges which have the point as a vertex)
			e = &e->oPrev();
			subdiv->remove(e->oNext());
		}

		/**
		 * Connect the new point to the vertices of the containing triangle 
		 * (or quadrilateral, if the new point fell on an existing edge.)
		 */
		QuadEdge* base = &subdiv->makeEdge(e->orig(), v);

		QuadEdge::splice(*base, *e);
		QuadEdge *startEdge = base;
		do {
			base = &subdiv->connect(*e, base->sym());
			e = &base->oPrev();
		} while (&e->lNext() != startEdge);
		

		// Examine suspect edges to ensure that the Delaunay condition
		// is satisfied.
		do {
			QuadEdge& t = e->oPrev();
			if (t.dest().rightOf(*e) &&
					v.isInCircle(e->orig(), t.dest(), e->dest())) {
				QuadEdge::swap(*e);
				e = &e->oPrev();
			} else if (&e->oNext() == startEdge) {
				return *base; // no more suspect edges.
			} else {
				e = &e->oNext().lPrev();
			}
		} while (true);
	}

};

} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_INCREMENTALDELAUNAYTRIANGULATOR_H

