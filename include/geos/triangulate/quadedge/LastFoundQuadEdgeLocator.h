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
 * Last port: triangulate/quadedge/LastFoundQuadEdgeLocator.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_LASTFOUNDQUADEDGELOCATOR_H
#define GEOS_TRIANGULATE_QUADEDGE_LASTFOUNDQUADEDGELOCATOR_H

#include <geos/triangulate/quadedge/QuadEdgeLocator.h>

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

//fwd declarations
class QuadEdgeSubdivision;

class LastFoundQuadEdgeLocator : public QuadEdgeLocator {
private:
	QuadEdgeSubdivision* subdiv;
	QuadEdge*			lastEdge;

public:
	LastFoundQuadEdgeLocator(QuadEdgeSubdivision *subdiv) : subdiv(subdiv){
		init();
	}

private:
	virtual void init() {
		lastEdge = findEdge();
	}

	virtual QuadEdge* findEdge() {
		return NULL;
		//TODO: BLC XXX FIXME IMPLEMENT
		//Collection edges = subdiv.getEdges();
		//// assume there is an edge - otherwise will get an exception
		//return (QuadEdge) edges.iterator().next();
	}

public:
	/**
	 * Locates an edge e, such that either v is on e, or e is an edge of a triangle containing v.
	 * The search starts from the last located edge amd proceeds on the general direction of v.
	 */
	virtual QuadEdge* locate(const Vertex &v) {
		return NULL;
		//TODO: BLC XXX FIXME IMPLEMENT
		//if (! lastEdge.isLive()) {
			//init();
		//}

		//QuadEdge e = subdiv.locateFromEdge(v, lastEdge);
		//lastEdge = e;
		//return e;
	}
}; 

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //  GEOS_TRIANGULATE_QUADEDGE_LASTFOUNDQUADEDGELOCATOR_H

