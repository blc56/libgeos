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
 * Last port: triangulate/quadedge/QuadEdgeLocator.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_QUADEDGELOCATOR_H
#define GEOS_TRIANGULATE_QUADEDGE_QUADEDGELOCATOR_H

#include <geos/triangulate/quadedge/Vertex.h>
#include <geos/triangulate/quadedge/QuadEdge.h>

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

/**
 * An interface for classes which locate an edge in a {@link QuadEdgeSubdivision}
 * which either contains a given {@link Vertex} V 
 * or is an edge of a triangle which contains V. 
 * Implementors may utilized different strategies for
 * optimizing locating containing edges/triangles.
 * 
 * @author JTS: Martin Davis
 * @author Ben Campbell
 */
class QuadEdgeLocator {
public:
	virtual ~QuadEdgeLocator() = 0; //not implemented
	virtual QuadEdge* locate(const Vertex &v) = 0; //not implemented
};

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_QUADEDGELOCATOR_H
