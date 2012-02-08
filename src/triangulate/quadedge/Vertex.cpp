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
 * Last port: triangulate/Vertex.java rev. 1.12
 *
 **********************************************************************/

#include <geos/triangulate/quadedge/Vertex.h>
#include <geos/triangulate/quadedge/QuadEdge.h>


namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

bool Vertex::rightOf(const QuadEdge &e) const {
	return isCCW(e.dest(), e.orig());
}

bool Vertex::leftOf(const QuadEdge &e) const {
	return isCCW(e.orig(), e.dest());
}

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace geos
