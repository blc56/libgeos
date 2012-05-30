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

#include <geos/geom/Polygon.h>
#include <geos/triangulate/quadedge/QuadEdgeSubdivision.h>

using namespace geos::geom;

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

GeometryCollection* QuadEdgeSubdivision::getTriangles(
		const GeometryFactory &geomFact) {
	TriList triPtsList;
	getTriangleCoordinates(&triPtsList, false);
	std::vector<Geometry*> tris;

	for(TriList::const_iterator it = triPtsList.begin();
			it != triPtsList.end(); ++it)
	{
		CoordinateSequence *coordSeq = *it;
		Polygon *tri = geomFact.createPolygon(
				geomFact.createLinearRing(coordSeq), NULL);
		tris.push_back(static_cast<Geometry*>(tri));
	}
	GeometryCollection* ret =  geomFact.createGeometryCollection(tris);

	//release memory
	for(std::vector<Geometry*>::iterator it=tris.begin(); it!=tris.end(); ++it)
		delete *it;
	tris.clear();

	return ret;
}

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes
