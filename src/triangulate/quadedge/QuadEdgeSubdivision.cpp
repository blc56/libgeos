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

#include <geos/geom/CoordinateArraySequenceFactory.h>
#include <geos/geom/Polygon.h>
#include <geos/triangulate/quadedge/QuadEdgeSubdivision.h>

using namespace geos::geom;

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

GeometryCollection* QuadEdgeSubdivision::getTriangles(
		const GeometryFactory &geomFact) {
	TriList triPtsList = getTriangleCoordinates(false);
	std::vector<Geometry*> tris;
	CoordinateArraySequenceFactory coordSeqFact;

	for(TriList::const_iterator it = triPtsList.begin();
			it != triPtsList.end(); ++it)
	{
		Coordinate::Vect* tript = *it;
		CoordinateSequence *coordSeq =
			coordSeqFact.create(tript);
		Polygon *tri = geomFact.createPolygon(
				geomFact.createLinearRing(coordSeq), NULL);
		tris.push_back(dynamic_cast<Geometry*>(tri));

		//tris[i++] = geomfact
				//.createpolygon(geomfact.createlinearring(tript), null);
	}
	GeometryCollection* ret =  geomFact.createGeometryCollection(tris);
	//TODO: BLC: XXX don't leak memory!
	tris.clear();//calls destructors for polygon pointers
	return ret;
}

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes
