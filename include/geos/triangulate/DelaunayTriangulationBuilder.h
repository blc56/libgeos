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
 * Last port: triangulate/DelaunayTriangulationBuilder.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_DELAUNAYTRIANGULATIONBUILDER_H
#define GEOS_TRIANGULATE_DELAUNAYTRIANGULATIONBUILDER_H

#include <geos/triangulate/quadedge/QuadEdgeSubdivision.h>
#include <geos/triangulate/IncrementalDelaunayTriangulator.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/GeometryFactory.h>

#include <algorithm>

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

/**
 * A utility class which creates Delaunay Trianglulations
 * from collections of points and extract the resulting 
 * triangulation edges or triangles as geometries. 
 * 
 * @author JTS: Martin Davis
 * @author Benjamin Campbell
 *
 */
class GEOS_DLL DelaunayTriangulationBuilder 
{
public:
	/**
	 * Extracts the unique {@link Coordinate}s from the given {@link Geometry}.
	 * @param geom the geometry to extract from
	 * @return a List of the unique Coordinates. Caller takes ownership of the returned object.
	 */
	static geom::CoordinateSequence* extractUniqueCoordinates(const Geometry& geom)
	{
		geom::CoordinateSequence *coords = geom.getCoordinates();
		unique(*coords);
		return coords;
	}
	
	static void unique(CoordinateSequence& coords)
	{
		std::vector<geos::geom::Coordinate> coordVector;
		coords.toVector(coordVector);
		std::sort(coordVector.begin(), coordVector.end(), geos::geom::CoordinateLessThen());
		coords.setPoints(coordVector);
		coords.removeRepeatedPoints();
	}
	
	/**
	 * Converts all {@link Coordinate}s in a collection to {@link Vertex}es.
	 * @param coords the coordinates to convert
	 * @return a List of Vertex objects. Call takes ownership of returned object.
	 */
	static IncrementalDelaunayTriangulator::VertexList* toVertices(const CoordinateSequence &coords)
	{
		IncrementalDelaunayTriangulator::VertexList* vertexList =
			new IncrementalDelaunayTriangulator::VertexList();

		for(size_t iter=0; iter < coords.size(); ++iter)
		{
			vertexList->push_back(Vertex(coords.getAt(iter)));
		}
		return vertexList;
	}
	
private:
	geom::CoordinateSequence* siteCoords;
	double tolerance;
	QuadEdgeSubdivision *subdiv;
	
public:
	/**
	 * Creates a new triangulation builder.
	 *
	 */
	DelaunayTriangulationBuilder() : siteCoords(NULL), tolerance(0.0), subdiv(NULL)
	{
	}

	~DelaunayTriangulationBuilder() 
	{
		if(siteCoords)
			delete siteCoords;
		if(subdiv)
			delete subdiv;
	}
	
	/**
	 * Sets the sites (vertices) which will be triangulated.
	 * All vertices of the given geometry will be used as sites.
	 * 
	 * @param geom the geometry from which the sites will be extracted.
	 */
	void setSites(const Geometry& geom)
	{
		if(siteCoords)
			delete siteCoords;
		// remove any duplicate points (they will cause the triangulation to fail)
		siteCoords = extractUniqueCoordinates(geom);
	}
	
	/**
	 * Sets the sites (vertices) which will be triangulated
	 * from a collection of {@link Coordinate}s.
	 * 
	 * @param geom a CoordinateSequence.
	 */
	void setSites(const CoordinateSequence& coords)
	{
		if(siteCoords)
			delete siteCoords;
		siteCoords = coords.clone();
		// remove any duplicate points (they will cause the triangulation to fail)
		unique(*siteCoords);
	}
	
	/**
	 * Sets the snapping tolerance which will be used
	 * to improved the robustness of the triangulation computation.
	 * A tolerance of 0.0 specifies that no snapping will take place.
	 * 
	 * @param tolerance the tolerance distance to use
	 */
	void setTolerance(double tolerance)
	{
		this->tolerance = tolerance;
	}
	
private:
	void create()
	{
		if(subdiv != NULL || siteCoords == NULL)
			return;
		
		Envelope siteEnv;
		siteCoords ->expandEnvelope(siteEnv);
		IncrementalDelaunayTriangulator::VertexList* vertices = toVertices(*siteCoords);
		subdiv = new QuadEdgeSubdivision(siteEnv, tolerance);
		IncrementalDelaunayTriangulator triangulator = IncrementalDelaunayTriangulator(subdiv);
		triangulator.insertSites(*vertices);
		delete vertices;
	}
	
public:
	/**
	 * Gets the {@link QuadEdgeSubdivision} which models the computed triangulation.
	 * 
	 * @return the subdivision containing the triangulation
	 */
	QuadEdgeSubdivision& getSubdivision()
	{
		create();
		return *subdiv;
	}
	
	/**
	 * Gets the edges of the computed triangulation as a {@link MultiLineString}.
	 * 
	 * @param geomFact the geometry factory to use to create the output
	 * @return the edges of the triangulation. The caller takes ownership of the returned object.
	 */
	std::auto_ptr<geom::MultiLineString> getEdges(GeometryFactory geomFact)
	{
		create();
		return subdiv->getEdges(geomFact);
	}
	
	/**
	 * Gets the faces of the computed triangulation as a {@link GeometryCollection} 
	 * of {@link Polygon}.
	 * 
	 * @param geomFact the geometry factory to use to create the output
	 * @return the faces of the triangulation. The caller takes ownership of the returned object.
	 */
	std::auto_ptr<geom::GeometryCollection> getTriangles(geom::GeometryFactory& geomFact)
	{
		create();
		return subdiv->getTriangles(geomFact);
	}
};

}//namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_DELAUNAYTRIANGULATIONBUILDER_H

