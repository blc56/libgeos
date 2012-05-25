
// 
// Test Suite for geos::triangulate::quadedge::QuadEdge
//
// tut
#include <tut.hpp>
// geos
#include <geos/triangulate/quadedge/QuadEdgeSubdivision.h>
#include <geos/triangulate/IncrementalDelaunayTriangulator.h>
#include <geos/triangulate/DelaunayTriangulationBuilder.h>
#include <geos/io/WKTWriter.h>
#include <geos/io/WKTReader.h>
#include <geos/geom/GeometryCollection.h>
#include <geos/geom/GeometryFactory.h>

#include <stdio.h>

using namespace geos::triangulate;
using namespace geos::triangulate::quadedge;
using namespace geos::geom;
using namespace geos::io;

namespace tut
{
	//
	// Test Group
	//

	// dummy data, not used
	struct test_incdelaunaytri_data
	{
		test_incdelaunaytri_data()
		{
		}
	};

	typedef test_group<test_incdelaunaytri_data> group;
	typedef group::object object;

	group test_incdelaunaytri_group("geos::triangulate::IncrementalDelaunayTriangulator");

	//helper function for funning triangulation
	void runDelaunay(const char *sitesWkt, bool computeTriangles, const char *expectedWkt)
	{
		WKTReader reader;
		Geometry *results;
		Geometry *sites = reader.read(sitesWkt);
		Geometry *expected = reader.read(expectedWkt);
		DelaunayTriangulationBuilder builder;
		GeometryFactory geomFact;

		builder.setSites(*sites);
		if(computeTriangles)
			results=builder.getTriangles(geomFact);
		//else
			//results=builder.getEdges(geomFact);
			
		ensure(results->equalsExact(expected));

		delete sites;
		delete expected;
		delete results;
	}

	//
	// Test Cases
	//

	// 1 - Basic function test
	template<>
	template<>
	void object::test<1>()
	{
		//Create a subdivision centered at (0,0)
		QuadEdgeSubdivision sub(Envelope(-100, 100, -100, 100), .00001);
		//make a triagulaor to work on sub
		IncrementalDelaunayTriangulator triangulator(&sub);

		triangulator.insertSite(Vertex(0, 0));

		//extract the triangles from the subdivision
		GeometryFactory geomFact;
		GeometryCollection *tris = sub.getTriangles(geomFact);
		WKTWriter wkt;
		printf("%s\n", wkt.writeFormatted(tris).c_str());
		delete tris;
	}

	// 2 - Test grid
	template<>
	template<>
	void object::test<2>()
	{
		char * wkt = "MULTIPOINT ((10 10), (10 20), (20 20), (20 10), (20 0), (10 0), (0 0), (0 10), (0 20))";
		char * expectedTri = "GEOMETRYCOLLECTION (POLYGON ((0 20, 0 10, 10 10, 0 20)), POLYGON ((0 20, 10 10, 10 20, 0 20)), POLYGON ((10 20, 10 10, 20 10, 10 20)), POLYGON ((10 20, 20 10, 20 20, 10 20)), POLYGON ((10 0, 20 0, 10 10, 10 0)), POLYGON ((10 0, 10 10, 0 10, 10 0)), POLYGON ((10 0, 0 10, 0 0, 10 0)), POLYGON ((10 10, 20 0, 20 10, 10 10)))";

		runDelaunay(wkt, true, expectedTri);
	}
} // namespace tut

