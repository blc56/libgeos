
// 
// Test Suite for geos::triangulate::quadedge::QuadEdge
//
// tut
#include <tut.hpp>
// geos
#include <geos/triangulate/quadedge/QuadEdgeSubdivision.h>
#include <geos/triangulate/IncrementalDelaunayTriangulator.h>
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
	//void runDelaunay(const char *sitesWkt, bool computTriangles, const char *expectedWkt)
	//{
		//WKTReader reader;
		//Geometry *sites = reader.read(sitesWkt);
		//QuadEdgeSubdivision sub(*sites->getEnvelopeInternal(), .00001);
		//IncrementalDelaunayTriangulator triangulator(&sub);
	//}

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
} // namespace tut

