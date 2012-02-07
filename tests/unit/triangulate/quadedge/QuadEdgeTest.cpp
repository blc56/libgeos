// 
// Test Suite for geos::triangulate::quadedge::QuadEdge
//
// tut
#include <tut.hpp>
// geos
#include <geos/triangulate/quadedge/Vertex.h>
#include <geos/triangulate/quadedge/QuadEdge.h>
// std
#include <stdio.h>

using namespace geos::triangulate::quadedge;

namespace tut
{
	//
	// Test Group
	//

	// dummy data, not used
	struct test_quadedge_data
	{
		test_quadedge_data()
		{
		}
	};

	typedef test_group<test_quadedge_data> group;
	typedef group::object object;

	group test_quadedge_group("geos::triangulate::quadedge::QuadEdge");


	//
	// Test Cases
	//

	// 1 - QuadEdge::connect()
	template<>
	template<>
	void object::test<1>()
	{		 
		Vertex v1(0, 0);
		Vertex v2(0, 1);

		Vertex v3(1, 0);
		Vertex v4(1, 1);

		QuadEdge *q0, *q1, *q2, *q3;
		QuadEdge *r0, *r1, *r2, *r3;
		QuadEdge *s0, *s1, *s2, *s3;

		QuadEdge::makeEdge(v1, v2, &q0, &q1, &q2, &q3);
		QuadEdge::makeEdge(v3, v4, &r0, &r1, &r2, &r3);

		QuadEdge::connect(*q0, *r0, &s0, &s1, &s2, &s3);

		//verify properties ensured by connect()
		//the new edge connects q0->orig() and r0->dest()
		ensure(s0->orig().equals(q0->dest()));
		ensure(s0->dest().equals(r0->orig()));
		//q0, r0, and s0 should have the same left face
		ensure(q0->lNext() == s0);
		ensure(s0->lNext() == r0);

		delete q0; delete q1; delete q2; delete q3;
		delete r0; delete r1; delete r2; delete r3;
		delete s0; delete s1; delete s2; delete s3;
	}

	// 2 - QuadEdge::connect(), causing a loop
	template<>
	template<>
	void object::test<2>()
	{		 
		Vertex v1(0, 0);
		Vertex v2(0, 1);

		Vertex v3(1, 0);
		Vertex v4(1, 1);

		QuadEdge *q0, *q1, *q2, *q3;
		QuadEdge *r0, *r1, *r2, *r3;
		QuadEdge *s0, *s1, *s2, *s3;

		QuadEdge::makeEdge(v1, v2, &q0, &q1, &q2, &q3);
		QuadEdge::makeEdge(v2, v3, &r0, &r1, &r2, &r3);

		QuadEdge::connect(*q0, *r0, &s0, &s1, &s2, &s3);

		//verify properties ensured by connect()
		//the new edge connects q0->orig() and r0->dest()
		ensure(s0->orig().equals(q0->dest()));
		ensure(s0->dest().equals(r0->orig()));
		//q0, r0, and s0 should have the same left face
		ensure(q0->lNext() == s0);
		ensure(s0->lNext() == r0);

		delete q0; delete q1; delete q2; delete q3;
		delete r0; delete r1; delete r2; delete r3;
		delete s0; delete s1; delete s2; delete s3;
	}
} // namespace tut


