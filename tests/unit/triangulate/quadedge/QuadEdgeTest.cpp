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

	// 3 - QuadEdge::swap()
	template<>
	template<>
	void object::test<3>()
	{		 
		Vertex v1(0, 0);
		Vertex v2(0, 1);

		Vertex v3(1, 0);
		Vertex v4(1, 1);

		QuadEdge *q0, *q1, *q2, *q3;
		QuadEdge *r0, *r1, *r2, *r3;
		QuadEdge *s0, *s1, *s2, *s3;
		QuadEdge *t0, *t1, *t2, *t3;
		QuadEdge *u0, *u1, *u2, *u3;

		//make a quadilateral
		QuadEdge::makeEdge(v1, v2, &q0, &q1, &q2, &q3);
		QuadEdge::makeEdge(v4, v3, &r0, &r1, &r2, &r3);
		QuadEdge::connect(*q0, *r0, &s0, &s1, &s2, &s3);
		QuadEdge::connect(*r0, *q0, &t0, &t1, &t2, &t3);

		//printf("\n=====================\n");
		//printf("r0->orig(): %f %f\n", r0->orig().getX(), r0->orig().getY());
		//printf("r0->dest(): %f %f\n", r0->dest().getX(), r0->dest().getY());
		//printf("s0->orig(): %f %f\n", s0->orig().getX(), s0->orig().getY());
		//printf("s0->dest(): %f %f\n", s0->dest().getX(), s0->dest().getY());

		//add an interior edge to make 2 triangles
		QuadEdge::connect(*t0, *r0, &u0, &u1, &u2, &u3);
		//printf("\n=====================\n");
		//printf("q0->orig(): %f %f\n", q0->orig().getX(), q0->orig().getY());
		//printf("q0->dest(): %f %f\n", q0->dest().getX(), q0->dest().getY());
		//printf("r0->orig(): %f %f\n", r0->orig().getX(), r0->orig().getY());
		//printf("r0->dest(): %f %f\n", r0->dest().getX(), r0->dest().getY());
		//printf("s0->orig(): %f %f\n", s0->orig().getX(), s0->orig().getY());
		//printf("s0->dest(): %f %f\n", s0->dest().getX(), s0->dest().getY());
		//printf("t0->orig(): %f %f\n", t0->orig().getX(), t0->orig().getY());
		//printf("t0->dest(): %f %f\n", t0->dest().getX(), t0->dest().getY());
		//printf("u0->orig(): %f %f\n", u0->orig().getX(), u0->orig().getY());
		//printf("u0->dest(): %f %f\n", u0->dest().getX(), u0->dest().getY());
		ensure(t0->dest().equals(u0->orig()));
		ensure(u0->dest().equals(r0->orig()));

		//now swap the interior edge
		QuadEdge::swap(*u0);
		//printf("\n=====================\n");
		//printf("q0->orig(): %f %f\n", q0->orig().getX(), q0->orig().getY());
		//printf("q0->dest(): %f %f\n", q0->dest().getX(), q0->dest().getY());
		//printf("r0->orig(): %f %f\n", r0->orig().getX(), r0->orig().getY());
		//printf("r0->dest(): %f %f\n", r0->dest().getX(), r0->dest().getY());
		//printf("s0->orig(): %f %f\n", s0->orig().getX(), s0->orig().getY());
		//printf("s0->dest(): %f %f\n", s0->dest().getX(), s0->dest().getY());
		//printf("t0->orig(): %f %f\n", t0->orig().getX(), t0->orig().getY());
		//printf("t0->dest(): %f %f\n", t0->dest().getX(), t0->dest().getY());
		//printf("u0->orig(): %f %f\n", u0->orig().getX(), u0->orig().getY());
		//printf("u0->dest(): %f %f\n", u0->dest().getX(), u0->dest().getY());
		ensure(r0->dest().equals(u0->dest()));
		ensure(u0->orig().equals(q0->dest()));

		delete q0; delete q1; delete q2; delete q3;
		delete r0; delete r1; delete r2; delete r3;
		delete s0; delete s1; delete s2; delete s3;
		delete t0; delete t1; delete t2; delete t3;
		delete u0; delete u1; delete u2; delete u3;
	}
} // namespace tut


