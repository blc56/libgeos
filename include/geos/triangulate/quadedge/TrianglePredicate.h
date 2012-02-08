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
 * Last port: triangulate/quadedge/Vertex.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_TRIANGLEPREDICATE_H
#define GEOS_TRIANGULATE_QUADEDGE_TRIANGLEPREDICATE_H

#include <geos/geom/Coordinate.h>
#include <geos/geom/Triangle.h>
#include <geos/geom/CoordinateArraySequence.h>

namespace geos {
namespace geom { // geos.geom

/**
 * Algorithms for computing values and predicates
 * associated with triangles.
 * For some algorithms extended-precision
 * implementations are provided, which are more robust
 * (i.e. they produce correct answers in more cases).
 * Also, some more robust formulations of
 * some algorithms are provided, which utilize
 * normalization to the origin.
 * 
 * @author JTS: Martin Davis
 * @author Benjamin Campbell
 *
 */
class TrianglePredicate 
{
public:
  /**
   * Tests if a point is inside the circle defined by 
   * the triangle with vertices a, b, c (oriented counter-clockwise). 
   * This test uses simple
   * double-precision arithmetic, and thus may not be robust.
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   * @param P the point to test
   * @return true if this point is inside the circle defined by the points a, b, c
   */
  static bool isInCircleNonRobust(
	  const Coordinate &a, const Coordinate &b, const Coordinate &c, 
	  const Coordinate &p) {
	bool isInCircle = 
			  (a.x * a.x + a.y * a.y) * triArea(b, c, p)
			- (b.x * b.x + b.y * b.y) * triArea(a, c, p)
			+ (c.x * c.x + c.y * c.y) * triArea(a, b, p)
			- (p.x * p.x + p.y * p.y) * triArea(a, b, c) 
			> 0;
	return isInCircle;
  }
  
  /**
   * Tests if a point is inside the circle defined by 
   * the triangle with vertices a, b, c (oriented counter-clockwise). 
   * This test uses simple
   * double-precision arithmetic, and thus is not 10% robust.
   * However, by using normalization to the origin
   * it provides improved robustness and increased performance.
   * <p>
   * Based on code by J.R.Shewchuk.
   * 
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   * @param P the point to test
   * @return true if this point is inside the circle defined by the points a, b, c
   */
  static bool isInCircleNormalized(
	  const Coordinate &a, const Coordinate &b, const Coordinate &c, 
	  const Coordinate &p) {
	double adx = a.x - p.x;
	double ady = a.y - p.y;
	double bdx = b.x - p.x;
	double bdy = b.y - p.y;
	double cdx = c.x - p.x;
	double cdy = c.y - p.y;

	double abdet = adx * bdy - bdx * ady;
	double bcdet = bdx * cdy - cdx * bdy;
	double cadet = cdx * ady - adx * cdy;
	double alift = adx * adx + ady * ady;
	double blift = bdx * bdx + bdy * bdy;
	double clift = cdx * cdx + cdy * cdy;

	double disc = alift * bcdet + blift * cadet + clift * abdet;
	return disc > 0;
  }
private: 
  /**
   * Computes twice the area of the oriented triangle (a, b, c), i.e., the area is positive if the
   * triangle is oriented counterclockwise.
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   */
  static double triArea(const Coordinate &a,
		  const Coordinate &b, const Coordinate &c) {
	  return (b.x - a.x) * (c.y - a.y) 
		   - (b.y - a.y) * (c.x - a.x);
  }

public:
  /**
   * Tests if a point is inside the circle defined by 
   * the triangle with vertices a, b, c (oriented counter-clockwise). 
   * This method uses more robust computation.
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   * @param P the point to test
   * @return true if this point is inside the circle defined by the points a, b, c
   */
  static bool isInCircleRobust(
	  const Coordinate &a, const Coordinate &b, const Coordinate &c, 
	  const Coordinate &p) 
  {
	//checkRobustInCircle(a, b, c, p);
//	return isInCircleNonRobust(a, b, c, p);	   
	return isInCircleNormalized(a, b, c, p);	   
  }

  /**
   * Tests if a point is inside the circle defined by 
   * the triangle with vertices a, b, c (oriented counter-clockwise). 
   * The computation uses {@link DD} arithmetic for robustness.
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   * @param P the point to test
   * @return true if this point is inside the circle defined by the points a, b, c
   */
  //public static bool isInCircleDDSlow(
	  //Coordinate a, Coordinate b, Coordinate c,
	  //Coordinate p) {
	//DD px = DD.valueOf(p.x);
	//DD py = DD.valueOf(p.y);
	//DD ax = DD.valueOf(a.x);
	//DD ay = DD.valueOf(a.y);
	//DD bx = DD.valueOf(b.x);
	//DD by = DD.valueOf(b.y);
	//DD cx = DD.valueOf(c.x);
	//DD cy = DD.valueOf(c.y);

	//DD aTerm = (ax.multiply(ax).add(ay.multiply(ay)))
		//.multiply(triAreaDDSlow(bx, by, cx, cy, px, py));
	//DD bTerm = (bx.multiply(bx).add(by.multiply(by)))
		//.multiply(triAreaDDSlow(ax, ay, cx, cy, px, py));
	//DD cTerm = (cx.multiply(cx).add(cy.multiply(cy)))
		//.multiply(triAreaDDSlow(ax, ay, bx, by, px, py));
	//DD pTerm = (px.multiply(px).add(py.multiply(py)))
		//.multiply(triAreaDDSlow(ax, ay, bx, by, cx, cy));

	//DD sum = aTerm.subtract(bTerm).add(cTerm).subtract(pTerm);
	//bool isInCircle = sum.doubleValue() > 0;

	//return isInCircle;
  //}

  /**
   * Computes twice the area of the oriented triangle (a, b, c), i.e., the area
   * is positive if the triangle is oriented counterclockwise.
   * The computation uses {@link DD} arithmetic for robustness.
   * 
   * @param ax the x ordinate of a vertex of the triangle
   * @param ay the y ordinate of a vertex of the triangle
   * @param bx the x ordinate of a vertex of the triangle
   * @param by the y ordinate of a vertex of the triangle
   * @param cx the x ordinate of a vertex of the triangle
   * @param cy the y ordinate of a vertex of the triangle
   */
  //public static DD triAreaDDSlow(DD ax, DD ay,
	  //DD bx, DD by, DD cx, DD cy) {
	//return (bx.subtract(ax).multiply(cy.subtract(ay)).subtract(by.subtract(ay)
		//.multiply(cx.subtract(ax))));
  //}

  //public static bool isInCircleDDFast(
	  //Coordinate a, Coordinate b, Coordinate c,
	  //Coordinate p) {
	//DD aTerm = (DD.sqr(a.x).selfAdd(DD.sqr(a.y)))
		//.selfMultiply(triAreaDDFast(b, c, p));
	//DD bTerm = (DD.sqr(b.x).selfAdd(DD.sqr(b.y)))
		//.selfMultiply(triAreaDDFast(a, c, p));
	//DD cTerm = (DD.sqr(c.x).selfAdd(DD.sqr(c.y)))
		//.selfMultiply(triAreaDDFast(a, b, p));
	//DD pTerm = (DD.sqr(p.x).selfAdd(DD.sqr(p.y)))
		//.selfMultiply(triAreaDDFast(a, b, c));

	//DD sum = aTerm.selfSubtract(bTerm).selfAdd(cTerm).selfSubtract(pTerm);
	//bool isInCircle = sum.doubleValue() > 0;

	//return isInCircle;
  //}

  //public static DD triAreaDDFast(
	  //Coordinate a, Coordinate b, Coordinate c) {
	
	//DD t1 = DD.valueOf(b.x).selfSubtract(a.x)
		  //.selfMultiply(
			  //DD.valueOf(c.y).selfSubtract(a.y));
	
	//DD t2 = DD.valueOf(b.y).selfSubtract(a.y)
		  //.selfMultiply(
			  //DD.valueOf(c.x).selfSubtract(a.x));
	
	//return t1.selfSubtract(t2);
  //}

  //public static bool isInCircleDDNormalized(
	  //Coordinate a, Coordinate b, Coordinate c,
	  //Coordinate p) {
	//DD adx = DD.valueOf(a.x).selfSubtract(p.x);
	//DD ady = DD.valueOf(a.y).selfSubtract(p.y);
	//DD bdx = DD.valueOf(b.x).selfSubtract(p.x);
	//DD bdy = DD.valueOf(b.y).selfSubtract(p.y);
	//DD cdx = DD.valueOf(c.x).selfSubtract(p.x);
	//DD cdy = DD.valueOf(c.y).selfSubtract(p.y);

	//DD abdet = adx.multiply(bdy).selfSubtract(bdx.multiply(ady));
	//DD bcdet = bdx.multiply(cdy).selfSubtract(cdx.multiply(bdy));
	//DD cadet = cdx.multiply(ady).selfSubtract(adx.multiply(cdy));
	//DD alift = adx.multiply(adx).selfAdd(ady.multiply(ady));
	//DD blift = bdx.multiply(bdx).selfAdd(bdy.multiply(bdy));
	//DD clift = cdx.multiply(cdx).selfAdd(cdy.multiply(cdy));

	//DD sum = alift.selfMultiply(bcdet)
	//.selfAdd(blift.selfMultiply(cadet))
	//.selfAdd(clift.selfMultiply(abdet));
	
	//bool isInCircle = sum.doubleValue() > 0;

	//return isInCircle;
  //}

  /**
   * Computes the inCircle test using distance from the circumcentre. 
   * Uses standard double-precision arithmetic.
   * <p>
   * In general this doesn't
   * appear to be any more robust than the standard calculation. However, there
   * is at least one case where the test point is far enough from the
   * circumcircle that this test gives the correct answer. 
   * <pre>
   * LINESTRING
   * (1507029.9878 518325.7547, 1507022.1120341457 518332.8225183258,
   * 1507029.9833 518325.7458, 1507029.9896965567 518325.744909031)
   * </pre>
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   * @param p the point to test
   * @return true if this point is inside the circle defined by the points a, b, c
   */
  //static bool isInCircleCC(const Coordinate &a, const Coordinate &b,
		//const Coordinate &c, const Coordinate &p) {
	//Coordinate cc = Triangle.circumcentre(a, b, c);
	//double ccRadius = a.distance(cc);
	//double pRadiusDiff = p.distance(cc) - ccRadius;
	//return pRadiusDiff <= 0;
  //}
  
  /**
   * Checks if the computed value for isInCircle is correct, using
   * double-double precision arithmetic.
   * 
   * @param a a vertex of the triangle
   * @param b a vertex of the triangle
   * @param c a vertex of the triangle
   * @param p the point to test
   */
//private static void checkRobustInCircle(Coordinate a, Coordinate b, Coordinate c,
	//Coordinate p) 
//{
  //bool nonRobustInCircle = isInCircleNonRobust(a, b, c, p);
  //bool isInCircleDD = TrianglePredicate.isInCircleDDSlow(a, b, c, p);
  //bool isInCircleCC = TrianglePredicate.isInCircleCC(a, b, c, p);

  //Coordinate circumCentre = Triangle.circumcentre(a, b, c);
  //System.out.println("p radius diff a = "
	  //+ Math.abs(p.distance(circumCentre) - a.distance(circumCentre))
	  /// a.distance(circumCentre));

  //if (nonRobustInCircle != isInCircleDD || nonRobustInCircle != isInCircleCC) {
	//System.out.println("inCircle robustness failure (double result = "
		//+ nonRobustInCircle 
		//+ ", DD result = " + isInCircleDD
		//+ ", CC result = " + isInCircleCC + ")");
	//System.out.println(WKTWriter.toLineString(new CoordinateArraySequence(
		//new Coordinate[] { a, b, c, p })));
	//System.out.println("Circumcentre = " + WKTWriter.toPoint(circumCentre)
		//+ " radius = " + a.distance(circumCentre));
	//System.out.println("p radius diff a = "
		//+ Math.abs(p.distance(circumCentre)/a.distance(circumCentre) - 1));
	//System.out.println("p radius diff b = "
		//+ Math.abs(p.distance(circumCentre)/b.distance(circumCentre) - 1));
	//System.out.println("p radius diff c = "
		//+ Math.abs(p.distance(circumCentre)/c.distance(circumCentre) - 1));
	//System.out.println();
  //}
//}


} ;

} // namespace geos.geom
} // namespace geos

#endif //GEOS_TRIANGULATE_QUADEDGE_TRIANGLEPREDICATE_H

