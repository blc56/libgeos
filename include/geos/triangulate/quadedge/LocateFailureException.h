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
 * Last port: triangulate/quadedge/QuadEdge.java rev. 1.12
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_LOCATEFAILUREEXCEPTION_H
#define GEOS_TRIANGULATE_QUADEDGE_LOCATEFAILUREEXCEPTION_H

#include <stdexcept>
#include <string>

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

//TODO:BLC XXX Implement this
//class GEOS_DLL LocateFailureException : public std::exception 
//{
//private:
	//static String msgWithSpatial(String msg, LineSegment seg) {
		//if (seg != null)
			//return msg + " [ " + seg + " ]";
		//return msg;
	//}

	//private LineSegment seg = null;

	//public LocateFailureException(String msg) {
		//super(msg);
	//}

	//public LocateFailureException(String msg, LineSegment seg) {
		//super(msgWithSpatial(msg, seg));
		//this.seg = new LineSegment(seg);
	//}

	//public LocateFailureException(LineSegment seg) {
		//super(
				//"Locate failed to converge (at edge: "
						//+ seg
						//+ ").  Possible causes include invalid Subdivision topology or very close sites");
		//this.seg = new LineSegment(seg);
	//}

	//public LineSegment getSegment() {
		//return seg;
	//}

//};

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_LOCATEFAILUREEXCEPTION_H
