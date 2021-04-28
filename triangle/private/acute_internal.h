#ifndef ACUTE_INTERNAL_H
#define ACUTE_INTERNAL_H

#include "../triangle.h"

void findNewSPLocationWithoutMaxAngle(mesh *m, behavior *b,
                      vertex torg, vertex tdest, vertex tapex,
                      vertex circumcenter, TRIANGLE_MACRO_REAL *xi, TRIANGLE_MACRO_REAL *eta, int offcenter, struct otri badotri);
void findNewSPLocationWithMaxAngle(mesh *m, behavior *b,
                      vertex torg, vertex tdest, vertex tapex,
                      vertex circumcenter, TRIANGLE_MACRO_REAL *xi, TRIANGLE_MACRO_REAL *eta, int offcenter, struct otri badotri);
int longestShortestEdge(TRIANGLE_MACRO_REAL aodist, TRIANGLE_MACRO_REAL dadist, TRIANGLE_MACRO_REAL dodist);
int doSmoothing(mesh *m, behavior *b, struct otri badotri,
		vertex torg, vertex tdest, vertex tapex, TRIANGLE_MACRO_REAL *newloc);
int getStarPoints(mesh *m, struct otri badotri,
			vertex p, vertex q, vertex r, int whichPoint, TRIANGLE_MACRO_REAL *points);
int getNeighborsVertex(mesh *m, struct otri badotri,
				TRIANGLE_MACRO_REAL first_x, TRIANGLE_MACRO_REAL first_y, TRIANGLE_MACRO_REAL second_x, TRIANGLE_MACRO_REAL second_y,
				TRIANGLE_MACRO_REAL *thirdpoint, struct otri *neighotri);
int getWedgeIntersectionWithoutMaxAngle(mesh *m, behavior *b, 
			                int numpoints, TRIANGLE_MACRO_REAL *points, TRIANGLE_MACRO_REAL *newloc);
int getWedgeIntersectionWithMaxAngle(mesh *m, behavior *b, 
			             int numpoints, TRIANGLE_MACRO_REAL *points, TRIANGLE_MACRO_REAL *newloc);
int polygonAngles(behavior *b,int numpoints, TRIANGLE_MACRO_REAL *points);
int testPolygonAngle(behavior *b, TRIANGLE_MACRO_REAL *x1, TRIANGLE_MACRO_REAL *y1, TRIANGLE_MACRO_REAL *x2, TRIANGLE_MACRO_REAL *y2, TRIANGLE_MACRO_REAL *x3, TRIANGLE_MACRO_REAL *y3 );
void lineLineIntersection(TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL x3, TRIANGLE_MACRO_REAL y3, TRIANGLE_MACRO_REAL x4, TRIANGLE_MACRO_REAL y4 , TRIANGLE_MACRO_REAL *p);
int halfPlaneIntersection(int numvertices, TRIANGLE_MACRO_REAL *convexPoly, TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2);
int splitConvexPolygon(int numvertices,TRIANGLE_MACRO_REAL *convexPoly, TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL *polys[]);
int linePointLocation(TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL x, TRIANGLE_MACRO_REAL y);
void lineLineSegmentIntersection(TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL x3, TRIANGLE_MACRO_REAL y3, TRIANGLE_MACRO_REAL x4, TRIANGLE_MACRO_REAL y4 , TRIANGLE_MACRO_REAL *p);
void findPolyCentroid(int numpoints, TRIANGLE_MACRO_REAL *points, TRIANGLE_MACRO_REAL *centroid);
void circleLineIntersection (TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL x3, TRIANGLE_MACRO_REAL y3, TRIANGLE_MACRO_REAL r , TRIANGLE_MACRO_REAL *p);
int chooseCorrectPoint (TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL x3, TRIANGLE_MACRO_REAL y3, int isObtuse );
void pointBetweenPoints(TRIANGLE_MACRO_REAL x1, TRIANGLE_MACRO_REAL y1, TRIANGLE_MACRO_REAL x2, TRIANGLE_MACRO_REAL y2, TRIANGLE_MACRO_REAL x, TRIANGLE_MACRO_REAL y, TRIANGLE_MACRO_REAL *p);
int testTriangleAngle(behavior *b, TRIANGLE_MACRO_REAL *x1, TRIANGLE_MACRO_REAL *y1, TRIANGLE_MACRO_REAL *x2, TRIANGLE_MACRO_REAL *y2, TRIANGLE_MACRO_REAL *x3, TRIANGLE_MACRO_REAL *y3 );
TRIANGLE_MACRO_REAL minDistanceToNeigbor(mesh *m, behavior *b, TRIANGLE_MACRO_REAL newlocX, TRIANGLE_MACRO_REAL newlocY, struct otri *searchtri);

#endif /* ACUTE_INTERNAL_H */
