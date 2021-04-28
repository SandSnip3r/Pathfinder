#ifndef PREDICATES_H
#define PREDICATES_H

#include "../triangle.h"

void exactinit();

TRIANGLE_MACRO_REAL counterclockwise(mesh *m, behavior *b,
                      vertex pa, vertex pb, vertex pc);

TRIANGLE_MACRO_REAL incircle(mesh *m, behavior *b,
              vertex pa, vertex pb, vertex pc, vertex pd);

TRIANGLE_MACRO_REAL nonregular(mesh *m, behavior *b,
                vertex pa, vertex pb, vertex pc, vertex pd);

void findcircumcenter(mesh *m, behavior *b,
                      vertex torg, vertex tdest, vertex tapex,
                      vertex circumcenter, TRIANGLE_MACRO_REAL *xi, TRIANGLE_MACRO_REAL *eta, int offcenter);

/********* Private methods *********/

int fast_expansion_sum_zeroelim(int elen, TRIANGLE_MACRO_REAL *e, int flen, TRIANGLE_MACRO_REAL *f, TRIANGLE_MACRO_REAL *h);

int scale_expansion_zeroelim(int elen, TRIANGLE_MACRO_REAL *e, TRIANGLE_MACRO_REAL b, TRIANGLE_MACRO_REAL *h);

TRIANGLE_MACRO_REAL estimate(int elen, TRIANGLE_MACRO_REAL *e);

TRIANGLE_MACRO_REAL counterclockwiseadapt(vertex pa, vertex pb, vertex pc, TRIANGLE_MACRO_REAL detsum);

TRIANGLE_MACRO_REAL incircleadapt(vertex pa, vertex pb, vertex pc, vertex pd, TRIANGLE_MACRO_REAL permanent);

TRIANGLE_MACRO_REAL orient3dadapt(vertex pa, vertex pb, vertex pc, vertex pd,
                   TRIANGLE_MACRO_REAL aheight, TRIANGLE_MACRO_REAL bheight, TRIANGLE_MACRO_REAL cheight, TRIANGLE_MACRO_REAL dheight,
                   TRIANGLE_MACRO_REAL permanent);

TRIANGLE_MACRO_REAL orient3d(mesh *m, behavior *b,
              vertex pa, vertex pb, vertex pc, vertex pd,
              TRIANGLE_MACRO_REAL aheight, TRIANGLE_MACRO_REAL bheight, TRIANGLE_MACRO_REAL cheight, TRIANGLE_MACRO_REAL dheight);


#endif /* PREDICATES_H */
