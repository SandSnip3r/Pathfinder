#ifndef ACUTE_H
#define ACUTE_H

#include "../triangle.h"

typedef struct acutepool_t {
    int size;
    // getWedgeIntersection (fixed size)
    TRIANGLE_MACRO_REAL *initialpoly;
    // getWedgeIntersection (dynamic size)
    TRIANGLE_MACRO_REAL *petalx;
    TRIANGLE_MACRO_REAL *petaly;
    TRIANGLE_MACRO_REAL *petalr;
    TRIANGLE_MACRO_REAL *wedges;
    // doSmoothing (fixed size [500])
    TRIANGLE_MACRO_REAL *points_p;
    TRIANGLE_MACRO_REAL *points_q;
    TRIANGLE_MACRO_REAL *points_r;
} acutepool;

void findNewSPLocation(mesh *m, behavior *b,
                      vertex torg, vertex tdest, vertex tapex,
                      vertex circumcenter, TRIANGLE_MACRO_REAL *xi, TRIANGLE_MACRO_REAL *eta, int offcenter, struct otri badotri);

void acutepool_init(int n, acutepool **mp);

void acutepool_resize(int n, acutepool *p);

void acutepool_deinit(acutepool *p);

#endif /* ACUTE_H */
