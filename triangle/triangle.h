/*****************************************************************************/
/*                                                                           */
/*  (triangle.h)                                                             */
/*                                                                           */
/*  Include file for programs that call Triangle.                            */
/*                                                                           */
/*  Accompanies Triangle Version 1.6                                         */
/*  July 28, 2005                                                            */
/*                                                                           */
/*  Copyright 1996, 2005                                                     */
/*  Jonathan Richard Shewchuk                                                */
/*  2360 Woolsey #H                                                          */
/*  Berkeley, California  94705-1927                                         */
/*  jrs@cs.berkeley.edu                                                      */
/*                                                                           */
/*****************************************************************************/
#ifndef TRIANGLE_H
#define TRIANGLE_H

/* #define NO_ACUTE */

#include "triangle_config.h"

#ifdef __cplusplus
namespace triangle {
#endif

/*****************************************************************************/
/*                                                                           */
/*  The `triangleio' structure.                                              */
/*                                                                           */
/*  Used to pass data into and out of Triangle.                              */
/*                                                                           */
/*****************************************************************************/

typedef struct triangleio_t {
	/* An array of point coordinates. Each point occupies two REALs. */
	TRIANGLE_MACRO_REAL *pointlist;

	/* An array of point attributes. Each point's attributes occupy */
	/* 'numberofpointattributes' REALs. */
	TRIANGLE_MACRO_REAL *pointattributelist;

	/* An array of point markers; one int per point. */
	int *pointmarkerlist;

	int numberofpoints;
	int numberofpointattributes;
	
	/* An array of triangle corners.  Each triangle occupies */
	/* 'numberofcorners' ints. */
	int *trianglelist;

	/* An array of triangle attributes. Each triangle's attributes occupy */
	/* 'numberoftriangleattributes' REALs. */
	TRIANGLE_MACRO_REAL *triangleattributelist;

	/* An array of triangle area constraints; one TRIANGLE_MACRO_REAL per triangle. Input only. */
	TRIANGLE_MACRO_REAL *trianglearealist;

	/* An array of triangle neighbors; three ints per triangle. Output only. */
	int *neighborlist;

	int numberoftriangles;
	int numberofcorners;
	int numberoftriangleattributes;

	/* An array of segment endpoints.Two ints per segment. */
	int *segmentlist;

	/* An array of segment markers. One int per segment. */
	int *segmentmarkerlist;

	int numberofsegments;

	/* An array of holes. Two REALs per hole. Input only. */
	TRIANGLE_MACRO_REAL *holelist;

	int numberofholes;

	/* An array of regional attributes and area constraints.  */
	/* Four REALs per area constraint. Input only. */
	TRIANGLE_MACRO_REAL *regionlist;

	int numberofregions;

	/* An array of edge endpoints. Two ints per edge. Output only. */
	int *edgelist;

	/* An array of edge markers; one int per edge. Output only. */
	int *edgemarkerlist;

  /*  An array of normal vectors, used for infinite rays in                    */
  /*    Voronoi diagrams.  The first normal vector's x and y magnitudes are    */
  /*    at indices [0] and [1], followed by the remaining vectors.  For each   */
  /*    finite edge in a Voronoi diagram, the normal vector written is the     */
  /*    zero vector.  Two REALs per edge.  Output only.                        */
  TRIANGLE_MACRO_REAL *normlist;

	int numberofedges;
} triangleio;


/* Labels that signify the result of point location.  The result of a        */
/*   search indicates that the point falls in the interior of a triangle, on */
/*   an edge, on a vertex, or outside the mesh.                              */

enum locateresult {INTRIANGLE, ONEDGE, ONVERTEX, OUTSIDE};

/* Labels that signify the result of vertex insertion.  The result indicates */
/*   that the vertex was inserted with complete success, was inserted but    */
/*   encroaches upon a subsegment, was not inserted because it lies on a     */
/*   segment, or was not inserted because another vertex occupies the same   */
/*   location.                                                               */

enum insertvertexresult {SUCCESSFULVERTEX, ENCROACHINGVERTEX, VIOLATINGVERTEX,
                         DUPLICATEVERTEX};

/* Labels that signify the result of direction finding.  The result          */
/*   indicates that a segment connecting the two query points falls within   */
/*   the direction triangle, along the left edge of the direction triangle,  */
/*   or along the right edge of the direction triangle.                      */

enum finddirectionresult {WITHIN, LEFTCOLLINEAR, RIGHTCOLLINEAR};

/*****************************************************************************/
/*                                                                           */
/*  The basic mesh data structures                                           */
/*                                                                           */
/*  There are three:  vertices, triangles, and subsegments (abbreviated      */
/*  `subseg').  These three data structures, linked by pointers, comprise    */
/*  the mesh.  A vertex simply represents a mesh vertex and its properties.  */
/*  A triangle is a triangle.  A subsegment is a special data structure used */
/*  to represent an impenetrable edge of the mesh (perhaps on the outer      */
/*  boundary, on the boundary of a hole, or part of an internal boundary     */
/*  separating two triangulated regions).  Subsegments represent boundaries, */
/*  defined by the user, that triangles may not lie across.                  */
/*                                                                           */
/*  A triangle consists of a list of three vertices, a list of three         */
/*  adjoining triangles, a list of three adjoining subsegments (when         */
/*  segments exist), an arbitrary number of optional user-defined            */
/*  floating-point attributes, and an optional area constraint.  The latter  */
/*  is an upper bound on the permissible area of each triangle in a region,  */
/*  used for mesh refinement.                                                */
/*                                                                           */
/*  For a triangle on a boundary of the mesh, some or all of the neighboring */
/*  triangles may not be present.  For a triangle in the interior of the     */
/*  mesh, often no neighboring subsegments are present.  Such absent         */
/*  triangles and subsegments are never represented by NULL pointers; they   */
/*  are represented by two special records:  `dummytri', the triangle that   */
/*  fills "outer space", and `dummysub', the omnipresent subsegment.         */
/*  `dummytri' and `dummysub' are used for several reasons; for instance,    */
/*  they can be dereferenced and their contents examined without violating   */
/*  protected memory.                                                        */
/*                                                                           */
/*  However, it is important to understand that a triangle includes other    */
/*  information as well.  The pointers to adjoining vertices, triangles, and */
/*  subsegments are ordered in a way that indicates their geometric relation */
/*  to each other.  Furthermore, each of these pointers contains orientation */
/*  information.  Each pointer to an adjoining triangle indicates which face */
/*  of that triangle is contacted.  Similarly, each pointer to an adjoining  */
/*  subsegment indicates which side of that subsegment is contacted, and how */
/*  the subsegment is oriented relative to the triangle.                     */
/*                                                                           */
/*  The data structure representing a subsegment may be thought to be        */
/*  abutting the edge of one or two triangle data structures:  either        */
/*  sandwiched between two triangles, or resting against one triangle on an  */
/*  exterior boundary or hole boundary.                                      */
/*                                                                           */
/*  A subsegment consists of a list of four vertices--the vertices of the    */
/*  subsegment, and the vertices of the segment it is a part of--a list of   */
/*  two adjoining subsegments, and a list of two adjoining triangles.  One   */
/*  of the two adjoining triangles may not be present (though there should   */
/*  always be one), and neighboring subsegments might not be present.        */
/*  Subsegments also store a user-defined integer "boundary marker".         */
/*  Typically, this integer is used to indicate what boundary conditions are */
/*  to be applied at that location in a finite element simulation.           */
/*                                                                           */
/*  Like triangles, subsegments maintain information about the relative      */
/*  orientation of neighboring objects.                                      */
/*                                                                           */
/*  Vertices are relatively simple.  A vertex is a list of floating-point    */
/*  numbers, starting with the x, and y coordinates, followed by an          */
/*  arbitrary number of optional user-defined floating-point attributes,     */
/*  followed by an integer boundary marker.  During the segment insertion    */
/*  phase, there is also a pointer from each vertex to a triangle that may   */
/*  contain it.  Each pointer is not always correct, but when one is, it     */
/*  speeds up segment insertion.  These pointers are assigned values once    */
/*  at the beginning of the segment insertion phase, and are not used or     */
/*  updated except during this phase.  Edge flipping during segment          */
/*  insertion will render some of them incorrect.  Hence, don't rely upon    */
/*  them for anything.                                                       */
/*                                                                           */
/*  Other than the exception mentioned above, vertices have no information   */
/*  about what triangles, subfacets, or subsegments they are linked to.      */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Handles                                                                  */
/*                                                                           */
/*  The oriented triangle (`otri') and oriented subsegment (`osub') data     */
/*  structures defined below do not themselves store any part of the mesh.   */
/*  The mesh itself is made of `triangle's, `subseg's, and `vertex's.        */
/*                                                                           */
/*  Oriented triangles and oriented subsegments will usually be referred to  */
/*  as "handles."  A handle is essentially a pointer into the mesh; it       */
/*  allows you to "hold" one particular part of the mesh.  Handles are used  */
/*  to specify the regions in which one is traversing and modifying the mesh.*/
/*  A single `triangle' may be held by many handles, or none at all.  (The   */
/*  latter case is not a memory leak, because the triangle is still          */
/*  connected to other triangles in the mesh.)                               */
/*                                                                           */
/*  An `otri' is a handle that holds a triangle.  It holds a specific edge   */
/*  of the triangle.  An `osub' is a handle that holds a subsegment.  It     */
/*  holds either the left or right side of the subsegment.                   */
/*                                                                           */
/*  Navigation about the mesh is accomplished through a set of mesh          */
/*  manipulation primitives, further below.  Many of these primitives take   */
/*  a handle and produce a new handle that holds the mesh near the first     */
/*  handle.  Other primitives take two handles and glue the corresponding    */
/*  parts of the mesh together.  The orientation of the handles is           */
/*  important.  For instance, when two triangles are glued together by the   */
/*  bond() primitive, they are glued at the edges on which the handles lie.  */
/*                                                                           */
/*  Because vertices have no information about which triangles they are      */
/*  attached to, I commonly represent a vertex by use of a handle whose      */
/*  origin is the vertex.  A single handle can simultaneously represent a    */
/*  triangle, an edge, and a vertex.                                         */
/*                                                                           */
/*****************************************************************************/

/* The triangle data structure.  Each triangle contains three pointers to    */
/*   adjoining triangles, plus three pointers to vertices, plus three        */
/*   pointers to subsegments (declared below; these pointers are usually     */
/*   `dummysub').  It may or may not also contain user-defined attributes    */
/*   and/or a floating-point "area constraint."  It may also contain extra   */
/*   pointers for nodes, when the user asks for high-order elements.         */
/*   Because the size and structure of a `triangle' is not decided until     */
/*   runtime, I haven't simply declared the type `triangle' as a struct.     */

typedef TRIANGLE_MACRO_REAL **triangle;            /* Really:  typedef triangle *triangle   */

/* An oriented triangle:  includes a pointer to a triangle and orientation.  */
/*   The orientation denotes an edge of the triangle.  Hence, there are      */
/*   three possible orientations.  By convention, each edge always points    */
/*   counterclockwise about the corresponding triangle.                      */

struct otri {
  triangle *tri;
  int orient;                                         /* Ranges from 0 to 2. */
};

/* The subsegment data structure.  Each subsegment contains two pointers to  */
/*   adjoining subsegments, plus four pointers to vertices, plus two         */
/*   pointers to adjoining triangles, plus one boundary marker, plus one     */
/*   segment number.                                                         */

typedef TRIANGLE_MACRO_REAL **subseg;                  /* Really:  typedef subseg *subseg   */

/* An oriented subsegment:  includes a pointer to a subsegment and an        */
/*   orientation.  The orientation denotes a side of the edge.  Hence, there */
/*   are two possible orientations.  By convention, the edge is always       */
/*   directed so that the "side" denoted is the right side of the edge.      */

struct osub {
  subseg *ss;
  int ssorient;                                       /* Ranges from 0 to 1. */
};

/* The vertex data structure.  Each vertex is actually an array of REALs.    */
/*   The number of REALs is unknown until runtime.  An integer boundary      */
/*   marker, and sometimes a pointer to a triangle, is appended after the    */
/*   REALs.                                                                  */

typedef TRIANGLE_MACRO_REAL *vertex;

/* A queue used to store encroached subsegments.  Each subsegment's vertices */
/*   are stored so that we can check whether a subsegment is still the same. */

struct badsubseg {
  subseg encsubseg;                             /* An encroached subsegment. */
  vertex subsegorg, subsegdest;                         /* Its two vertices. */
};

/* A queue used to store bad triangles.  The key is the square of the cosine */
/*   of the smallest angle of the triangle.  Each triangle's vertices are    */
/*   stored so that one can check whether a triangle is still the same.      */

struct badtriang {
  triangle poortri;                       /* A skinny or too-large triangle. */
  TRIANGLE_MACRO_REAL key;                             /* cos^2 of smallest (apical) angle. */
  vertex triangorg, triangdest, triangapex;           /* Its three vertices. */
  struct badtriang *nexttriang;             /* Pointer to next bad triangle. */
};

/* A stack of triangles flipped during the most recent vertex insertion.     */
/*   The stack is used to undo the vertex insertion if the vertex encroaches */
/*   upon a subsegment.                                                      */

struct flipstacker {
  triangle flippedtri;                       /* A recently flipped triangle. */
  struct flipstacker *prevflip;               /* Previous flip in the stack. */
};

/* A node in a heap used to store events for the sweepline Delaunay          */
/*   algorithm.  Nodes do not point directly to their parents or children in */
/*   the heap.  Instead, each node knows its position in the heap, and can   */
/*   look up its parent and children in a separate array.  The `eventptr'    */
/*   points either to a `vertex' or to a triangle (in encoded format, so     */
/*   that an orientation is included).  In the latter case, the origin of    */
/*   the oriented triangle is the apex of a "circle event" of the sweepline  */
/*   algorithm.  To distinguish site events from circle events, all circle   */
/*   events are given an invalid (smaller than `xmin') x-coordinate `xkey'.  */

struct event {
  TRIANGLE_MACRO_REAL xkey, ykey;                              /* Coordinates of the event. */
  TRIANGLE_MACRO_VOID *eventptr;      /* Can be a vertex or the location of a circle event. */
  int heapposition;              /* Marks this event's position in the heap. */
};

/* A node in the splay tree.  Each node holds an oriented ghost triangle     */
/*   that represents a boundary edge of the growing triangulation.  When a   */
/*   circle event covers two boundary edges with a triangle, so that they    */
/*   are no longer boundary edges, those edges are not immediately deleted   */
/*   from the tree; rather, they are lazily deleted when they are next       */
/*   encountered.  (Since only a random sample of boundary edges are kept    */
/*   in the tree, lazy deletion is faster.)  `keydest' is used to verify     */
/*   that a triangle is still the same as when it entered the splay tree; if */
/*   it has been rotated (due to a circle event), it no longer represents a  */
/*   boundary edge and should be deleted.                                    */

struct splaynode {
  struct otri keyedge;                     /* Lprev of an edge on the front. */
  vertex keydest;           /* Used to verify that splay node is still live. */
  struct splaynode *lchild, *rchild;              /* Children in splay tree. */
};

/* A type used to allocate memory.  firstblock is the first block of items.  */
/*   nowblock is the block from which items are currently being allocated.   */
/*   nextitem points to the next slab of free memory for an item.            */
/*   deaditemstack is the head of a linked list (stack) of deallocated items */
/*   that can be recycled.  unallocateditems is the number of items that     */
/*   remain to be allocated from nowblock.                                   */
/*                                                                           */
/* Traversal is the process of walking through the entire list of items, and */
/*   is separate from allocation.  Note that a traversal will visit items on */
/*   the "deaditemstack" stack as well as live items.  pathblock points to   */
/*   the block currently being traversed.  pathitem points to the next item  */
/*   to be traversed.  pathitemsleft is the number of items that remain to   */
/*   be traversed in pathblock.                                              */
/*                                                                           */
/* alignbytes determines how new records should be aligned in memory.        */
/*   itembytes is the length of a record in bytes (after rounding up).       */
/*   itemsperblock is the number of items allocated at once in a single      */
/*   block.  itemsfirstblock is the number of items in the first block,      */
/*   which can vary from the others.  items is the number of currently       */
/*   allocated items.  maxitems is the maximum number of items that have     */
/*   been allocated at once; it is the current number of items plus the      */
/*   number of records kept on deaditemstack.                                */

struct memorypool {
  TRIANGLE_MACRO_VOID **firstblock, **nowblock;
  TRIANGLE_MACRO_VOID *nextitem;
  TRIANGLE_MACRO_VOID *deaditemstack;
  TRIANGLE_MACRO_VOID **pathblock;
  TRIANGLE_MACRO_VOID *pathitem;
  int alignbytes;
  int itembytes;
  int itemsperblock;
  int itemsfirstblock;
  long items, maxitems;
  int unallocateditems;
  int pathitemsleft;
};


/* Data structure for command line switches and file names.  This structure  */
/*   is used (instead of global variables) to allow reentrancy.              */

typedef struct behavior_t {

  /* Triangulate a Planar Straight Line Graph (-p switch). */
  int poly;

  /* Refine a previously generated mesh (-r switch). */
  int refine;

  /* Quality mesh generation (-q switch). */
  int quality;

  /* Apply an area constraint per triangle (-a switch without number). */
  int vararea;

  /* Apply a global maximum triangle area constraint (-a switch with number). */
  int fixedarea;

  /* Apply a user-defined triangle constraint (-u switch). */
  int usertest;

  /* Apply attributes to identify triangles in certain regions. (-A switch). */
  int regionattrib;

  /* Enclose the convex hull with segments. (-c switch). */
  int convex;

  /* Weighted Delaunay triangulation (1 for -w switch) or regular triangulation */
  /* ie. lower hull of a height field (2 for -W switch). */
  int weighted;

  /* Jettison unused vertices from output (-j switch). */
  int jettison;

  /* Number all items starting from zero or one (0 for -z switch). */
  int firstnumber;

  /* Voronoi */
  int voronoi;

  /* Generate a list of triangle neighbors (-n switch). */
  int neighbors;

  /* Suppress output of boundary information (-B switch). */
  int nobound;

  /* Ignore holes (-O switch). */
  int noholes;

  /* Suppress use of exact arithmetic (-X switch). */
  int noexact;

  /* Conforming Delaunay: all triangles are truly Delaunay (-D switch). */
  int conformdel;

  /* Use incremental method (-i switch). */
  int incremental;

  /* Use Fortune's sweepline algorithm (-F switch). */
  int sweepline;

  /* Use alternating cuts for divide-and-conquer (inverse of -l switch). */
  int dwyer;

  /* Force segments into mesh by splitting instead of using CDT (-s switch). */
  int splitseg;

  /* Determine whether segments are used at all (-p, -r, -q, or -c switch). */
  int usesegments;

  /* Element order (specified after -o switch). */
  int order;

  /* Suppress boundary segment splitting (-Y switch). */
  int nobisect;

  /* Maximum number of added Steiner points (specified after -S switch).   */
  int steiner;

  /* Minimum angle bound (specified after -q switch). */
  TRIANGLE_MACRO_REAL minangle;

  /* Cosine squared of minangle. */
  TRIANGLE_MACRO_REAL goodangle;

  /* Constant used to place off-center Steiner points. */
  TRIANGLE_MACRO_REAL offconstant;

  /* Maximum area bound (specified after -a switch). */
  TRIANGLE_MACRO_REAL maxarea;

#ifndef NO_ACUTE
  /* Maximum angle bound (specified after -U switch). */
  TRIANGLE_MACRO_REAL maxangle;

  /* Cosine of maxangle. */
  TRIANGLE_MACRO_REAL maxgoodangle;
#endif

  /* Callback function for user-defined mesh sizing. */
  /* Arguments are int (vertex triorg, vertex tridest, vertex triapex, TRIANGLE_MACRO_REAL area) */
  /* Should return 1 if triangle has to be further refined or 0 if not. */
  int (*triunsuitable_user_func)(vertex, vertex, vertex, TRIANGLE_MACRO_REAL);
} behavior;                                     /* End of `struct behavior'. */

/* Forward declaration of acute memorypool struct */
#ifndef NO_ACUTE
typedef struct acutepool_t acutepool;
#endif

/* Mesh data structure.  Triangle operates on only one mesh, but the mesh    */
/*   structure is used (instead of global variables) to allow reentrancy.    */

typedef struct mesh_t {

/* Variables used to allocate memory for triangles, subsegments, vertices,   */
/*   viri (triangles being eaten), encroached segments, bad (skinny or too   */
/*   large) triangles, and splay tree nodes.                                 */

  struct memorypool triangles;
  struct memorypool subsegs;
  struct memorypool vertices;
  struct memorypool viri;
  struct memorypool badsubsegs;
  struct memorypool badtriangles;
  struct memorypool flipstackers;
  struct memorypool splaynodes;

#ifndef NO_ACUTE
  acutepool *acute_mem;
#endif

/* Variables that maintain the bad triangle queues.  The queues are          */
/*   ordered from 4095 (highest priority) to 0 (lowest priority).            */

  struct badtriang *queuefront[4096];
  struct badtriang *queuetail[4096];
  int nextnonemptyq[4096];
  int firstnonemptyq;

/* Variable that maintains the stack of recently flipped triangles.          */

  struct flipstacker *lastflip;

/* Other variables. */

  TRIANGLE_MACRO_REAL xmin, xmax, ymin, ymax;                            /* x and y bounds. */
  TRIANGLE_MACRO_REAL xminextreme;      /* Nonexistent x value used as a flag in sweepline. */
  int invertices;                               /* Number of input vertices. */
  int inelements;                              /* Number of input triangles. */
  int insegments;                               /* Number of input segments. */
  int holes;                                       /* Number of input holes. */
  int regions;                                   /* Number of input regions. */
  int undeads;    /* Number of input vertices that don't appear in the mesh. */
  long edges;                                     /* Number of output edges. */
  int mesh_dim;                                /* Dimension (ought to be 2). */
  int nextras;                           /* Number of attributes per vertex. */
  int eextras;                         /* Number of attributes per triangle. */
  long hullsize;                          /* Number of edges in convex hull. */
  int steinerleft;                 /* Number of Steiner points not yet used. */
  int vertexmarkindex;         /* Index to find boundary marker of a vertex. */
  int vertex2triindex;     /* Index to find a triangle adjacent to a vertex. */
  int highorderindex;  /* Index to find extra nodes for high-order elements. */
  int elemattribindex;            /* Index to find attributes of a triangle. */
  int areaboundindex;             /* Index to find area bound of a triangle. */
  int checksegments;         /* Are there segments in the triangulation yet? */
  int checkquality;                  /* Has quality triangulation begun yet? */
  long samples;              /* Number of random samples for point location. */

  long incirclecount;                 /* Number of incircle tests performed. */
  long counterclockcount;     /* Number of counterclockwise tests performed. */
  long orient3dcount;           /* Number of 3D orientation tests performed. */
  long hyperbolacount;      /* Number of right-of-hyperbola tests performed. */
  long circumcentercount;  /* Number of circumcenter calculations performed. */
  long circletopcount;       /* Number of circle top calculations performed. */

/* Triangular bounding box vertices.                                         */

  vertex infvertex1, infvertex2, infvertex3;

/* Pointer to the `triangle' that occupies all of "outer space."             */

  triangle *dummytri;
  triangle *dummytribase;    /* Keep base address so we can free() it later. */

/* Pointer to the omnipresent subsegment.  Referenced by any triangle or     */
/*   subsegment that isn't really connected to a subsegment at that          */
/*   location.                                                               */

  subseg *dummysub;
  subseg *dummysubbase;      /* Keep base address so we can free() it later. */

/* Pointer to a recently visited triangle.  Improves point location if       */
/*   proximate vertices are inserted sequentially.                           */

  struct otri recenttri;

} mesh;                                                  /* End of `struct mesh'. */

typedef struct quality_t {
	TRIANGLE_MACRO_REAL shortest, longest;
	TRIANGLE_MACRO_REAL smallestarea, biggestarea;
	TRIANGLE_MACRO_REAL smallestangle, biggestangle;
	TRIANGLE_MACRO_REAL minaltitude;
	TRIANGLE_MACRO_REAL worstaspect;
	int angletable[18];
	int aspecttable[16];
} quality;

	
typedef struct rect_t {
	TRIANGLE_MACRO_REAL xmin;
	TRIANGLE_MACRO_REAL ymin;
	TRIANGLE_MACRO_REAL xmax;
	TRIANGLE_MACRO_REAL ymax;
} rect;

#ifdef __cplusplus
} // namespace triangle
#endif

#endif /* TRIANGLE_H */