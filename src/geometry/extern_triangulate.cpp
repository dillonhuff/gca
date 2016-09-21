#include "geometry/extern_triangulate.h"

#include "geometry/vtk_utils.h"
#include "geometry/vtk_debug.h"

#define ANSI_DECLARATORS

#ifdef SINGLE

#define REAL float

#else

#define REAL double

#endif

#include <string>
#include <stdio.h>
#include <stdlib.h>

extern "C" {
#include "triangle_lib/triangle.h"
}

namespace gca {

  void report(struct triangulateio *io,
	      int markers,
	      int reporttriangles,
	      int reportneighbors,
	      int reportsegments,
	      int reportedges,
	      int reportnorms) {

    int i, j;

    for (i = 0; i < io->numberofpoints; i++) {
      printf("Point %4d:", i);
      for (j = 0; j < 2; j++) {
	printf("  %.6g", io->pointlist[i * 2 + j]);
      }
      if (io->numberofpointattributes > 0) {
	printf("   attributes");
      }
      for (j = 0; j < io->numberofpointattributes; j++) {
	printf("  %.6g",
	       io->pointattributelist[i * io->numberofpointattributes + j]);
      }
      if (markers) {
	printf("   marker %d\n", io->pointmarkerlist[i]);
      } else {
	printf("\n");
      }
    }
    printf("\n");

    if (reporttriangles || reportneighbors) {
      for (i = 0; i < io->numberoftriangles; i++) {
	if (reporttriangles) {
	  printf("Triangle %4d points:", i);
	  for (j = 0; j < io->numberofcorners; j++) {
	    printf("  %4d", io->trianglelist[i * io->numberofcorners + j]);
	  }
	  if (io->numberoftriangleattributes > 0) {
	    printf("   attributes");
	  }
	  for (j = 0; j < io->numberoftriangleattributes; j++) {
	    printf("  %.6g", io->triangleattributelist[i *
						       io->numberoftriangleattributes + j]);
	  }
	  printf("\n");
	}
	if (reportneighbors) {
	  printf("Triangle %4d neighbors:", i);
	  for (j = 0; j < 3; j++) {
	    printf("  %4d", io->neighborlist[i * 3 + j]);
	  }
	  printf("\n");
	}
      }
      printf("\n");
    }

    if (reportsegments) {
      for (i = 0; i < io->numberofsegments; i++) {
	printf("Segment %4d points:", i);
	for (j = 0; j < 2; j++) {
	  printf("  %4d", io->segmentlist[i * 2 + j]);
	}
	if (markers) {
	  printf("   marker %d\n", io->segmentmarkerlist[i]);
	} else {
	  printf("\n");
	}
      }
      printf("\n");
    }

    if (reportedges) {
      for (i = 0; i < io->numberofedges; i++) {
	printf("Edge %4d points:", i);
	for (j = 0; j < 2; j++) {
	  printf("  %4d", io->edgelist[i * 2 + j]);
	}
	if (reportnorms && (io->edgelist[i * 2 + 1] == -1)) {
	  for (j = 0; j < 2; j++) {
	    printf("  %.6g", io->normlist[i * 2 + j]);
	  }
	}
	if (markers) {
	  printf("   marker %d\n", io->edgemarkerlist[i]);
	} else {
	  printf("\n");
	}
      }
      printf("\n");
    }
  }
  
  std::vector<triangle> triangulate_flat_3d(const polygon_3& p) {
    struct triangulateio in, mid;

    /* Define input points. */

    in.numberofpoints = 8;
    in.numberofpointattributes = 0;

    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.pointlist[0] = 0.0;
    in.pointlist[1] = 0.0;
    in.pointlist[2] = 1.0;
    in.pointlist[3] = 0.0;
    in.pointlist[4] = 1.0;
    in.pointlist[5] = 10.0;
    in.pointlist[6] = 0.0;
    in.pointlist[7] = 10.0;

    in.pointlist[8] = 0.1;
    in.pointlist[9] = 0.1;
    in.pointlist[10] = 0.9;
    in.pointlist[11] = 0.1;
    in.pointlist[12] = 0.9;
    in.pointlist[13] = 9.0;
    in.pointlist[14] = 0.1;
    in.pointlist[15] = 9.0;
  
    in.pointattributelist = NULL;

    in.pointmarkerlist = (int *) malloc(in.numberofpoints * sizeof(int));
    in.pointmarkerlist[0] = 0;
    in.pointmarkerlist[1] = 0;
    in.pointmarkerlist[2] = 0;
    in.pointmarkerlist[3] = 0;

    in.pointmarkerlist[4] = 0;
    in.pointmarkerlist[5] = 0;
    in.pointmarkerlist[6] = 0;
    in.pointmarkerlist[7] = 0;

    in.numberofsegments = 8;
    /* Needed only if segments are output (-p or -c) and -P not used: */
    in.segmentlist = static_cast<int*>(malloc(2*in.numberofsegments*sizeof(int))); //(int *) NULL;
    in.segmentlist[0] = 0;
    in.segmentlist[1] = 1;
    in.segmentlist[2] = 1;
    in.segmentlist[3] = 2;
    in.segmentlist[4] = 2;
    in.segmentlist[5] = 3;
    in.segmentlist[6] = 3;
    in.segmentlist[7] = 0;

    in.segmentlist[8] = 0 + 4;
    in.segmentlist[9] = 1 + 4;
    in.segmentlist[10] = 1 + 4;
    in.segmentlist[11] = 2 + 4;
    in.segmentlist[12] = 2 + 4;
    in.segmentlist[13] = 3 + 4;
    in.segmentlist[14] = 3 + 4;
    in.segmentlist[15] = 0 + 4;
  
    in.segmentmarkerlist = static_cast<int*>(malloc(2*in.numberofsegments*sizeof(int)));
    in.segmentmarkerlist[0] = 0;
    in.segmentmarkerlist[1] = 0;
    in.segmentmarkerlist[2] = 0;
    in.segmentmarkerlist[3] = 0;

    in.segmentmarkerlist[4] = 0;
    in.segmentmarkerlist[5] = 0;
    in.segmentmarkerlist[6] = 0;
    in.segmentmarkerlist[7] = 0;


    // Need to add holes
    in.numberofholes = 0;
    in.numberofregions = 0; //1;
    in.regionlist = NULL; // (REAL *) malloc(in.numberofregions * 4 * sizeof(REAL));
    // in.regionlist[0] = 0.5;
    // in.regionlist[1] = 5.0;
    // in.regionlist[2] = 7.0;            /* Regional attribute (for whole mesh). */
    // in.regionlist[3] = 0.1;          /* Area constraint that will not be used. */

    printf("Input point set:\n\n");
    report(&in, 1, 0, 0, 0, 0, 0);

    /* Make necessary initializations so that Triangle can return a */
    /*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

    mid.pointlist = (REAL *) NULL;            /* Not needed if -N switch used. */
    /* Not needed if -N switch used or number of point attributes is zero: */
    mid.pointattributelist = (REAL *) NULL;
    mid.pointmarkerlist = (int *) NULL; /* Not needed if -N or -B switch used. */
    mid.trianglelist = (int *) NULL;          /* Not needed if -E switch used. */
    /* Not needed if -E switch used or number of triangle attributes is zero: */
    mid.triangleattributelist = (REAL *) NULL;
    mid.neighborlist = (int *) NULL;         /* Needed only if -n switch used. */

    /* Needed only if segments are output (-p or -c) and -P and -B not used: */
    mid.segmentmarkerlist = (int *) NULL;
    mid.edgelist = (int *) NULL;             /* Needed only if -e switch used. */
    mid.edgemarkerlist = (int *) NULL;   /* Needed if -e used and -B not used. */

    /* Triangulate the points.  Switches are chosen to read and write a  */
    /*   PSLG (p), preserve the convex hull (c), number everything from  */
    /*   zero (z), assign a regional attribute to each element (A), and  */
    /*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
    /*   neighbor list (n).                                              */

    char settings[] = "pzen";
    triangulate(&(settings[0]), &in, &mid, NULL);

    printf("Initial triangulation:\n\n");
    report(&mid, 1, 1, 1, 1, 1, 0);

    cout << "mid.numberofcorners = " << mid.numberofcorners << endl;

    vector<point> point_vec;
    for (unsigned i = 0; i < mid.numberofpoints; i++) {
      REAL px = mid.pointlist[2*i];
      REAL py = mid.pointlist[2*i + 1];
      point_vec.push_back(point(px, py, 0.0));
    }

    cout << "POINTS" << endl;
    for (auto p : point_vec) {
      cout << p << endl;
    }
  
    vector<triangle> tris;
    for (unsigned i = 0; i < mid.numberoftriangles; i++) {
      vector<point> pts;
      for (unsigned j = 0; j < mid.numberofcorners; j++) {
	auto ind = mid.trianglelist[i * mid.numberofcorners + j];
	pts.push_back(point_vec[ind]);
      }

      DBG_ASSERT(pts.size() == 3);

      point v0 = pts[0];
      point v1 = pts[1];
      point v2 = pts[2];
    
      point q1 = v1 - v0;
      point q2 = v2 - v0;
      point norm = cross(q2, q1).normalize();
      tris.push_back(triangle(norm, v0, v1, v2));
    }

    cout << "Number of triangles = " << tris.size() << endl;
    for (auto t : tris) {
      cout << t << endl;
    }

    auto pd = polydata_for_triangles(tris);
    auto act = polydata_actor(pd);
    visualize_actors({act});

    /* Free all allocated arrays, including those allocated by Triangle. */

    free(in.pointlist);
    free(in.pointattributelist);
    free(in.pointmarkerlist);
    free(in.regionlist);

    free(mid.pointlist);
    free(mid.pointattributelist);
    free(mid.pointmarkerlist);
    free(mid.trianglelist);
    free(mid.triangleattributelist);
    free(mid.trianglearealist);
    free(mid.neighborlist);
    free(mid.segmentlist);
    free(mid.segmentmarkerlist);
    free(mid.edgelist);
    free(mid.edgemarkerlist);

    DBG_ASSERT(false);
  }

}
