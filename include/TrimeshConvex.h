/*
  TrimeshConvex.h
*/

#ifndef __TRIMESHCONVEX_H__
#define __TRIMESHCONVEX_H__

#define A_SIZE(a) (sizeof(a) / sizeof(a[0]))

struct trimeshvi {
  unsigned int vtxCount;
  dReal *vtx;
  dTriIndex *indices;
  unsigned int indexCount; // count of all dTriIndex elements
};

struct convexfvp {
  unsigned int faceCount; // planeCount
  dReal *faces; // planes
  unsigned int vtxCount; // pointCount
  dReal *vtx; // points
  unsigned int *polygons;
};

// extern trimeshvi *CreateTrimeshTemplate(trimeshvi *tmv, , , , );
extern dGeomID CreateTrimeshFromVI(dWorldID world, dSpaceID space,
  dReal density, trimeshvi *tmv);

// extern convexfvp *CreateConvexTemplate(convexfvp *fvp, , , , , );
extern dGeomID CreateConvexFromFVP(dWorldID world, dSpaceID space,
  dReal density, convexfvp *fvp);

extern void DrawTrimeshObject(dGeomID geom, trimeshvi *tmv,
  dReal R, dReal G, dReal B, int ws);
extern void DrawConvexObject(dGeomID geom, convexfvp *fvp,
  dReal R, dReal G, dReal B);

#endif // __TRIMESHCONVEX_H__
