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

struct metatrimesh {
  dReal density;
  trimeshvi *tmv;
  dReal *colour;
};

struct metaconvex {
  dReal density;
  convexfvp *fvp;
  dReal *colour;
};

// extern trimeshvi *CreateTrimeshTemplate(trimeshvi *tmv, , , , );
extern dGeomID CreateGeomTrimeshFromVI(dSpaceID space, trimeshvi *tmv);
extern dBodyID CreateTrimeshFromVI(dWorldID world, dSpaceID space,
  const char *key, metatrimesh *mt);

// extern convexfvp *CreateConvexTemplate(convexfvp *fvp, , , , , );
extern dGeomID CreateGeomConvexFromFVP(dSpaceID space, convexfvp *fvp);
extern dBodyID CreateConvexFromFVP(dWorldID world, dSpaceID space,
  const char *key, metaconvex *mc);

#endif // __TRIMESHCONVEX_H__
