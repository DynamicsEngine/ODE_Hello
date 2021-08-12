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

extern void FreeTriMeshVI(trimeshvi *tmv);
extern void FreeMetaTriMesh(metatrimesh *mt);

extern void FreeConvexFVP(convexfvp *fvp);
extern void FreeMetaConvex(metaconvex *mc);

extern trimeshvi *CvtTriMeshVIFromConvexFVP(convexfvp *fvp, dReal sc);
extern metatrimesh *CvtMetaTriMeshFromMetaConvex(metaconvex *mc, dReal sc);

extern convexfvp *CvtConvexFVPFromTriMeshVI(trimeshvi *tmv, dReal sc);
extern metaconvex *CvtMetaConvexFromMetaTriMesh(metatrimesh *mt, dReal sc);

extern trimeshvi *ScaleTriMeshVI(trimeshvi *tmv, dReal sc);
extern trimeshvi *CopyTriMeshVI(trimeshvi *dst, trimeshvi *src, dReal sc);
extern metatrimesh *CopyMetaTriMesh(
  metatrimesh *dst, metatrimesh *src, dReal sc);

extern dGeomID CreateGeomTrimeshFromVI(dSpaceID space, trimeshvi *tmv);
extern dBodyID CreateTrimeshFromVI(dWorldID world, dSpaceID space,
  const char *key, metatrimesh *mt);

extern convexfvp *ScaleConvexFVP(convexfvp *fvp, dReal sc);
extern convexfvp *CopyConvexFVP(convexfvp *dst, convexfvp *src, dReal sc);
extern metaconvex *CopyMetaConvex(
  metaconvex *dst, metaconvex *src, dReal sc);

extern dGeomID CreateGeomConvexFromFVP(dSpaceID space, convexfvp *fvp);
extern dBodyID CreateConvexFromFVP(dWorldID world, dSpaceID space,
  const char *key, metaconvex *mc);

#endif // __TRIMESHCONVEX_H__
