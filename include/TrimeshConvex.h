/*
  TrimeshConvex.h
*/

#ifndef __TRIMESHCONVEX_H__
#define __TRIMESHCONVEX_H__

#define QI {1.0, 0.0, 0.0, 0.0}
#define A_SIZE(a) (sizeof(a) / sizeof(a[0]))

struct cmaterial {
  int texID;
  dReal *colour;
};

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
  cmaterial *cm;
};

struct metaconvex {
  dReal density;
  convexfvp *fvp;
  cmaterial *cm;
};

extern void SetScaleLimit(dReal sclim);

extern void Cross3(dReal *c, dReal *a, dReal *b); // c[3] = a[3] x b[3]
extern void Normal4(dReal *n, dReal *v); // n[4] = normal(v[9])
extern void RecalcFaces(convexfvp *fvp);

extern void FreeTriMeshVI(trimeshvi *tmv);
extern void FreeMetaTriMesh(metatrimesh *mt);

extern void FreeConvexFVP(convexfvp *fvp);
extern void FreeMetaConvex(metaconvex *mc);

extern trimeshvi *CvtTriMeshVIFromConvexFVP(convexfvp *fvp, dReal sc);
extern metatrimesh *CvtMetaTriMeshFromConvex(metaconvex *mc, dReal sc);

extern convexfvp *CvtConvexFVPFromTriMeshVI(trimeshvi *tmv, dReal sc);
extern metaconvex *CvtMetaConvexFromTriMesh(metatrimesh *mt, dReal sc);

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

extern void _MassSetConvexAsTrimesh(dMass *m, dReal density, dGeomID g);

#endif // __TRIMESHCONVEX_H__
