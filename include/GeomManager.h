/*
  GeomManager.h
*/

#ifndef __GEOMMANAGER_H__
#define __GEOMMANAGER_H__

extern trimeshvi *UnMapGeomTriMesh(dGeomID geom);
extern dGeomID MapGeomTriMesh(dGeomID geom, trimeshvi *tmv);
extern trimeshvi *FindTriMesh(dGeomID geom);
extern convexfvp *UnMapGeomConvex(dGeomID geom);
extern dGeomID MapGeomConvex(dGeomID geom, convexfvp *fvp);
extern convexfvp *FindConvex(dGeomID geom);
extern cmaterial *UnMapGeomMaterial(dGeomID geom);
extern dGeomID MapGeomMaterial(dGeomID geom, cmaterial *cm);
extern cmaterial *FindMaterial(dGeomID geom);
extern dBodyID MapBody(const char *key, dBodyID body);
extern dBodyID FindBody(const char *key);
extern dBodyID OrderBody(dBodyID body, int pos);

extern void DestroyObject(dBodyID body);
extern void DestroyObjects();
extern void DrawObjects(int ws);
extern void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot, int ws);

#endif // __GEOMMANAGER_H__
