/*
  GeomManager.h
*/

#ifndef __GEOMMANAGER_H__
#define __GEOMMANAGER_H__

extern dGeomID MapGeomConvex(dGeomID geom, convexfvp *fvp);
extern dGeomID MapGeomColour(dGeomID geom, const dReal *colour);
extern dBodyID MapBody(const char *key, dBodyID body);
extern dBodyID FindBody(const char *key);
extern dBodyID OrderBody(dBodyID body, int pos);

extern void DestroyObject(dBodyID body);
extern void DestroyObjects();
extern void DrawObjects(int ws);
extern void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot, int ws);

#endif // __GEOMMANAGER_H__
