/*
  TrimeshConvex.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <geommanager.h>

#include <iostream>
#include <exception>
#include <stdexcept>

using namespace std;

void FreeTriMeshVI(trimeshvi *tmv)
{
  if(!tmv) return;
  if(tmv->vtx) delete[] tmv->vtx;
  if(tmv->indices) delete[] tmv->indices;
  delete tmv;
}

void FreeMetaTriMesh(metatrimesh *mt)
{
  if(!mt) return;
  if(mt->tmv) FreeTriMeshVI(mt->tmv);
  delete mt;
}

void FreeConvexFVP(convexfvp *fvp)
{
  if(!fvp) return;
  if(fvp->faces) delete[] fvp->faces;
  if(fvp->vtx) delete[] fvp->vtx;
  if(fvp->polygons) delete[] fvp->polygons;
  delete fvp;
}

void FreeMetaConvex(metaconvex *mc)
{
  if(!mc) return;
  if(mc->fvp) FreeConvexFVP(mc->fvp);
  delete mc;
}

trimeshvi *CvtTriMeshVIFromConvexFVP(convexfvp *fvp, dReal sc)
{
  trimeshvi *tmv;
  return tmv;
}

metatrimesh *CvtMetaTriMeshFromMetaConvex(metaconvex *mc, dReal sc)
{
  metatrimesh *mt;
  return mt;
}

convexfvp *CvtConvexFVPFromTriMeshVI(trimeshvi *tmv, dReal sc)
{
  convexfvp *fvp;
  return fvp;
}

metaconvex *CvtMetaConvexFromMetaTriMesh(metatrimesh *mt, dReal sc)
{
  metaconvex *mc;
  return mc;
}

trimeshvi *ScaleTriMeshVI(trimeshvi *tmv, dReal sc)
{
  for(int i = 0; i < 3 * tmv->vtxCount; ++i) tmv->vtx[i] *= sc;
  return tmv;
}

trimeshvi *CopyTriMeshVI(trimeshvi *dst, trimeshvi *src, dReal sc)
{
#if 0
  cout << "CopyTriMeshVI: " << (dst ? "copy" : "create") << endl;
  cout << " vtxCount: " << src->vtxCount << endl;
  cout << " indexCount: " << src->indexCount << endl;
#endif
  if(!dst){
    dst = new trimeshvi;
    if(!dst) throw runtime_error("can't create trimeshvi");
    dst->vtx = new dReal[3 * src->vtxCount];
    if(!dst->vtx) throw runtime_error("can't create trimeshvi.vtx");
    dst->indices = new dTriIndex[src->indexCount];
    if(!dst->indices) throw runtime_error("can't create trimeshvi.indices");
  }
  dst->vtxCount = src->vtxCount;
#if 0
  for(int i = 0; i < 3 * dst->vtxCount; ++i) dst->vtx[i] = sc * src->vtx[i];
#else
  memcpy(dst->vtx, src->vtx, sizeof(dReal) * 3 * dst->vtxCount);
  ScaleTriMeshVI(dst, sc);
#endif
  dst->indexCount = src->indexCount;
  memcpy(dst->indices, src->indices, sizeof(dTriIndex) * dst->indexCount);
  return dst;
}

metatrimesh *CopyMetaTriMesh(
  metatrimesh *dst, metatrimesh *src, dReal sc)
{
  int create = dst == NULL;
  if(create){
    dst = new metatrimesh;
    if(!dst) throw runtime_error("can't create metatrimesh");
  }
  dst->density = src->density;
  dst->tmv = CopyTriMeshVI(create ? NULL : dst->tmv, src->tmv, sc);
  dst->colour = src->colour;
  return dst;
}

dGeomID CreateGeomTrimeshFromVI(dSpaceID space, trimeshvi *tmv)
{
  dTriMeshDataID tmd = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildDouble(tmd,
    tmv->vtx, 3 * sizeof(dReal), tmv->vtxCount,
    tmv->indices, tmv->indexCount, 3 * sizeof(dTriIndex));
  dGeomTriMeshDataPreprocess2(tmd,
    (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);
  dGeomID geom = dCreateTriMesh(space, tmd, 0, 0, 0);
#if 0 // do not release here
  if(tmd){ dGeomTriMeshDataDestroy(tmd); tmd = NULL; }
#else
  dGeomSetData(geom, tmd);
#endif
  return geom;
}

dBodyID CreateTrimeshFromVI(dWorldID world, dSpaceID space,
  const char *key, metatrimesh *mt)
{
  dGeomID geom = CreateGeomTrimeshFromVI(space, mt->tmv);
  dMass mass;
  dMassSetZero(&mass);
  dMassSetTrimesh(&mass, mt->density, geom); // mass at .25 .25 .25
  // dMassSetTrimeshTotal(&mass, weight, geom); // mass at .25 .25 .25
//printf("mass at %f %f %f\n", mass.c[0], mass.c[1], mass.c[2]);
  dGeomSetPosition(geom, -mass.c[0], -mass.c[1], -mass.c[2]);
  dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  dBodyID b = dBodyCreate(world);
  dBodySetMass(b, &mass);
  dGeomSetBody(geom, b);
  MapGeomColour(geom, mt->colour);
  return MapBody(key, OrderBody(b, 0));
}

convexfvp *ScaleConvexFVP(convexfvp *fvp, dReal sc)
{
  for(int i = 0; i < 3 * fvp->vtxCount; ++i) fvp->vtx[i] *= sc;
  return fvp;
}

convexfvp *CopyConvexFVP(convexfvp *dst, convexfvp *src, dReal sc)
{
  size_t count = 0;
  unsigned int *s = src->polygons;
  for(int i = 0; i < src->faceCount; ++i){
    unsigned int n = *s++;
    s += n;
    count += (1 + n);
  }
#if 0
  cout << "CopyConvexFVP: " << (dst ? "copy" : "create") << endl;
  cout << " faceCount: " << src->faceCount << endl;
  cout << " vtxCount: " << src->vtxCount << endl;
  cout << " indexCount of Polygons: " << count << endl;
#endif
  if(!dst){
    dst = new convexfvp;
    if(!dst) throw runtime_error("can't create convexfvp");
    dst->faces = new dReal[4 * src->faceCount];
    if(!dst->faces) throw runtime_error("can't create convexfvp.faces");
    dst->vtx = new dReal[3 * src->vtxCount];
    if(!dst->vtx) throw runtime_error("can't create convexfvp.vtx");
    dst->polygons = new unsigned int[count];
    if(!dst->polygons) throw runtime_error("can't create convexfvp.polygons");
  }
  dst->faceCount = src->faceCount;
  memcpy(dst->faces, src->faces, sizeof(dReal) * 4 * dst->faceCount);
  dst->vtxCount = src->vtxCount;
#if 0
  for(int i = 0; i < 3 * dst->vtxCount; ++i) dst->vtx[i] = sc * src->vtx[i];
#else
  memcpy(dst->vtx, src->vtx, sizeof(dReal) * 3 * dst->vtxCount);
  ScaleConvexFVP(dst, sc);
#endif
  memcpy(dst->polygons, src->polygons, sizeof(unsigned int) * count);
  return dst;
}

metaconvex *CopyMetaConvex(
  metaconvex *dst, metaconvex *src, dReal sc)
{
  int create = dst == NULL;
  if(create){
    dst = new metaconvex;
    if(!dst) throw runtime_error("can't create metaconvex");
  }
  dst->density = src->density;
  dst->fvp = CopyConvexFVP(create ? NULL : dst->fvp, src->fvp, sc);
  dst->colour = src->colour;
  return dst;
}

dGeomID CreateGeomConvexFromFVP(dSpaceID space, convexfvp *fvp)
{
  dGeomID geom = dCreateConvex(space,
    fvp->faces, fvp->faceCount, fvp->vtx, fvp->vtxCount, fvp->polygons);
  MapGeomConvex(geom, fvp);
  return geom;
}

dBodyID CreateConvexFromFVP(dWorldID world, dSpaceID space,
  const char *key, metaconvex *mc)
{
  dGeomID geom = CreateGeomConvexFromFVP(space, mc->fvp);
  dMass mass;
  dMassSetZero(&mass);
  //dMassSetSphereTotal(&mass, weight, radius); // (must convert to trimesh)
  dMassSetSphere(&mass, mc->density, 0.5); // (must convert to trimesh)
  //dMassSetBox(&mass, mc->density, 0.25, 0.25, 0.25); // (must convert to trimesh)
  dGeomSetPosition(geom, -mass.c[0], -mass.c[1], -mass.c[2]);
  dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  dBodyID b = dBodyCreate(world);
  dBodySetMass(b, &mass);
  dGeomSetBody(geom, b);
  MapGeomColour(geom, mc->colour);
  return MapBody(key, OrderBody(b, 0));
}
