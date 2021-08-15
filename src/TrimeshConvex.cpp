/*
  TrimeshConvex.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <geommanager.h>

#include <cmath>

#include <iostream>
#include <exception>
#include <stdexcept>

using namespace std;

static dReal scale_min_limit = 1e-5;

inline int isRescale(dReal sc){
  dReal f = sc - 1.0;
  return (f < -scale_min_limit) || (f > scale_min_limit);
}

void SetScaleLimit(dReal sclim)
{
  scale_min_limit = sclim;
}

inline void Cross3(dReal *c, dReal *a, dReal *b) // c[3] = a[3] x b[3]
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

inline void Normal4(dReal *n, dReal *v) // n[4] = normal(v[9])
{
  dReal a[] = {v[3] - v[0], v[4] - v[1], v[5] - v[2]};
  dReal b[] = {v[6] - v[0], v[7] - v[1], v[8] - v[2]};
  Cross3(n, a, b);
  dReal r = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  for(int j = 0; j < 3; ++j) n[j] /= r;
  n[3] = 0.333;
}

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
  trimeshvi *tmv = new trimeshvi;
  if(!tmv) throw runtime_error("can't convert to trimeshvi");
  tmv->vtxCount = fvp->vtxCount;
  tmv->vtx = new dReal[3 * tmv->vtxCount];
  if(!tmv->vtx) throw runtime_error("can't convert to trimeshvi.vtx");
  memcpy(tmv->vtx, fvp->vtx, sizeof(dReal) * 3 * tmv->vtxCount);
  if(isRescale(sc)) ScaleTriMeshVI(tmv, sc);
  // get triangles count
  size_t tcnt = 0;
  unsigned int *s = fvp->polygons;
  for(int i = 0; i < fvp->faceCount; ++i){
    unsigned int n = *s++;
    if(n <= 2) throw runtime_error("can't convert to trimeshvi polygon <= 2");
    s += n;
    tcnt += (n - 2); // triangles in the polygon
  }
  tmv->indexCount = 3 * tcnt;
  tmv->indices = new dTriIndex[tmv->indexCount];
  if(!tmv->indices) throw runtime_error("can't convert to trimeshvi.indices");
  // set triangle indices
  dTriIndex *p = tmv->indices;
  s = fvp->polygons;
  for(int i = 0; i < fvp->faceCount; ++i){
    unsigned int n = *s++;
    for(int t = 0; t < (n - 2); ++t){ // triangle number
      *p++ = (dTriIndex)*s++; // [0] [2] [3] [4] ...
      *p++ = (dTriIndex)*s; // [1] [3] [4] [5] ...
      if(!t) *p++ = (dTriIndex)*++s; // [2]
      else *p++ = (dTriIndex)*(s - 2 - t); // [0]
    }
    ++s;
  }
  return tmv;
}

metatrimesh *CvtMetaTriMeshFromConvex(metaconvex *mc, dReal sc)
{
  metatrimesh *mt = new metatrimesh;
  if(!mt) throw runtime_error("can't create metatrimesh cvt");
  mt->density = mc->density;
  mt->tmv = CvtTriMeshVIFromConvexFVP(mc->fvp, sc);
  mt->colour = mc->colour;
  return mt;
}

convexfvp *CvtConvexFVPFromTriMeshVI(trimeshvi *tmv, dReal sc)
{
  convexfvp *fvp = new convexfvp;
  if(!fvp) throw runtime_error("can't convert to convexfvp");
  fvp->faceCount = tmv->indexCount / 3;
  fvp->faces = new dReal[4 * fvp->faceCount];
  if(!fvp->faces) throw runtime_error("can't convert to convexfvp.faces");
  fvp->vtxCount = tmv->vtxCount;
  fvp->vtx = new dReal[3 * fvp->vtxCount];
  if(!fvp->vtx) throw runtime_error("can't convert to convexfvp.vtx");
  memcpy(fvp->vtx, tmv->vtx, sizeof(dReal) * 3 * fvp->vtxCount);
  if(isRescale(sc)) ScaleConvexFVP(fvp, sc);
  fvp->polygons = new unsigned int[4 * fvp->faceCount];
  if(!fvp->polygons) throw runtime_error("can't convert to convexfvp.polygons");
  // set triangle indices
  dTriIndex *s = tmv->indices;
  unsigned int *p = fvp->polygons;
  for(int i = 0; i < fvp->faceCount; ++i){
    *p++ = 3;
    for(int j = 0; j < 3; ++j) *p++ = (unsigned int)*s++;
  }
  // set normal of faces
  dReal *vtx = tmv->vtx;
  s = tmv->indices;
  for(int i = 0; i < fvp->faceCount; ++i){
    dTriIndex idx[] = {*s++, *s++, *s++};
    dReal v[] = {
      vtx[idx[0] * 3 + 0], vtx[idx[0] * 3 + 1], vtx[idx[0] * 3 + 2],
      vtx[idx[1] * 3 + 0], vtx[idx[1] * 3 + 1], vtx[idx[1] * 3 + 2],
      vtx[idx[2] * 3 + 0], vtx[idx[2] * 3 + 1], vtx[idx[2] * 3 + 2]};
    Normal4(&fvp->faces[i * 4], v);
  }
  return fvp;
}

metaconvex *CvtMetaConvexFromTriMesh(metatrimesh *mt, dReal sc)
{
  metaconvex *mc = new metaconvex;
  if(!mc) throw runtime_error("can't create metaconvex cvt");
  mc->density = mt->density;
  mc->fvp = CvtConvexFVPFromTriMeshVI(mt->tmv, sc);
  mc->colour = mt->colour;
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
  if(isRescale(sc)) ScaleTriMeshVI(dst, sc);
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
  if(isRescale(sc)) ScaleConvexFVP(dst, sc);
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
