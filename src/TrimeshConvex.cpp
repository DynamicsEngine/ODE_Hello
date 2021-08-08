/*
  TrimeshConvex.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>

// trimeshvi *CreateTrimeshTemplate(trimeshvi *tmv, , , , )

dGeomID CreateTrimeshFromVI(dWorldID world, dSpaceID space,
  dReal density, trimeshvi *tmv)
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
  dMass mass;
  dMassSetZero(&mass);
  dMassSetTrimesh(&mass, density, geom); // mass at .25 .25 .25
  // dMassSetTrimeshTotal(&mass, weight, geom); // mass at .25 .25 .25
//printf("mass at %f %f %f\n", mass.c[0], mass.c[1], mass.c[2]);
  dGeomSetPosition(geom, -mass.c[0], -mass.c[1], -mass.c[2]);
  // dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  dBodyID b = dBodyCreate(world);
  dBodySetMass(b, &mass);
  dGeomSetBody(geom, b);
  return geom;
}

// convexfvp *CreateConvexTemplate(convexfvp *fvp, , , , , )

dGeomID CreateConvexFromFVP(dWorldID world, dSpaceID space,
  dReal density, convexfvp *fvp)
{
  dGeomID geom = dCreateConvex(space,
    fvp->faces, fvp->faceCount, fvp->vtx, fvp->vtxCount, fvp->polygons);
  dMass mass;
  dMassSetZero(&mass);
  //dMassSetSphereTotal(&mass, weight, radius); // (must convert to trimesh)
  dMassSetSphere(&mass, density, 0.5); // (must convert to trimesh)
  //dMassSetBox(&mass, density, 0.25, 0.25, 0.25); // (must convert to trimesh)
  dBodyID b = dBodyCreate(world);
  dBodySetMass(b, &mass);
  dGeomSetBody(geom, b);
  return geom;
}

void DrawTrimeshObject(dGeomID geom, trimeshvi *tmv,
  dReal R, dReal G, dReal B, int ws)
{
  dsSetColor(R, G, B);
#if 0
  const dReal *pos = dGeomGetPosition(geom);
  const dReal *rot = dGeomGetRotation(geom);
  dReal *vtx = tmv->vtx;
  dTriIndex *idx = tmv->indices;
  for(int i = 0; i < tmv->indexCount; i += 3){
    dReal v[] = { // explicit conversion from type of vtx to dReal (now same)
      vtx[idx[i + 0] * 3 + 0],
      vtx[idx[i + 0] * 3 + 1],
      vtx[idx[i + 0] * 3 + 2],
      vtx[idx[i + 1] * 3 + 0],
      vtx[idx[i + 1] * 3 + 1],
      vtx[idx[i + 1] * 3 + 2],
      vtx[idx[i + 2] * 3 + 0],
      vtx[idx[i + 2] * 3 + 1],
      vtx[idx[i + 2] * 3 + 2]};
    dsDrawTriangleD(pos, rot, &v[0], &v[3], &v[6], ws);
  }
#else
  dVector3 tpos = {0.0, 0.0, 0.0};
  dMatrix3 trot;
  dRSetIdentity(trot);
  int triCount = dGeomTriMeshGetTriangleCount(geom);
  for(int i = 0; i < triCount; ++i){
    dVector3 v0, v1, v2;
    dGeomTriMeshGetTriangle(geom, i, &v0, &v1, &v2); // already transformed
    dsDrawTriangleD(tpos, trot, v0, v1, v2, ws);
  }
#endif
}

void DrawConvexObject(dGeomID geom, convexfvp *fvp,
  dReal R, dReal G, dReal B)
{
  dsSetColor(R, G, B);
  const dReal *pos = dGeomGetPosition(geom);
  const dReal *rot = dGeomGetRotation(geom);
  dsDrawConvexD(pos, rot,
    fvp->faces, fvp->faceCount, fvp->vtx, fvp->vtxCount, fvp->polygons);
}
