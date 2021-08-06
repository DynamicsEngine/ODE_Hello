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
  dGeomTriMeshDataBuildSingle(tmd,
    tmv->vtx, 3 * sizeof(float), tmv->vtxCount,
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
