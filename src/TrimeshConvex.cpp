/*
  TrimeshConvex.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <geommanager.h>

// trimeshvi *CreateTrimeshTemplate(trimeshvi *tmv, , , , )

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
  metatrimesh *mt)
{
  dGeomID geom = CreateGeomTrimeshFromVI(space, mt->tmv);
  dMass mass;
  dMassSetZero(&mass);
  dMassSetTrimesh(&mass, mt->density, geom); // mass at .25 .25 .25
  // dMassSetTrimeshTotal(&mass, weight, geom); // mass at .25 .25 .25
//printf("mass at %f %f %f\n", mass.c[0], mass.c[1], mass.c[2]);
  dGeomSetPosition(geom, -mass.c[0], -mass.c[1], -mass.c[2]);
  // dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  dBodyID b = dBodyCreate(world);
  dBodySetMass(b, &mass);
  dGeomSetBody(geom, b);
  MapGeomColour(geom, mt->colour);
  return b;
}

// convexfvp *CreateConvexTemplate(convexfvp *fvp, , , , , )

dGeomID CreateGeomConvexFromFVP(dSpaceID space, convexfvp *fvp)
{
  dGeomID geom = dCreateConvex(space,
    fvp->faces, fvp->faceCount, fvp->vtx, fvp->vtxCount, fvp->polygons);
  MapGeomConvex(geom, fvp);
  return geom;
}

dBodyID CreateConvexFromFVP(dWorldID world, dSpaceID space,
  metaconvex *mc)
{
  dGeomID geom = CreateGeomConvexFromFVP(space, mc->fvp);
  dMass mass;
  dMassSetZero(&mass);
  //dMassSetSphereTotal(&mass, weight, radius); // (must convert to trimesh)
  dMassSetSphere(&mass, mc->density, 0.5); // (must convert to trimesh)
  //dMassSetBox(&mass, mc->density, 0.25, 0.25, 0.25); // (must convert to trimesh)
  dBodyID b = dBodyCreate(world);
  dBodySetMass(b, &mass);
  dGeomSetBody(geom, b);
  MapGeomColour(geom, mc->colour);
  return b;
}
