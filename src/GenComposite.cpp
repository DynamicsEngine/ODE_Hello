/*
  GenComposite.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <geommanager.h>
#include <gencomposite.h>

#include <iostream>
#include <unordered_map>
#include <deque>

using namespace std;

dBodyID CreateComposite(dWorldID world, dSpaceID space,
  const char *key, metacomposite *mc, int numcomposite)
{
  deque<dGeomID> gto; // to keep order
  gto.clear();
  unordered_map<dGeomID, pair<dGeomID, const dReal *> > gts; // gtrans, gsub, o
  gts.clear();
  dBodyID b = dBodyCreate(world);
  dMass mass;
  dMassSetZero(&mass);
  for(int j = 0; j < numcomposite; ++j){
    dMass subm;
    dMassSetZero(&subm);
    dGeomID gsub = NULL, gtrans = dCreateGeomTransform(space);
    dGeomTransformSetCleanup(gtrans, 1);
    dReal dm = mc[j].dm;
    dReal *param = mc[j].params;
    switch(mc[j].clsID){
    case dSphereClass: {
      gsub = dCreateSphere(0, param[0]);
      dMassSetSphere(&subm, dm, param[0]);
    } break;
    case dBoxClass: {
      gsub = dCreateBox(0, param[0], param[1], param[2]);
      dMassSetBox(&subm, dm, param[0], param[1], param[2]);
      //dMassSetBoxTotal(&subm, subweight, param[0], param[1], param[2]);
    } break;
    case dCapsuleClass: {
      gsub = dCreateCapsule(0, param[0], param[1]);
      dMassSetCapsule(&subm, dm, 3, param[0], param[1]); // 123: xyz
    } break;
    case dCylinderClass: {
      gsub = dCreateCylinder(0, param[0], param[1]);
      dMassSetCylinder(&subm, dm, 3, param[0], param[1]); // 123: xyz
    } break;
    case dPlaneClass: {
      gsub = dCreatePlane(0, param[0], param[1], param[2], param[3]);
      dMassSetBox(&subm, dm, 10.0, 10.0, 0.05); // ***
    } break;
    case dConvexClass: {
      gsub = CreateGeomConvexFromFVP(0, (convexfvp *)mc[j].v);
      _MassSetConvexAsTrimesh(&subm, dm, gsub);
      dGeomSetPosition(gsub, -subm.c[0], -subm.c[1], -subm.c[2]); // ***
      dMassTranslate(&subm, -subm.c[0], -subm.c[1], -subm.c[2]); // ***
    } break;
    case dTriMeshClass: {
      gsub = CreateGeomTrimeshFromVI(0, (trimeshvi *)mc[j].v);
      dMassSetTrimesh(&subm, dm, gsub);
      dGeomSetPosition(gsub, -subm.c[0], -subm.c[1], -subm.c[2]); // ***
      dMassTranslate(&subm, -subm.c[0], -subm.c[1], -subm.c[2]); // ***
    } break;
    default:
      printf("not implemented type dGeomGetClass() in CreateComposite\n");
      printf(" geomID: %p, classID: %d\n", gtrans, mc[j].clsID);
      break;
    }
    dGeomTransformSetGeom(gtrans, gsub);
    // MapGeomMaterial(gtrans, mc[j].cm); // trans will not be shown
    MapGeomMaterial(gsub, mc[j].cm);
    dReal *o = mc[j].offset;
    gto.insert(gto.begin(), gtrans); // first <-> last gto.push_back(gtrans);
    gts.insert(make_pair(gtrans, make_pair(gsub, o)));
    dGeomSetPosition(gsub, o[0], o[1], o[2]);
    dMassTranslate(&subm, o[0], o[1], o[2]);
    // dQuaternion q;
    // dQSetIdentity(mc[j].q); // dQFromAxisAndAngle(q, , , , M_PI / 2);
    dGeomSetQuaternion(gsub, mc[j].q);
    dMatrix3 rot;
    dRfromQ(rot, mc[j].q);
    dMassRotate(&subm, rot);
    dMassAdd(&mass, &subm);
  } // CG != (0, 0, 0)
  for(auto it = gts.begin(); it != gts.end(); ++it){
    dGeomID gsub = it->second.first;
    const dReal *o = it->second.second;
    dGeomSetPosition(gsub, o[0]-mass.c[0], o[1]-mass.c[1], o[2]-mass.c[2]);
  }
  dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  dBodySetMass(b, &mass); // CG == (0, 0, 0)
  for(auto it = gto.begin(); it != gto.end(); ++it) dGeomSetBody(*it, b);
  gts.clear();
  gto.clear();
  return MapBody(key, OrderBody(b, 0));
}

dBodyID CreateSphere(dWorldID world, dSpaceID space,
  const char *key, metasphere *s)
{
  dBodyID b = dBodyCreate(world);
  dMass mass;
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, s->m, s->r);
  dBodySetMass(b, &mass);
  dGeomID geom = dCreateSphere(space, s->r);
  dGeomSetBody(geom, b);
  MapGeomMaterial(geom, s->cm);
  return MapBody(key, OrderBody(b, 0));
}

dBodyID CreatePlane(dWorldID world, dSpaceID space,
  const char *key, metaplane *p)
{
  dBodyID b = dBodyCreate(world);
  dMass mass;
  dMassSetZero(&mass);
  dMassSetBox(&mass, p->dm, p->lxyz[0], p->lxyz[1], p->lxyz[2]);
  dBodySetMass(b, &mass);
  dGeomID geom = dCreatePlane(space, p->v[0], p->v[1], p->v[2], p->v[3]);
  dGeomSetBody(geom, b);
  MapGeomMaterial(geom, p->cm);
  return MapBody(key, OrderBody(b, 0));
}
