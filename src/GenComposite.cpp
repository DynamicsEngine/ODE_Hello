/*
  GenComposite.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <gencomposite.h>

#include <iostream>
#include <map>

using namespace std;

dBodyID CreateComposite(dWorldID world, dSpaceID space,
  metacomposite *mc)
{
  map<dGeomID, pair<dGeomID, const dReal *> > gts; // <trans, <sub, offset> >
  gts.clear();
  dBodyID b = dBodyCreate(world);
  dMass mass;
  dMassSetZero(&mass);
  for(int j = 0; j < mc->numcomposite; ++j){
    dMass subm;
    dMassSetZero(&subm);
    dGeomID gsub = NULL, gtrans = dCreateGeomTransform(space);
    dGeomTransformSetCleanup(gtrans, 1);
    dReal dm = mc->dm[j];
    dReal *param = mc->params[j];
    switch(mc->clsID[j]){
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
    default:
      printf("not implemented type dGeomGetClass() in CreateComposite\n");
      printf(" geomID: %p, classID: %d\n", gtrans, mc->clsID[j]);
      break;
    }
    dGeomTransformSetGeom(gtrans, gsub);
    dReal *o = mc->offset[j];
    gts.insert(make_pair(gtrans, make_pair(gsub, o)));
    dGeomSetPosition(gsub, o[0], o[1], o[2]);
    dMassTranslate(&subm, o[0], o[1], o[2]);
    // dQuaternion q;
    // dQSetIdentity(mc->q[j]); // dQFromAxisAndAngle(q, , , , M_PI / 2);
    dGeomSetQuaternion(gsub, mc->q[j]);
    dMatrix3 rot;
    dRfromQ(rot, mc->q[j]);
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
  for(auto it = gts.begin(); it != gts.end(); ++it) dGeomSetBody(it->first, b);
  gts.clear();
  return b;
}
