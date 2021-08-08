/*
  ODE_Hello.cpp

  ODE Open Dynamics Engine sample

  *** IME setting - use old IME ***
*/

#include <iostream>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>

using namespace std;

#define TEX_RES "C:/ode-0.16.2/drawstuff/textures"

#define WIDTH 640
#define HEIGHT 480

#define MAX_COLLIDE_NUM 10

#define TICK_DELTA 0.01
#define DENSITY 5.0

int sw_viewpoint = 0; // xyz: camera position, hpr: look at
float xyz[][3] = {{5.0, 0.0, 2.0}, {5.36, 2.02, 4.28}, {-8.3, -14.1, 3.1}};
float hpr[][3] = {{-180.0, 0.0, 0.0}, {-162.0, -31.0, 0.0}, {84.5, 1.0, 0.0}};

float move_delta = 0.1; // delta for x, y, z move
int wire_solid = 1; // 0: wireframe, 1: solid (for bunny)
int polyfill_wireframe = 0; // 0: solid, 1: wireframe (for all)

dWorldID world; // for body
dSpaceID space; // for geom collision
dGeomID ground;
dJointGroupID contactgroup;

extern trimeshvi tmvTetra;
extern convexfvp fvpTetra;

extern trimeshvi tmvCube;
extern convexfvp fvpCube;

extern trimeshvi tmvIcosahedron;
extern convexfvp fvpIcosahedron;

extern trimeshvi tmvBunny;
extern convexfvp fvpBunny; // body drawing by fvpBunny causes rocky

extern trimeshvi tmvCustom; // now tetra
extern convexfvp fvpCustom; // now tetra

dGeomID geomTmTetra;
dGeomID geomTetra;
dGeomID geomTmCube;
dGeomID geomCube;
dGeomID geomTmIcosahedron;
dGeomID geomIcosahedron;
dGeomID geomTmBunny;
dGeomID geomBunny;
dGeomID geomTmCustom;
dGeomID geomCustom;

dReal slopeSz[] = {8.0, 0.1, 2.0};
dReal slopeLR[] = {2.0, 1.0};
dReal slopeO[][3] = {{0.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}}; // offset
const int slopeNC = A_SIZE(slopeO); // number of composite parts
dGeomID geomSlope[slopeNC][2]; // composite [n][] = {geomTrans, geomSub}

struct sphere {
  dBodyID body;
  dGeomID geom;
  dReal r, m;
  dVector4 colour;
  dReal gBounce;
};

struct sphere apple, ball, roll;

dVector4 palette[] = {
  {0.8, 0.4, 0.4, 1.0}, // apple
  {0.4, 0.4, 0.8, 1.0}, // ball
  {0.4, 0.8, 0.4, 1.0}, // roll
  {1.0, 1.0, 1.0, 1.0}, // geomSlope
  {0.8, 0.6, 0.2, 1.0}, // geomTmTetra
  {0.4, 0.8, 0.4, 1.0}, // geomTetra
  {0.6, 0.8, 0.2, 1.0}, // geomTmCube
  {0.8, 0.8, 0.4, 1.0}, // geomCube
  {0.2, 0.8, 0.6, 1.0}, // geomTmIcosahedron
  {0.4, 0.8, 0.8, 1.0}, // geomIcosahedron
  {0.8, 0.2, 0.6, 1.0}, // geomTmBunny
  {0.8, 0.4, 0.8, 1.0}, // geomBunny
  {0.6, 0.2, 0.8, 1.0}, // geomTmCustom
  {0.2, 0.6, 0.8, 1.0}, // geomCustom
  {1.0, 1.0, 1.0, 1.0}
};

void DestroyCompositeObject(dGeomID (*geomcomposite)[2], int num);
void DestroyObject(dGeomID geom);
void DestroyObjects();
void CreateObjects(dWorldID world);
void DrawObjects();
void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot,
  const dReal *colour, int ws);

void CreateSphere(struct sphere *s,
  dWorldID world, dReal r, dReal m, dReal bounce, const dReal *colour);

dReal getgBounce(dGeomID id);
void nearCallback(void *data, dGeomID o1, dGeomID o2);

void showViewPoint(int save);
void command(int cmd);
void setParameters();
void simLoop(int pause);

void drawStuffStart();
void setDrawStuff(dsFunctions *fn);

void DestroyCompositeObject(dGeomID (*geomcomposite)[2], int num)
{
  dBodyDestroy(dGeomGetBody(geomcomposite[0][0]));
  for(int j = 0; j < num; ++j) dGeomDestroy(geomcomposite[j][0]);
}

void DestroyObject(dGeomID geom)
{
  dBodyDestroy(dGeomGetBody(geom));
  dGeomDestroy(geom);
}

void DestroyObjects()
{
  DestroyObject(apple.geom);
  DestroyObject(ball.geom);
  DestroyObject(roll.geom);

  DestroyCompositeObject(geomSlope, slopeNC);

  DestroyObject(geomTmTetra);
  DestroyObject(geomTetra);
  DestroyObject(geomTmCube);
  DestroyObject(geomCube);
  DestroyObject(geomTmIcosahedron);
  DestroyObject(geomIcosahedron);
  DestroyObject(geomTmBunny);
  DestroyObject(geomBunny);
  DestroyObject(geomTmCustom);
  DestroyObject(geomCustom);
}

void CreateObjects(dWorldID world)
{
cout << "Sphere red" << endl;
  CreateSphere(&apple, world, 0.2, 1.0, 1.0, palette[0]);
  dBodySetPosition(apple.body, -0.15, 0.31, 2.5); // x, y on the bunny
  dBodyDisable(apple.body);
cout << "Sphere blue" << endl;
  CreateSphere(&ball, world, 0.1, 1.0, 0.5, palette[1]);
  dBodySetPosition(ball.body, 0.5, 0.0, ball.r);
cout << "Sphere green" << endl;
  CreateSphere(&roll, world, 0.2, 1.0, 0.8, palette[2]);
  dBodySetPosition(roll.body, -12.0, 0.0, 1.2); // on the slope

cout << "Slope" << endl;
  dBodyID s = dBodyCreate(world);
  dMass mass;
  dMassSetZero(&mass);
  for(int j = 0; j < slopeNC; ++j){
    dMass subm;
    dMassSetZero(&subm);
    dReal *o = slopeO[j];
    dGeomID *g = geomSlope[j];
    g[0] = dCreateGeomTransform(space);
    dGeomTransformSetCleanup(g[0], 1);
    switch(j){
    case 0:
      g[1] = dCreateBox(0, slopeSz[0], slopeSz[1], slopeSz[2]);
      dMassSetBox(&subm, DENSITY, slopeSz[0], slopeSz[1], slopeSz[2]);
      //dMassSetBoxTotal(&subm, subweight, slopeSz[0], slopeSz[1], slopeSz[2]);
      break;
    case 1:
      g[1] = dCreateCylinder(0, slopeLR[1], slopeLR[0]);
      dMassSetCylinder(&subm, DENSITY, 3, slopeLR[1], slopeLR[0]); // 123: xyz
      break;
    }
    dGeomTransformSetGeom(g[0], g[1]);
    dGeomSetPosition(g[1], o[0], o[1], o[2]);
    dMassTranslate(&subm, o[0], o[1], o[2]);
    dQuaternion q;
    dQSetIdentity(q); // dQFromAxisAndAngle(q, , , , M_PI / 2);
    dGeomSetQuaternion(g[1], q);
    dMatrix3 rot;
    dRfromQ(rot, q);
    dMassRotate(&subm, rot);
    dMassAdd(&mass, &subm);
  } // CG != (0, 0, 0)
  for(int j = 0; j < slopeNC; ++j){
    dReal *o = slopeO[j];
    dGeomID *g = geomSlope[j];
    dGeomSetPosition(g[1], o[0]-mass.c[0], o[1]-mass.c[1], o[2]-mass.c[2]);
  }
  dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  dBodySetMass(s, &mass); // CG == (0, 0, 0)
  for(int j = 0; j < slopeNC; ++j) dGeomSetBody(geomSlope[j][0], s);
  dBodySetPosition(s, -13.5, 0.0, 1.2);
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
    dQFromAxisAndAngle(p, 0, 1, 0, M_PI / 18);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(s, o);
  }
  dBodyEnable(s); // dBodyDisable(s);

cout << "TmTetra" << endl;
  geomTmTetra = CreateTrimeshFromVI(world, space, DENSITY, &tmvTetra);
  dBodyID t = dGeomGetBody(geomTmTetra);
  dBodySetPosition(t, 0.0, -1.5, 0.5);
  dBodyEnable(t); // dBodyDisable(t);
cout << "Tetra" << endl;
  geomTetra = CreateConvexFromFVP(world, space, DENSITY, &fvpTetra);
  dBodyID b = dGeomGetBody(geomTetra);
  dBodySetPosition(b, 0.0, 1.5, 0.5);
  dBodyEnable(b);
cout << "TmCube" << endl;
  geomTmCube = CreateTrimeshFromVI(world, space, DENSITY, &tmvCube);
  dBodyID e = dGeomGetBody(geomTmCube);
  dBodySetPosition(e, -1.5, -3.0, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(e, q);
  }
  dBodyEnable(e);
cout << "Cube" << endl;
  geomCube = CreateConvexFromFVP(world, space, DENSITY, &fvpCube);
  dBodyID c = dGeomGetBody(geomCube);
  dBodySetPosition(c, -1.5, -1.5, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(c, q);
  }
  dBodyEnable(c);
cout << "TmIcosahedron" << endl;
  geomTmIcosahedron = CreateTrimeshFromVI(world, space, DENSITY, &tmvIcosahedron);
  dBodyID h = dGeomGetBody(geomTmIcosahedron);
  dBodySetPosition(h, -1.5, 3.0, 0.5);
  dBodyEnable(h);
cout << "Icosahedron" << endl;
  geomIcosahedron = CreateConvexFromFVP(world, space, DENSITY, &fvpIcosahedron);
  dBodyID i = dGeomGetBody(geomIcosahedron);
  dBodySetPosition(i, -1.5, 1.5, 0.5);
  dBodyEnable(i);
cout << "TmBunny" << endl;
  geomTmBunny = CreateTrimeshFromVI(world, space, DENSITY, &tmvBunny);
  dBodyID m = dGeomGetBody(geomTmBunny);
  dBodySetPosition(m, 0.0, 0.25, 0.88); // to (-0.109884, 0.304591, 1.217693)
  dQuaternion q;
  dQSetIdentity(q);
  dMatrix3 rot;
  dRSetIdentity(rot);
#if 0
#if 0
  dRFromAxisAndAngle(rot, 1, 0, 0, M_PI / 2);
#else
  dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
  dRfromQ(rot, q);
#endif
  dBodySetRotation(m, rot);
#else
#if 0
  dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
#else
  dRFromEulerAngles(rot, -M_PI / 2, 0, 0); // phi=-x, theta=-y, psi=-z
  dQfromR(q, rot);
#endif
  dBodySetQuaternion(m, q);
#endif
  dBodyEnable(m); // dBodyDisable(m);
cout << "Bunny" << endl;
  geomBunny = CreateConvexFromFVP(world, space, DENSITY, &fvpBunny);
  dBodyID r = dGeomGetBody(geomBunny);
  dBodySetPosition(r, -3.0, -1.5, 2.0);
  dBodyEnable(r);
cout << "TmCustom" << endl;
  geomTmCustom = CreateTrimeshFromVI(world, space, DENSITY, &tmvCustom);
  dBodyID d = dGeomGetBody(geomTmCustom);
  dBodySetPosition(d, -3.0, 3.0, 0.5);
  dBodyEnable(d);
cout << "Custom" << endl;
  geomCustom = CreateConvexFromFVP(world, space, DENSITY, &fvpCustom);
  dBodyID o = dGeomGetBody(geomCustom);
  dBodySetPosition(o, -3.0, 1.5, 0.5);
  dBodyEnable(o);
}

void DrawObjects()
{
  dsSetTexture(DS_WOOD); // DS_SKY DS_GROUND DS_CHECKERED

  DrawGeom(apple.geom, NULL, NULL, apple.colour, wire_solid);
  DrawGeom(ball.geom, NULL, NULL, ball.colour, wire_solid);
  DrawGeom(roll.geom, NULL, NULL, roll.colour, wire_solid);

  for(int j = 0; j < slopeNC; ++j)
    DrawGeom(geomSlope[j][0], NULL, NULL, palette[3], wire_solid);

  DrawGeom(geomTmTetra, NULL, NULL, palette[4], wire_solid);
  DrawConvexObject(geomTetra, &fvpTetra, palette[5]);
  DrawGeom(geomTmCube, NULL, NULL, palette[6], wire_solid);
  DrawConvexObject(geomCube, &fvpCube, palette[7]);
  DrawGeom(geomTmIcosahedron, NULL, NULL, palette[8], wire_solid);
  DrawConvexObject(geomIcosahedron, &fvpIcosahedron, palette[9]);
  DrawGeom(geomTmBunny, NULL, NULL, palette[10], wire_solid);
  DrawConvexObject(geomBunny, &fvpBunny, palette[11]);
  DrawGeom(geomTmCustom, NULL, NULL, palette[12], wire_solid);
  DrawConvexObject(geomCustom, &fvpCustom, palette[13]);
}

void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot,
  const dReal *colour, int ws)
{
  if(!geom) return;
  if(!pos) pos = dGeomGetPosition(geom);
  if(!rot) rot = dGeomGetRotation(geom);
  if(colour) dsSetColor(colour[0], colour[1], colour[2]);
  switch(dGeomGetClass(geom)){
  case dSphereClass: {
    dsDrawSphereD(pos, rot, dGeomSphereGetRadius(geom));
  } break;
  case dBoxClass: {
    dVector3 lxyz;
    dGeomBoxGetLengths(geom, lxyz);
    dsDrawBoxD(pos, rot, lxyz);
  } break;
  case dCylinderClass: {
    dReal len, radius;
    dGeomCylinderGetParams(geom, &radius, &len);
    dsDrawCylinderD(pos, rot, len, radius);
  } break;
  case dCapsuleClass: {
    dReal len, radius;
    dGeomCapsuleGetParams(geom, &radius, &len);
    dsDrawCapsuleD(pos, rot, len, radius);
  } break;
  case dTriMeshClass: {
    dVector3 tpos = {0.0, 0.0, 0.0};
    dMatrix3 trot;
    dRSetIdentity(trot);
    int triCount = dGeomTriMeshGetTriangleCount(geom);
    for(int i = 0; i < triCount; ++i){
      dVector3 v0, v1, v2;
      dGeomTriMeshGetTriangle(geom, i, &v0, &v1, &v2); // already transformed
      dsDrawTriangleD(tpos, trot, v0, v1, v2, ws);
    }
  } break;
  case dGeomTransformClass: {
    dGeomID gt = dGeomTransformGetGeom(geom);
    const dReal *gtpos = dGeomGetPosition(gt);
    const dReal *gtrot = dGeomGetRotation(gt);
    dVector3 rpos;
    dMatrix3 rrot;
    dMULTIPLY0_331(rpos, rot, gtpos);
    for(int i = 0; i < A_SIZE(rpos); ++i) rpos[i] += pos[i];
    dMULTIPLY0_333(rrot, rot, gtrot);
    DrawGeom(gt, rpos, rrot, colour, ws);
  } break;
  default: printf("not implemented type dGeomGetClass() in DrawGeom\n"); break;
  }
}

void CreateSphere(struct sphere *s,
  dWorldID world, dReal r, dReal m, dReal bounce, const dReal *colour)
{
  s->body = dBodyCreate(world);
  s->r = r, s->m = m;
  s->gBounce = bounce;
  for(int i = 0; i < 4; ++i) s->colour[i] = colour[i];
  dMass mass;
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, m, r);
  dBodySetMass(s->body, &mass);
  s->geom = dCreateSphere(space, r);
  dGeomSetBody(s->geom, s->body);
}

dReal getgBounce(dGeomID id)
{
  if(id == apple.geom) return apple.gBounce;
  if(id == ball.geom) return ball.gBounce;
  return 0.9; // 1.0;
}

void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = MAX_COLLIDE_NUM;
  dContact contact[N]; // contact points
  int isGround = ((ground == o1) || (ground == o2));
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  if(isGround){
    dGeomID id = ground == o1 ? o2 : o1;
    dReal bounce = getgBounce(id);
    for(int i = 0; i < n; ++i){
      dContact *p = &contact[i];
      p->surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
      p->surface.bounce = bounce;
      p->surface.bounce_vel = 0.0; // minimum velocity for bounce
      p->surface.mu = 0.5; // or dInfinity
      p->surface.soft_erp = 0.2;
      p->surface.soft_cfm = 0.001;
      dJointID c = dJointCreateContact(world, contactgroup, p);
      // dJointAttach(c, dGeomGetBody(p->geom.g1), dGeomGetBody(p->geom.g2));
      dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
    }
  }else{
    dReal bounce = getgBounce(o1) * getgBounce(o2);
    for(int i = 0; i < n; i++){
      dContact *p = &contact[i];
      p->surface.mode = dContactBounce;
      p->surface.bounce = bounce;
      p->surface.bounce_vel = 0.01; // minimum velocity for bounce
      p->surface.mu = 0.5; // or dInfinity
      dJointID c = dJointCreateContact(world, contactgroup, p);
      dJointAttach(c, dGeomGetBody(p->geom.g1), dGeomGetBody(p->geom.g2));
      // dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
    }
  }
}

void showViewPoint(int save)
{
  float p[3], l_hpr[3];
  dsGetViewpoint(p, l_hpr);
  printf("view point: %d\n", sw_viewpoint);
  printf(" xyz(%f, %f, %f)\n", p[0], p[1], p[2]);
  printf(" hpr(%f, %f, %f)\n", l_hpr[0], l_hpr[1], l_hpr[2]);
  if(save){
    for(int i = 0; i < 3; ++i){
      xyz[sw_viewpoint][i] = p[i];
      hpr[sw_viewpoint][i] = l_hpr[i];
    }
    printf(" saved\n");
  }
}

void command(int cmd)
{
  dBodyID ba = apple.body;
  const dReal *apos = dBodyGetPosition(ba);
  dReal ax = apos[0], ay = apos[1], az = apos[2];
  switch(cmd){
  case ' ':
    if(dBodyIsEnabled(ba)) dBodyDisable(ba);
    else dBodyEnable(ba);
    break;
  case 'u': dBodySetPosition(ba, ax - move_delta, ay, az); break;
  case 'n': dBodySetPosition(ba, ax + move_delta, ay, az); break;
  case 'h': dBodySetPosition(ba, ax, ay - move_delta, az); break;
  case 'l': dBodySetPosition(ba, ax, ay + move_delta, az); break;
  case 'j': dBodySetPosition(ba, ax, ay, az - move_delta); break;
  case 'k': dBodySetPosition(ba, ax, ay, az + move_delta); break;
  case 'm': printf("move_delta = %f\n", move_delta *= 0.9); break;
  case 'i': printf("move_delta = %f\n", move_delta *= 1.1); break;
  case 't': {
    dBodySetTorque(dGeomGetBody(apple.geom), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(ball.geom), 0.0, 0.0, 0.5);

    dBodySetTorque(dGeomGetBody(roll.geom), -0.5, 0.0, 0.0);

    dBodySetTorque(dGeomGetBody(geomTmTetra), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomTetra), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomTmCube), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomCube), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomTmIcosahedron), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomIcosahedron), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomTmBunny), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomBunny), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomTmCustom), 0.0, 0.0, 0.5);
    dBodySetTorque(dGeomGetBody(geomCustom), 0.0, 0.0, 0.5);
  } break;
  case 'o': {
    dBodyID b = dGeomGetBody(apple.geom);
    const dReal *p = dBodyGetPosition(b);
    printf("sphere red (%f, %f, %f)\n", p[0], p[1], p[2]);
  } break;
  case 'p': dsSetDrawMode(polyfill_wireframe = 1 - polyfill_wireframe); break;
  case 'w': wire_solid = 1 - wire_solid; break;
  case 'v': showViewPoint(0); break;
  case 's': {
    showViewPoint(1);
    sw_viewpoint = (sw_viewpoint + 1) % A_SIZE(hpr);
    dsSetViewpoint(xyz[sw_viewpoint], hpr[sw_viewpoint]); // set camera
    showViewPoint(0);
  } break;
  case 'r':
    DestroyObjects();
    dJointGroupDestroy(contactgroup);
    contactgroup = dJointGroupCreate(0);
    setParameters();
    CreateObjects(world);
    break;
  default:
    break;
  }
}

void setParameters()
{
  printf("' ': drop sphere red\n");
  printf("u: --x\n");
  printf("n: ++x\n");
  printf("h: --y\n");
  printf("l: ++y\n");
  printf("j: --z\n");
  printf("k: ++z\n");
  printf("m: --move_delta\n");
  printf("i: ++move_delta\n");
  printf("t: set torque\n");
  printf("o: show sphere red location(x, y, z)\n");
  printf("p: dsSetDrawMode(polyfill_wireframe) for all\n");
  printf("w: dsDrawTriangle(..., wire_solid) for bunny\n");
  printf("v: show view point (x, y, z) (hpr)\n");
  printf("s: switch view point\n");
  printf("r: reset all objects\n");
}

void simLoop(int pause)
{
  if(!pause){
    dSpaceCollide(space, 0, nearCallback);
    // skip // append fource
    // skip // append torque
    // skip // append joint (contact) parameters
    dWorldStep(world, TICK_DELTA);
    dJointGroupEmpty(contactgroup);
  }
  DrawObjects();
}

void drawStuffStart()
{
  dsSetViewpoint(xyz[sw_viewpoint], hpr[sw_viewpoint]); // set camera
  showViewPoint(0);
  dsSetSphereQuality(3); // default sphere 1
  dsSetCapsuleQuality(3); // default capsule 3
}

void setDrawStuff(dsFunctions *fn)
{
  fn->version = DS_VERSION;
  fn->start = drawStuffStart;
  fn->step = simLoop;
  fn->command = command;
  fn->stop = NULL;
  fn->path_to_textures = TEX_RES; // utf-8
}

int main(int ac, char **av)
{
  dInitODE();
  world = dWorldCreate();
  dWorldSetGravity(world, 0, 0, -0.2); // -9.8
  space = dHashSpaceCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);

  contactgroup = dJointGroupCreate(0);
  setParameters();
  CreateObjects(world);

  dsFunctions fn;
  setDrawStuff(&fn);
  dsSimulationLoop(ac, av, WIDTH, HEIGHT, &fn);

  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
