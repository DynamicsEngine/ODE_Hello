/*
  ODE_Hello.cpp

  ODE Open Dynamics Engine sample

  *** IME setting - use old IME ***
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <geommanager.h>
#include <gencomposite.h>

#include <iostream>
#include <map>

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

dVector4 palette[] = {
  {0.8, 0.4, 0.4, 1.0}, // apple
  {0.4, 0.4, 0.8, 1.0}, // ball
  {0.4, 0.8, 0.4, 1.0}, // roll
  {1.0, 0.8, 0.2, 0.6}, // geomSlope
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
  {0.2, 1.0, 0.8, 0.4} // geomPlane
};

int slopeC[] = {dBoxClass, dCylinderClass};
dReal slopeD[] = {DENSITY, DENSITY};
dReal slopeO[][3] = {{0.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}}; // offset
dReal slopeP[][4] = {{6.0, 0.1, 8.0}, {1.0, 2.0}}; // lxyz, RL
dQuaternion slopeQ[] = {{}, {}};
void *slopeV[] = {NULL, NULL};
dReal *slopeA[] = {palette[3], palette[14]};
metacomposite slope = {
  A_SIZE(slopeC), slopeC, slopeD, slopeO, slopeP, slopeQ, slopeV, slopeA};

int uballC[] = {dSphereClass, dSphereClass};
dReal uballD[] = {DENSITY, DENSITY};
dReal uballO[][3] = {{0.0, 0.0, 0.0}, {0.05, 0.0, 0.0}}; // offset
dReal uballP[][4] = {{0.2}, {0.1}}; // Rout, Rin
dQuaternion uballQ[] = {{}, {}};
void *uballV[] = {NULL, NULL};
dReal *uballA[] = {palette[14], palette[3]};
metacomposite uball = {
  A_SIZE(uballC), uballC, uballD, uballO, uballP, uballQ, uballV, uballA};

metasphere apple = {0.2, 1.0, 1.0, palette[0]};
metasphere ball = {0.1, 1.0, 0.5, palette[1]};
metasphere roll = {0.2, 1.0, 0.8, palette[2]};

void CreateObjects(dWorldID world);

dReal getgBounce(dGeomID id);
void nearCallback(void *data, dGeomID o1, dGeomID o2);

void showViewPoint(int save);
void command(int cmd);
void setParameters();
void simLoop(int pause);

void drawStuffStart();
void setDrawStuff(dsFunctions *fn);

void CreateObjects(dWorldID world)
{
cout << "Sphere red" << endl;
  dBodyID ba = CreateSphere(world, space, "apple", &apple);
  dBodySetPosition(ba, -0.15, 0.31, 2.5); // x, y on the bunny
  dBodyDisable(ba);
cout << "Sphere blue" << endl;
  dBodyID bb = CreateSphere(world, space, "ball", &ball);
  dBodySetPosition(bb, 0.5, 0.0, ball.r);
cout << "Sphere green" << endl;
  dBodyID br = CreateSphere(world, space, "roll", &roll);
  dBodySetPosition(br, -12.0, 0.0, 1.2); // on the slope

cout << "Slope" << endl;
  dBodyID s = CreateComposite(world, space, "slope", &slope);
  dBodySetPosition(s, -13.5, 0.0, 1.2);
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
    dQFromAxisAndAngle(p, 0, 1, 0, M_PI / 18);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(s, o);
  }
  dBodyEnable(s); // dBodyDisable(s);

cout << "U ball" << endl;
  dBodyID u = CreateComposite(world, space, "uball", &uball);
  dBodySetPosition(u, -12.0, 1.0, 1.2); // on the slope
  dBodyEnable(u);
cout << "LU ball" << endl;
  dBodyID lu = CreateComposite(world, space, "luball", &uball);
  dBodySetPosition(lu, -12.0, 2.0, 1.2); // on the slope
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI / 2);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(lu, o);
  }
  dBodyEnable(lu);
cout << "RU ball" << endl;
  dBodyID ru = CreateComposite(world, space, "ruball", &uball);
  dBodySetPosition(ru, -12.0, -1.0, 1.2); // on the slope
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 0, 0, 1, -M_PI / 2);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(ru, o);
  }
  dBodyEnable(ru);

cout << "TmTetra" << endl;
  metatrimesh mttetra = {DENSITY, &tmvTetra, palette[4]};
  dBodyID t = CreateTrimeshFromVI(world, space, "tmtetra", &mttetra);
  dBodySetPosition(t, 0.0, -1.5, 0.5);
  dBodyEnable(t); // dBodyDisable(t);
cout << "Tetra" << endl;
  metaconvex mctetra = {DENSITY, &fvpTetra, palette[5]};
  dBodyID b = CreateConvexFromFVP(world, space, "tetra", &mctetra);
  dBodySetPosition(b, 0.0, 1.5, 0.5);
  dBodyEnable(b);
cout << "TmCube" << endl;
  metatrimesh mtcube = {DENSITY, &tmvCube, palette[6]};
  dBodyID e = CreateTrimeshFromVI(world, space, "tmcube", &mtcube);
  dBodySetPosition(e, -1.5, -3.0, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(e, q);
  }
  dBodyEnable(e);
cout << "Cube" << endl;
  metaconvex mccube = {DENSITY, &fvpCube, palette[7]};
  dBodyID c = CreateConvexFromFVP(world, space, "cube", &mccube);
  dBodySetPosition(c, -1.5, -1.5, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(c, q);
  }
  dBodyEnable(c);
cout << "TmIcosahedron" << endl;
  metatrimesh mticosahedron = {DENSITY, &tmvIcosahedron, palette[8]};
  dBodyID h = CreateTrimeshFromVI(world, space, "tmicosahedron", &mticosahedron);
  dBodySetPosition(h, -1.5, 3.0, 0.5);
  dBodyEnable(h);
cout << "Icosahedron" << endl;
  metaconvex mcicosahedron = {DENSITY, &fvpIcosahedron, palette[9]};
  dBodyID i = CreateConvexFromFVP(world, space, "icosahedron", &mcicosahedron);
  dBodySetPosition(i, -1.5, 1.5, 0.5);
  dBodyEnable(i);
cout << "TmBunny" << endl;
  metatrimesh mtbunny = {DENSITY, &tmvBunny, palette[10]};
  dBodyID m = CreateTrimeshFromVI(world, space, "tmbunny", &mtbunny);
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
  metaconvex mcbunny = {DENSITY, &fvpBunny, palette[11]};
  dBodyID r = CreateConvexFromFVP(world, space, "bunny", &mcbunny);
  dBodySetPosition(r, -3.0, -1.5, 2.0);
  dBodyEnable(r);
cout << "TmCustom" << endl;
  metatrimesh mtcustom = {DENSITY, &tmvCustom, palette[12]};
  dBodyID d = CreateTrimeshFromVI(world, space, "tmcustom", &mtcustom);
  dBodySetPosition(d, -3.0, 3.0, 0.5);
  dBodyEnable(d);
cout << "Custom" << endl;
  metaconvex mccustom = {DENSITY, &fvpCustom, palette[13]};
  dBodyID o = CreateConvexFromFVP(world, space, "custom", &mccustom);
  dBodySetPosition(o, -3.0, 1.5, 0.5);
  dBodyEnable(o);
cout << "Plane" << endl;
  if(0){
    metaplane plane = {{0, 0, 1, 0}, {10.0, 10.0, 0.05}, DENSITY, palette[14]};
    dBodyID p = CreatePlane(world, space, "plane", &plane);
    dBodySetPosition(p, 0.0, 0.0, 0.05);
    dMatrix3 rot;
    dRSetIdentity(rot);
    // dRFromAxisAndAngle(rot, 1, 0, 0, M_PI / 2);
    dBodySetRotation(p, rot);
    dBodyEnable(p);
  }
}

dReal getgBounce(dGeomID id)
{
  if(id == dBodyGetFirstGeom(FindBody("apple"))) return apple.gBounce;
  if(id == dBodyGetFirstGeom(FindBody("ball"))) return ball.gBounce;
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
      // dJointAttach(c, dGeomGetBody(p->geom.g1), dGeomGetBody(p->geom.g2));
      dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
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
  dBodyID ba = FindBody("apple");
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
    char *torqueX[] = {"apple", "roll",
      "cube", "icosahedron", "custom"};
    for(int i = 0; i < A_SIZE(torqueX); ++i)
      dBodySetTorque(FindBody(torqueX[i]), -0.5, 0.0, 0.0);
    char *torqueZ[] = {"ball", "tmbunny", "bunny", "tmtetra", "tetra",
      "tmcube", "tmicosahedron", "tmcustom"};
    for(int i = 0; i < A_SIZE(torqueZ); ++i)
      dBodySetTorque(FindBody(torqueZ[i]), 0.0, 0.0, 0.5);
  } break;
  case 'o': {
    dBodyID b = FindBody("apple");
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
  DrawObjects(wire_solid);
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
