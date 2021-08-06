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

float move_delta = 0.1; // delta for x, y, z move
int wire_solid = 1; // 0: wireframe, 1: solid (for bunny)
int polyfill_wireframe = 0; // 0: solid, 1: wireframe (for all)
dsFunctions fn; // must be global
dWorldID world; // for body
dSpaceID space; // for geom collision
dGeomID ground;
dJointGroupID contactgroup;

extern trimeshvi tmvTetra;
extern convexfvp fvpTetra;

extern trimeshvi tmvCube; // not implemented (now tetra)
extern convexfvp fvpCube;

extern trimeshvi tmvIcosahedron; // not implemented (now tetra)
extern convexfvp fvpIcosahedron;

extern trimeshvi tmvBunny;
extern convexfvp fvpBunny; // body drawing by fvpBunny causes rocky

extern trimeshvi tmvCustom; // not implemented (now tetra)
extern convexfvp fvpCustom; // not implemented (now tetra)

dGeomID geomTmTetra;
dGeomID geomTetra;
dGeomID geomCube;
dGeomID geomIcosahedron;
dGeomID geomTmBunny;
dGeomID geomBunny;
dGeomID geomCustom;

struct sphere {
  dBodyID body;
  dGeomID geom;
  dReal r, m;
  dReal R, G, B;
  dReal gBounce;
};

struct sphere apple, ball;

void DestroyObject(dGeomID geom);
void DestroyObjects();
void CreateObjects(dWorldID w);
void DrawObjects();

void CreateSphere(struct sphere *s,
  dWorldID world, dReal r, dReal m, dReal bounce, dReal R, dReal G, dReal B);
void DrawSphere(struct sphere *s);

dReal getgBounce(dGeomID id);
void nearCallback(void *data, dGeomID o1, dGeomID o2);

void command(int cmd);
void setParameters();
void simLoop(int pause);

void drawStuffStart();
void setDrawStuff(dsFunctions *fn);

void DestroyObject(dGeomID geom)
{
  dBodyDestroy(dGeomGetBody(geom));
  dGeomDestroy(geom);
}

void DestroyObjects()
{
  DestroyObject(apple.geom);
  DestroyObject(ball.geom);

  DestroyObject(geomTmTetra);
  DestroyObject(geomTetra);
  DestroyObject(geomCube);
  DestroyObject(geomIcosahedron);
  DestroyObject(geomTmBunny);
  DestroyObject(geomBunny);
  DestroyObject(geomCustom);
}

void CreateObjects(dWorldID world)
{
cout << "Sphere red" << endl;
  CreateSphere(&apple, world, 0.2, 1.0, 1.0, 0.8, 0.4, 0.4);
  dBodySetPosition(apple.body, -0.15, 0.31, 2.5); // x, y on the bunny
cout << "Sphere blue" << endl;
  CreateSphere(&ball, world, 0.1, 1.0, 0.5, 0.4, 0.4, 0.8);
  dBodySetPosition(ball.body, 0.5, 0.0, ball.r);
  dBodyDisable(apple.body);
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
cout << "Cube" << endl;
  geomCube = CreateConvexFromFVP(world, space, DENSITY, &fvpCube);
  dBodyID c = dGeomGetBody(geomCube);
  dBodySetPosition(c, -1.5, -1.5, 0.5);
  dBodyEnable(c);
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
cout << "Custom" << endl;
  geomCustom = CreateConvexFromFVP(world, space, DENSITY, &fvpCustom);
  dBodyID o = dGeomGetBody(geomCustom);
  dBodySetPosition(o, -3.0, 1.5, 0.5);
  dBodyEnable(o);
}

void DrawObjects()
{
  DrawSphere(&apple);
  DrawSphere(&ball);

  DrawTrimeshObject(geomTmTetra, &tmvTetra, 0.8, 0.6, 0.2, wire_solid);
  DrawConvexObject(geomTetra, &fvpTetra, 0.4, 0.8, 0.4);
  DrawConvexObject(geomCube, &fvpCube, 0.8, 0.8, 0.4);
  DrawConvexObject(geomIcosahedron, &fvpIcosahedron, 0.4, 0.8, 0.8);
  DrawTrimeshObject(geomTmBunny, &tmvBunny, 0.2, 0.8, 0.6, wire_solid);
  DrawConvexObject(geomBunny, &fvpBunny, 0.8, 0.4, 0.8);
  DrawConvexObject(geomCustom, &fvpCustom, 0.2, 0.6, 0.8);
}

void CreateSphere(struct sphere *s,
  dWorldID world, dReal r, dReal m, dReal bounce, dReal R, dReal G, dReal B)
{
  s->body = dBodyCreate(world);
  s->r = r, s->m = m, s->R = R, s->G = G, s->B = B;
  s->gBounce = bounce;
  dMass mass;
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, m, r);
  dBodySetMass(s->body, &mass);
  s->geom = dCreateSphere(space, r);
  dGeomSetBody(s->geom, s->body);
}

void DrawSphere(struct sphere *s)
{
  dsSetColor(s->R, s->G, s->B);
  const dReal *pos = dBodyGetPosition(s->body);
  const dReal *rot = dBodyGetRotation(s->body);
  dsDrawSphereD(pos, rot, s->r);
}

dReal getgBounce(dGeomID id)
{
  if(id == apple.geom) return apple.gBounce;
  if(id == ball.geom) return ball.gBounce;
  return 1.0;
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
      p->surface.soft_erp = 0.2;
      p->surface.soft_cfm = 0.001;
      p->surface.mu = 0.5; // or float('inf') for max;
      p->surface.bounce_vel = 0.0; // minimum velocity for bounce
      dJointID c = dJointCreateContact(world, contactgroup, p);
      dJointAttach(c, dGeomGetBody(p->geom.g1), dGeomGetBody(p->geom.g2));
    }
  }else{
    for(int i = 0; i < n; i++){
      dContact *p = &contact[i];
      p->surface.mode = dContactBounce;
      p->surface.bounce = 1.0;
      p->surface.bounce_vel = 0.01; // minimum velocity for bounce
      dJointID c = dJointCreateContact(world, contactgroup, p);
      dJointAttach(c, dGeomGetBody(p->geom.g1), dGeomGetBody(p->geom.g2));
    }
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
  case 'o': {
    dBodyID b = dGeomGetBody(apple.geom);
    const dReal *p = dBodyGetPosition(b);
    printf("sphere red (%f, %f, %f)\n", p[0], p[1], p[2]);
  } break;
  case 'p': dsSetDrawMode(polyfill_wireframe = 1 - polyfill_wireframe); break;
  case 'w': wire_solid = 1 - wire_solid; break;
  case 'v': {
    float p[3], hpr[3];
    dsGetViewpoint(p, hpr);
    printf("view point xyz(%f, %f, %f)\n", p[0], p[1], p[2]);
    printf("view point hpr(%f, %f, %f)\n", hpr[0], hpr[1], hpr[2]);
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
  printf("o: show sphere red location(x, y, z)\n");
  printf("p: dsSetDrawMode(polyfill_wireframe) for all\n");
  printf("w: dsDrawTriangle(..., wire_solid) for bunny\n");
  printf("v: show view point(x, y, z)\n");
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
  static float xyz[3] = {5.36, 2.02, 4.28}; // camera position {3.0, 0.0, 1.0}
  static float hpr[3] = {-162.0, -31.0, 0.0}; // look at {-180.0, 0.0, 0.0}
  dsSetViewpoint(xyz, hpr); // set camera
  dsSetSphereQuality(3); // default sphere 1
  dsSetCapsuleQuality(3); // default capsule 3
}

void setDrawStuff(dsFunctions *fn)
{
  fn->version = DS_VERSION;
  fn->start = drawStuffStart;
  fn->step = simLoop;
  fn->command = command;
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

  setDrawStuff(&fn);
  dsSimulationLoop(ac, av, WIDTH, HEIGHT, &fn);

  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
