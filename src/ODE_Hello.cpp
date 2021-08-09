/*
  ODE_Hello.cpp

  ODE Open Dynamics Engine sample

  *** IME setting - use old IME ***
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
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

map<dGeomID, convexfvp *> geom_convex_manager;
map<const char *, pair<dBodyID, const dReal *> > geom_body_manager;

int slopeC[] = {dBoxClass, dCylinderClass};
dReal slopeD[] = {DENSITY, DENSITY};
dReal slopeO[][3] = {{0.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}}; // offset
dReal slopeP[][4] = {{8.0, 0.1, 2.0}, {1.0, 2.0}}; // lxyz, RL
dQuaternion slopeQ[] = {{}, {}};
metacomposite slope = {A_SIZE(slopeC), slopeC, slopeD, slopeO, slopeP, slopeQ};

struct sphere {
  dBodyID body;
  dGeomID geom;
  dReal r, m;
  dVector4 colour;
  dReal gBounce;
};

sphere apple, ball, roll;

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
  {1.0, 1.0, 1.0, 0.6} // geomPlane
};

void DestroyObject(dBodyID body);
void DestroyObjects();
void CreateObjects(dWorldID world);
void DrawObjects();
void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot,
  const dReal *colour, int ws); // dVector3, dMatrix3, dVector4

void CreateSphere(sphere *s, dWorldID world, dSpaceID space,
  dReal r, dReal m, dReal bounce, const dReal *colour);

dReal getgBounce(dGeomID id);
void nearCallback(void *data, dGeomID o1, dGeomID o2);

void showViewPoint(int save);
void command(int cmd);
void setParameters();
void simLoop(int pause);

void drawStuffStart();
void setDrawStuff(dsFunctions *fn);

void DestroyObject(dBodyID body)
{
  dGeomID geom = dBodyGetFirstGeom(body);
  while(geom){
    dGeomID nextgeom = dBodyGetNextGeom(geom);
    dGeomDestroy(geom);
    geom = nextgeom;
  }
  dBodyDestroy(body);
}

void DestroyObjects()
{
  for(auto it = geom_body_manager.begin(); it != geom_body_manager.end(); ++it)
    DestroyObject(it->second.first);

  geom_convex_manager.clear();
  geom_body_manager.clear();
}

void CreateObjects(dWorldID world)
{
cout << "Sphere red" << endl;
  CreateSphere(&apple, world, space, 0.2, 1.0, 1.0, palette[0]);
  dBodySetPosition(apple.body, -0.15, 0.31, 2.5); // x, y on the bunny
  dBodyDisable(apple.body);
  geom_body_manager.insert(make_pair("apple", make_pair(apple.body, apple.colour)));
cout << "Sphere blue" << endl;
  CreateSphere(&ball, world, space, 0.1, 1.0, 0.5, palette[1]);
  dBodySetPosition(ball.body, 0.5, 0.0, ball.r);
  geom_body_manager.insert(make_pair("ball", make_pair(ball.body, ball.colour)));
cout << "Sphere green" << endl;
  CreateSphere(&roll, world, space, 0.2, 1.0, 0.8, palette[2]);
  dBodySetPosition(roll.body, -12.0, 0.0, 1.2); // on the slope
  geom_body_manager.insert(make_pair("roll", make_pair(roll.body, roll.colour)));

cout << "Slope" << endl;
  dBodyID s = CreateComposite(world, space, &slope);
  dBodySetPosition(s, -13.5, 0.0, 1.2);
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
    dQFromAxisAndAngle(p, 0, 1, 0, M_PI / 18);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(s, o);
  }
  dBodyEnable(s); // dBodyDisable(s);
  geom_body_manager.insert(make_pair("slope", make_pair(s, palette[3])));

cout << "TmTetra" << endl;
  dGeomID geomTmTetra = CreateTrimeshFromVI(world, space, DENSITY, &tmvTetra);
  dBodyID t = dGeomGetBody(geomTmTetra);
  dBodySetPosition(t, 0.0, -1.5, 0.5);
  dBodyEnable(t); // dBodyDisable(t);
  geom_body_manager.insert(make_pair("tmtetra", make_pair(t, palette[4])));
cout << "Tetra" << endl;
  dGeomID geomTetra = CreateConvexFromFVP(world, space, DENSITY, &fvpTetra);
  geom_convex_manager.insert(make_pair(geomTetra, &fvpTetra));
  dBodyID b = dGeomGetBody(geomTetra);
  dBodySetPosition(b, 0.0, 1.5, 0.5);
  dBodyEnable(b);
  geom_body_manager.insert(make_pair("tetra", make_pair(b, palette[5])));
cout << "TmCube" << endl;
  dGeomID geomTmCube = CreateTrimeshFromVI(world, space, DENSITY, &tmvCube);
  dBodyID e = dGeomGetBody(geomTmCube);
  dBodySetPosition(e, -1.5, -3.0, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(e, q);
  }
  dBodyEnable(e);
  geom_body_manager.insert(make_pair("tmcube", make_pair(e, palette[6])));
cout << "Cube" << endl;
  dGeomID geomCube = CreateConvexFromFVP(world, space, DENSITY, &fvpCube);
  geom_convex_manager.insert(make_pair(geomCube, &fvpCube));
  dBodyID c = dGeomGetBody(geomCube);
  dBodySetPosition(c, -1.5, -1.5, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(c, q);
  }
  dBodyEnable(c);
  geom_body_manager.insert(make_pair("cube", make_pair(c, palette[7])));
cout << "TmIcosahedron" << endl;
  dGeomID geomTmIcosahedron = CreateTrimeshFromVI(world, space, DENSITY, &tmvIcosahedron);
  dBodyID h = dGeomGetBody(geomTmIcosahedron);
  dBodySetPosition(h, -1.5, 3.0, 0.5);
  dBodyEnable(h);
  geom_body_manager.insert(make_pair("tmicosahedron", make_pair(h, palette[8])));
cout << "Icosahedron" << endl;
  dGeomID geomIcosahedron = CreateConvexFromFVP(world, space, DENSITY, &fvpIcosahedron);
  geom_convex_manager.insert(make_pair(geomIcosahedron, &fvpIcosahedron));
  dBodyID i = dGeomGetBody(geomIcosahedron);
  dBodySetPosition(i, -1.5, 1.5, 0.5);
  dBodyEnable(i);
  geom_body_manager.insert(make_pair("icosahedron", make_pair(i, palette[9])));
cout << "TmBunny" << endl;
  dGeomID geomTmBunny = CreateTrimeshFromVI(world, space, DENSITY, &tmvBunny);
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
  geom_body_manager.insert(make_pair("tmbunny", make_pair(m, palette[10])));
cout << "Bunny" << endl;
  dGeomID geomBunny = CreateConvexFromFVP(world, space, DENSITY, &fvpBunny);
  geom_convex_manager.insert(make_pair(geomBunny, &fvpBunny));
  dBodyID r = dGeomGetBody(geomBunny);
  dBodySetPosition(r, -3.0, -1.5, 2.0);
  dBodyEnable(r);
  geom_body_manager.insert(make_pair("bunny", make_pair(r, palette[11])));
cout << "TmCustom" << endl;
  dGeomID geomTmCustom = CreateTrimeshFromVI(world, space, DENSITY, &tmvCustom);
  dBodyID d = dGeomGetBody(geomTmCustom);
  dBodySetPosition(d, -3.0, 3.0, 0.5);
  dBodyEnable(d);
  geom_body_manager.insert(make_pair("tmcustom", make_pair(d, palette[12])));
cout << "Custom" << endl;
  dGeomID geomCustom = CreateConvexFromFVP(world, space, DENSITY, &fvpCustom);
  geom_convex_manager.insert(make_pair(geomCustom, &fvpCustom));
  dBodyID o = dGeomGetBody(geomCustom);
  dBodySetPosition(o, -3.0, 1.5, 0.5);
  dBodyEnable(o);
  geom_body_manager.insert(make_pair("custom", make_pair(o, palette[13])));
cout << "Plane" << endl;
  if(1){
    dGeomID geomPlane = dCreatePlane(space, 0, 0, 1, 0);
    dBodyID p = dBodyCreate(world);
    dMass mass;
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, 1.0, 10.0, 10.0, 0.05);
    dBodySetMass(p, &mass);
    dGeomSetBody(geomPlane, p);
    dBodySetPosition(p, 0.0, 0.0, 0.05);
    dMatrix3 rot;
    dRSetIdentity(rot);
    // dRFromAxisAndAngle(rot, 1, 0, 0, M_PI / 2);
    dBodySetRotation(p, rot);
    dBodyEnable(p);
    geom_body_manager.insert(make_pair("plane", make_pair(p, palette[14])));
  }
}

void DrawObjects()
{
  dsSetTexture(DS_WOOD); // DS_SKY DS_GROUND DS_CHECKERED

  for(auto it = geom_body_manager.begin(); it != geom_body_manager.end(); ++it)
    for(dGeomID g = dBodyGetFirstGeom(it->second.first); g; g = dBodyGetNextGeom(g))
      DrawGeom(g, NULL, NULL, it->second.second, wire_solid);
}

void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot,
  const dReal *colour, int ws)
{
  if(!geom) return;
  if(!pos) pos = dGeomGetPosition(geom);
  if(!rot) rot = dGeomGetRotation(geom);
#if 0
  if(colour) dsSetColor(colour[0], colour[1], colour[2]);
#else
  if(colour) dsSetColorAlpha(colour[0], colour[1], colour[2], colour[3]);
#endif
  int clsID = dGeomGetClass(geom);
  switch(clsID){
  case dSphereClass: { // 0
    dsDrawSphereD(pos, rot, dGeomSphereGetRadius(geom));
  } break;
  case dBoxClass: { // 1
    dVector3 lxyz;
    dGeomBoxGetLengths(geom, lxyz);
    dsDrawBoxD(pos, rot, lxyz);
  } break;
  case dCylinderClass: { // 3
    dReal len, radius;
    dGeomCylinderGetParams(geom, &radius, &len);
    dsDrawCylinderD(pos, rot, len, radius);
  } break;
  case dCapsuleClass: { // 2
    dReal len, radius;
    dGeomCapsuleGetParams(geom, &radius, &len);
    dsDrawCapsuleD(pos, rot, len, radius);
  } break;
  case dConvexClass: { // 6
    convexfvp *fvp = geom_convex_manager[geom];
    if(!fvp) printf("not managed convex in DrawGeom (geomID: %p)\n", geom);
    else dsDrawConvexD(pos, rot,
      fvp->faces, fvp->faceCount, fvp->vtx, fvp->vtxCount, fvp->polygons);
  } break;
  case dTriMeshClass: { // 8
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
  case dGeomTransformClass: { // 7
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
  case dPlaneClass: { // 4
    dVector4 v;
    dGeomPlaneGetParams(geom, v);
    dVector3 lxyz = {10.0, 10.0, 0.05};
    dsDrawBoxD(pos, rot, lxyz);
  } break;
#if 0
  case dRayClass: { // 5
  } break;
  case dHeightfieldClass: { // 9
  } break;
#endif
  default:
    printf("not implemented type dGeomGetClass() in DrawGeom\n");
    printf(" geomID: %p, classID: %d\n", geom, clsID);
    break;
  }
}

void CreateSphere(sphere *s, dWorldID world, dSpaceID space,
  dReal r, dReal m, dReal bounce, const dReal *colour)
{
  s->body = dBodyCreate(world);
  s->r = r, s->m = m;
  s->gBounce = bounce;
  for(int i = 0; i < A_SIZE(s->colour); ++i) s->colour[i] = colour[i];
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
    char *torqueX[] = {"apple", "roll",
      "cube", "icosahedron", "custom"};
    for(int i = 0; i < A_SIZE(torqueX); ++i)
      dBodySetTorque(geom_body_manager[torqueX[i]].first, -0.5, 0.0, 0.0);
    char *torqueZ[] = {"ball", "tmbunny", "bunny", "tmtetra", "tetra",
      "tmcube", "tmicosahedron", "tmcustom"};
    for(int i = 0; i < A_SIZE(torqueZ); ++i)
      dBodySetTorque(geom_body_manager[torqueZ[i]].first, 0.0, 0.0, 0.5);
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
