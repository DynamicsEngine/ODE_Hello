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
#include <memory>

using namespace std;

#define TEX_RES "./textures"

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
  {1.0, 0.8, 0.2, 0.6}, // slope(Box), tmball(TriMesh)
  {0.8, 0.6, 0.2, 1.0}, // tmtetra
  {0.4, 0.8, 0.4, 1.0}, // tetra
  {0.6, 0.8, 0.2, 1.0}, // tmtube
  {0.8, 0.8, 0.4, 1.0}, // cube
  {0.2, 0.8, 0.6, 1.0}, // tmicosahedron
  {0.4, 0.8, 0.8, 1.0}, // icosahedron
  {0.8, 0.2, 0.6, 1.0}, // tmbunny
  {0.8, 0.4, 0.8, 1.0}, // bunny
  {0.6, 0.2, 0.8, 1.0}, // tmcustom
  {0.2, 0.6, 0.8, 1.0}, // custom
  {0.6, 0.2, 0.8, 0.6}, // uball in Sphere
  {0.2, 0.8, 0.6, 0.4}, // uball out Sphere
  {0.6, 0.2, 0.8, 0.6}, // luball in Sphere
  {0.2, 0.8, 0.6, 0.4}, // luball out Sphere
  {0.6, 0.2, 0.8, 0.6}, // ruball in Sphere
  {0.2, 0.8, 0.6, 0.4}, // ruball out Sphere
  {0.6, 0.2, 0.8, 0.6}, // vball in Box
  {0.8, 0.2, 0.6, 0.6}, // vball in Capsule
  {0.2, 0.8, 0.6, 0.4}, // vball out Sphere
  {0.6, 0.2, 0.8, 0.6}, // lvball in Box
  {0.8, 0.2, 0.6, 0.6}, // lvball in Capsule
  {0.2, 0.8, 0.6, 0.4}, // lvball out Sphere
  {0.6, 0.2, 0.8, 0.6}, // rvball in Box
  {0.8, 0.2, 0.6, 0.6}, // rvball in Capsule
  {0.2, 0.8, 0.6, 0.4}, // rvball out Sphere
  {0.2, 1.0, 0.8, 0.4} // plane, slope(Cylinder), tmball(Sphere)
};

cmaterial material[A_SIZE(palette)][1]; // a[][1] to use like array of pointer

metatrimesh mtbunny = {DENSITY, &tmvBunny, material[10]};
shared_ptr<metatrimesh> mtbunny2;
shared_ptr<metatrimesh> mtbunny3;
metaconvex mcbunny = {DENSITY, &fvpBunny, material[11]};
shared_ptr<metaconvex> mcbunny2;
shared_ptr<metaconvex> mcbunny3;

metacomposite tmball[] = { // trimesh, Rout
  {dTriMeshClass, DENSITY * 0.1, {0, 0, 0}, {}, QI, &tmvBunny, material[3]},
  {dSphereClass, DENSITY * 0.1, {0, 0, 0}, {1.2}, QI, NULL, material[29]}};

metacomposite slope[] = { // lxyz, RL
  {dBoxClass, DENSITY, {0, 0, 0}, {6.0, 0.1, 8.0}, QI, NULL, material[3]},
  {dCylinderClass, DENSITY, {-3.0, 0, 0}, {1.0, 2.0}, QI, NULL, material[29]}};

metasphere apple = {0.2, 1.0, 1.0, material[0]};
metasphere ball = {0.1, 1.0, 0.5, material[1]};
metasphere roll = {0.2, 1.0, 0.8, material[2]};

metacomposite uball[] = { // Rin, Rout
  {dSphereClass, DENSITY, {0.05, 0, 0}, {0.1}, QI, NULL, material[14]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[15]}};
metacomposite luball[] = { // Rin, Rout
  {dSphereClass, DENSITY, {0.05, 0, 0}, {0.1}, QI, NULL, material[16]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[17]}};
metacomposite ruball[] = { // Rin, Rout
  {dSphereClass, DENSITY, {0.05, 0, 0}, {0.1}, QI, NULL, material[18]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[19]}};

metacomposite vball[] = { // lxyz, RL, Rout
  {dBoxClass, DENSITY, {0.05, 0, 0}, {0.2, 0.1, 0.1}, QI, NULL, material[20]},
  {dCapsuleClass, DENSITY, {0, 0, 0.03}, {0.05, 0.2}, QI, NULL, material[21]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[22]}};
metacomposite lvball[] = { // lxyz, RL, Rout
  {dBoxClass, DENSITY, {0.05, 0, 0}, {0.2, 0.1, 0.1}, QI, NULL, material[23]},
  {dCapsuleClass, DENSITY, {0, 0, 0.03}, {0.05, 0.2}, QI, NULL, material[24]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[25]}};
metacomposite rvball[] = { // lxyz, RL, Rout
  {dBoxClass, DENSITY, {0.05, 0, 0}, {0.2, 0.1, 0.1}, QI, NULL, material[26]},
  {dCapsuleClass, DENSITY, {0, 0, 0.03}, {0.05, 0.2}, QI, NULL, material[27]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[28]}};

metacomposite ihball[] = { // 4 * lxyz, Rout (Radius of Gyration) RG > (slow)
  {dBoxClass, 100.0, {0.09, 0, 0}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dBoxClass, 100.0, {0, 0, 0.09}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dBoxClass, 100.0, {-0.09, 0, 0}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dBoxClass, 100.0, {0, 0, -0.09}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[29]}};

metacomposite iiball[] = { // 4 * lxyz, Rout (Radius of Gyration) RG < (fast)
  {dBoxClass, 100.0, {0.05, 0, 0}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dBoxClass, 100.0, {0, 0, 0.05}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dBoxClass, 100.0, {-0.05, 0, 0}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dBoxClass, 100.0, {0, 0, -0.05}, {0.1, 0.1, 0.1}, QI, NULL, material[3]},
  {dSphereClass, DENSITY, {0, 0, 0}, {0.2}, QI, NULL, material[29]}};

void CreateObjects(dWorldID world);

dReal getgBounce(dGeomID id);
void nearCallback(void *data, dGeomID o1, dGeomID o2);

void showViewPoint(int save);
void command(int cmd);
void setParameters();
void simLoop(int pause);

void drawStuffStart();
void setDrawStuff(dsFunctions *fn);

void disp_Mass(dBodyID b, const char *n)
{
  dMass m;
  dBodyGetMass(b, &m);
  printf("%s %17.13f(%9.5f %9.5f %9.5f)\n", n, m.mass, m.c[0], m.c[1], m.c[2]);
  printf(" I1k [%17.13f %17.13f %17.13f]\n", m.I[0], m.I[1], m.I[2]);
  printf(" I2k [%17s %17.13f %17.13f]\n", "*", m.I[5], m.I[6]);
  printf(" I3k [%17s %17s %17.13f]\n", "*", "*", m.I[10]);
}

void create_TmBall()
{
  dBodyID b = CreateComposite(world, space, "tmball", tmball, A_SIZE(tmball));
  dBodySetPosition(b, -14.0, -3.0, 3.0);
  if(1){
    dQuaternion o, p, q;
#if 0
    dQFromAxisAndAngle(q, 0, 1, 0, M_PI / 2);
    dQFromAxisAndAngle(p, 1, 0, 0, M_PI / 2);
#else
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
    dQSetIdentity(p);
#endif
    dQMultiply0(o, p, q);
    dBodySetQuaternion(b, o);
  }
  dBodyEnable(b);
  disp_Mass(b, "TmBall");
}

void create_Slope()
{
  dBodyID b = CreateComposite(world, space, "slope", slope, A_SIZE(slope));
  dBodySetPosition(b, -13.5, 0.0, 1.2);
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
    dQFromAxisAndAngle(p, 0, 1, 0, M_PI / 18);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(b, o);
  }
  dBodyEnable(b); // dBodyDisable(s);
  disp_Mass(b, "Slope");
}

void create_SphereApple()
{
  dBodyID b = CreateSphere(world, space, "apple", &apple);
  dBodySetPosition(b, -0.15, 0.31, 2.5); // x, y on the bunny
  dBodyDisable(b);
  disp_Mass(b, "Sphere apple");
}

void create_SphereBall()
{
  dBodyID b = CreateSphere(world, space, "ball", &ball);
  dBodySetPosition(b, 0.5, 0.0, ball.r);
  disp_Mass(b, "Sphere ball");
}

void create_SphereRoll()
{
  dBodyID b = CreateSphere(world, space, "roll", &roll);
  dBodySetPosition(b, -12.0, 0.0, 1.2); // on the slope
  disp_Mass(b, "Sphere roll");
}

void create_UBall()
{
  dBodyID b = CreateComposite(world, space, "uball", uball, A_SIZE(uball));
  dBodySetPosition(b, -12.0, 0.5, 1.2); // on the slope
  dBodyEnable(b);
  disp_Mass(b, "U ball");
}

void create_LUBall()
{
  dBodyID b = CreateComposite(world, space, "luball", luball, A_SIZE(luball));
  dBodySetPosition(b, -12.0, 2.5, 1.2); // on the slope
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI / 2);
    dQSetIdentity(p);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(b, o);
  }
  dBodyEnable(b);
  disp_Mass(b, "LU ball");
}

void create_RUBall()
{
  dBodyID b = CreateComposite(world, space, "ruball", ruball, A_SIZE(ruball));
  dBodySetPosition(b, -12.0, -0.5, 1.2); // on the slope
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 0, 0, 1, -M_PI / 2);
    dQSetIdentity(p);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(b, o);
  }
  dBodyEnable(b);
  disp_Mass(b, "RU ball");
}

void create_VBall()
{
  dBodyID b = CreateComposite(world, space, "vball", vball, A_SIZE(vball));
  dBodySetPosition(b, -12.0, 1.5, 1.2); // on the slope
  dBodyEnable(b);
  disp_Mass(b, "V ball");
}

void create_LVBall()
{
  dBodyID b = CreateComposite(world, space, "lvball", lvball, A_SIZE(lvball));
  dBodySetPosition(b, -12.0, 3.5, 1.2); // on the slope
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI / 2);
    dQSetIdentity(p);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(b, o);
  }
  dBodyEnable(b);
  disp_Mass(b, "LV ball");
}

void create_RVBall()
{
  dBodyID b = CreateComposite(world, space, "rvball", rvball, A_SIZE(rvball));
  dBodySetPosition(b, -12.0, -1.5, 1.2); // on the slope
  if(1){
    dQuaternion o, p, q;
    dQFromAxisAndAngle(q, 0, 0, 1, -M_PI / 2);
    dQSetIdentity(p);
    dQMultiply0(o, p, q);
    dBodySetQuaternion(b, o);
  }
  dBodyEnable(b);
  disp_Mass(b, "RV ball");
}

void create_IHBall()
{
  dBodyID b = CreateComposite(world, space, "ihball", ihball, A_SIZE(ihball));
  dBodySetPosition(b, -12.0, -2.5, 1.2); // on the slope
  dBodyEnable(b);
  disp_Mass(b, "IH ball");
}

void create_IIBall()
{
  dBodyID b = CreateComposite(world, space, "iiball", iiball, A_SIZE(iiball));
  dBodySetPosition(b, -12.0, -3.5, 1.2); // on the slope
  dBodyEnable(b);
  disp_Mass(b, "II ball");
}

void create_TmTetra()
{
  metatrimesh mttetra = {DENSITY, &tmvTetra, material[4]};
  dBodyID b = CreateTrimeshFromVI(world, space, "tmtetra", &mttetra);
  dBodySetPosition(b, 0.0, -1.5, 0.5);
  dBodyEnable(b); // dBodyDisable(t);
  disp_Mass(b, "TmTetra");
}

void create_Tetra()
{
  metaconvex mctetra = {DENSITY, &fvpTetra, material[5]};
  dBodyID b = CreateConvexFromFVP(world, space, "tetra", &mctetra);
  dBodySetPosition(b, 0.0, 1.5, 0.5);
  dBodyEnable(b);
  disp_Mass(b, "Tetra");
}

void create_TmCube()
{
  metatrimesh mtcube = {DENSITY, &tmvCube, material[6]};
  dBodyID b = CreateTrimeshFromVI(world, space, "tmcube", &mtcube);
  dBodySetPosition(b, -1.5, -3.0, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(b, q);
  }
  dBodyEnable(b);
  disp_Mass(b, "TmCube");
}

void create_Cube()
{
  metaconvex mccube = {DENSITY, &fvpCube, material[7]};
  dBodyID b = CreateConvexFromFVP(world, space, "cube", &mccube);
  dBodySetPosition(b, -1.5, -1.5, 0.5);
  if(1){
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 1, 1, M_PI / 4);
    dBodySetQuaternion(b, q);
  }
  dBodyEnable(b);
  disp_Mass(b, "Cube");
}

void create_TmIcosahedron()
{
  metatrimesh mticosahedron = {DENSITY, &tmvIcosahedron, material[8]};
  dBodyID b = CreateTrimeshFromVI(world, space, "tmicosahedron", &mticosahedron);
  dBodySetPosition(b, -1.5, 3.0, 0.5);
  dBodyEnable(b);
  disp_Mass(b, "TmIcosahedron");
}

void create_Icosahedron()
{
  metaconvex mcicosahedron = {DENSITY, &fvpIcosahedron, material[9]};
  dBodyID b = CreateConvexFromFVP(world, space, "icosahedron", &mcicosahedron);
  dBodySetPosition(b, -1.5, 1.5, 0.5);
  dBodyEnable(b);
  disp_Mass(b, "Icosahedron");
}

void create_TmBunny()
{
  dBodyID b = CreateTrimeshFromVI(world, space, "tmbunny", &mtbunny);
  dBodySetPosition(b, 0.0, 0.25, 0.88); // to (-0.109884, 0.304591, 1.217693)
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
  dBodySetRotation(b, rot);
#else
#if 0
  dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2);
#else
  dRFromEulerAngles(rot, -M_PI / 2, 0, 0); // phi=-x, theta=-y, psi=-z
  dQfromR(q, rot);
#endif
  dBodySetQuaternion(b, q);
#endif
  dBodyEnable(b); // dBodyDisable(b);
  disp_Mass(b, "TmBunny");
}

void create_Bunny()
{
  dBodyID b = CreateConvexFromFVP(world, space, "bunny", &mcbunny);
  dBodySetPosition(b, -3.0, -1.5, 2.0);
  dBodyEnable(b);
  disp_Mass(b, "Bunny");
}

void create_TmCustom()
{
  metatrimesh mtcustom = {DENSITY, &tmvCustom, material[12]};
  dBodyID b = CreateTrimeshFromVI(world, space, "tmcustom", &mtcustom);
  dBodySetPosition(b, -3.0, 3.0, 0.5);
  dBodyEnable(b);
  disp_Mass(b, "TmCustom");
}

void create_Custom()
{
  metaconvex mccustom = {DENSITY, &fvpCustom, material[13]};
  dBodyID b = CreateConvexFromFVP(world, space, "custom", &mccustom);
  dBodySetPosition(b, -3.0, 1.5, 0.5);
  dBodyEnable(b);
  disp_Mass(b, "Custom");
}

void create_Plane()
{
  metaplane plane = {{0, 0, 1, 0}, {10.0, 10.0, 0.05}, DENSITY, material[29]};
  dBodyID b = CreatePlane(world, space, "plane", &plane);
  dBodySetPosition(b, 0.0, 0.0, 0.05);
  if(1){
    dMatrix3 rot;
    dRSetIdentity(rot);
    // dRFromAxisAndAngle(rot, 1, 0, 0, M_PI / 2);
    dBodySetRotation(b, rot);
  }
  dBodyEnable(b);
  disp_Mass(b, "Plane");
}

void create_TmBunny2()
{
  dBodyID b = CreateTrimeshFromVI(world, space, "tmbunny2", mtbunny2.get());
  dBodySetPosition(b, -3.0, -3.5, 2.0);
  if(1){
    dMatrix3 rot;
    dRFromAxisAndAngle(rot, 1, 0, 0, M_PI / 2);
    dBodySetRotation(b, rot);
  }
  dBodyEnable(b);
  disp_Mass(b, "TmBunny2");
}

void create_TmBunny3()
{
  dBodyID b = CreateTrimeshFromVI(world, space, "tmbunny3", mtbunny3.get());
  dBodySetPosition(b, -3.0, -4.0, 2.0);
  if(1){
    dMatrix3 rot;
    dRFromAxisAndAngle(rot, 1, 0, 0, M_PI / 2);
    dBodySetRotation(b, rot);
  }
  dBodyEnable(b);
  disp_Mass(b, "TmBunny3");
}

void create_Bunny2()
{
  dBodyID b = CreateConvexFromFVP(world, space, "bunny2", mcbunny2.get());
  dBodySetPosition(b, -3.0, -2.5, 2.0);
  dBodyEnable(b);
  disp_Mass(b, "Bunny2");
}

void create_Bunny3()
{
  dBodyID b = CreateConvexFromFVP(world, space, "bunny3", mcbunny3.get());
  dBodySetPosition(b, -3.0, -3.0, 2.0);
  dBodyEnable(b);
  disp_Mass(b, "Bunny3");
}

void CreateObjects(dWorldID world)
{
  create_TmBall();
  create_Slope();
  create_SphereApple(); create_SphereBall(); create_SphereRoll();
  create_UBall(); create_LUBall(); create_RUBall();
  create_VBall(); create_LVBall(); create_RVBall();
  create_IHBall(); create_IIBall();
  create_TmTetra(); create_Tetra();
  create_TmCube(); create_Cube();
  create_TmIcosahedron(); create_Icosahedron();
  create_TmBunny(); create_Bunny();
  create_TmCustom(); create_Custom();
  if(0) create_Plane();
  create_TmBunny2(); create_TmBunny3();
  create_Bunny2(); create_Bunny3();
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
    char *torqueX[] = {"apple", "roll", "tmball",
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
  if(1){
    static char buf[80] = {0};
    static int framecount = 0;
    static double spf = 0.0;
    spf += dsElapsedTime();
    if(++framecount == 120){
      sprintf(buf, "%5.2f", framecount / spf);
      // printf("%s\n", buf);
      framecount = 0;
      spf = 0.0;
    }
    int len = strlen(buf);
    for(int i = 0; i < len; ++i){
      char b = buf[i];
      int n = b == '.' ? -1 : (b - '0');
      dsSetTexture(10 + n); // 9: [.] 10-19-20-25: [0-9-A-F] 26: [[R G] [B Y]]
      dsSetColorAlpha(1.0, 1.0, 1.0, 0.4);
      dVector3 lxyz = {1.0, 1.0, 1.0};
      dVector3 pos = {0.5 - 6.0 + i, 0.5 + 1.0 + 0.6 * i, 0.5 + 2.0 - 0.4 * i};
      dMatrix3 rot;
      dRFromEulerAngles(rot, 0, -M_PI / 12, -M_PI / 4);
#if 1
      dReal s = -0.4, t = 0.3;
      dReal v[] = {1 + s, 0 + t, 0, 1 + s, 1 + t, 0, 0 + s, 1 + t, 0};
      dReal w[] = {0 + s, 1 + t, 0, 0 + s, 0 + t, 0, 1 + s, 0 + t, 0};
      dsDrawTriangleD(pos, rot, &v[0], &v[3], &v[6], 1);
      dsDrawTriangleD(pos, rot, &w[0], &w[3], &w[6], 1);
#else
      dsDrawBoxD(pos, rot, lxyz);
#endif
    }
  }
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

  int ids[] = {5, 6, 7, 8};
  for(int i = 0; i < A_SIZE(palette); ++i){
    // texID (enum DS_NONE _WOOD _CHECKERED _GROUND _SKY) or >= 5 from map.txt
    material[i]->texID = ids[i % A_SIZE(ids)];
    material[i]->colour = palette[i];
  }
  material[3]->texID = 7;

  mtbunny2 = shared_ptr<metatrimesh>(CopyMetaTriMesh(NULL, &mtbunny, 0.2),
    [](metatrimesh *p){ FreeMetaTriMesh(p); });
  mtbunny3 = shared_ptr<metatrimesh>(CvtMetaTriMeshFromConvex(&mcbunny, 0.2),
    [](metatrimesh *p){ FreeMetaTriMesh(p); });

  mcbunny2 = shared_ptr<metaconvex>(CopyMetaConvex(NULL, &mcbunny, 0.2),
    [](metaconvex *p){ FreeMetaConvex(p); });
  mcbunny3 = shared_ptr<metaconvex>(CvtMetaConvexFromTriMesh(&mtbunny, 0.2),
    [](metaconvex *p){ FreeMetaConvex(p); });

  RecalcFaces(mcbunny.fvp); // right normal
  RecalcFaces(mcbunny2.get()->fvp); // right normal

  contactgroup = dJointGroupCreate(0);
  setParameters();
  CreateObjects(world);

  if(0){
    printf("sizeof(dReal): %d\n", sizeof(dReal)); // 8
    dQuaternion q;
    dQSetIdentity(q); // [1 0 0 0] (cos(t/2), sin(t/2)x, sin(t/2)y, sin(t/2)z)
    // dQFromAxisAndAngle(q, 1, 0, 0, M_PI); // [0 1 0 0]
    // dQFromAxisAndAngle(q, 0, 1, 0, M_PI); // [0 0 1 0]
    // dQFromAxisAndAngle(q, 0, 0, 1, M_PI); // [0 0 0 1]
    // dQFromAxisAndAngle(q, 1, 0, 0, M_PI / 2); // [0.7071 0.7071 0 0]
    printf("sizeof(dQuaternion): %d\n", sizeof(dQuaternion)); // 32 (8 x 4)
//    printf("sizeof(dQuaternion[0]): %d\n", sizeof(dQuaternion[0])); // 0 !
//    printf("A_SIZE(dQuaternion): %d\n", A_SIZE(dQuaternion)); // hung up !!
    for(int i = 0; i < 4; ++i) printf("%17.13f ", ((dReal *)q)[i]);
    printf("\n");
    dMatrix3 rot;
    dRfromQ(rot, q); // rot[[1 0 0 0] [0 1 0 0] [0 0 1 0]] <- q[1 0 0 0]
    printf("sizeof(dMatrix3): %d\n", sizeof(dMatrix3)); // 96 (8 x 12)
//    printf("sizeof(dMatrix3[0]): %d\n", sizeof(dMatrix3[0])); // 0 !
//    printf("A_SIZE(dMatrix3): %d\n", A_SIZE(dMatrix3)); // hung up !!
    for(int r = 0; r < 3; ++r){
      for(int c = 0; c < 4; ++c) printf("%17.13f ", ((dReal *)rot)[r * 4 + c]);
      printf("\n");
    }
  }

  dsFunctions fn;
  setDrawStuff(&fn);
  dsSimulationLoop(ac, av, WIDTH, HEIGHT, &fn);

  DestroyObjects();
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
