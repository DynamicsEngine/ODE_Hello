/*
  GenComposite.h
*/

#ifndef __GENCOMPOSITE_H__
#define __GENCOMPOSITE_H__

struct metasphere {
  dReal r, m, gBounce;
  dReal *colour;
};

struct metaplane {
  dVector4 v;
  dVector3 lxyz;
  dReal dm;
  dReal *colour;
};

struct metacomposite {
  int numcomposite;
  int *clsID;
  dReal *dm; // density or mass
  dReal (*offset)[3];
  dReal (*params)[4];
  dQuaternion *q;
  void **v; // trimeshvi or convexfvp
  dReal **colour;
};

extern dBodyID CreateComposite(dWorldID world, dSpaceID space,
  metacomposite *mc);

extern dBodyID CreateSphere(dWorldID world, dSpaceID space, metasphere *s);
extern dBodyID CreatePlane(dWorldID world, dSpaceID space, metaplane *p);

#endif // __GENCOMPOSITE_H__
