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
  int clsID;
  dReal dm; // density or mass
  dVector3 offset;
  dVector4 params;
  dQuaternion q;
  void *v; // trimeshvi or convexfvp
  dReal *colour;
};

extern dBodyID CreateComposite(dWorldID world, dSpaceID space,
  const char *key, metacomposite *mc, int numcomposite);

extern dBodyID CreateSphere(dWorldID world, dSpaceID space,
  const char *key, metasphere *s);
extern dBodyID CreatePlane(dWorldID world, dSpaceID space,
  const char *key, metaplane *p);

#endif // __GENCOMPOSITE_H__
