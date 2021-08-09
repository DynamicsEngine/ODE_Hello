/*
  GenComposite.h
*/

#ifndef __GENCOMPOSITE_H__
#define __GENCOMPOSITE_H__

struct metacomposite {
  int numcomposite;
  int *clsID;
  dReal *dm; // density or mass
  dReal (*offset)[3];
  dReal (*params)[4];
  dQuaternion *q;
};

extern dBodyID CreateComposite(dWorldID world, dSpaceID space,
  metacomposite *mc);

#endif // __GENCOMPOSITE_H__
