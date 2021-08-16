/*
  TrimeshConvexCustom.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>

static dTriIndex Indices[] = { // 4 * 3
  3, 1, 0,
  3, 2, 1,
  3, 0, 2,
  2, 0, 1};

static dReal Planes[] = { // 4 * 4
  0.0f, 0.0f, -1.0f, 0.0f,
  -1.0f, 0.0f, 0.0f, 0.0f,
  0.0f, -1.0f, 0.0f, 0.0f,
  0.57735f, 0.57735f, 0.57735f, 0.57735f};
static dReal Vtx[] = { // 4 * 3;
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f};
static unsigned int Polygons[] = { // 4 * (1 + 3)
  3, 3, 1, 0,
  3, 3, 2, 1,
  3, 3, 0, 2,
  3, 2, 0, 1};

trimeshvi tmvCustom = {A_SIZE(Vtx) / 3, Vtx, Indices, A_SIZE(Indices)};
convexfvp fvpCustom = {
  A_SIZE(Planes) / 4, Planes, A_SIZE(Vtx) / 3, Vtx, Polygons};
