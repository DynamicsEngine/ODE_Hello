/*
  TrimeshConvexTetra.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>

static float Vtx[] = { // 4 * 3;
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f};
static dTriIndex Indices[] = { // 4 * 3
  3, 1, 0,
  3, 2, 1,
  3, 0, 2,
  2, 0, 1};

static dReal Planes[] = { // 4 * 4
  0.0f, 0.0f, -1.0f, 0.333f,
  -1.0f, 0.0f, 0.0f, 0.333f,
  0.0f, -1.0f, 0.0f, 0.333f,
  1.0f, 1.0f, 1.0f, 0.1f};
static dReal Points[] = { // 4 * 3;
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f};
static unsigned int Polygons[] = { // 4 * (1 + 3)
  3, 3, 1, 0,
  3, 3, 2, 1,
  3, 3, 0, 2,
  3, 2, 0, 1};

trimeshvi tmvTetra = {A_SIZE(Vtx) / 3, Vtx, Indices, A_SIZE(Indices)};
convexfvp fvpTetra = {
  A_SIZE(Planes) / 4, Planes, A_SIZE(Points) / 3, Points, Polygons};
