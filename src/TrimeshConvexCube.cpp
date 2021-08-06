/*
  TrimeshConvexCube.cpp
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

#if 1
static dReal Planes[] = { // 6 * 4
  1.0f, 0.0f, 0.0f, 0.25f,
  0.0f, 1.0f, 0.0f, 0.25f,
  0.0f, 0.0f, 1.0f, 0.25f,
  0.0f, 0.0f, -1.0f, 0.25f,
  0.0f, -1.0f, 0.0f, 0.25f,
  -1.0f, 0.0f, 0.0f, 0.25f};
#else
static dReal Planes[] = { // 6 * 4
  1.0f, 0.0f, 0.0f, 2.0f,
  0.0f, 1.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 1.0f, 1.0f,
  0.0f, 0.0f, -1.0f, 1.0f,
  0.0f, -1.0f, 0.0f, 1.0f,
  -1.0f, 0.0f, 0.0f, 0.0f};
#endif
static dReal Points[] = { // 8 * 3;
  0.25f, 0.25f, 0.25f,
  -0.25f, 0.25f, 0.25f,
  0.25f, -0.25f, 0.25f,
  -0.25f, -0.25f, 0.25f,
  0.25f, 0.25f, -0.25f,
  -0.25f, 0.25f, -0.25f,
  0.25f, -0.25f, -0.25f,
  -0.25f, -0.25f, -0.25f};
static unsigned int Polygons[] = { // 6 * (1 + 4)
  4, 0, 2, 6, 4, // +X
  4, 1, 0, 4, 5, // +Y
  4, 0, 1, 3, 2, // +Z
  4, 3, 1, 5, 7, // -X
  4, 2, 3, 7, 6, // -Y
  4, 5, 4, 6, 7}; // -Z

trimeshvi tmvCube = {A_SIZE(Vtx) / 3, Vtx, Indices, A_SIZE(Indices)};
convexfvp fvpCube = {
  A_SIZE(Planes) / 4, Planes, A_SIZE(Points) / 3, Points, Polygons};
