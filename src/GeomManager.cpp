/*
  GeomManager.cpp
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <trimeshconvex.h>
#include <geommanager.h>

#include <unordered_map>
#include <vector>

using namespace std;

static unordered_map<dGeomID, convexfvp *> geom_convex_manager;
static unordered_map<dGeomID, const dReal *> geom_colour_manager;
static unordered_map<string, dBodyID> geom_body_manager;
static vector<dBodyID> geom_body_order;

dGeomID MapGeomConvex(dGeomID geom, convexfvp *fvp)
{
  geom_convex_manager.insert(make_pair(geom, fvp));
  return geom;
}

dGeomID MapGeomColour(dGeomID geom, const dReal *colour)
{
  geom_colour_manager.insert(make_pair(geom, colour));
  return geom;
}

dBodyID MapBody(const char *key, dBodyID body)
{
  if(key) geom_body_manager.insert(make_pair(key, body));
  return body;
}

dBodyID FindBody(const char *key)
{
  return geom_body_manager[key];
}

dBodyID OrderBody(dBodyID body, int pos)
{
  if(pos < 0) geom_body_order.insert(geom_body_order.end() + (pos + 1), body);
  else geom_body_order.insert(geom_body_order.begin() + pos, body);
  return body;
}

void DestroyObject(dBodyID body)
{
  dGeomID geom = dBodyGetFirstGeom(body);
  while(geom){
    dGeomID nextgeom = dBodyGetNextGeom(geom);
    // dGeomTriMeshDataDestroy(tmd); // when geom has dTriMeshDataID tmd
    dGeomDestroy(geom);
    geom = nextgeom;
  }
  dBodyDestroy(body);
}

void DestroyObjects()
{
  for(auto it = geom_body_manager.begin(); it != geom_body_manager.end(); ++it)
    DestroyObject(it->second);

  geom_body_order.clear();
  geom_body_manager.clear();
  geom_colour_manager.clear();
  geom_convex_manager.clear();
}

void DrawObjects(int ws)
{
  dsSetTexture(DS_WOOD); // DS_SKY DS_GROUND DS_CHECKERED

  for(auto it = geom_body_order.begin(); it != geom_body_order.end(); ++it)
    for(dGeomID g = dBodyGetFirstGeom(*it); g; g = dBodyGetNextGeom(g))
      DrawGeom(g, NULL, NULL, ws);
}

void DrawGeom(dGeomID geom, const dReal *pos, const dReal *rot, int ws)
{
  if(!geom) return;
  if(!pos) pos = dGeomGetPosition(geom);
  if(!rot) rot = dGeomGetRotation(geom);
  const dReal *colour = geom_colour_manager[geom];
  if(colour) dsSetColorAlpha(colour[0], colour[1], colour[2], colour[3]);
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
  case dCapsuleClass: { // 2
    dReal len, radius;
    dGeomCapsuleGetParams(geom, &radius, &len);
    dsDrawCapsuleD(pos, rot, len, radius);
  } break;
  case dCylinderClass: { // 3
    dReal len, radius;
    dGeomCylinderGetParams(geom, &radius, &len);
    dsDrawCylinderD(pos, rot, len, radius);
  } break;
  case dPlaneClass: { // 4
    dVector4 v;
    dGeomPlaneGetParams(geom, v);
    dVector3 lxyz = {10.0, 10.0, 0.05}; // ***
    dsDrawBoxD(pos, rot, lxyz);
  } break;
#if 0
  case dRayClass: { // 5
  } break;
#endif
  case dConvexClass: { // 6
    convexfvp *fvp = geom_convex_manager[geom];
    if(!fvp) printf("not managed convex in DrawGeom (geomID: %p)\n", geom);
    else dsDrawConvexD(pos, rot,
      fvp->faces, fvp->faceCount, fvp->vtx, fvp->vtxCount, fvp->polygons);
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
    DrawGeom(gt, rpos, rrot, ws);
  } break;
  case dTriMeshClass: { // 8
    int is_composite = (dGeomGetSpace(geom) == 0);
    dVector3 tpos = {0.0, 0.0, 0.0};
    dMatrix3 trot;
    dRSetIdentity(trot);
    int triCount = dGeomTriMeshGetTriangleCount(geom);
    for(int i = 0; i < triCount; ++i){
      dVector3 v0, v1, v2;
      dGeomTriMeshGetTriangle(geom, i, &v0, &v1, &v2); // already transformed
      if(!is_composite) dsDrawTriangleD(tpos, trot, v0, v1, v2, ws); // top
      else dsDrawTriangleD(pos, rot, v0, v1, v2, ws); // in the dTransformClass
    }
  } break;
#if 0
  case dHeightfieldClass: { // 9
  } break;
#endif
  default:
    printf("not implemented type dGeomGetClass() in DrawGeom\n");
    printf(" geomID: %p, classID: %d\n", geom, clsID);
    break;
  }
}
