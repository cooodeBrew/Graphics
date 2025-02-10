#include "trimesh.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <float.h>
#include <string.h>
#include "../ui/TraceUI.h"
extern TraceUI *traceUI;
extern TraceUI *traceUI;

using namespace std;

Trimesh::~Trimesh() {
  for (auto f : faces)
    delete f;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3 &v) { vertices.emplace_back(v); }

void Trimesh::addNormal(const glm::dvec3 &n) { normals.emplace_back(n); }

void Trimesh::addColor(const glm::dvec3 &c) { vertColors.emplace_back(c); }

void Trimesh::addUV(const glm::dvec2 &uv) { uvCoords.emplace_back(uv); }

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace(int a, int b, int c) {
  int vcnt = vertices.size();

  if (a >= vcnt || b >= vcnt || c >= vcnt)
    return false;

  TrimeshFace *newFace = new TrimeshFace(this, a, b, c);
  if (!newFace->degen)
    faces.push_back(newFace);
  else
    delete newFace;

  // Don't add faces to the scene's object list so we can cull by bounding
  // box
  return true;
}

// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
const char *Trimesh::doubleCheck() {
  if (!vertColors.empty() && vertColors.size() != vertices.size())
    return "Bad Trimesh: Wrong number of vertex colors.";
  if (!uvCoords.empty() && uvCoords.size() != vertices.size())
    return "Bad Trimesh: Wrong number of UV coordinates.";
  if (!normals.empty() && normals.size() != vertices.size())
    return "Bad Trimesh: Wrong number of normals.";

  return 0;
}

bool Trimesh::intersectLocal(ray &r, isect &i) const {
  bool have_one = false;
  for (auto face : faces) {
    isect cur;
    if (face->intersectLocal(r, cur)) {
      if (!have_one || (cur.getT() < i.getT())) {
        i = cur;
        have_one = true;
      }
    }
  }
  if (!have_one)
    i.setT(1000.0);
  return have_one;
}

bool TrimeshFace::intersect(ray &r, isect &i) const {
  return intersectLocal(r, i);
}


// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray &r, isect &i) const {
  // YOUR CODE HERE
  //
  // FIXME: Add ray-trimesh intersection

  /* To determine the color of an intersection, use the following rules:
     - If the parent mesh has non-empty `uvCoords`, barycentrically interpolate
       the UV coordinates of the three vertices of the face, then assign it to
       the intersection using i.setUVCoordinates().
     - Otherwise, if the parent mesh has non-empty `vertexColors`,
       barycentrically interpolate the colors from the three vertices of the
       face. Create a new material by copying the parent's material, set the
       diffuse color of this material to the interpolated color, and then 
       assign this material to the intersection.
     - If neither is true, assign the parent's material to the intersection.
  */

  // get the 3D coordinates for the three vertices of the triangle
  glm::dvec3 x_coord = parent->vertices[ids[0]];
  glm::dvec3 y_coord = parent->vertices[ids[1]];
  glm::dvec3 z_coord = parent->vertices[ids[2]];

  // dot product between normal and rat's direction
  double dotP = glm::dot(normal, r.getDirection());

  // if dot product is positive, the ray is hitting the back face relative to normal
  if (dotP > RAY_EPSILON) {
    return false;
  }

  // get the parameter along the ray where the intersection
  // with the plane containing the triangle occurs.
  double hitParam = glm::dot(normal, x_coord - r.getPosition()) / dotP;

  if (hitParam < RAY_EPSILON) {
    // the intersection point is behind the ray's origin or too close
    return false;
  }

  i.setT(hitParam);

  // get the intersection point on the ray using hit parameter
  glm::dvec3 P = r.at(hitParam);

  // get vectors between vertices of the triangle
  glm::dvec3 vec_yx = (x_coord - y_coord); // y to x
  glm::dvec3 vec_zx = (x_coord - z_coord); // z to x
  glm::dvec3 vec_zy = (y_coord - z_coord); // z to y

  // vertices to intersection point
  glm::dvec3 vec_x_point = P - x_coord;
  glm::dvec3 vec_y_point = P - y_coord;
  glm::dvec3 vec_z_point = P - z_coord;

  // area of triangle using cross product
  double tri_area = glm::length(glm::cross(vec_zx, vec_zy)) * 0.5;

  // sub-areas for the subtriangles formed by the intersection point
  double yz_area = glm::length(glm::cross(vec_zy, vec_y_point)) * 0.5;
  double xz_area = glm::length(glm::cross(vec_zx, vec_z_point)) * 0.5;
  double xy_area = glm::length(glm::cross(vec_yx, vec_x_point)) * 0.5;

  // if total area is nearly 0, then the triangle is degenerate
  if (tri_area < RAY_EPSILON) {
    return false;
  }

  // get barycentric coordinates
  double alpha = yz_area / tri_area;
  double beta = xz_area / tri_area;
  double gamma = xy_area / tri_area;

  // check if all of them are non-negative and sum to 1
  if (alpha >= 0 && beta >= 0 && gamma >= 0 && std::fabs(1.0 - alpha - beta - gamma) < RAY_EPSILON) {
    i.setBary(alpha, beta, gamma);

    if (!parent->normals.empty()) {
      glm::dvec3 x_norm = parent->normals[ids[0]];
      glm::dvec3 y_norm = parent->normals[ids[1]];
      glm::dvec3 z_norm = parent->normals[ids[2]];

      glm::dvec3 N = x_norm * alpha + y_norm * beta + z_norm * gamma;
      i.setN(glm::normalize(N));
    } else {
      i.setN(glm::normalize(normal));
    }

    // assign texture coordinates or vertec colors to the intersection
    if (!parent->uvCoords.empty()) {
      glm::dvec2 uv = parent->uvCoords[ids[0]] * alpha + parent->uvCoords[ids[1]] * beta + parent->uvCoords[ids[2]] * gamma;
      i.setUVCoordinates(uv);
    } else if (!parent->vertColors.empty()) {
      glm::dvec3 color1 = parent->vertColors[ids[0]];
      glm::dvec3 color2 = parent->vertColors[ids[1]];
      glm::dvec3 color3 = parent->vertColors[ids[2]];

      glm::dvec3 color = color1 * alpha + color2 * beta + color3 * gamma;

      Material material = parent->getMaterial();

      material.setDiffuse(color);

      i.setMaterial(material);
    } else {
      // neither uv coordinates nor vertex colors are available
      i.setMaterial(parent->getMaterial());
    }

    i.setObject(this->parent);
    return true;
  } else {
    return false;
  }
}

// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals() {
  int cnt = vertices.size();
  normals.resize(cnt);
  std::vector<int> numFaces(cnt, 0);

  for (auto face : faces) {
    glm::dvec3 faceNormal = face->getNormal();

    for (int i = 0; i < 3; ++i) {
      normals[(*face)[i]] += faceNormal;
      ++numFaces[(*face)[i]];
    }
  }

  for (int i = 0; i < cnt; ++i) {
    if (numFaces[i])
      normals[i] /= numFaces[i];
  }

  vertNorms = true;
}

