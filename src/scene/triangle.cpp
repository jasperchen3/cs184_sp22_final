#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

Vector3D Triangle::moller_trumbore(const Ray &r) const {
  Vector3D E_1 = p1 - p3;
  Vector3D E_2 = p2 - p3;
  Vector3D S = r.o - p3;
  Vector3D S_1 = cross(r.d, E_2);
  Vector3D S_2 = cross(S, E_1);
  Vector3D t_b1_b2 = 1 / dot(S_1, E_1) * Vector3D(dot(S_2, E_2), dot(S_1, S), dot(S_2, r.d));
  return t_b1_b2;
}

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  Vector3D t_b1_b2 = moller_trumbore(r);
  double t = t_b1_b2[0];
  double b1 = t_b1_b2[1];
  double b2 = t_b1_b2[2];
  double b3 = 1 - b1 - b2;
  
  return r.min_t <= t && t <= r.max_t && 0 <= b1 && b1 <= 1 && 0 <= b2 && b2 <= 1 && 0 <= b3 && b3 <= 1;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D t_b1_b2 = moller_trumbore(r);

  double t = t_b1_b2[0];
  double b1 = t_b1_b2[1];
  double b2 = t_b1_b2[2];
  double b3 = 1 - b1 - b2;
  if (!has_intersection(r)) {
    return false;
  }
  r.max_t = t;
  isect->t = t;
  isect->n = b1 * n1 + b2 * n2 + b3 * n3;
  isect->primitive = this;
  isect->bsdf = get_bsdf();
  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
