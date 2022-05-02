#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  float t_x_min = std::min((min.x - r.o.x) / r.d.x, (max.x - r.o.x) / r.d.x);
  float t_x_max = std::max((min.x - r.o.x) / r.d.x, (max.x - r.o.x) / r.d.x);
  float t_y_min = std::min((min.y - r.o.y) / r.d.y, (max.y - r.o.y) / r.d.y);
  float t_y_max = std::max((min.y - r.o.y) / r.d.y, (max.y - r.o.y) / r.d.y);
  float t_z_min = std::min((min.z - r.o.z) / r.d.z, (max.z - r.o.z) / r.d.z);
  float t_z_max = std::max((min.z - r.o.z) / r.d.z, (max.z - r.o.z) / r.d.z);

  
  //float first_intersection = std::min(std::min(t_x_max, t_y_max), t_z_max);
  //float second_intersection = std::max(std::max(t_x_min, t_y_min), t_z_min);
    float first_intersection = std::max(std::max(t_x_min, t_y_min), t_z_min);
    float second_intersection = std::min(std::min(t_x_max, t_y_max), t_z_max);

  if (second_intersection < r.min_t || r.max_t < first_intersection || first_intersection > second_intersection) {
    return false;
  }
  
  t0 = std::max(first_intersection, (float) r.min_t);
  t1 = std::min(second_intersection, (float) r.max_t);
  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
