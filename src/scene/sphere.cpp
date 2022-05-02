#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - this->o, r.d);
  double c = dot(r.o - this->o, r.o - this->o) - this->r2;
  
  double determinant = pow(b, 2) - 4 * a * c;
  
  
  
  if (determinant < 0) {
    return false;
  }
  
  double first_intersection = (-b - sqrt(determinant)) / (2 * a);
  double second_intersection = (-b + sqrt(determinant)) / (2 * a);
  if (second_intersection < r.min_t || r.max_t < first_intersection){
    return false;
  }
  
  t1 = first_intersection;
  t2 = second_intersection;
  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double first_intersection;
  double second_intersection;
  
  return test(r, first_intersection, second_intersection);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  
  double first_intersection;
  double second_intersection;
  if (!test(r, first_intersection, second_intersection)) {
    return false;
  }
  if (first_intersection < r.min_t) {
    r.max_t = second_intersection;
    i->t = second_intersection;
    i->n = normal(r.at_time(second_intersection));
  } else {
    r.max_t = first_intersection;
    i->t = first_intersection;
    i->n = normal(r.at_time(first_intersection));
  }
  i->primitive = this;
  i->bsdf = get_bsdf();
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
