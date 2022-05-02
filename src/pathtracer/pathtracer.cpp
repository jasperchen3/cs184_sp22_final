#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p_w = r.o + r.d * isect.t;
  const Vector3D w_out_o = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading

  for (int i = 0; i < num_samples; i++) {
    
    Vector3D w_in_o = hemisphereSampler->get_sample();
    Vector3D w_in_w = o2w * w_in_o;
    Vector3D w_out_w = o2w * w_out_o;
    
    Vector3D L_i = isect.bsdf->f(w_out_o, w_in_o);

    Ray sampled_ray = Ray(hit_p_w, w_in_w);
    sampled_ray.min_t = EPS_D;

    Intersection i_next;
    double cos_theta = dot(w_in_w, isect.n);
    if (bvh->intersect(sampled_ray, &i_next)) {
      L_out += cos_theta * i_next.bsdf->get_emission() * L_i;
    }
  }
  return 2 * PI * L_out / num_samples;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  
  Vector3D L_out;
  Vector3D wi;
  double distToLight;
  double pdf;
  
  for (SceneLight* light : scene->lights) {
    Vector3D L_light = Vector3D();
    for (size_t i = 0; i < ns_area_light; i++) {

      Vector3D L_i = light->sample_L(hit_p, &wi, &distToLight, &pdf);
      
      Ray emitted_ray = Ray(hit_p, wi);
      emitted_ray.min_t = EPS_F;
      emitted_ray.max_t = distToLight - EPS_F;
      Intersection i_next;
      double cos_theta = dot(wi, isect.n);

      if (cos_theta > 0 && !bvh->intersect(emitted_ray, &i_next)) {
        Vector3D f = isect.bsdf->f(w_out, wi);

        L_light += cos_theta * f * L_i / pdf;
      }
      if (light->is_delta_light()) {
        L_light *= ns_area_light;
        break;
      }
    }
    L_out += L_light / ns_area_light;
  }
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d); //object space

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.

  L_out = one_bounce_radiance(r, isect);

  if (r.depth <= 1) { //terminate the ray if it reaches this
      return L_out;
  }

  Vector3D wi;
  double pdf;
  Vector3D w_out_w = o2w * w_out; //world space
  Vector3D emission = isect.bsdf->sample_f(w_out_w, &wi, &pdf);
  if (r.has_bounced == false) {
      wi = o2w  * wi; // world space
      Ray next_ray = Ray(hit_p, wi);
      next_ray.min_t = EPS_F;
      next_ray.depth = r.depth - 1;
      next_ray.has_bounced = true;
      Intersection next_int;
      double cos_theta = dot(wi, isect.n);
      if (bvh->intersect(next_ray, &next_int) && cos_theta > 0) {
          //cout << "got here" << endl;
          L_out += at_least_one_bounce_radiance(next_ray, next_int) * emission * cos_theta / pdf;
      } else{
          return L_out;
      }
  } else if (coin_flip(.65)) {
      wi = o2w  * wi;
      Ray next_ray = Ray(hit_p, wi);
      next_ray.min_t = EPS_F;
      next_ray.depth = r.depth - 1;
      next_ray.has_bounced = true;
      Intersection next_int;
      double cos_theta = dot(wi, isect.n);
      if (bvh->intersect(next_ray, &next_int) && cos_theta > 0) {
          L_out += at_least_one_bounce_radiance(next_ray, next_int) * emission * cos_theta / pdf / 0.65;
      } else{
          return L_out;
      }
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;
  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  //L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  
  
  Vector3D est_radiance = Vector3D();
  float s1 = 0;
  float s2 = 0;
  float n = 0;
  for (int i = 0; i < num_samples ; i++) {
      if (i % samplesPerBatch == samplesPerBatch - 1) {
          float mean = s1 / n;
          float variance = (s2 - s1 * s1 / n) / (n - 1);
          float I_squared = pow(1.96, 2) * variance / n;
          if (I_squared <= pow(maxTolerance * mean, 2)) {
              break;
          }
      }
      Vector2D rand_vec = gridSampler->get_sample();
      Ray ray = camera->generate_ray((x + rand_vec.x) / sampleBuffer.w, (y + rand_vec.y) / sampleBuffer.h);
      ray.depth = max_ray_depth;
      Vector3D rad = est_radiance_global_illumination(ray);
      est_radiance += rad;
      
      float illum = rad.illum();
      s1 += illum;
      s2 += illum * illum;
      n++;
  }
  est_radiance /= n;
  sampleBuffer.update_pixel(est_radiance, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = n;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
