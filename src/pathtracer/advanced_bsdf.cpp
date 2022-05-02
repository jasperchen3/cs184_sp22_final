#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Project 3-2: Part 1
  // Implement MirrorBSDF
  *pdf = 1.0;
  reflect(wo, wi);
  return reflectance / abs_cos_theta(*wi);
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO Project 3-2: Part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.

  double theta_h = getTheta(h / h.norm());
  double numerator = pow(M_E, -pow(tan(theta_h), 2)/pow(alpha, 2));
  double denominator = M_PI * pow(alpha, 2) * pow(cos(theta_h), 4);  
  return numerator / denominator;
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO Project 3-2: Part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.
  //Vector3D eta = Vector3D(0.059193, 0.059881, 0.047366); //For Silver
  //Vector3D k = Vector3D(4.1283, 3.5892 , 2.8132);
  double cos_theta_i = wi.z;
  Vector3D Rs_num = (eta * eta) + (k * k) - 2 * eta * cos_theta_i + pow(cos_theta_i, 2);
  Vector3D Rs_den = (eta * eta) + (k * k) + 2 * eta * cos_theta_i + pow(cos_theta_i, 2);
  Vector3D Rp_num = ((eta * eta) + (k * k)) * pow(cos_theta_i, 2) - 2 * eta * cos_theta_i + 1;
  Vector3D Rp_den = ((eta * eta) + (k * k)) * pow(cos_theta_i, 2) + 2 * eta * cos_theta_i + 1;
  Vector3D Rs = Rs_num / Rs_den;
  Vector3D Rp = Rp_num / Rp_den; 
  return (Rs + Rp) / 2;
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO Project 3-2: Part 2
  // Implement microfacet model here.

  //Vector3D h = (wo + wi) / (wo + wi).norm();
  Vector3D n = Vector3D(0, 0, 1);
  if (dot(n, wi) <= 0 || dot(n, wo) <= 0) {
    return Vector3D();
  }
  Vector3D h = wo.norm() * wi + wi.norm() * wo;
  
  return (F(wi) * G(wo, wi) * D(h)) / (4 * dot(n, wo) * dot(n, wi));  
  //return Vector3D();
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Project 3-2: Part 2
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  //*wi = cosineHemisphereSampler.get_sample(pdf);
  Vector3D n = Vector3D(0, 0, 1);
  Vector2D samples = sampler.get_sample();
  double r1 = samples[0];
  double r2 = samples[1];
  double theta_h = atan(sqrt(-pow(alpha, 2) * log(1 - r1)));
  //std::cout << theta_h << std::endl;
  double sin_t = sin(theta_h);
  double cos_t = cos(theta_h);
  double tan_t = tan(theta_h);
  double phi_h = 2 * M_PI * r2;
  Vector3D h = Vector3D(sin(theta_h) * cos(phi_h), sin(phi_h) * sin(theta_h), cos(theta_h));
  *wi =  2 * dot(h, wo) * h - wo;  	
  //std::cout << h << std::endl;

  //if wi is invalid
  if (dot(n, *wi) <= 0 || dot(n, wo) <= 0) {
     *pdf = 0;
     return 0;
  }	  

  //calculating p_theta and p_phi
  double p_theta_num = (2 * sin_t) * (pow(M_E, -pow(tan_t, 2) / pow(alpha, 2)));
  double p_theta_den = pow(alpha, 2) * pow(cos_t, 3); 
  double p_theta = p_theta_num/p_theta_den;
  //std::cout << p_theta << std::endl;
  double p_phi = 1 / (2 * M_PI);
  
  //calculating p_w_h and p_w_wi, the pdf
  double p_w_h = p_theta * p_phi / sin_t;
  double p_w_wi = p_w_h / (4 * dot(*wi, h));
  *pdf = p_w_wi;
  //std::cout << p_w_wi << std::endl;

  //std::cout << MicrofacetBSDF::f(wo, *wi) << std::endl;
  return MicrofacetBSDF::f(wo, *wi);
}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Project 3-2: Part 1
  // Implement RefractionBSDF

  if (refract(wo, wi, ior)) {
    *pdf = 1;
    //entering : exiting
    double eta = wo.z > 0 ? 1.0f / ior : ior;
    return transmittance / abs_cos_theta(*wi) / pow(eta, 2);
  } else {
    return Vector3D();
  }
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Project 3-2: Part 1
  // Compute Fresnel coefficient and either reflect or refract based on it.

  // compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305
  double eta = wo.z > 0 ? 1.0 / ior : ior;
  double wi_z_squared = 1 - pow(eta, 2) * (1 - pow(wo.z, 2));
  // Total internal reflection
  if (wi_z_squared < 0) {
    reflect(wo, wi);
    *pdf = 1.0;
    return reflectance / abs_cos_theta(*wi);
  } else {
    double R_0 = pow((1 - eta) / (1 + eta), 2);
    double R = R_0 + (1 - R_0) * pow((1 - abs(wo.z)), 5);
    if (coin_flip(R)) {	    
      reflect(wo, wi);
      *pdf = R;
      return R * reflectance / abs_cos_theta(*wi);
    } else {	    
      refract(wo, wi, ior);
      *pdf = 1 - R;
      return (1 - R) * transmittance / abs_cos_theta(*wi) / pow(eta, 2);
    }
  }
  
}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // TODO Project 3-2: Part 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo.x, -wo.y, wo.z);
}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // TODO Project 3-2: Part 1
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  
  // entering : exiting
  double eta = wo.z > 0 ? 1.0f / ior : ior;
  double wi_z_squared = 1 - pow(eta, 2) * (1 - pow(wo.z, 2));
  if (wi_z_squared < 0) {
    return false;
  }
  wi->x = -eta * wo.x;
  wi->y = -eta * wo.y;
  wi->z = (2 * (wo.z < 0) - 1) * sqrt(wi_z_squared);
  return true;

}

} // namespace CGL
