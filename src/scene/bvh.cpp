#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
  
  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

float BVHAccel::helper(int* axis, float minx, float maxx, float sumx, float miny, float maxy, float sumy,
                       float minz, float maxz, float sumz) {
  float diffx = maxx - minx;
  float diffy = maxy - miny;
  float diffz = maxz - minz;
  
  if (std::max({diffx, diffy, diffz}) == diffx) {
    *axis = 0;
  } else if (std::max({diffx, diffy, diffz}) == diffy) {
    *axis = 1;
  } else {
    *axis = 2;
  }
  
  if (*axis == 0) {
    return sumx;
  } else if (*axis == 1) {
    return sumy;
  } else {
    return sumz;
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {
  
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  BBox bbox;
  size_t num_nodes = 0;
  
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    num_nodes = num_nodes + 1;
  }
  
  BVHNode *node = new BVHNode(bbox);
  
  if (num_nodes <= max_leaf_size) {
    node->start = start;
    node->end = end;
  } else {
    Vector3D centroid = (*start)->get_bbox().centroid();
    
    float minx = centroid[0];
    float maxx = centroid[0];
    float sumx = 0;
    float miny = centroid[1];
    float maxy = centroid[1];
    float sumy = 0;
    float minz = centroid[2];
    float maxz = centroid[2];
    float sumz = 0;
    for (auto p = start; p != end; p++) {
      centroid = (*p)->get_bbox().centroid();
      float currx = centroid[0];
      float curry = centroid[1];
      float currz = centroid[2];
      
      sumx += currx;
      sumy += curry;
      sumz += currz;
      
      maxx = max(maxx, currx);
      maxy = max(maxy, curry);
      maxz = max(maxz, currz);
      minx = min(minx, currx);
      miny = min(miny, curry);
      minz = min(minz, currz);
      
    }
    
    int* axis = new int(0);
    float mean = helper(axis, minx, maxx, sumx, miny, maxy, sumy, minz, maxz, sumz) / (float) num_nodes;
    
    std::vector<Primitive *>::iterator mid;
    mid = partition(start, end, [mean, axis](Primitive* &em) {return em->get_bbox().centroid()[*axis] < mean;});
    node->l = construct_bvh(start, mid, max_leaf_size);
    node->r = construct_bvh(mid, end, max_leaf_size);
  }
  
  return node;
  
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):

    /*
    for (auto p : primitives) {
      total_isects++;
      if (p->has_intersection(ray))
        return true;
    }
    return false;
    */

      double t0;
      double t1;
      if (!node->bb.intersect(ray, t0, t1)) { //if the ray doesn't pass through the bbox
          return false;
      }

      if (node->isLeaf()) {
          for (auto p = node->start; p != node->end; p++) {
              total_isects++;
              if ((*p)->has_intersection(ray))
                  return true;
          }
      } else {
          return has_intersection(ray, node->l) || has_intersection(ray, node->r);
      }
      return false;

   }

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
    /*
         bool hit = false;
         for (auto p : primitives) {
           total_isects++;
           hit = p->intersect(ray, i) || hit;
         }
         return hit;
    */

  double t0;
  double t1;
  if (!node->bb.intersect(ray, t0, t1)) { //if the ray doesn't pass through the bbox
      return false;
  }

  total_isects++;
  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit;
  } else {
    bool right_intersect = intersect(ray, i, node->r);
    return intersect(ray, i, node->l) || right_intersect;
  }

}

} // namespace SceneObjects
} // namespace CGL
