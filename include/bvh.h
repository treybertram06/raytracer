//
// Created by treyb on 2025-01-24.
//

#ifndef BVH_H
#define BVH_H

#include "aabb.h"
#include "hittable.h"
#include "hittable_list.h"
#include "usefulstuff.h"

#include <thread>
#include <algorithm>

class bvh_node : public hittable {
public:
  bvh_node(hittable_list list) : bvh_node(list.objects, 0, list.objects.size()) {
    // There's a C++ subtlety here. This constructor (without span indices) creates an
    // implicit copy of the hittable list, which we will modify. The lifetime of the copied
    // list only extends until this constructor exits. That's OK, because we only need to
    // persist the resulting bounding volume hierarchy.
  }

  bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end) {
    // Compute bounding box for the entire object span
    bbox = aabb::empty;

    std::vector<aabb> bounding_boxes;
    bounding_boxes.reserve(end - start);

    for (size_t i = start; i < end; ++i) {
      aabb box = objects[i]->bounding_box();
      bounding_boxes.push_back(box);
      bbox = aabb(bbox, box);  // Merge bounding boxes
    }

    int axis = bbox.longest_axis();

    auto comparator = (axis == 0) ? box_x_compare
                    : (axis == 1) ? box_y_compare
                                  : box_z_compare;

    size_t object_span = end - start;

    if (object_span == 1) {
      left = right = objects[start];
    } else if (object_span == 2) {
      if (comparator(objects[start], objects[start + 1])) {
        left = objects[start];
        right = objects[start + 1];
      } else {
        left = objects[start + 1];
        right = objects[start];
      }
    } else {
      // Use bounding_boxes to compare instead of calling bounding_box() repeatedly
      std::sort(std::begin(objects) + start, std::begin(objects) + end, comparator);

      auto mid = start + object_span / 2;
      left = make_shared<bvh_node>(objects, start, mid);
      right = make_shared<bvh_node>(objects, mid, end);
    }
  }

  bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
    if (!bbox.hit(r, ray_t))
      return false;

    bool is_left_first = r.direction()[bbox.longest_axis()] < 0;
    const auto first = is_left_first ? right : left;
    const auto second = is_left_first ? left : right;

    bool hit_first = first->hit(r, ray_t, rec);
    bool hit_second = second->hit(r, interval(ray_t.min, hit_first ? rec.t : ray_t.max), rec);

    return hit_first || hit_second;
  }


  aabb bounding_box() const override { return bbox; }

private:
  shared_ptr<hittable> left;
  shared_ptr<hittable> right;
  aabb bbox;

  static bool box_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b, int axis_index) {
    auto a_axis_interval = a->bounding_box().axis_interval(axis_index);
    auto b_axis_interval = b->bounding_box().axis_interval(axis_index);
    return a_axis_interval.min < b_axis_interval.min;
  }

  static bool box_x_compare (const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
    return box_compare(a, b, 0);
  }

  static bool box_y_compare (const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
    return box_compare(a, b, 1);
  }

  static bool box_z_compare (const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
    return box_compare(a, b, 2);
  }


};




#endif //BVH_H
