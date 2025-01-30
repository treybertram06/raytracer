//
// Created by treyb on 2025-01-21.
//

#ifndef HITTABLE_H
#define HITTABLE_H

#include "usefulstuff.h"

class material;

class hit_record {
  public:
    point3 p;
    vec3 normal;
    shared_ptr<material> mat;
    double t;
    double u;
    double v;
    bool front_face;

    void set_face_normal(const ray& r, const vec3& outward_normal) {
        // sets the hit record normal vector
        // outward_normal is assumed ot have unit length
        front_face = dot(r.direction(), outward_normal) < 0;
        //if f_f, o_n else !o_n
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable {
  public:
    virtual ~hittable() = default;

    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;

    virtual aabb bounding_box() const = 0;
};

#endif //HITTABLE_H
