//
// Created by treyb on 2025-01-21.
//

#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "material.h"

using namespace std;
#include <vector>
#include <thread>
#include <mutex>

class camera {
public:
  int imageWidth;
  int imageHeight;
  int samples_per_pixel;
  int maxDepth;

  double vfov; //vertical fov
  point3 lookfrom = point3(0,0,0);
  point3 lookat = point3(0,0,-1);
  vec3 vup = vec3(0,1,0);



  void render(const hittable& world) {
    initialize();

    cout << "P3\n" << imageWidth << ' ' << imageHeight << "\n255\n";

     for (int j = 0; j < imageHeight; j++) {
       clog << "\rScanlines remaining: " << (imageHeight - j) << ' ' << flush;
       for (int i = 0; i < imageWidth; i++) {

        color pixel_color(0,0,0);
        for (int sample = 0; sample < samples_per_pixel; sample++) {
          ray r = get_ray(i, j);
          pixel_color += ray_color(r, maxDepth, world);
        }
        write_color(std::cout, pixel_samples_scale * pixel_color);

       }
     }

      clog << "\rDone.            \n";
    }

private:
  vec3 pixel00Loc; //location of pixel 0,0
  vec3 pixelDeltaU;
  vec3 pixelDeltaV;
  point3 center;
  double pixel_samples_scale; //colour scale factor for a sum of pixel samples
  vec3 u, v, w;





    void initialize() {
      auto focalLength = (lookfrom - lookat).length();

      //viewport width is calculated based off height and image aspect ratio
      double theta = degrees_to_radians(vfov);
      double h = std::tan(theta/2);
      auto viewportHeight = 2*h * focalLength;
      double viewportWidth = viewportHeight * (static_cast<double>(imageWidth) / static_cast<double>(imageHeight));

      w = unit_vector(lookfrom - lookat);
      u = unit_vector(cross(vup, w));
      v = cross(w, u);

      pixel_samples_scale = 1.0 / samples_per_pixel;

      center = lookfrom;


      //horizontal vector points to the right
      vec3 viewportU = viewportWidth * u;
      //and vertical points down
      vec3 viewportV = viewportHeight * -v;

      //Calculate the delta vectors from pixel to pixel
      pixelDeltaU = viewportU / imageWidth;
      pixelDeltaV = viewportV / imageHeight;

      //Calculate the location of the upper left pixel
      //viewportUpperLeft = {half of width, half of height, distance from camera to viewport}
      vec3 viewportUpperLeft = center - (focalLength * w) - viewportU/2 - viewportV/2;
      pixel00Loc = viewportUpperLeft + 0.5 * (pixelDeltaU + pixelDeltaV);
    }

    ray get_ray(int i, int j) const {
        // Construct a camera ray originating from the origin and directed at randomly sampled
        // point around the pixel location i, j.

        auto offset = sample_square();
        auto pixel_sample = pixel00Loc
                          + ((i + offset.x()) * pixelDeltaU)
                          + ((j + offset.y()) * pixelDeltaV);

        auto ray_origin = center;
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
      }

    vec3 sample_square() const {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
      }

    color ray_color(const ray& r, int depth, const hittable& world) const {
      //if we've exceeded the ray bounce limit, no more light is gathered
      if (depth <= 0)
        return color(0,0,0);
      hit_record rec;
      if (world.hit(r, interval(0.001, infinity), rec)) {
        ray scattered;
        color attenuation;
        if (rec.mat->scatter(r, rec, attenuation, scattered)) {
          return attenuation * ray_color(scattered, depth-1, world);
        }
        return color(0,0,0);
      }

      vec3 unitDirection = unit_vector(r.direction());
      double a = 0.5*(unitDirection.y() + 1.0);
      return (1.0-a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
    }

};


#endif //CAMERA_H
