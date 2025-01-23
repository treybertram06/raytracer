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

  double vfov = 90; //vertical fov



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
  double focalLength = 1.0;
  vec3 pixel00Loc; //location of pixel 0,0
  vec3 pixelDeltaU;
  vec3 pixelDeltaV;
  point3 center;
  double pixel_samples_scale; //colour scale factor for a sum of pixel samples

  double theta = degrees_to_radians(vfov);
  double h = std::tan(theta/2);
  double viewportHeight = 2*h * focalLength;



    void initialize() {
      //viewport width is calculated based off height and image aspect ratio
      double viewportWidth = viewportHeight * (static_cast<double>(imageWidth) / static_cast<double>(imageHeight));
      center = point3(0, 0, 0);

      pixel_samples_scale = 1.0 / samples_per_pixel;

      // Calculate the vectors across the horizontal and down vertical edges

      //horizontal vector points to the right
      vec3 viewportU = vec3(viewportWidth, 0, 0);
      //and vertical points down
      vec3 viewportV = vec3(0, -viewportHeight, 0);

      //Calculate the delta vectors from pixel to pixel
      pixelDeltaU = viewportU / imageWidth;
      pixelDeltaV = viewportV / imageHeight;

      //Calculate the location of the upper left pixel
      //viewportUpperLeft = {half of width, half of height, distance from camera to viewport}
      vec3 viewportUpperLeft = center - vec3(0, 0, focalLength) - viewportU/2 - viewportV/2;
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
