//
// Created by treyb on 2025-01-21.
//

#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "material.h"
#include "pdf.h"

using namespace std;
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

class camera {
public:
  int image_width;
  int image_height;
  int samples_per_pixel;
  int max_depth;
  color background;

  double vfov; //vertical fov
  point3 lookfrom = point3(0,0,0);
  point3 lookat = point3(0,0,-1);
  vec3 vup = vec3(0,1,0);

  double defocus_angle;
  double focus_dist;

  int threads_left = 99999;



  void render(const hittable& world, const hittable& lights) {
    initialize();
    clog << "Rendering image...\n";
    vector<vector<color>> image(image_height, vector<color>(image_width));  // Properly preallocate
    //cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    atomic<int> completed_lines(0);
    atomic<int> remaining_lines(image_height);

    int max_threads = max(1, static_cast<int>(thread::hardware_concurrency()) - threads_left);
    atomic<int> next_line(0);
    mutex print_mutex;  // Ensure safe printing

    auto render_worker = [&]() {
      while (true) {
        int j = next_line.fetch_add(1);
        if (j >= image_height) break;  // Prevent out-of-bounds access

        vector<color> line(image_width);  // Store locally to avoid conflicts
        for (int i = 0; i < image_width; i++) {
          color pixel_color(0, 0, 0);

          for (int s_j = 0; s_j < sqrt_spp; s_j++) {
            for (int s_i = 0; s_i < sqrt_spp; s_i++) {
              ray r = get_ray(i, j, s_i, s_j);
              pixel_color += ray_color(r, max_depth, world, lights);
            }
          }


          line[i] = pixel_samples_scale * pixel_color;
        }

        // Copy line safely into the main image buffer
        image[j] = std::move(line);

        completed_lines++;
        remaining_lines--;

        // Safe printing using mutex
        {
          lock_guard<mutex> lock(print_mutex);
          clog << "\rLines completed: " << completed_lines
               << " | Lines remaining: " << remaining_lines << flush;
        }
      }
    };

    // Spawn worker threads
    vector<thread> threads;
    for (int i = 0; i < max_threads; i++) {
      threads.emplace_back(render_worker);
    }

    // Join threads
    for (auto& t : threads) {
      t.join();
    }

    clog << "\nDone.\n";
    clog << "Writing to file...\n";
    write_image_buffer_to_file(image);
  }

private:
  vec3 pixel00_loc; //location of pixel 0,0
  vec3 pixel_delta_u;
  vec3 pixel_delta_v;
  point3 center;
  double pixel_samples_scale; //colour scale factor for a sum of pixel samples
  int    sqrt_spp;             // Square root of number of samples per pixel
  double recip_sqrt_spp;       // 1 / sqrt_spp
  vec3 u, v, w;
  vec3 defocus_disk_u;
  vec3 defocus_disk_v;


  void write_image_buffer_to_file(vector< vector<color> >& image) {

    int height = image.size();
    int width = image[0].size();

    // PPM header
    cout << "P3\n" << width << " " << height << "\n255\n";

    // Write pixel data
    for (const auto& row : image) {
      for (const auto& pixel : row) {
        write_color(std::cout, pixel);
      }
      cout << "\n";
    }
  }



    void initialize() {

      //viewport width is calculated based off height and image aspect ratio
      double theta = degrees_to_radians(vfov);
      double h = std::tan(theta/2);
      auto viewport_height = 2*h * focus_dist;
      double viewport_width = viewport_height * (static_cast<double>(image_width) / static_cast<double>(image_height));

      w = unit_vector(lookfrom - lookat);
      u = unit_vector(cross(vup, w));
      v = cross(w, u);

      sqrt_spp = int(std::sqrt(samples_per_pixel));
      pixel_samples_scale = 1.0 / (sqrt_spp * sqrt_spp);
      recip_sqrt_spp = 1.0 / sqrt_spp;

      center = lookfrom;


      //horizontal vector points to the right
      vec3 viewport_u = viewport_width * u;
      //and vertical points down
      vec3 viewport_v = viewport_height * -v;

      //Calculate the delta vectors from pixel to pixel
      pixel_delta_u = viewport_u / image_width;
      pixel_delta_v = viewport_v / image_height;

      //Calculate the location of the upper left pixel
      //viewportUpperLeft = {half of width, half of height, distance from camera to viewport}
      vec3 viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
      pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

      //calculate defocus disk basis vectors
      auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
      defocus_disk_u = u * defocus_radius;
      defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j, int s_i, int s_j) const {
        // Construct a camera ray originating from the defocus disk and directed at randomly sampled
        // point around the pixel location i, j for stratified sample square s_i, s_j

        auto offset = sample_square_stratified(s_i, s_j);
        auto pixel_sample = pixel00_loc
                          + ((i + offset.x()) * pixel_delta_u)
                          + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
      }

    vec3 sample_square_stratified(int s_i, int s_j) const {
      // Returns the vector to a random point in the square sub-pixel specified by grid
      // indices s_i and s_j, for an idealized unit square pixel [-.5,-.5] to [+.5,+.5].

      auto px = ((s_i + random_double()) * recip_sqrt_spp) - 0.5;
      auto py = ((s_j + random_double()) * recip_sqrt_spp) - 0.5;

      return vec3(px, py, 0);
    }

    vec3 sample_square() const {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
      }

    point3 defocus_disk_sample() const {
      //returns a random point inside the defocus disk
      auto p = random_in_unit_disk();
      return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world, const hittable& lights) const {
      //if we've exceeded the ray bounce limit, no more light is gathered
      if (depth <= 0)
        return color(0,0,0);
      hit_record rec;

      // If the ray hits nothing, return the background color.
      if (!world.hit(r, interval(0.001, infinity), rec))
        return background;

      ray scattered;
      color attenuation;
      double pdf_value;
      color color_from_emission = rec.mat->emitted(r, rec, rec.u, rec.v, rec.p);

      if (!rec.mat->scatter(r, rec, attenuation, scattered, pdf_value))
        return color_from_emission;

      hittable_pdf light_pdf(lights, rec.p);
      scattered = ray(rec.p, light_pdf.generate(), r.time());
      pdf_value = light_pdf.value(scattered.direction());

     double scattering_pdf = rec.mat->scattering_pdf(r, rec, scattered);

    color sample_color = ray_color(scattered, depth-1, world, lights);
    color color_from_scatter = (attenuation * scattering_pdf * sample_color) / pdf_value;

      return color_from_emission + color_from_scatter;
    }

};


#endif //CAMERA_H
