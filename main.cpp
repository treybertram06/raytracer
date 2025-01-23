#include "include/usefulstuff.h"

#include "include/hittable.h"
#include "include/hittable_list.h"
#include "include/material.h"
#include "include/sphere.h"
#include "include/camera.h"

using namespace std;


int main() {

    hittable_list world;

    /*
    auto material_ground = make_shared<lambertian>(color(0.8, 0.8, 0.0));
    auto material_center = make_shared<lambertian>(color(0.1, 0.2, 0.5));
    auto material_left   = make_shared<dielectric>(1.50);
    auto material_bubble = make_shared<dielectric>(1.00 / 1.50);
    auto material_right  = make_shared<metal>(color(0.8, 0.6, 0.2), 0.1);

    world.add(make_shared<sphere>(point3( 0.0, -100.5, -1.0), 100.0, material_ground));
    world.add(make_shared<sphere>(point3( 0.0,    0.0, -1.2),   0.5, material_center));
    world.add(make_shared<sphere>(point3(-1.0,    0.0, -1.0),   0.5, material_left));
    world.add(make_shared<sphere>(point3(-1.0,    0.0, -1.0),   0.4, material_bubble));
    world.add(make_shared<sphere>(point3( 1.0,    0.0, -1.0),   0.5, material_right));
    */

    auto R = std::cos(pi/4);

    auto material_left = make_shared<lambertian>(color(0,0,1));
    auto material_right = make_shared<lambertian>(color(1,0,0));

    world.add(make_shared<sphere>(point3(-R, 0, -1), R, material_left));
    world.add(make_shared<sphere>(point3( R, 0, -1), R, material_right));



    camera cam;

    cam.imageWidth = 640;
    cam.imageHeight = 360;
    cam.samples_per_pixel = 25;
    cam.maxDepth = 25;

    cam.vfov = 60;

    /*
    clog << "Enter image width: ";
    cin >> cam.imageWidth;
    clog << "Enter image height: ";
    cin >> cam.imageHeight;
    clog << "Enter sampling rate: ";
    cin >> cam.samples_per_pixel;
    */


    cam.render(world);





    return 0;
}

