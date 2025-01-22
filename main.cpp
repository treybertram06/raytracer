#include "include/usefulstuff.h"

#include "include/hittable.h"
#include "include/hittable_list.h"
#include "include/sphere.h"
#include "include/camera.h"

using namespace std;


int main() {

    hittable_list world;

    world.add(make_shared<sphere>(point3(0,0,-1), 0.5));
    world.add(make_shared<sphere>(point3(0.5,0.5,-1), 0.25));
    //floorish
    world.add(make_shared<sphere>(point3(0,-1000.5,-1), 1000));

    camera cam;

    cam.imageWidth = 1280;
    cam.imageHeight = 720;
    cam.samples_per_pixel = 25;

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

