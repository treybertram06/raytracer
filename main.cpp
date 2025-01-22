#include "include/usefulstuff.h"

#include "include/hittable.h"
#include "include/hittable_list.h"
#include "include/sphere.h"

using namespace std;


color rayColor(const ray& r, const hittable& world) {

    hit_record rec;
    if (world.hit(r, 0, infinity, rec)) {
        return 0.5 * (rec.normal + color(1,1,1));
    }

    vec3 unitDirection = unit_vector(r.direction());
    double a = 0.5*(unitDirection.y() + 1.0);
    return (1.0-a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);

}

int main() {

    constexpr int imageWidth = 1280;
    constexpr int imageHeight = 720;

    //Camera
    double focalLength = 1.0;
    double viewportHeight = 2.0;

    //world

    hittable_list world;

    world.add(make_shared<sphere>(point3(0,0,-1), 0.5));
    world.add(make_shared<sphere>(point3(0,-100.5,-1), 100));


    //viewport width is calculated based off height and image aspect ratio
    double viewportWidth = viewportHeight * (static_cast<double>(imageWidth) / static_cast<double>(imageHeight));
    point3 cameraCenter = point3(0, 0, 0);

    // Calculate the vectors across the horizontal and down vertical edges

    //horizontal vector points to the right
    vec3 viewportU = vec3(viewportWidth, 0, 0);
    //and vertical points down
    vec3 viewportV = vec3(0, -viewportHeight, 0);

    //Calculate the delta vectors from pixel to pixel
    vec3 pixelDeltaU = viewportU / imageWidth;
    vec3 pixelDeltaV = viewportV / imageHeight;

    //Calculate the location of the upper left pixel
    //viewportUpperLeft = {half of width, half of height, distance from camera to viewport}
    vec3 viewportUpperLeft = cameraCenter - vec3(0, 0, focalLength) - viewportU/2 - viewportV/2;
    vec3 pixel00Loc = viewportUpperLeft + 0.5 * (pixelDeltaU + pixelDeltaV);





    //Render
    cout << "P3\n" << imageWidth << ' ' << imageHeight << "\n255\n";

    for (int j = 0; j < imageHeight; j++) {
        clog << "\rScanlines remaining: " << (imageHeight - j) << ' ' << flush;
        for (int i = 0; i < imageWidth; i++) {

            vec3 pixelCenter = pixel00Loc + (i * pixelDeltaU) + (j * pixelDeltaV);
            vec3 rayDirection = pixelCenter - cameraCenter;
            ray r(cameraCenter, rayDirection);

            color pixelColor = rayColor(r, world);


            write_color(std::cout, pixelColor);

        }
    }

    clog << "\rDone.            \n";

    return 0;
}

