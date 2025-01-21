#include "include/color.h"
#include "include/vec3.h"
#include "include/ray.h"
#include <iostream>

using namespace std;

double hitSphere(const point3& center, double radius, const ray& r) {
    vec3 oc = center - r.origin();
    double a = dot(r.direction(), r.direction());
    double b = -2.0 * dot(r.direction(), oc);
    double c = dot(oc, oc) - radius*radius;
    double discriminant = b*b - 4*a*c;

    if (discriminant < 0) {
        return -1.0;
    } else {
        return (-b - sqrt(discriminant) ) / (2.0*a);
    }
}

color rayColor(const ray& r) {

    auto t = hitSphere(point3(0,0,-1), 0.5, r);
    if (t > 0.0) {
        vec3 N = unit_vector(r.at(t) - vec3(0,0,-1));
        return 0.5*color(N.x()+1, N.y()+1, N.z()+1);
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

            color pixelColor = rayColor(r);


            write_color(std::cout, pixelColor);

        }
    }

    clog << "\rDone.            \n";

    return 0;
}

