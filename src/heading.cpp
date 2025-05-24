// Used to determine the heading
#include <cmath>

const float PI = 3.14159265f;

float heading(float mag_y, float mag_x){
    float heading = atan2f(mag_y, -mag_x) * 180 / PI;
    if (heading < 0){
        heading += 360.0;
    }

    //TODO could use the declination to get it in terms of true north
    return heading;
}
