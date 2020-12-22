#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include "../headers/point_cloud.h"

#define PI 3.14159265
#define RADIUS 100

using namespace std;
using namespace algorithms;

vector<Vec3D *> point_cloud_generator::get_spherical_points() {
    int point_count;
    cout << "Number of Points: ";
    cin >> point_count;

    ofstream file("Resource\\random_out.txt");
    vector<Vec3D *> points = vector<Vec3D *>();
    points.reserve(point_count);

    srand((unsigned) time(NULL));
    for (int i = 0; i < point_count; i++) {
        Vec3D *new_pointts = get_evenly_distributed_points();
        file << "# " << new_pointts->X << " " << new_pointts->Y << " " << new_pointts->Z << " "
             << new_pointts->R << " " << new_pointts->G << " " << new_pointts->B << " " << endl;
        points.push_back(new_pointts);
    }
    file.close();
    return points;
}

Vec3D *point_cloud_generator::get_random_point() {
    // use spherical coordinate
    double phi = (rand() % 360) * PI / 180;
    double theta = (rand() % 360) * PI / 180;

    double x = RADIUS * sin(theta) * cos(phi);
    double y = RADIUS * sin(theta) * sin(phi);
    double z = RADIUS * cos(theta);

    return new Vec3D(x, y, z);
}

Vec3D *point_cloud_generator::get_evenly_distributed_points() {
    // project random dot in cartesian coordinate to unit sphere
    double x = rand() % 2000 - 1000;
    double y = rand() % 2000 - 1000;
    double z = rand() % 2000 - 1000;

    double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    x = RADIUS * x / r;
    y = RADIUS * y / r;
    z = RADIUS * z / r;

    return new Vec3D(x, y, z);
}