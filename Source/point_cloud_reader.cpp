#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "../headers/point_cloud.h"

using namespace std;
using namespace algorithms;

vector<Vec3D *> point_cloud_reader::get_point_cloud() {
    vector<Vec3D *> points = vector<Vec3D *>();

    cout << "Loading: ";
    string filename = "Resource/sample.txt";

    ifstream file(filename);

    double x = 0, y = 0, z = 0;
    int _red = 0, _green = 0, _blue = 0;
    char hex;

    while (file >> hex) {
        file >> x >> y >> z >> _red >> _green >> _blue;
        Vec3D *dot = new Vec3D(x, y, z, (uint8_t) _red, (uint8_t) _green, (uint8_t) _blue);
        points.push_back(dot);
    }

    file.close();

    return points;
}
