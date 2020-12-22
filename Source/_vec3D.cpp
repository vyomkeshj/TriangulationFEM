#include <cmath>
#include <string>
#include "../headers/data_structures.h"

using namespace std;
using namespace algorithms;

Vec3D::Vec3D(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b) {
    Id = generate_id();

    X = x;
    Y = y;
    Z = z;

    R = r;
    G = g;
    B = b;
}

Vec3D::Vec3D(double x, double y, double z, bool isAuxiliaryDot, uint8_t r, uint8_t g, uint8_t b) {
    Id = generate_id();

    is_auxillary_point = isAuxiliaryDot;

    X = x;
    Y = y;
    Z = z;

    R = r;
    G = g;
    B = b;
}

Vec3D::Vec3D(Vec3D *dot, double lengthAfterProjection) {
    Id = dot->Id;
    was_visited = dot->was_visited;
    is_auxillary_point = dot->is_auxillary_point;

    double length = sqrt(pow(dot->X, 2) + pow(dot->Y, 2) + pow(dot->Z, 2));
    double scaleFactor = lengthAfterProjection / length;

    X = scaleFactor * dot->X;
    Y = scaleFactor * dot->Y;
    Z = scaleFactor * dot->Z;

    R = dot->R;
    G = dot->G;
    B = dot->B;
}

Vec3D::~Vec3D() {
}

int Vec3D::generate_id() {
    static int id = 0;
    return id++;
}

bool Vec3D::check_for_coincidence(Vec3D *dot) {
    return (X == dot->X && Y == dot->Y && Z == dot->Z);
}

string Vec3D::to_string() {
    return "Dot ID: " + ::to_string(Id) + "; "
           + ::to_string(X) + ", " + ::to_string(Y) + ", " + ::to_string(Z) + "; "
           + ::to_string(R) + ", " + ::to_string(G) + ", " + ::to_string(B) + "; "
           + "was_visited: " + (was_visited ? "true" : "false") + "; "
           + "is_auxillary_point: " + (is_auxillary_point ? "true" : "false") + ";\n";
}

