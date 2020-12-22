#include <string>
#include "../headers/data_structures.h"

using namespace std;
using namespace algorithms;

Triangle::Triangle(Vec3D *v0, Vec3D *v1, Vec3D *v2) {
    id_ = generate_id();
    _vertices[0] = v0;
    _vertices[1] = v1;
    _vertices[2] = v2;
}

Triangle::~Triangle() {
}

int Triangle::generate_id() {
    static int id = 0;
    return id++;
}

bool Triangle::has_vertex_coincident_with_d(Vec3D *dot) {
    return _vertices[0]->check_for_coincidence(dot)
           || _vertices[1]->check_for_coincidence(dot)
           || _vertices[2]->check_for_coincidence(dot);
}

void Triangle::assign_neighbouring_triangles(Triangle *n0, Triangle *n1, Triangle *n2) {
    _neighbours[0] = n0;
    _neighbours[1] = n1;
    _neighbours[2] = n2;
}

string Triangle::to_string() {
    return "Triangle ID: " + ::to_string(id_) + ";\n"
           + "_vertices[0]: " + _vertices[0]->to_string()
           + "_vertices[1]: " + _vertices[1]->to_string()
           + "_vertices[2]: " + _vertices[2]->to_string()
           + "_neighbours[0] ID: " + ::to_string(_neighbours[0]->id_) + ", "
           + "_neighbours[1] ID: " + ::to_string(_neighbours[1]->id_) + ", "
           + "_neighbours[2] ID: " + ::to_string(_neighbours[2]->id_) + ";\n";
}