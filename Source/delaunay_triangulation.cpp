#include <regex>
#include <string>
#include <tuple>
#include <vector>
#include "../headers/delaunay_triangulation.h"
#include <iostream>

using namespace std;
using namespace algorithms;

delaunay_triangulation_process::delaunay_triangulation_process() {
    for (int i = 0; i < INIT_VERTICES_COUNT; i++) {
        _aux_points[i] = new Vec3D(
                (i % 2 == 0 ? 1 : -1) * (i / 2 == 0 ? VECTOR_LENGTH : 0),
                (i % 2 == 0 ? 1 : -1) * (i / 2 == 1 ? VECTOR_LENGTH : 0),
                (i % 2 == 0 ? 1 : -1) * (i / 2 == 2 ? VECTOR_LENGTH : 0),
                true, 0, 0, 0
        );
    }

    _projected_points = new vector<Vec3D *>();
    _mesh = new vector<Triangle *>();

    for (int i = 0; i < sizeof(stats) / sizeof(long); i++) {
        stats[i] = 0;
    }
}

delaunay_triangulation_process::~delaunay_triangulation_process() {
    for (int i = 0; i < INIT_VERTICES_COUNT; i++) {
        delete _aux_points[i];
    }

    vector<Vec3D *>::iterator point_iterator;
    for (point_iterator = _projected_points->begin(); point_iterator != _projected_points->end(); point_iterator++) {
        delete *point_iterator;
    }

    vector<Triangle *>::iterator mesh_iterator;
    for (mesh_iterator = _mesh->begin(); mesh_iterator != _mesh->end(); mesh_iterator++) {
        delete *mesh_iterator;
    }

    delete _projected_points;
    delete _mesh;
}

vector<tuple<int, int, int> *> delaunay_triangulation_process::get_triangulation_result(vector<Vec3D *> &dots) {
    stats[2] = clock();

    _projected_points->reserve(dots.size());
    _mesh->reserve(8 + (dots.size() - 6) * 2);  // n random points = form 8+(N-6)*2 triangles

    // project dots to an unit shpere for triangulation
    vector<Vec3D *>::iterator itDots;     // iterator over the dots vector input, here we convert from input dots to _ProjDots
    //TODO: OMP acceleration possible, this is a long loop, but linear is faster because of the critical section

    for (itDots = dots.begin(); itDots != dots.end(); itDots++) {
        Vec3D *projectedDot = new Vec3D((*itDots), VECTOR_LENGTH);
        _projected_points->push_back(projectedDot);
    }
    // prepare initial convex hull with 6 vertices and 8 triangle faces
    find_convex_hull(_projected_points);

//TODO: examine open mp acceleration case
//For each dot in projected dots, if the dot has not been prev. visited, insert_point_to_cloud(dot)

    long test1 = clock();
#pragma omp parallel shared(_projected_points)
    {
#pragma omp for nowait
        for (int i = 0; i < _projected_points->size(); i++) {
            Vec3D *dot = _projected_points->at(i);
            if (!dot->was_visited) {
                insert_point_to_cloud(dot);
            }
        }
    }


/*
    for (itDots = _projected_points->begin(); itDots != _projected_points->end(); itDots++) {
        Vec3D *dot = *itDots;    //dereference the iterator to get_distance the dot vector
        if (!dot->was_visited) {
            insert_point_to_cloud(dot);
        }
    }*/
    long test2 = clock();
    cout << "vector translation took" << test2 - test1 << endl;



    // remove trianges connected with auxiliary dots
    remove_extra_triangles();

    // generate output
    vector<tuple<int, int, int> *> mesh = vector<tuple<int, int, int> *>();

    vector<Triangle *>::iterator itMesh;
    for (itMesh = _mesh->begin(); itMesh != _mesh->end(); itMesh++) {
        Triangle *triangle = *itMesh;
        mesh.push_back(new tuple<int, int, int>(
                triangle->_vertices[0]->Id,
                triangle->_vertices[1]->Id,
                triangle->_vertices[2]->Id
        ));
    }

    stats[3] = clock();

    return mesh;
}

void delaunay_triangulation_process::find_convex_hull(vector<Vec3D *> *dots) {
    Vec3D *initialVertices[INIT_VERTICES_COUNT];
    Triangle *initialHullFaces[INIT_FACES_COUNT];

    for (int i = 0; i < INIT_VERTICES_COUNT; i++) {
        initialVertices[i] = _aux_points[i];
    }

    // if close enough, use input dots to replace auxiliary dots so won't be removed in the end
    double minDistance[INIT_VERTICES_COUNT] = {0, 0, 0, 0, 0, 0};
    vector<Vec3D *>::iterator it;
    for (it = dots->begin(); it != dots->end(); it++) {
        double distance[INIT_VERTICES_COUNT];
        for (int i = 0; i < INIT_VERTICES_COUNT; i++) {
            distance[i] = get_distance(_aux_points[i], *it);
            if (minDistance[i] == 0 || distance[i] < minDistance[i]) {
                minDistance[i] = distance[i];
            }
        }

        for (int i = 0; i < INIT_VERTICES_COUNT; i++) {
            if (minDistance[i] == distance[i] && is_min_in_array(distance, INIT_VERTICES_COUNT, i)) {
                initialVertices[i] = *it;
            }
        }
    }

    int vertex0Index[] = {0, 0, 0, 0, 1, 1, 1, 1};
    int vertex1Index[] = {4, 3, 5, 2, 2, 4, 3, 5};
    int vertex2Index[] = {2, 4, 3, 5, 4, 3, 5, 2};

    for (int i = 0; i < INIT_FACES_COUNT; i++) {
        Vec3D *v0 = initialVertices[vertex0Index[i]];
        Vec3D *v1 = initialVertices[vertex1Index[i]];
        Vec3D *v2 = initialVertices[vertex2Index[i]];

        Triangle *triangle = new Triangle(v0, v1, v2);
        initialHullFaces[i] = triangle;

        _mesh->push_back(triangle);
    }

    int neighbor0Index[] = {1, 2, 3, 0, 7, 4, 5, 6};
    int neighbor1Index[] = {4, 5, 6, 7, 0, 1, 2, 3};
    int neighbor2Index[] = {3, 0, 1, 2, 5, 6, 7, 4};

    for (int i = 0; i < INIT_FACES_COUNT; i++) {
        Triangle *n0 = initialHullFaces[neighbor0Index[i]];
        Triangle *n1 = initialHullFaces[neighbor1Index[i]];
        Triangle *n2 = initialHullFaces[neighbor2Index[i]];
        initialHullFaces[i]->assign_neighbouring_triangles(n0, n1, n2);
    }

    // dot already in the mesh, avoid being visited by insert_point_to_cloud() again
    for (int i = 0; i < INIT_VERTICES_COUNT; i++) {
        initialVertices[i]->was_visited = true;
    }
}

void delaunay_triangulation_process::insert_point_to_cloud(Vec3D *dot) {
    double det_1 = 0, det_2 = 0, det_3 = 0;

    vector<Triangle *>::iterator it;
    it = _mesh->begin();
    Triangle *triangle = *it;
    while (it != _mesh->end()) {
        stats[0]++;


        det_1 = get_determinant(triangle->_vertices[0], triangle->_vertices[1], dot);
        det_2 = get_determinant(triangle->_vertices[1], triangle->_vertices[2], dot);
        det_3 = get_determinant(triangle->_vertices[2], triangle->_vertices[0], dot);

        // if this dot projected into an existing triangle, split the existing triangle to 3 new ones
        if (det_1 >= 0 && det_2 >= 0 && det_3 >= 0) {
            if (!triangle->has_vertex_coincident_with_d(dot)) {

                split_triangle(triangle,
                               dot);       //the call inside the ifs, 2463 -> 1187 or 1448 ~1000ms, around 1-2 msec per exec
            }
            return;
        } else if (det_2 >= 0 && det_3 >= 0)  // on one side, search neighbors
            triangle = triangle->_neighbours[0];
        else if (det_1 >= 0 && det_3 >= 0)
            triangle = triangle->_neighbours[1];
        else if (det_1 >= 0 && det_2 >= 0)
            triangle = triangle->_neighbours[2];

            // cannot determine effectively
        else if (det_1 > 0)
            triangle = triangle->_neighbours[1];
        else if (det_2 > 0)
            triangle = triangle->_neighbours[2];
        else if (det_3 > 0)
            triangle = triangle->_neighbours[0];
        else {
            triangle = *it++;
        }


    }
}

void delaunay_triangulation_process::remove_extra_triangles() {
    vector<Triangle *>::iterator it;
    for (it = _mesh->begin(); it != _mesh->end();) {
        Triangle *triangle = *it;
        bool isExtraTriangle = false;
        for (int i = 0; i < 3; i++) {
            if (triangle->_vertices[i]->is_auxillary_point) {
                isExtraTriangle = true;
                break;
            }
        }

        if (isExtraTriangle) {
            delete *it;
            it = _mesh->erase(it);
        } else {
            it++;
        }
    }
}

// takes 1-2 msec per call
void delaunay_triangulation_process::split_triangle(Triangle *triangle, Vec3D *dot) {

    Triangle *newTriangle1 = new Triangle(dot, triangle->_vertices[1], triangle->_vertices[2]);
    Triangle *newTriangle2 = new Triangle(dot, triangle->_vertices[2], triangle->_vertices[0]);

    triangle->_vertices[2] = triangle->_vertices[1];
    triangle->_vertices[1] = triangle->_vertices[0];
    triangle->_vertices[0] = dot;

    newTriangle1->assign_neighbouring_triangles(triangle, triangle->_neighbours[1], newTriangle2);
    newTriangle2->assign_neighbouring_triangles(newTriangle1, triangle->_neighbours[2], triangle);
    triangle->assign_neighbouring_triangles(newTriangle2, triangle->_neighbours[0], newTriangle1);

    fix_neighbourhood(newTriangle1->_neighbours[1], triangle, newTriangle1);
    fix_neighbourhood(newTriangle2->_neighbours[1], triangle, newTriangle2);

    _mesh->push_back(newTriangle1);
    _mesh->push_back(newTriangle2);
    // optimize triangles according to delaunay triangulation definition
//#pragma omp parallel
//    {
//#pragma omp single
//        {
//#pragma omp task
//            {
    perform_local_optimization(triangle, triangle->_neighbours[1]);
//            }
//#pragma omp task
//            {
    perform_local_optimization(newTriangle1, newTriangle1->_neighbours[1]);
//            }
//#pragma omp task
//            {
    perform_local_optimization(newTriangle2, newTriangle2->_neighbours[1]);
//            }
//        }
//    }
}


void delaunay_triangulation_process::fix_neighbourhood(Triangle *target, Triangle *oldNeighbor, Triangle *newNeighbor) {
    for (int i = 0; i < 3; i++) {
        if (target->_neighbours[i] == oldNeighbor) {
            target->_neighbours[i] = newNeighbor;
            break;
        }
    }
}

void delaunay_triangulation_process::perform_local_optimization(Triangle *t0, Triangle *t1) {
    stats[1]++;

    for (int i = 0; i < 3; i++) {
        if (t1->_vertices[i] == t0->_vertices[0] ||
            t1->_vertices[i] == t0->_vertices[1] ||
            t1->_vertices[i] == t0->_vertices[2]) {
            continue;
        }

        double matrix[] = {
                t1->_vertices[i]->X - t0->_vertices[0]->X,
                t1->_vertices[i]->Y - t0->_vertices[0]->Y,
                t1->_vertices[i]->Z - t0->_vertices[0]->Z,

                t1->_vertices[i]->X - t0->_vertices[1]->X,
                t1->_vertices[i]->Y - t0->_vertices[1]->Y,
                t1->_vertices[i]->Z - t0->_vertices[1]->Z,

                t1->_vertices[i]->X - t0->_vertices[2]->X,
                t1->_vertices[i]->Y - t0->_vertices[2]->Y,
                t1->_vertices[i]->Z - t0->_vertices[2]->Z
        };

        if (get_determinant(matrix) <= 0) {
            // terminate after optimized
            break;
        }

        if (swap_diagonal(t0, t1)) {
            return;
        }
    }
}

bool delaunay_triangulation_process::swap_diagonal(Triangle *t0, Triangle *t1) {
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            if (t0->_vertices[j] != t1->_vertices[0] &&
                t0->_vertices[j] != t1->_vertices[1] &&
                t0->_vertices[j] != t1->_vertices[2] &&
                t1->_vertices[k] != t0->_vertices[0] &&
                t1->_vertices[k] != t0->_vertices[1] &&
                t1->_vertices[k] != t0->_vertices[2]) {
                t0->_vertices[(j + 2) % 3] = t1->_vertices[k];
                t1->_vertices[(k + 2) % 3] = t0->_vertices[j];

                t0->_neighbours[(j + 1) % 3] = t1->_neighbours[(k + 2) % 3];
                t1->_neighbours[(k + 1) % 3] = t0->_neighbours[(j + 2) % 3];
                t0->_neighbours[(j + 2) % 3] = t1;
                t1->_neighbours[(k + 2) % 3] = t0;


                fix_neighbourhood(t0->_neighbours[(j + 1) % 3], t1, t0);
                fix_neighbourhood(t1->_neighbours[(k + 1) % 3], t0, t1);

                perform_local_optimization(t0, t0->_neighbours[j]);
                perform_local_optimization(t0, t0->_neighbours[(j + 1) % 3]);
                perform_local_optimization(t1, t1->_neighbours[k]);
                perform_local_optimization(t1, t1->_neighbours[(k + 1) % 3]);
            }
        }
    }

    return true;
}


bool delaunay_triangulation_process::is_min_in_array(double *arr, int length, int index) {
    for (int i = 0; i < length; i++) {
        if (arr[i] < arr[index]) {
            return false;
        }
    }

    return true;
}

double delaunay_triangulation_process::get_distance(Vec3D *v0, Vec3D *v1) {
    return sqrt(pow((v0->X - v1->X), 2) +
                pow((v0->Y - v1->Y), 2) +
                pow((v0->Z - v1->Z), 2));
}

double delaunay_triangulation_process::get_determinant(Vec3D *v0, Vec3D *v1, Vec3D *v2) {
    double matrix[] = {
            v0->X, v0->Y, v0->Z,
            v1->X, v1->Y, v1->Z,
            v2->X, v2->Y, v2->Z
    };

    return get_determinant(matrix);
}

double delaunay_triangulation_process::get_determinant(double *matrix) {
//todo: can I break it down and parallelize?
    // inversed for left handed coordinate system
    double determinant = matrix[2] * matrix[4] * matrix[6]
                         + matrix[0] * matrix[5] * matrix[7]
                         + matrix[1] * matrix[3] * matrix[8]
                         - matrix[0] * matrix[4] * matrix[8]
                         - matrix[1] * matrix[5] * matrix[6]
                         - matrix[2] * matrix[3] * matrix[7];

    // adjust result based on float number accuracy, otherwise causing deadloop
    return abs(determinant) <= DBL_EPSILON ? 0 : determinant;
}

string delaunay_triangulation_process::get_stats() {
    // display thousands separator
    regex regex("\\d{1,3}(?=(\\d{3})+$)");

    return "\nTriangle count: "
           + regex_replace(to_string(_mesh->size()), regex, "$&,")
           + "\nTriangle search operations: "
           + regex_replace(to_string(stats[0]), regex, "$&,")
           + "\nLocal optimizations: "
           + regex_replace(to_string(stats[1]), regex, "$&,")
           + "\nTriangulation cost: "
           + to_string(stats[3] - stats[2])
           + "ms\n";
}