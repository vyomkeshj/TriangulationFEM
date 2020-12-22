#include <iostream>
#include <string>
#include <vector>
#include "headers/point_cloud.h"
#include "headers/delaunay_triangulation.h"
#include "headers/_viz.h"

using namespace std;
using namespace algorithms;

void ClearMemory(vector<Vec3D *> &, vector<tuple<int, int, int> *> &);

int main() {
    try {
        int cmd;
        cout << "\n 1 = Random Points\n 2 = Import From File!\n";
        cin >> cmd;

        vector<Vec3D *> points
                = cmd == 1
                  ? point_cloud_generator().get_spherical_points()
                  : point_cloud_reader().get_point_cloud();

        delaunay_triangulation_process triangulation = delaunay_triangulation_process();
        vector<tuple<int, int, int> *> mesh = triangulation.get_triangulation_result(points);
        cout << triangulation.get_stats() << endl;

        _viz visualization = _viz(false);
        visualization.create_viz(points, mesh);

        ClearMemory(points, mesh);
    }
    catch (exception e) {
        cout << e.what() << endl;
        //system("pause");
    }

    return 0;
}

void ClearMemory(vector<Vec3D *> &points, vector<tuple<int, int, int> *> &_mesh) {
    vector<Vec3D *>::iterator points_vec_iterator;
    for (points_vec_iterator = points.begin(); points_vec_iterator != points.end(); points_vec_iterator++) {
        delete *points_vec_iterator;
    }

    vector<tuple<int, int, int> *>::iterator mesh_vec_iterator;
    for (mesh_vec_iterator = _mesh.begin(); mesh_vec_iterator != _mesh.end(); mesh_vec_iterator++) {
        delete *mesh_vec_iterator;
    }
}
