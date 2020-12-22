#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#define INIT_VERTICES_COUNT 6 /* count of vertices in the initial hull */
#define INIT_FACES_COUNT 8 /* count of faces in the initial hull */
#define VECTOR_LENGTH 0.5 /* radius of unit sphere the dots projected into */

#define DBL_EPSILON 1

#include "data_structures.h"

namespace algorithms {
    class delaunay_triangulation_process {
    private:
        Vec3D *_aux_points[INIT_VERTICES_COUNT];
        std::vector<Vec3D *> *_projected_points;
        std::vector<Triangle *> *_mesh;
        long stats[4];

        void find_convex_hull(std::vector<Vec3D *> *dots);

        void insert_point_to_cloud(Vec3D *dot);

        void remove_extra_triangles();

        void split_triangle(Triangle *triangle, Vec3D *dot);

        void fix_neighbourhood(Triangle *target, Triangle *oldNeighbor, Triangle *newNeighbor);

        void perform_local_optimization(Triangle *t0, Triangle *t1);

        bool swap_diagonal(Triangle *t0, Triangle *t1);

        bool is_min_in_array(double *arr, int length, int index);

        double get_distance(Vec3D *v0, Vec3D *v1);

        double get_determinant(double *matrix);

        double get_determinant(Vec3D *v0, Vec3D *v1, Vec3D *v2);


    public:
        delaunay_triangulation_process();

        ~delaunay_triangulation_process();

        std::vector<std::tuple<int, int, int> *> get_triangulation_result(std::vector<Vec3D *> &dots);

        std::string get_stats();
    };
}

#endif