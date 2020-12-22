#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

namespace algorithms {
    class Vec3D {
    private:
        int generate_id();

    public:
        int Id = 0;

        // coordinate
        double X, Y, Z;

        // color
        uint8_t R, G, B;

        bool was_visited = false;
        bool is_auxillary_point = false;

        Vec3D(double x, double y, double z, uint8_t r = 255, uint8_t g = 248, uint8_t b = 220);

        Vec3D(double x, double y, double z, bool isAuxiliaryDot, uint8_t r = 255, uint8_t g = 248, uint8_t b = 220);

        Vec3D(Vec3D *dot, double lengthAfterProjection);

        ~Vec3D();

        bool check_for_coincidence(Vec3D *dot);

        std::string to_string();
    };

    class Triangle {
    private:
        int generate_id();

    public:
        int id_ = 0;

        // 3 vertices
        Vec3D *_vertices[3];

        // 3 neighbors
        Triangle *_neighbours[3];

        Triangle(Vec3D *v0, Vec3D *v1, Vec3D *v2);

        ~Triangle();

        bool has_vertex_coincident_with_d(Vec3D *dot);

        void assign_neighbouring_triangles(Triangle *n0, Triangle *n1, Triangle *n2);

        std::string to_string();
    };
}

#endif