#ifndef DOTCLOUD_H
#define DOTCLOUD_H

#include "data_structures.h"

namespace algorithms {
    class point_cloud_generator {
    private:
        Vec3D *get_random_point();

        Vec3D *get_evenly_distributed_points();

    public:
        std::vector<Vec3D *> get_spherical_points();
    };

    class point_cloud_reader {
    public:
        std::vector<Vec3D *> get_point_cloud();
    };
}

#endif