#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "data_structures.h"

namespace algorithms {
    class _viz {
    public:
        bool show_wireframe;

        _viz(bool isShowWireframe = true);
        ~_viz();

        void create_viz(std::vector<Vec3D *> &dots, std::vector<std::tuple<int, int, int> *> &mesh);
    };
}

#endif