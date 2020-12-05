//
// Created by 安頔 on 2020/12/5.
//

#ifndef PROJECT_GRAPHCUT_H
#define PROJECT_GRAPHCUT_H

#include "texture.h"

class GraphCut{
public:
    explicit GraphCut(const char *);
    ~GraphCut();

    bool run(int, int, int iter = -1);

private:
    Texture *patch;
    Texture *result;
};

#endif //PROJECT_GRAPHCUT_H
