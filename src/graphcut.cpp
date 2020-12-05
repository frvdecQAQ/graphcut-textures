//
// Created by 安頔 on 2020/12/5.
//

#include <graphcut.h>

GraphCut::GraphCut(const char *img_path) {
    patch = new Texture(img_path);
    result = nullptr;
}

GraphCut::~GraphCut() {
    delete patch;
    delete result;
}

bool GraphCut::run(int h, int w, int iter) {
    if(!patch->get_has_read())return false;
    result = new Texture(h, w, patch->texture.type());
    return true;
}



