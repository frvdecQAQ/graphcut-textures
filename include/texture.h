//
// Created by 安頔 on 2020/12/5.
//

#ifndef PROJECT_TEXTURE_H
#define PROJECT_TEXTURE_H

#include <cstdio>
#include <opencv2/opencv.hpp>

class Texture{
public:
    cv::Mat texture;
    explicit Texture(const char *);
    Texture(int, int, int);
    ~Texture();
    int get_height() const;
    int get_width() const;
    int get_channels();
    bool get_has_read() const;

private:
    int width;
    int height;
    int channels;
    bool has_read;
};

#endif //PROJECT_TEXTURE_H
