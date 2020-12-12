//
// Created by 安頔 on 2020/12/5.
//
#include <texture.h>

Texture::Texture(const char *img_path) {
    texture = cv::imread(img_path);
    has_read = true;
    if(texture.empty()) {
        cv::VideoCapture capture;
        cv::Mat img_gif;
        img_gif = capture.open(img_path);
        if(!capture.isOpened()) {
            printf("Error: Cannot open image path\n");
            has_read = false;
            capture.release();
            return;
        }
        while(capture.read(img_gif)) {
            texture = img_gif.clone();
        }
        capture.release();
        if (texture.empty()){
            printf("Error: image is empty\n");
            has_read = false;
            return;
        }
    }
    width = texture.cols;
    height = texture.rows;
    channels = texture.channels();
}
Texture::Texture(int height, int width, int type){
    has_read = true;
    this->width = width;
    this->height = height;
    this->channels = (int)(type/8)+1;
    texture = cv::Mat::zeros(height, width, type);
}
Texture::~Texture() {
    texture.release();
}
int Texture::get_height() const {
    return height;
}
int Texture::get_width() const {
    return width;
}
int Texture::get_channels() {
    return channels;
}
bool Texture::get_has_read() const {
    return has_read;
}


