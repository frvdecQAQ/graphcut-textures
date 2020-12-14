//
// Created by 安頔 on 2020/12/5.
//

#ifndef PROJECT_GRAPHCUT_H
#define PROJECT_GRAPHCUT_H

#include <random>
#include <map>
#include <cmath>
#include <texture.h>
#include <MaxFlow.h>

enum Strategy{
    kRandomChoice,
    kGlobalBestChoice,
    kLocalBestChoice
};

class GraphCut{
public:
    explicit GraphCut(const char* patch_path);
    ~GraphCut();
    bool run(int h, int w, int iter);
    void set_choose_option(enum Strategy option);
    void set_para_k(double k);
    void set_use_grad(bool use);
    void showResult();
    void store(const char* result_path);
    void storeSeam(const char* seam_path);

private:
    constexpr static const int dx[4] = {1, -1, 0, 0};
    constexpr static const int dy[4] = {0, 0, 1, -1};
    double cut_cost{0.0}; // sum of cut cost

    int result_h{-1}, result_w{-1};
    int patch_h, patch_w;
    int sub_patch_h, sub_patch_w;

    int offset_x{-1}, offset_y{-1};
    int sub_offset_x{-1}, sub_offset_y{-1};

    int pixel_vis_cnt{0};
    int *pixel_vis{nullptr};
    std::pair<int, int> *pixel_belong{nullptr};
    int *pixel_vis_sum{nullptr};

    const double min_overlap = 0.1;
    enum Strategy choose_option{kRandomChoice};

    double *seam_left_sum{nullptr};
    double *seam_up_sum{nullptr};

    bool use_grad = false;

    double para_k = 0.05;
    double para_scale = 1000;

    Texture *init_patch;
    Texture *combine_patch;
    Texture *result;
    MaxFlow *flow;
    void initRun(int h, int w);
    static int posToId(int w, int i, int j);
    static int randomInt(int st, int ed);
    void buildGraph(int run_iter);
    static double vecLength(const cv::Vec3i& v);
    static double vecLength2(const cv::Vec3i& v);
    static bool checkIn(int h, int w, int i, int j);
    void calculatePixelVisSum() const;
    double calculateRectSeamCost(int i, int j, int bound_x, int bound_y);
    void calculateSeamSum();
    int calculateRectPixel(int a, int b, int c, int d);
    void chooseOffset();
    void chooseErrorRegion(int &error_region_x, int &error_region_y,
                           int &error_region_last_x, int &error_region_last_y,
                           int min_overlap_coef);
    void chooseOffsetRandom();
    void chooseOffsetGlobal();
    void chooseOffsetLocal();
    double calculateCost(const cv::Vec3i &origin_color_u, const cv::Vec3i &origin_color_v,
                  const cv::Vec3i &patch_color_u, const cv::Vec3i &patch_color_v) const;
    static cv::Mat fft(Texture *a, Texture* b);
    double resultPixelVar();

    static double randomDouble(double st, double ed);
};

#endif //PROJECT_GRAPHCUT_H
