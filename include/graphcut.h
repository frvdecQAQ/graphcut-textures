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
    explicit GraphCut(const char *);
    ~GraphCut();
    void set_choice(enum Strategy);
    void set_para_k(double);
    bool run(int, int, int iter = -1);
    void show();
    void store(const char *);

private:
    bool chooseOffset(int&, int&, std::pair<int,int>*);
    bool combineAndCut(std::pair<int, int> *, int, int, int&);
    double resultPixelVar(std::pair<int, int>*);
    static bool checkIn(int, int, int, int);
    static double vecLength(const cv::Vec3b&);
    static long long vecLength2(cv::Vec3b);
    double calculateCutCost(int, int, int, int, int, int);
    static int randomInt(int, int);
    static double randomDouble(double, double);
    static int transformId(int, int, int, int);
    constexpr static const int dx[4] = {0, 0, -1, 1};
    constexpr static const int dy[4] = {1, -1, 0, 0};
    Texture *patch;
    Texture *result;
    MaxFlow *flow;
    enum Strategy choice;
    double para_k = 0.5;
};

#endif //PROJECT_GRAPHCUT_H
