//
// Created by 安頔 on 2020/12/5.
//

#ifndef PROJECT_MAXFLOW_H
#define PROJECT_MAXFLOW_H

#include <queue>
#include <algorithm>
#include <cmath>

class MaxFlow{
public:
    MaxFlow(int, int);
    ~MaxFlow();
    void clear();
    void set_s(int);
    void set_t(int);
    void set_node_cnt(int);
    void addEdge(int, int, double);
    double dinic();
    void dfsFromStart(bool*);

    int node_cnt;
    constexpr static const double inf = 1e12;
    constexpr static const double eps = 1e-8;

private:
    void add(int, int, double);
    bool bfs();
    double dfs(int, double);
    double *cap;
    int *h, *nxt, *to, *vis;
    int s{0}, t{1}, cnt{1};
    std::queue<int>q;
};

#endif //PROJECT_MAXFLOW_H
