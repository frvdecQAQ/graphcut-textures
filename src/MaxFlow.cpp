//
// Created by 安頔 on 2020/12/5.
//

#include <MaxFlow.h>

MaxFlow::MaxFlow(int n, int m) {
    h = new int[n];
    vis = new int[n];
    nxt = new int[m];
    to = new int[m];
    cap = new double[m];
    node_cnt = n;
    clear();
}
MaxFlow::~MaxFlow() {
    delete[] h;
    delete[] vis;
    delete[] nxt;
    delete[] to;
    delete[] cap;
}
void MaxFlow::clear() {
    for(int i = 0; i <= node_cnt; ++i)h[i] = 0;
    cnt = 1;
}
void MaxFlow::set_s(int st) {
    s = st;
}
void MaxFlow::set_t(int ed) {
    t = ed;
}
void MaxFlow::set_node_cnt(int cnt) {
    node_cnt = cnt;
    printf("%d\n", cnt);
}
void MaxFlow::addEdge(int u, int v, double w) {
    add(u, v, w);
    add(v, u, w);
}
void MaxFlow::add(int u, int v, double w) {
    ++cnt;
    to[cnt] = v;
    cap[cnt] = w;
    nxt[cnt] = h[u];
    h[u] = cnt;
}
bool MaxFlow::bfs() {
    while(!q.empty())q.pop();
    for(int i = 0; i <= node_cnt; ++i)vis[i] = -1;
    q.push(s);
    vis[s] = 0;
    while(!q.empty()){
        int now = q.front();
        q.pop();
        for(int i = h[now]; i; i = nxt[i]){
            if(cap[i] > 0 && vis[to[i]] == -1){
                q.push(to[i]);
                vis[to[i]] = vis[now]+1;
            }
        }
    }
    //for(int i = 0; i <= node_cnt; ++i)printf("%d ",vis[i]);
    //printf("\n");
    if(vis[t] == -1)return false;
    return true;
}
double MaxFlow::dfs(int x, double f) {
    if(x == t){
        //printf("f = %lf\n", f);
        return f;
    }
    double w, used = 0;
    for(int i = h[x]; i; i = nxt[i]){
        if(cap[i] > 0 && vis[to[i]] == vis[x]+1){
            w = f-used;
            w = dfs(to[i], std::min(w, cap[i]));
            cap[i] -= w;
            cap[i^1] += w;
            used += w;
            if(std::fabs(f-used) < eps)return f;
        }
    }
    if(std::fabs(used) < eps)vis[x] = -1;
    return used;
}
double MaxFlow::dinic() {
    double ans = 0;
    while(bfs()){
        ans += dfs(s, inf);
        //printf("%lf\n", ans);
    }return ans;
}

void MaxFlow::dfsFromStart(bool *s_arrive) {
    while(!q.empty())q.pop();
    for(int i = 0; i <= node_cnt; ++i)vis[i] = 0;
    for(int i = 0; i <= node_cnt; ++i)s_arrive[i] = false;
    q.push(s);
    vis[s] = 1;
    while(!q.empty()){
        int now = q.front();
        q.pop();
        for(int i = h[now]; i; i = nxt[i]){
            if(!vis[to[i]] && cap[i] > 0){
                vis[to[i]] = 1;
                q.push(to[i]);
                s_arrive[to[i]] = true;
            }
        }
    }
}
