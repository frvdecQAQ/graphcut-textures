//
// Created by 安頔 on 2020/12/5.
//

#include <graphcut.h>

GraphCut::GraphCut(const char *img_path) {
    patch = new Texture(img_path);
    result = nullptr;
    flow = nullptr;
    choice = kRandomChoice;
}
GraphCut::~GraphCut() {
    delete patch;
    delete result;
    delete flow;
}
void GraphCut::set_choice(enum Strategy para) {
    choice = para;
}
void GraphCut::set_para_k(double k){
    para_k = k;
}

int GraphCut::randomInt(int st, int ed) {
    std::default_random_engine e(time(nullptr));
    std::uniform_int_distribution<int> u(st, ed);
    return u(e);
}
double GraphCut::randomDouble(double st, double ed){
    std::default_random_engine e(time(nullptr));
    std::uniform_real_distribution<double> u(st, ed);
    return u(e);
}
bool GraphCut::checkIn(int h, int w, int i, int j) {
    return i >= 0 && i < h && j >= 0 && j < w;
}
int GraphCut::transformId(int h, int w, int i, int j) {
    return i*w+j;
}
long long GraphCut::vecLength2(cv::Vec3b v) {
    return v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
}
double GraphCut::vecLength(const cv::Vec3b& v) {
    return std::sqrt((double)vecLength2(v));
}
double GraphCut::calculateCutCost(int x, int y, int nxt_x, int nxt_y, int offset_x, int offset_y) {
    cv::Vec3b original_color_u = result->texture.at<cv::Vec3b>(x, y);
    cv::Vec3b original_color_v = result->texture.at<cv::Vec3b>(nxt_x, nxt_y);
    cv::Vec3b patch_color_u = patch->texture.at<cv::Vec3b>(x-offset_x, y-offset_y);
    cv::Vec3b patch_color_v = patch->texture.at<cv::Vec3b>(nxt_x-offset_x, nxt_y-offset_y);
    cv::Vec3b color_u = cv::Vec3b(abs(original_color_u[0]-patch_color_u[0]),
                                  abs(original_color_u[1]-patch_color_u[1]),
                                  abs(original_color_u[2]-patch_color_u[2]));
    cv::Vec3b color_v = cv::Vec3b(abs(original_color_v[0]-patch_color_v[1]),
                                  abs(original_color_v[1]-patch_color_v[1]),
                                  abs(original_color_v[2]-patch_color_v[2]));
    return vecLength(color_u)+vecLength(color_v);
}
double GraphCut::resultPixelVar(std::pair<int, int> *pixel_source_pos) {
    int h = result->get_height();
    int w = result->get_width();
    double avg = 0;
    int pixel_cnt = 0;
    for(int i = 0; i < h; ++i){
        for(int j = 0; j < w; ++j){
            int id = transformId(h, w, i, j);
            if(pixel_source_pos[id].first == -1)continue;
            pixel_cnt++;
            avg += vecLength(result->texture.at<cv::Vec3b>(i, j));
        }
    }
    avg /= pixel_cnt;
    double var = 0;
    for(int i = 0; i < h; ++i){
        for(int j = 0; j < w; ++j){
            int id = transformId(h, w, i, j);
            if(pixel_source_pos[id].first == -1)continue;
            double tmp = vecLength(result->texture.at<cv::Vec3b>(i, j))-avg;
            var += (tmp*tmp);
        }
    }
    var /= pixel_cnt;
    return var;
}
bool GraphCut::chooseOffset(int &x, int &y, std::pair<int,int> *pixel_source_pos) {
    int limit_x = result->get_height()-patch->get_height()/2;
    int limit_y = result->get_width()-patch->get_width()/2;
    switch (choice) {
        case kRandomChoice:{
            x = randomInt(0, limit_x);
            y = randomInt(0, limit_y);
            break;
        }
        case kGlobalBestChoice:{
            int h = result->get_height();
            int w = result->get_width();
            int patch_height = patch->get_height();
            int patch_width = patch->get_width();
            double var = resultPixelVar(pixel_source_pos);
            double p_sum = 0;
            double p;
            std::vector<double>p_record;
            for(int i = 0; i < limit_x; ++i){
                for(int j = 0; j < limit_y; ++j){
                    int overlap_cnt = 0;
                    int patch_pixel_cnt = 0;
                    long long diff = 0;
                    for(int pos_x = i; pos_x < std::min(h, i+patch_height); ++pos_x){
                        for(int pos_y = j; pos_y < std::min(w, j+patch_width); ++pos_y){
                            int id = transformId(h, w, pos_x, pos_y);
                            patch_pixel_cnt++;
                            if(pixel_source_pos[id].first == -1)continue;
                            overlap_cnt++;
                            cv::Vec3b original_color = result->texture.at<cv::Vec3b>(pos_x, pos_y);
                            cv::Vec3b patch_color = patch->texture.at<cv::Vec3b>(pos_x-i, pos_y-j);
                            diff += vecLength2(cv::Vec3b(abs(original_color[0]-patch_color[0]),
                                                         abs(original_color[1]-patch_color[1]),
                                                         abs(original_color[2]-patch_color[2])));
                        }
                    }
                    if(overlap_cnt == 0 || patch_pixel_cnt == overlap_cnt){
                        p = 0.0;
                    }
                    else{
                        diff /= overlap_cnt;
                        p = para_p_scale*exp((double)(-diff)/(para_k*var));
                    }
                    p_record.push_back(p);
                    p_sum += p;
                }
            }
            double p_random = randomDouble(0, p_sum);
            int p_record_cnt = 0;
            for(int i = 0; i < limit_x; ++i){
                for(int j = 0; j < limit_y; ++j){
                    p_random -= p_record[p_record_cnt];
                    if(p_random <= 0){
                        x = i;
                        y = j;
                        return true;
                    }
                    p_record_cnt++;
                }
            }
            break;
        }
        case kLocalBestChoice:{
            x = 1;
            y = 0;
            break;
        }
        default:{
            printf("Error:enum Strategy don't have correct value!\n");
            return false;
        }
    }return true;
}
bool GraphCut::combineAndCut(std::pair<int,int> *pixel_source_pos, int x, int y, int& pixel_has_source) {
    int h = result->get_height();
    int w = result->get_width();
    int patch_height = patch->get_height();
    int patch_width = patch->get_width();
    int s = 0;
    int t = 1;
    flow->clear();
    flow->set_s(s);
    flow->set_t(t);
    std::map<int, int>id_to_node;
    std::vector<std::pair<int, int>>overlap_pos;
    int node_cnt = 1;
    for(int i = x; i < std::min(x+patch_height, h); ++i) {
        for (int j = y; j < std::min(y + patch_width, w); ++j) {
            int id = transformId(h, w, i, j);
            if (pixel_source_pos[id].first != -1) {
                id_to_node[id] = ++node_cnt;
                overlap_pos.emplace_back(i, j);
            }
        }
    }
    flow->set_node_cnt(node_cnt);
    for(auto pos: overlap_pos){
        int belong_init = 0;
        int belong_patch = 0;
        int id = transformId(h, w, pos.first, pos.second);
        int node_id = id_to_node[id];
        for(int k = 0; k < 4; ++k){
            int nxt_x = pos.first+dx[k];
            int nxt_y = pos.second+dy[k];
            int nxt_id = transformId(h, w, nxt_x, nxt_y);
            if(!checkIn(h, w, nxt_x, nxt_y)){
                if(checkIn(patch_height, patch_width, nxt_x-x, nxt_y-y))belong_patch++;
                continue;
            }
            if(id_to_node.count(nxt_id)){
                if(k&1)continue;
                double w_cut = calculateCutCost(pos.first, pos.second, nxt_x, nxt_y, x, y);
                //printf("w = %lf\n", w_cut);
                if(fabs(w_cut) > MaxFlow::eps)flow->addEdge(node_id, id_to_node[nxt_id], w_cut);
            }
            else{
                if(checkIn(patch_height, patch_width, nxt_x-x, nxt_y-y)){
                    belong_patch++;
                }
                else if(pixel_source_pos[nxt_id].first != -1){
                    belong_init++;
                }
            }
        }
        if(belong_patch+belong_init){
            if(belong_patch >= belong_init){
                flow->addEdge(node_id, t, MaxFlow::inf);
                //printf("t %d\n", node_id);
            }
            else{
                flow->addEdge(s, node_id, MaxFlow::inf);
                //printf("s %d\n", node_id);
            }
        }
    }
    double max_flow_ans = flow->dinic();
    printf("max_flow_ans = %lf\n", max_flow_ans);
    bool *s_arrive = new bool[node_cnt+1];
    flow->dfsFromStart(s_arrive);
    for(auto pos: overlap_pos){
        int id = transformId(h, w, pos.first, pos.second);
        int node_id = id_to_node[id];
        if(!s_arrive[node_id]){
            pixel_source_pos[id] = std::make_pair(pos.first-x, pos.second-y);
            result->texture.at<cv::Vec3b>(pos.first, pos.second) =
                    patch->texture.at<cv::Vec3b>(pos.first-x, pos.second-y);
        }
    }
    for(int i = x; i < std::min(x+patch_height, h); ++i){
        for(int j = y; j < std::min(y+patch_width, w); ++j){
            int id = transformId(h, w, i, j);
            if(pixel_source_pos[id].first == -1) {
                pixel_source_pos[id] = std::make_pair(i - x, j - y);
                result->texture.at<cv::Vec3b>(i, j) = patch->texture.at<cv::Vec3b>(i - x, j - y);
                pixel_has_source++;
            }
        }
    }
    delete[] s_arrive;
    return true;
}
bool GraphCut::run(int h, int w, int iter) {
    if(!patch->get_has_read())return false;
    int patch_height = patch->get_height();
    int patch_width = patch->get_width();
    if(h < patch_height || w < patch_width)return false;
    result = new Texture(h, w, patch->texture.type());
    flow = new MaxFlow(3*h*w, 8*h*w);
    std::pair<int, int> pixel_source_pos[h*w];
    for(int i = 0; i < h; ++i){
        for(int j = 0; j < w; ++j){
            if(i < patch_height && j < patch_width) {
                pixel_source_pos[transformId(h, w, i, j)] = std::make_pair(i, j);
                result->texture.at<cv::Vec3b>(i, j) = patch->texture.at<cv::Vec3b>(i, j);
            }
            else{
                pixel_source_pos[transformId(h, w, i ,j)] = std::make_pair(-1, -1);
                result->texture.at<cv::Vec3b>(i, j) = {0, 0, 0};
            }
        }
    }
    int pixel_has_source = patch_height*patch_width;
    int run_iter = 0;
    int left_up_x, left_up_y;
    while(iter == -1 || run_iter < iter){
        if(pixel_has_source == h*w)break;
        if(!chooseOffset(left_up_x, left_up_y, pixel_source_pos))return false;
        printf("offset_x = %d, offset_y = %d\n", left_up_x, left_up_y);
        if(!combineAndCut(pixel_source_pos, left_up_x, left_up_y, pixel_has_source))continue;
        printf("combine done!\n");
        run_iter++;
        printf("run_iter = %d\n", run_iter);
        printf("pixel = %d\n", pixel_has_source);
        //show();
    }
    return true;
}
void GraphCut::show() {
    cv::imshow("result", result->texture);
    cv::waitKey(0);
}
void GraphCut::store(const char *img_path) {
    cv::imwrite(img_path, result->texture);
}

