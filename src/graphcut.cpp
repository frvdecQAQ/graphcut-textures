//
// Created by 安頔 on 2020/12/5.
//

#include <graphcut.h>

GraphCut::GraphCut(const char *patch_path) {
    init_patch = new Texture(patch_path);
    patch_h = init_patch->get_height();
    patch_w = init_patch->get_width();
    sub_patch_h = patch_h/3;
    sub_patch_w = patch_w/3;
    combine_patch = nullptr;
    result = nullptr;
    flow = nullptr;
}

GraphCut::~GraphCut() {
    delete init_patch;
    delete combine_patch;
    delete result;
    delete flow;
    delete[] pixel_vis;
    delete[] pixel_belong;
    delete[] pixel_vis_sum;
    delete[] seam_left_sum;
    delete[] seam_up_sum;
}

int GraphCut::randomInt(int st, int ed) {
    static std::default_random_engine e(time(nullptr));
    std::uniform_int_distribution<int> u(st, ed);
    return u(e);
}

void GraphCut::buildGraph(int run_iter) {
    flow->clear();
    flow->set_s(0);
    flow->set_t(1);
    int node_cnt = 1;
    int combine_patch_h = combine_patch->get_height();
    int combine_patch_w = combine_patch->get_width();
    std::map<int, int>id_to_node;
    std::vector<std::pair<int, int>>overlap_pos;
    for(int i = offset_x; i < std::min(offset_x+combine_patch_h, result_h); ++i){
        for(int j = offset_y; j < std::min(offset_y+combine_patch_w, result_w); ++j){
            int id = posToId(result_w, i, j);
            if(pixel_vis[id] != -1){
                id_to_node[id] = ++node_cnt;
                overlap_pos.emplace_back(i, j);
            }
        }
    }
    double cut_cost_copy = cut_cost;
    for(auto pos: overlap_pos){
        int id = posToId(result_w, pos.first, pos.second);
        int node_id = id_to_node[id];
        int result_belong = 0;
        int patch_belong = 0;
        for(int k = 0; k < 4; ++k){
            int nxt_x = pos.first+dx[k];
            int nxt_y = pos.second+dy[k];
            if(!checkIn(result_h, result_w, nxt_x, nxt_y)){
                if(pixel_vis_cnt != result_w*result_h &&
                    checkIn(combine_patch_h, combine_patch_w, nxt_x-offset_x, nxt_y-offset_y)){
                    patch_belong++;
                }continue;
            }
            int nxt_id = posToId(result_w, nxt_x, nxt_y);
            if(id_to_node.count(nxt_id)){
                if(k&1)continue;
                cv::Vec3i patch_color_u = combine_patch->texture
                        .at<cv::Vec3b>(pos.first-offset_x, pos.second-offset_y);
                cv::Vec3i patch_color_v = combine_patch->texture
                        .at<cv::Vec3b>(nxt_x-offset_x, nxt_y-offset_y);
                cv::Vec3i color_u_from_u = result->texture.at<cv::Vec3b>(pos.first, pos.second);
                cv::Vec3i color_v_from_v = result->texture.at<cv::Vec3b>(nxt_x, nxt_y);
                if(pixel_vis[id] == pixel_vis[nxt_id]) {
                    double w_cut = calculateCost(color_u_from_u, color_v_from_v, patch_color_u, patch_color_v);
                    flow->addEdge(node_id, id_to_node[nxt_id], w_cut);
                }
                else{
                    ++node_cnt;
                    cv::Vec3i color_v_from_u =
                            init_patch->texture.at<cv::Vec3b>(
                                    pixel_belong[id].first+dx[k],
                                    pixel_belong[id].second+dy[k]);
                    cv::Vec3i color_u_from_v =
                            init_patch->texture.at<cv::Vec3b>(
                                    pixel_belong[nxt_id].first-dx[k],
                                    pixel_belong[nxt_id].second-dy[k]);
                    double w1_cut = calculateCost(color_u_from_u, color_v_from_u, patch_color_u, patch_color_v);
                    double w2_cut = calculateCost(patch_color_u, patch_color_v, color_u_from_v, color_v_from_v);
                    double w_cut = calculateCost(color_u_from_u, color_v_from_u, color_u_from_v, color_v_from_v);
                    flow->addEdge(node_cnt, 1, w_cut);
                    flow->addEdge(node_id, node_cnt, w1_cut);
                    flow->addEdge(node_cnt, id_to_node[nxt_id], w2_cut);
                    cut_cost -= w_cut;
                }
            }
            else{
                if(checkIn(combine_patch_h, combine_patch_w, nxt_x-offset_x, nxt_y-offset_y))patch_belong++;
                else if(pixel_vis[nxt_id] != -1)result_belong++;
            }
        }
        if(patch_belong+result_belong){
            if(patch_belong >= result_belong)flow->addEdge(node_id, 1, MaxFlow::inf);
            else flow->addEdge(0, node_id, MaxFlow::inf);
        }
    }
    flow->set_node_cnt(node_cnt+10);
    double max_flow_ans = flow->dinic();
    if(max_flow_ans  >= MaxFlow::inf){
        printf("max_flow_ans = %lf\n", max_flow_ans);
        printf("cut_cost = %lf\n", cut_cost);
        cut_cost = cut_cost_copy;
        printf(".....\n");
        return;
    }
    cut_cost += max_flow_ans;
    printf("cut_cost = %lf\n", cut_cost);
    assert(!(pixel_vis_cnt == result_h*result_w && cut_cost-cut_cost_copy > MaxFlow::eps));
    bool *s_arrive = new bool[node_cnt+10];
    flow->dfsFromStart(s_arrive);
    for(int i = offset_x; i < std::min(offset_x+combine_patch_h, result_h); ++i){
        for(int j = offset_y; j < std::min(offset_y+combine_patch_w, result_w); ++j){
            int id = posToId(result_w, i, j);
            if((pixel_vis[id] != -1 && !s_arrive[id_to_node[id]]) || pixel_vis[id] == -1){
                if(pixel_vis[id] == -1)pixel_vis_cnt++;
                result->texture.at<cv::Vec3b>(i, j) =
                        combine_patch->texture.at<cv::Vec3b>(i-offset_x, j-offset_y);
                pixel_vis[id] = run_iter;
                pixel_belong[id] =
                        std::make_pair(i-offset_x+sub_offset_x, j-offset_y+sub_offset_y);
            }
        }
    }
    delete []s_arrive;
}

void GraphCut::initRun(int h, int w) {
    result_h = h; result_w = w;
    result = new Texture(h, w, init_patch->texture.type());
    pixel_vis = new int[h*w];
    pixel_vis_sum = new int[h*w]{0};
    seam_left_sum = new double[h*w]{0.0};
    seam_up_sum = new double[h*w]{0.0};
    pixel_belong = new std::pair<int, int>[h*w];
    flow = new MaxFlow(3*h*w, 8*h*w);
    pixel_vis_cnt = patch_h*patch_w;
    for(int i = 0; i < h*w; ++i){
        pixel_vis[i] = -1;
        pixel_belong[i] = std::make_pair(-1, -1);
    }
    for(int i = 0; i < patch_h; ++i){
        for(int j = 0; j < patch_w; ++j){
            int id = posToId(w, i, j);
            pixel_vis[id] = 0;
            pixel_belong[id] = std::make_pair(i, j);
            result->texture.at<cv::Vec3b>(i, j) = init_patch->texture.at<cv::Vec3b>(i, j);
        }
    }
}

bool GraphCut::run(int h, int w, int iter) {
    if(patch_h > h || patch_w > w)return false;
    if(iter < 0)return false;
    initRun(h, w);
    int run_iter = 0;
    while(pixel_vis_cnt != result_h*result_w){
        run_iter++;
        calculatePixelVisSum();
        chooseOffset();
        buildGraph(run_iter);
        printf("%d\n", pixel_vis_cnt);
        //showResult();
    }
    calculatePixelVisSum();
    printf("------------------\n");
    while(iter--){
        //printf("%d\n", iter);
        run_iter++;
        chooseOffset();
        buildGraph(run_iter);
    }
    return true;
}

void GraphCut::showResult() {
    cv::imshow("result", result->texture);
    cv::waitKey(0);
}

int GraphCut::posToId(int w, int i, int j) {
    return i*w+j;
}

void GraphCut::set_choose_option(enum Strategy option) {
    choose_option = option;
}

void GraphCut::calculatePixelVisSum() const {
    for(int i = 0; i < result_h; ++i){
        for(int j = 0; j < result_w; ++j){
            int id = posToId(result_w, i, j);
            pixel_vis_sum[id] = (pixel_vis[id] != -1? 1: 0);
            pixel_vis_sum[id] += (i == 0? 0: pixel_vis_sum[posToId(result_w, i-1, j)]);
            pixel_vis_sum[id] += (j == 0? 0: pixel_vis_sum[posToId(result_w, i, j-1)]);
            pixel_vis_sum[id] -= ((i == 0 || j == 0)? 0: pixel_vis_sum[posToId(result_w, i-1, j-1)]);
        }
    }
}

int GraphCut::calculateRectPixel(int a, int b, int c, int d) {
    return pixel_vis_sum[posToId(result_w, c, d)]-
            (a == 0? 0: pixel_vis_sum[posToId(result_w, a-1, d)])-
            (b == 0? 0: pixel_vis_sum[posToId(result_w, c, b-1)])+
            ((a == 0 || b == 0)? 0: pixel_vis_sum[posToId(result_w, a-1, b-1)]);
}

void GraphCut::chooseOffset() {
    switch (choose_option) {
        case kRandomChoice:
            chooseOffsetRandom();
            break;
        case kGlobalBestChoice:
            chooseOffsetGlobal();
            break;
        case kLocalBestChoice:
            chooseOffsetLocal();
            break;
    }
}

void GraphCut::chooseOffsetRandom() {
    std::vector<std::pair<int, int>>can_random_pos;
    for(int i = 0; i < result_h; ++i){
        for(int j = 0; j < result_w; ++j){
            int bound_x = std::min(i+patch_h, result_h);
            int bound_y = std::min(j+patch_w, result_w);
            int rect_pixel_sum = calculateRectPixel(i, j, bound_x-1, bound_y-1);
            int rect_area = (bound_x-i)*(bound_y-j);
            if(rect_pixel_sum < rect_area*min_overlap)continue;
            if(rect_area == rect_pixel_sum && pixel_vis_cnt != result_w*result_h)continue;
            can_random_pos.emplace_back(i, j);
        }
    }
    int random_index = randomInt(0, (int)(can_random_pos.size()));
    offset_x = can_random_pos[random_index].first;
    offset_y = can_random_pos[random_index].second;
    sub_offset_x = 0;
    sub_offset_y = 0;
    combine_patch = new Texture(patch_h, patch_w, init_patch->texture.type());
    combine_patch->texture = init_patch->texture.clone();
}

void GraphCut::chooseOffsetGlobal() {
    int error_region_x, error_region_y;
    int error_region_last_x, error_region_last_y;
    chooseErrorRegion(error_region_x, error_region_y,
                      error_region_last_x, error_region_last_y, 1);
    std::vector<std::pair<int, int>>can_random_pos;
    for(int i = 0; i <= error_region_x; ++i){
        for(int j = 0; j <= error_region_y; ++j){
            int bound_x = std::min(i+patch_h, result_h);
            int bound_y = std::min(j+patch_w, result_w);
            if(bound_x < error_region_last_x)continue;
            if(bound_y < error_region_last_y)continue;
            can_random_pos.emplace_back(i, j);
        }
    }
    double error_c = MaxFlow::inf;
    int can_random_pos_size = (int)(can_random_pos.size());
    int T = 1000;
    offset_x = -1; offset_y = -1;
    while(T--){
        int rand_index = randomInt(0, can_random_pos_size-1);
        int x = can_random_pos[rand_index].first;
        int y = can_random_pos[rand_index].second;
        int overlap_cnt = 0;
        double diff = 0;
        //TODO
        for(int i = x; i < std::min(i+patch_h, result_h); ++i){
            for(int j = y; j < std::min(j+patch_w, result_w); ++j){
                int id = posToId(result_w, i, j);
                if(pixel_vis[id] == -1)continue;
                overlap_cnt++;
                cv::Vec3i result_color = result->texture.at<cv::Vec3b>(i, j);
                cv::Vec3i patch_color = init_patch->texture.at<cv::Vec3b>(i-x, j-y);
                diff += vecLength2(result_color-patch_color);
            }
        }
        diff /= overlap_cnt;
        if(diff < error_c){
            error_c = diff;
            offset_x = x;
            offset_y = y;
        }
    }
    assert(offset_x != -1);
    sub_offset_x = 0;
    sub_offset_y = 0;
    combine_patch = new Texture(patch_h, patch_w, init_patch->texture.type());
    combine_patch->texture = init_patch->texture.clone();
}

void GraphCut::chooseOffsetLocal() {
    int error_region_x, error_region_y;
    int error_region_last_x, error_region_last_y;
    chooseErrorRegion(error_region_x, error_region_y,
                      error_region_last_x, error_region_last_y, 3);
    int sub_height = sub_patch_h;
    int sub_width = sub_patch_w;
    if(error_region_x > 0){
        error_region_x--;
        sub_height++;
    }
    if(error_region_y > 0){
        error_region_y--;
        sub_width++;
    }
    if(error_region_x+sub_height < result_h)sub_height++;
    if(error_region_y+sub_width < result_w)sub_width++;
    double error_c = MaxFlow::inf;
    int limit_x = patch_h-sub_height;
    int limit_y = patch_w-sub_width;
    sub_offset_x = -1;
    sub_offset_y = -1;
    int T = 2000;
    while(T--){
        int x = randomInt(0, limit_x-1);
        int y = randomInt(0, limit_y-1);
        double diff = 0;
        for(int i = x; i < x+sub_height; ++i){
            for(int j = y; j < y+sub_width; ++j){
                int result_x = error_region_x+i-x;
                int result_y = error_region_y+j-y;
                int result_id = posToId(result_w, result_x, result_y);
                if(pixel_vis[result_id] == -1)continue;
                cv::Vec3i result_color = result->texture.at<cv::Vec3b>(result_x, result_y);
                cv::Vec3i patch_color = init_patch->texture.at<cv::Vec3b>(i, j);
                diff += vecLength2(result_color-patch_color);
            }
        }
        if(diff < error_c){
            error_c = diff;
            sub_offset_x = x;
            sub_offset_y = y;
        }
    }
    assert(sub_offset_x != -1);
    //printf("%d %d\n", sub_offset_x, sub_offset_y);
    offset_x = error_region_x;
    offset_y = error_region_y;
    combine_patch = new Texture(sub_height, sub_width, init_patch->texture.type());
    combine_patch->texture = init_patch->texture(cv::Rect(sub_offset_y, sub_offset_x,
                                                          sub_width, sub_height)).clone();
}

double GraphCut::calculateCost(const cv::Vec3i &origin_color_u,
                               const cv::Vec3i &origin_color_v,
                               const cv::Vec3i &patch_color_u,
                               const cv::Vec3i &patch_color_v) {
    cv::Vec3i grad_origin_u = origin_color_v-origin_color_u;
    cv::Vec3i grad_patch_u = patch_color_v-patch_color_u;
    double w_ans = vecLength(origin_color_u-patch_color_u)+vecLength(origin_color_v-patch_color_v);
    //double div = vecLength(grad_origin_u)+vecLength(grad_patch_u);
    //if(fabs(div) > MaxFlow::eps)w_ans /= div;
    return w_ans;
}

double GraphCut::vecLength(const cv::Vec3i& v) {
    return sqrt(vecLength2(v));
}

double GraphCut::vecLength2(const cv::Vec3i& v) {
    return (double)(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

bool GraphCut::checkIn(int h, int w, int i, int j) {
    return i >= 0 && i < h && j >= 0 && j < w;
}

void GraphCut::store(const char *result_path) {
    cv::imwrite(result_path, result->texture);
}

void GraphCut::storeSeam(const char *seam_path){
    auto *seam = new Texture(result_h, result_w, result->texture.type());
    seam->texture = result->texture.clone();
    for(int i = 0; i < result_h; ++i){
        for(int j = 0; j < result_w; ++j){
            bool flag = false;
            int id = posToId(result_w, i, j);
            for(int k = 0; k < 4; ++k){
                int nxt_x = dx[k]+i;
                int nxt_y = dy[k]+j;
                if(!checkIn(result_h, result_w, nxt_x, nxt_y))continue;
                int nxt_id = posToId(result_w, nxt_x, nxt_y);
                if(pixel_vis[nxt_id] != pixel_vis[id]){
                    flag = true;
                    break;
                }
            }
            if(flag){
                seam->texture.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 255);
            }
        }
    }
    //cv::imshow("seam", seam->texture);
    //cv::waitKey(0);
    cv::imwrite(seam_path, seam->texture);
    delete seam;
}

void GraphCut::calculateSeamSum() {
    for(int i = 0; i < result_h; ++i){
        for(int j = 0; j < result_w; ++j){
            int id = posToId(result_w, i, j);
            seam_left_sum[id] = seam_up_sum[id] = 0;
            if(pixel_vis[id] == -1)continue;
            for(int k = 1; k < 4; k += 2){
                int nxt_x = i+dx[k];
                int nxt_y = j+dy[k];
                if(!checkIn(result_h, result_w, nxt_x, nxt_y))continue;
                int nxt_id = posToId(result_w, nxt_x, nxt_y);
                if(pixel_vis[nxt_id] == -1)continue;
                if(pixel_vis[nxt_id] == pixel_vis[id])continue;
                cv::Vec3i color_u_from_u = result->texture.at<cv::Vec3b>(i, j);
                cv::Vec3i color_v_from_v = result->texture.at<cv::Vec3b>(nxt_x, nxt_y);
                cv::Vec3i color_v_from_u =
                        init_patch->texture.at<cv::Vec3b>(
                                pixel_belong[id].first+dx[k],
                                pixel_belong[id].second+dy[k]);
                cv::Vec3i color_u_from_v =
                        init_patch->texture.at<cv::Vec3b>(
                                pixel_belong[nxt_id].first-dx[k],
                                pixel_belong[nxt_id].second-dy[k]);
                double w_cut = calculateCost(color_u_from_u, color_v_from_u,
                                             color_u_from_v, color_v_from_v);
                if(k == 1)seam_left_sum[id] = w_cut;
                else seam_up_sum[id] = w_cut;
            }
        }
    }
    for(int i = 0; i < result_h; ++i){
        for(int j = 0; j < result_w; ++j){
            int id = posToId(result_w, i, j);
            seam_left_sum[id] += (i == 0? 0: seam_left_sum[posToId(result_w, i-1, j)]);
            seam_left_sum[id] += (j == 0? 0: seam_left_sum[posToId(result_w, i, j-1)]);
            seam_left_sum[id] -= ((i == 0 || j == 0)? 0: seam_left_sum[posToId(result_w, i-1, j-1)]);
            seam_up_sum[id] += (i == 0? 0: seam_up_sum[posToId(result_w, i-1, j)]);
            seam_up_sum[id] += (j == 0? 0: seam_up_sum[posToId(result_w, i, j-1)]);
            seam_up_sum[id] -= ((i == 0 || j == 0)? 0: seam_up_sum[posToId(result_w, i-1, j-1)]);
        }
    }
}

double GraphCut::calculateRectSeamCost(int i, int j, int bound_x, int bound_y) {
    double left_sum = seam_left_sum[posToId(result_w, bound_x, bound_y)]-
            seam_left_sum[posToId(result_w, i, bound_y)]-
            (j == 0? 0: seam_left_sum[posToId(result_w, bound_x, j-1)])+
            (j == 0? 0: seam_left_sum[posToId(result_w, i, j-1)]);
    double up_sum = seam_up_sum[posToId(result_w, bound_x, bound_y)]-
            (i == 0? 0: seam_up_sum[posToId(result_w, i-1, bound_y)])-
            seam_up_sum[posToId(result_w, bound_x, j)]+
            (i == 0? 0: seam_up_sum[posToId(result_w, i-1, j)]);
    return up_sum+left_sum;
}

void GraphCut::chooseErrorRegion(int &error_region_x, int &error_region_y,
                                 int &error_region_last_x, int &error_region_last_y,
                                 int min_overlap_coef) {
    calculateSeamSum();
    double error_region = -1;
    int new_pixel = 0;
    for(int i = 0; i < result_h; ++i){
        for(int j = 0; j < result_w; ++j){
            int bound_x = std::min(i+sub_patch_h, result_h);
            int bound_y = std::min(j+sub_patch_w, result_w);
            int rect_pixel_sum = calculateRectPixel(i, j, bound_x-1, bound_y-1);
            int rect_area = (bound_x-i)*(bound_y-j);
            if(rect_pixel_sum < min_overlap_coef*rect_area*min_overlap)continue;
            if(rect_area == rect_pixel_sum && pixel_vis_cnt != result_w*result_h)continue;
            double error_seam = calculateRectSeamCost(i, j, bound_x-1, bound_y-1);
            int area_new_pixel = rect_area-rect_pixel_sum;
            if(error_seam > error_region || (error_seam == error_region && area_new_pixel > new_pixel)){
                error_region = error_seam;
                new_pixel = area_new_pixel;
                error_region_x = i;
                error_region_y = j;
                error_region_last_x = bound_x;
                error_region_last_y = bound_y;
            }
        }
    }
}
