#include <iostream>
#include <graphcut.h>

void blend(){
    auto *back = new Texture("../data/background.jpg");
    auto *front = new Texture("../data/front.jpg");
    auto *front_edit = new Texture("../data/front_edit.jpg");
    //auto *back_edit = new Texture("../data/background_edit.jpg");
    int back_h = back->get_height();
    int back_w = back->get_width();
    int front_h = front->get_height();
    int front_w = front->get_width();
    printf("%d %d\n", back_h, back_w);
    printf("%d %d\n", front_h, front_w);
    int offset_x = back_h-front_h;
    int offset_y = back_w-front_w;
    auto *flow = new MaxFlow(2*front_h*front_w, 6*front_h*front_w);
    flow->set_s(0);
    flow->set_t(1);
    int node_cnt = 1;
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    std::map<int, int>id_to_node;
    for(int i = offset_x; i < std::min(back_h, offset_x+front_h); ++i){
        for(int j = offset_y; j < std::min(back_w, offset_y+front_w); ++j){
            int id = i*back_w+j;
            id_to_node[id] = ++node_cnt;
        }
    }
    flow->set_node_cnt(node_cnt);
    int therehold = 5;
    for(int i = offset_x; i < std::min(back_h, offset_x+front_h); ++i){
        for(int j = offset_y; j < std::min(back_w, offset_y+front_w); ++j){
            int id = i*back_w+j;
            cv::Vec3i edit_color = front_edit->texture.at<cv::Vec3b>(i-offset_x, j-offset_y);
            if(edit_color[0] < therehold && edit_color[1] < therehold && edit_color[2] < therehold){
                flow->addEdge(id_to_node[id], 1, MaxFlow::inf);
            }
            for(int k = 0; k < 4; ++k){
                int nxt_x = i+dx[k];
                int nxt_y = j+dy[k];
                int nxt_id = nxt_x*back_w+j;
                if(nxt_x < 0 || nxt_y < 0 || nxt_x >= back_h || nxt_y >= back_w){
                    //if(nxt_x >= back_h)flow->addEdge(id_to_node[id], 1, MaxFlow::inf);
                    continue;
                }
                if(!id_to_node.count(nxt_id)){
                    flow->addEdge(0, id_to_node[id], MaxFlow::inf);
                }
                else{
                    if(k&1)continue;
                    cv::Vec3i back_color_u = back->texture.at<cv::Vec3b>(i, j);
                    cv::Vec3i back_color_v = back->texture.at<cv::Vec3b>(nxt_x, nxt_y);
                    cv::Vec3i front_color_u = front->texture.at<cv::Vec3b>(i-offset_x, j-offset_y);
                    cv::Vec3i front_color_v = front->texture.at<cv::Vec3b>(nxt_x-offset_x, nxt_y-offset_y);
                    cv::Vec3i u_color_diff = back_color_u - front_color_u;
                    //std::cout << u_color_diff << std::endl;
                    cv::Vec3i v_color_diff = back_color_v - front_color_v;
                    cv::Vec3i grad_color_back = back_color_v - back_color_u;
                    cv::Vec3i grad_color_front = front_color_v - front_color_u;
                    double w_cut = sqrt((double)(u_color_diff[0] * u_color_diff[0] +
                                                 u_color_diff[1] * u_color_diff[1] +
                                                 u_color_diff[2] * u_color_diff[2])) +
                                   sqrt((double)(v_color_diff[0] * v_color_diff[0] +
                                                 v_color_diff[1] * v_color_diff[1] +
                                                 v_color_diff[2] * v_color_diff[2]));
                    double div = sqrt((double)(grad_color_back[0] * grad_color_back[0] +
                                               grad_color_back[1] * grad_color_back[1] +
                                               grad_color_back[2] * grad_color_back[2])) +
                                 sqrt((double)(grad_color_front[0] * grad_color_front[0] +
                                               grad_color_front[1] * grad_color_front[1] +
                                               grad_color_front[2] * grad_color_front[2]));
                    if(fabs(div) > MaxFlow::eps)w_cut /= div;
                    flow->addEdge(id_to_node[id], id_to_node[nxt_id], w_cut);
                }
            }

        }
    }
    double max_flow_ans = flow->dinic();
    printf("%lf\n", max_flow_ans);
    bool *s_arrive = new bool[node_cnt+10];
    flow->dfsFromStart(s_arrive);
    auto *mask = new Texture(front_h, front_w, 0);
    for(int i = offset_x; i < std::min(back_h, offset_x+front_h); ++i) {
        for (int j = offset_y; j < std::min(back_w, offset_y + front_w); ++j) {
            int id = i * back_w + j;
            if (!s_arrive[id_to_node[id]]) {
                mask->texture.at<uchar>(i - offset_x, j - offset_y) = 255;
                //back->texture.at<cv::Vec3b>(i, j) = front->texture.at<cv::Vec3b>(i-offset_x, j-offset_y);
            }
        }
    }
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,cv::Size(9, 9));
    cv::dilate(mask->texture, mask->texture, element);
    cv::GaussianBlur(mask->texture, mask->texture, cv::Size(5, 5), 0.9, 0.9);

    for(int i = offset_x; i < std::min(back_h, offset_x+front_h); ++i) {
        for (int j = offset_y; j < std::min(back_w, offset_y + front_w); ++j) {
            int id = i * back_w + j;
            cv::Vec3f front_color = front->texture.at<cv::Vec3b>(i-offset_x, j-offset_y);
            cv::Vec3f back_color = back->texture.at<cv::Vec3b>(i, j);
            double alpha = mask->texture.at<uchar>(i-offset_x, j-offset_y)/255.0;
            //printf("%lf\n", alpha);
            back->texture.at<cv::Vec3b>(i, j) = front_color*alpha+back_color*(1-alpha);
        }
    }

    cv::imwrite("../result/blend/blend.jpg", back->texture);
    delete back;
    delete front;
    delete front_edit;
    //delete back_edit;
    delete mask;
    delete [] s_arrive;
}

int main() {
    //blend();
    auto *graph_cut = new GraphCut("../data/akeyboard_small.gif");
    graph_cut->set_choose_option(kLocalBestChoice);
    graph_cut->set_para_k(0.1);
    graph_cut->set_use_grad(false);
    if(graph_cut->run(320, 320, 50))printf("DONE\n");
    else printf("ERROR\n");
    graph_cut->storeSeam("../result/overall/grad_keyboard_seam.jpg");
    graph_cut->store("../result/overall/grad_keyboard.jpg");
    delete graph_cut;
    return 0;
}
