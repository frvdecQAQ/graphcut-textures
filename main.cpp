#include <iostream>
#include <graphcut.h>

int main() {
    auto *graph_cut = new GraphCut("../data/green.gif");
    graph_cut->set_choice(kGlobalBestChoice);
    graph_cut->set_para_k(0.3);
    if(graph_cut->run(80, 80, -1)){
        printf("yes\n");
    }else{
        printf("no\n");
    }
    graph_cut->store("../data/test_green.jpg");
    delete graph_cut;
    return 0;
}
