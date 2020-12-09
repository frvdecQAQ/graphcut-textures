#include <iostream>
#include <graphcut.h>

int main() {
    auto *graph_cut = new GraphCut("../data/green.gif");
    graph_cut->set_choice(kGlobalBestChoice);
    if(graph_cut->run(80, 80, 3)){
        printf("yes\n");
    }else{
        printf("no\n");
    }
    graph_cut->store("../data/test.jpg");
    delete graph_cut;
    return 0;
}
