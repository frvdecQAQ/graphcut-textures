#include <iostream>
#include <graphcut.h>

int main() {
    auto *graph_cut = new GraphCut("../data/strawberries2.gif");
    graph_cut->set_choose_option(kGlobalBestChoice);
    if(graph_cut->run(300, 300, 20))printf("DONE\n");
    else printf("ERROR\n");
    graph_cut->storeSeam("../result/test_seam.jpg");
    graph_cut->store("../result/test.jpg");
    delete graph_cut;
    return 0;
}
