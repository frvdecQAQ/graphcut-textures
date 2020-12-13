#include <iostream>
#include <graphcut.h>

int main() {
    auto *graph_cut = new GraphCut("../data/akeyboard_small.gif");
    graph_cut->set_choose_option(kGlobalBestChoice);
    if(graph_cut->run(320, 320, 50))printf("DONE\n");
    else printf("ERROR\n");
    graph_cut->storeSeam("../result/overall/keyboard_seam.jpg");
    graph_cut->store("../result/overall/keyboard.jpg");
    delete graph_cut;
    return 0;
}
