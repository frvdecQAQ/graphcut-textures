#include <iostream>
#include "graphcut.h"

int main() {
    auto *graph_cut = new GraphCut("../data/green.gif");
    if(graph_cut->run(300, 300)){
        printf("yes\n");
    }else{
        printf("no\n");
    }
    delete graph_cut;
    return 0;
}
