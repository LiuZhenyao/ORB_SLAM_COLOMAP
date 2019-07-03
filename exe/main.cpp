#include <pose_graph.h>

int main() {

    std::cout << "MAIN FUNCTION RUNING" << std::endl;

    // testFunc();
    vins_PoseGraph_reader::loadPoseGraph();
    // orbslam2_PoseGraph_reader::loadKeyFrames();

    return 0;
}