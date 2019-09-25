#include <pose_graph.h>

int main() {

    std::cout << "MAIN FUNCTION RUNING" << std::endl;

    // delete old output file
    system("exec rm -r /home/shu/fangwenSHU/Monocular-SLAM-based-on-MYNTEYE/output/*");

    // testFunc();
    // vins_PoseGraph_reader::loadPoseGraph();
    orbslam2_PoseGraph_reader::loadPoseGraph();

    // vins_PoseGraph_reader::test_pg();
    // orbslam2_PoseGraph_reader::test_pg();

    return 0;
}