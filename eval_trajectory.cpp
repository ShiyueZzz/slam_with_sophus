#include <fstream>
#include <iostream>
#include <sophus/se3.hpp>
#include <unistd.h>
// #include <pangolin/pangolin.h> TODO build Pangolin someday

using namespace Sophus;
using namespace std;

using TrajectoryType = vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char **argv) {
    auto gt_file = "/home/shiyue/work/slam_with_sophus/data/trajectory_gt.txt";
    auto output_file = "/home/shiyue/work/slam_with_sophus/data/trajectory_output.txt";
    TrajectoryType gt = ReadTrajectory(gt_file);
    TrajectoryType output = ReadTrajectory(output_file);
    assert(!gt.empty() && !output.empty());
    assert(gt.size() == output.size());

    // computer RMSE (root-mean-squared error for each Lie algebra)
    double rmse = 0;
    for (size_t i = 0; i < output.size(); i++) {
        Sophus::SE3d p1 = output[i], p2 = gt[i];
        // point error is computed in product Lie group SE(3) element,
        // and calculate norm on the logarithemic map (Lie algebra).
        double err = (p2.inverse() * p1).log().norm();
        rmse += err * err;
    }
    rmse = rmse / double(output.size());
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    return 0;
}

TrajectoryType ReadTrajectory(const string &path) {
    ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin) {
        cerr << "trajectory " << path << " not found." << endl;
        return trajectory;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }
    return trajectory;
}
