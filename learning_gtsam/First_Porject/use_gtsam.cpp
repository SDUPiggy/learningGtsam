// #include<eigen3/Eigen/Core>
// #include<eigen3/Eigen/Dense>
    #include<gtsam/base/Vector.h>
    #include<gtsam/geometry/Pose2.h>
    #include<gtsam/slam/BetweenFactor.h>
    #include<gtsam/nonlinear/NonlinearFactorGraph.h>
    #include<gtsam/nonlinear/Values.h>
    #include<gtsam/nonlinear/Marginals.h>
    #include<gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
    // #include<gtsam/nonlinear/DoglegOptimizer.h>
    #include<gtsam/slam/PriorFactor.h>
    using namespace gtsam;
    using namespace std;
    int main(int argc, char** argv)
    {
        // 生成一个空的nonlinear factor graph
        NonlinearFactorGraph graph;
        Pose2 priorMean(0.0, 0.0, 0.0);
        // noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
        // 生成一个noise model
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
        // 添加Factor
        graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
        // odometry 这样构造为x,y,theta
        Pose2 odometry(2.0, 0.0, 0.0);
        // 生成odometry的noise
        auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
        
        graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
        graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));
        // 估计值
        Values initial;
        initial.insert(1, Pose2(0.5, 0.0, 0.2));
        initial.insert(2, Pose2(2.3, 0.1, -0.2));
        initial.insert(3, Pose2(4.1, 0.1, 0.1));
        initial.print("\n Initial Estimate:\n");
        
        Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
        result.print("Final Result:\n");

        cout.precision(2);
        Marginals marginals(graph, result);
        cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
        cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
        cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

        return 0;
    }
