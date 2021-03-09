#include <uav_obj.h>

UAVObj::UAVObj(double max_acc, double interval) //ros::NodeHandle& nh, 
{
    interval_ = interval;
    max_acc_ = max_acc;
}

void UAVObj::get_future(){
    tf2::Vector3 pose;
    std::cout << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z << " " << std::endl;
    tf2::convert(odom.pose.pose.position, pose);
    std::cout << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z << " " << std::endl;

    tf2::Quaternion quat;
    tf2::convert(odom.pose.pose.orientation, quat);

    tf2::Transform tf(quat, pose);
    eigen = tf_to_eigen(tf);
    // std::cout << matrix << std::endl;

    tf2::Vector3 twist;
    tf2::convert(odom.twist.twist.linear, twist);
    
    double max_x = get_future_pos(twist[0], MAX_ACC, INTERVAL);
    double min_x = get_future_pos(twist[0], -MAX_ACC, INTERVAL);
    double max_y = get_future_pos(twist[1], MAX_ACC, INTERVAL);
    double min_y = get_future_pos(twist[1], -MAX_ACC, INTERVAL);
    double max_z = get_future_pos(twist[2], MAX_ACC, INTERVAL);
    double min_z = get_future_pos(twist[2], -MAX_ACC, INTERVAL);

    std::cout << "covariance " << std::endl << get_covariance(max_x, max_y, max_z) << std::endl;

    std::cout << "eigen: " << std::endl << eigen << std::endl;

    quad[0] = eigen.transpose() * get_covariance(max_x, max_y, max_z) * eigen;
    quad[1] = eigen.transpose() * get_covariance(max_x, max_y, min_z) * eigen;
    quad[2] = eigen.transpose() * get_covariance(max_x, min_y, max_z) * eigen;
    quad[3] = eigen.transpose() * get_covariance(max_x, min_y, min_z) * eigen;
    quad[4] = eigen.transpose() * get_covariance(min_x, max_y, max_z) * eigen;
    quad[5] = eigen.transpose() * get_covariance(min_x, max_y, min_z) * eigen;
    quad[6] = eigen.transpose() * get_covariance(min_x, min_y, max_z) * eigen;
    quad[7] = eigen.transpose() * get_covariance(min_x, min_y, min_z) * eigen;
    std::cout << "ece0: " << std::endl << quad[0] << std::endl;
    std::cout << "ece4: " << std::endl << quad[4] << std::endl;
}

Eigen::Matrix4d tf_to_eigen(tf2::Transform tf){
    Eigen::Matrix4d eigen;
    tf2::Matrix3x3 basis = tf.getBasis();
    tf2::Vector3 origin = tf.getOrigin();
    // std::cout << "basis" << std::endl;
    // for (int i = 0; i < 3; i++){
    //     for (int j = 0; j < 3; j++){
    //         std::cout << basis[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << "origin" << std::endl;
    // for (int i = 0; i < 3; i++){
    //     std::cout << origin[i] << " ";
    // }
    // std::cout << std::endl;
    eigen << basis[0][0], basis[0][1], basis[0][2], origin[0],
            basis[1][0], basis[1][1], basis[1][2], origin[1],
            basis[2][0], basis[2][1], basis[2][2], origin[2],
            0, 0, 0, 1;
    return eigen;
}

void UAVObj::set_odom(nav_msgs::Odometry msg){
    odom = msg;
    get_future();
}

bool UAVObj::check_interference(UAVObj o){
    Eigen::Vector3d vector(o.odom.pose.pose.position.x - odom.pose.pose.position.x,
        o.odom.pose.pose.position.y - odom.pose.pose.position.y,
        o.odom.pose.pose.position.z - odom.pose.pose.position.z);
    vector.normalize();
    std::cout << "vector " << vector << std::endl;
    //check self
    std::cout << "self eigen " << std::endl << eigen << std::endl;
    int idx = 0;
    if (vector.dot(eigen.block<3,1>(0,0)) < 0){
        idx += 4;
    }
    if (vector.dot(eigen.block<3,1>(0,1)) < 0){
        idx += 2;
    }
    if (vector.dot(eigen.block<3,1>(0,2)) < 0){
        idx += 1;
    }
    std::cout << "self index: " << idx << " self traj: " << std::endl << quad[idx] << std::endl;
    Eigen::Matrix4d self_ellipse = quad[idx];
    //check other
    idx = 0;
    if (vector.dot(o.eigen.block<3,1>(0,0)) > 0){
        idx += 4;
    }
    if (vector.dot(o.eigen.block<3,1>(0,1)) > 0){
        idx += 2;
    }
    if (vector.dot(o.eigen.block<3,1>(0,2)) > 0){
        idx += 1;
    }
    std::cout << "other index: " << idx << " other traj: " << std::endl << o.quad[idx] << std::endl;
    Eigen::Matrix4d o_ellipse = o.quad[idx];
    return intercept_ellipse(self_ellipse, o_ellipse);
}

Eigen::Matrix4d get_covariance(double x, double y, double z){
    Eigen::Matrix4d covariance;
    covariance.block<3,3>(0,0) = Eigen::Vector3d(1/x/x, 1/y/y, 1/z/z).asDiagonal();
    covariance(3,3) = -1;
    covariance.block<1,3>(3,0) << 0, 0, 0;
    covariance.block<3,1>(0,3) << 0, 0, 0;
    return covariance;
}

// std::vector<double> determinant(std::vector<double> matrix[4][4], int n) {
//     std::vector<double> det(n + 1, 0);
//     std::vector<double> submatrix[4][4];
//     if (n == 2){
//         std::vector<double> result;
//         result.push_back(matrix[0][0][0] * matrix[1][1][0] - matrix[1][0][0] * matrix[0][1][0]);
//         result.push_back(matrix[0][0][0] * matrix[1][1][1] + matrix[0][0][1] * matrix[1][1][0]
//                         - matrix[1][0][0] * matrix[0][1][1] - matrix[1][0][1] * matrix[0][1][0]);
//         result.push_back(matrix[0][0][1] * matrix[1][1][1] - matrix[1][0][1] * matrix[0][1][1]);
//         return result;
//     }
//     else {
//         for (int x = 0; x < n; x++) {
//             int subi = 0;
//             for (int i = 1; i < n; i++) {
//                 int subj = 0;
//                 for (int j = 0; j < n; j++) {
//                     if (j == x){
//                         continue;
//                     }
//                     submatrix[subi][subj] = matrix[i][j];
//                     subj++;
//                 }
//                 subi++;
//             }
//             int sign = pow(-1, x);
//             std::vector<double> sub_det = determinant(submatrix, n - 1);
//             std::vector<double> curr_det;
//             det[0] += sign * matrix[0][x][0] * sub_det[0];
//             det[1] += sign * (matrix[0][x][0] * sub_det[1] + matrix[0][x][1] * sub_det[0]);
//             det[2] += sign * (matrix[0][x][0] * sub_det[2] + matrix[0][x][1] * sub_det[1]);
//             det[3] += sign * (matrix[0][x][0] * sub_det[3] + matrix[0][x][1] * sub_det[2]);
//             if (n == 4){
//                 det[4] += sign * (matrix[0][x][0] * sub_det[4] + matrix[0][x][1] * sub_det[3]);
//             }
//         }
//     }
//     return det;
// }

// double get_lambda(Eigen::Matrix4d a, Eigen::Matrix4d b){
//     std::vector<double> matrix[4][4];
//     for (int i = 0; i < 4; i++){
//         for (int j = 0; j < 4; j++){
//             matrix[i][j].push_back(a(i, j));
//             matrix[i][j].push_back(-b(i, j));
//         }
//     }
//     std::vector<double> det = determinant(matrix, 4);
//     std::complex<double>* solutions = solve_quartic(det[1] / det[0], det[2] / det[0], det[3] / det[0], det[4] / det[0]);
    
//     double lambda = std::numeric_limits<double>::quiet_NaN();
//     double min_det = 1;

//     for (int i = 0; i < 4; i++){
//         std::cout << solutions[i].real() << " " << solutions[i].imag() << std::endl;
//         if (solutions[i].imag() == 0.0){
//             double det = (solutions[i].real() * a - b).determinant();
//             if (std::isnan(lambda) || det < min_det){
//                 min_det = det;
//                 lambda = solutions[i].real();
//             }
//         }
//     }
//     return lambda;
// }

bool intercept_ellipse(Eigen::Matrix4d a, Eigen::Matrix4d b){
    // double lambda = get_lambda(a, b);
    // std::cout << "lambda: " << lambda << std::endl;
    // Eigen::Matrix4d x;
    // x << 0.25, 0, 0, 0,
    //     0, 1, 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, -1;
    
    // Eigen::Matrix4d y1;
    // y1 << 1, 0, 0, 0,
    //     0, 1, 0, 0,
    //     0, 0, 1, 0,
    //     -7, 0, 0, 1;
    // Eigen::Matrix4d y2;
    // y2 << 0.111, 0, 0, 0,
    //     0, 0.25, 0, 0,
    //     0, 0, 0.063, 0,
    //     0, 0, 0, -1;
    // Eigen::Matrix4d y = y1 * y2 * y1.transpose();
    // double c = get_lambda(x, y);
    // std::cout << "lambda: " << std::endl;
    // std::cout << c << std::endl;
    // std::cout << "x: " << std::endl;
    // std::cout << x << std::endl;
    // std::cout << "y: " << std::endl;
    // std::cout << y << std::endl;
    // std::cout << "lambda x - y: " << std::endl;
    // std::cout << c * x - y << std::endl;

    Eigen::EigenSolver<Eigen::Matrix4d> es(a.inverse() * b);
    std::cout << "es: " << es.eigenvalues() << " end es" << std::endl;
    std::cout << "ev: " << es.eigenvectors() << " end ev" << std::endl;
    std::complex<double> ev1 = es.eigenvalues()[0];
    std::complex<double> ev2 = es.eigenvalues()[1];
    if (es.eigenvalues()[0].imag() == 0){ // not intersect
        return false;
    }

    return true;
}

double get_future_pos(double vel, double acc, double time){
    return vel*time + 0.5 * acc * time * time;
}


void test(Eigen::Vector3d e1_size, Eigen::Matrix4d e1_tf, Eigen::Vector3d e2_size, Eigen::Matrix4d e2_tf){
    Eigen::Matrix3d m;
    // m = AngleAxisf(angle1, Vector3d::UnitZ())
    //     * AngleAxisf(angle2, Vector3d::UnitY())
    //     * AngleAxisf(angle3, Vector3d::UnitX());
}