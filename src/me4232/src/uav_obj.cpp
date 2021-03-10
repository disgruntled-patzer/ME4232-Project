#include <uav_obj.h>

UAVObj::UAVObj(double x_max_acc_, double y_max_acc_, double z_max_acc_)
{
    // In vehicle body frame
    x_max_acc = x_max_acc_;
    y_max_acc = y_max_acc_;
    z_max_acc = z_max_acc_;
}

// Create the 8 ellipses for the 8 possible acceleration directions
void UAVObj::get_future(){
    tf2::Vector3 pose;
    tf2::convert(odom.pose.pose.position, pose);
    tf2::Quaternion quat;
    tf2::convert(odom.pose.pose.orientation, quat);

    tf2::Transform tf(quat, pose);
    eigen = tf_to_eigen(tf);

    double x = odom.twist.twist.linear.x * INTERVAL;
    double y = odom.twist.twist.linear.y * INTERVAL;
    double z = odom.twist.twist.linear.z * INTERVAL;

    Eigen::Matrix4d twist = Eigen::Matrix4d::Identity();
    twist.block<3,1>(0,3) << x, y, z;
    eigen = eigen * twist;

    // tf2::Vector3 twist;
    // tf2::convert(odom.twist.twist.linear, twist);
    
    // double max_x = get_future_pos(twist[0], MAX_ACC, INTERVAL);
    // double min_x = get_future_pos(twist[0], -MAX_ACC, INTERVAL);
    // double max_y = get_future_pos(twist[1], MAX_ACC, INTERVAL);
    // double min_y = get_future_pos(twist[1], -MAX_ACC, INTERVAL);
    // double max_z = get_future_pos(twist[2], MAX_ACC, INTERVAL);
    // double min_z = get_future_pos(twist[2], -MAX_ACC, INTERVAL);

    // quad[0] = eigen.transpose() * get_covariance(max_x, max_y, max_z) * eigen;
    // quad[1] = eigen.transpose() * get_covariance(max_x, max_y, min_z) * eigen;
    // quad[2] = eigen.transpose() * get_covariance(max_x, min_y, max_z) * eigen;
    // quad[3] = eigen.transpose() * get_covariance(max_x, min_y, min_z) * eigen;
    // quad[4] = eigen.transpose() * get_covariance(min_x, max_y, max_z) * eigen;
    // quad[5] = eigen.transpose() * get_covariance(min_x, max_y, min_z) * eigen;
    // quad[6] = eigen.transpose() * get_covariance(min_x, min_y, max_z) * eigen;
    // quad[7] = eigen.transpose() * get_covariance(min_x, min_y, min_z) * eigen;
}

// Convert tf message to eigen matrix
Eigen::Matrix4d tf_to_eigen(tf2::Transform tf){
    Eigen::Matrix4d eigen;
    tf2::Matrix3x3 basis = tf.getBasis();
    tf2::Vector3 origin = tf.getOrigin();
    eigen << basis[0][0], basis[0][1], basis[0][2], origin[0],
            basis[1][0], basis[1][1], basis[1][2], origin[1],
            basis[2][0], basis[2][1], basis[2][2], origin[2],
            0, 0, 0, 1;
    return eigen;
}

// Set the lastest odometry of the UAV
void UAVObj::set_odom(nav_msgs::Odometry msg){
    odom = msg;
    get_future();
}

// Return bool of whether current UAV might collide with other UAV
bool UAVObj::check_interference(UAVObj o){
    Eigen::Vector3d vector(o.eigen(3,0) - eigen(3,0),
        o.eigen(3,1) - eigen(3,1),
        o.eigen(3,2) - eigen(3,2));
    vector.normalize();
    //check self
    double x_acc = vector.dot(eigen.block<3,1>(0,0)) < 0 ? -x_max_acc : x_max_acc;
    double a = get_future_pos(x_acc, INTERVAL);
    double y_acc = vector.dot(eigen.block<3,1>(0,1)) < 0 ? -y_max_acc : y_max_acc;
    double b = get_future_pos(y_acc, INTERVAL);
    double z_acc = vector.dot(eigen.block<3,1>(0,2)) < 0 ? -z_max_acc : z_max_acc;
    double c = get_future_pos(z_acc, INTERVAL);

    // Eigen::Matrix4d eigen_inverse = eigen.inverse();
    Eigen::Matrix4d self_ellipse = eigen.transpose() * get_covariance(a, b, c) * eigen;
    // std::cout << "self " << self_ellipse << std::endl;

    //check other
    x_acc = vector.dot(o.eigen.block<3,1>(0,0)) > 0 ? -x_max_acc : x_max_acc;
    a = get_future_pos(x_acc, INTERVAL);
    y_acc = vector.dot(o.eigen.block<3,1>(0,1)) > 0 ? -y_max_acc : y_max_acc;
    b = get_future_pos(y_acc, INTERVAL);
    z_acc = vector.dot(o.eigen.block<3,1>(0,2)) > 0 ? -z_max_acc : z_max_acc;
    c = get_future_pos(z_acc, INTERVAL);

    // eigen_inverse = o.eigen.inverse();
    Eigen::Matrix4d o_ellipse = o.eigen.transpose() * get_covariance(a, b, c) * o.eigen;
    // std::cout << "other " << o_ellipse << std::endl;
    return intercept_ellipse(self_ellipse, o_ellipse);
}

// Return covariance matrix for ellipse based on the x, y, z lengths
Eigen::Matrix4d get_covariance(double x, double y, double z){
    Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero(4, 4);
    covariance.block<3,3>(0,0) = Eigen::Vector3d(1/x/x, 1/y/y, 1/z/z).asDiagonal();
    covariance(3,3) = -1;
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

// Check if 2 ellipses intersect
bool intercept_ellipse(Eigen::Matrix4d a, Eigen::Matrix4d b){
    // double lambda = get_lambda(a, b);
    // std::cout << "lambda: " << lambda << std::endl;

    // std::cout << "a: " << std::endl << a << std::endl;
    // std::cout << "b: " << std::endl << b << std::endl;

    Eigen::EigenSolver<Eigen::Matrix4d> es(a.inverse() * b);
    for (int i = 0; i < 4; i++){
        if (abs(es.eigenvalues()[i].imag()) > 1e-9){
            // std::cout << "es: " << es.eigenvalues() << " end es" << std::endl;
            // std::cout << "ev: " << es.eigenvectors() << " end ev" << std::endl;
            return true;
        }
    }
    return false;
}

// S = 1/2 at^2
double get_future_pos(double acc, double time){
    return 0.5 * acc * time * time;
}