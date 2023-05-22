#include "ae_utils.hpp"


double Utils::YawFromQuaternion(tf2::Quaternion &q)
{
    return atan2(2.0f * (q.getW() * q.getZ() + q.getX() * q.getY()), 1.0f - 2.0f * (q.getY() * q.getY() + q.getZ() * q.getZ()));
}
Covariance6d Utils::RotateCovariance(const Covariance6d &cov, const tf2::Quaternion &q)
{
    Eigen::Quaterniond q_eig(q.getW(), q.getX(), q.getY(), q.getZ());

    Covariance6d cov_out;

    Matrix6d R = Matrix6d::Zero();

	EigenMapConstCovariance6d map_cov_in(cov.data());
	EigenMapCovariance6d map_cov_out(cov_out.data());

    R.block<3, 3>(0, 0) = R.block<3, 3>(3, 3) = q_eig.normalized().toRotationMatrix();

	map_cov_out = R * map_cov_in * R.transpose();

    return cov_out;
}

