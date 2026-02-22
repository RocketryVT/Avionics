#pragma once

#include <Eigen/Dense>
#include <cmath>

/**
 * @brief Right-Invariant Extended Kalman Filter (RIEKF) for attitude estimation.
 *
 * Implementation of the paper:
 * "Quaternion Invariant Extended Kalman Filtering for Spacecraft Attitude Estimation"
 * - https://arc.aiaa.org/doi/10.2514/1.G003177
 * Specifically only the RIEKF variant is implemented here.
 *
 * State: quaternion (4D) + gyro bias (3D), error state is 6D.
 * Propagation uses gyroscope measurements.
 * Update fuses body-frame vector observations (accelerometer, magnetometer).
 *
 * @tparam T Scalar type (float or double)
 */
template <typename T = float>
class RIEKF {
public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Mat6 = Eigen::Matrix<T, 6, 6>;
    using Vec6 = Eigen::Matrix<T, 6, 1>;

    struct Config {
        T sigma_gyro;      // gyro noise std dev (rad/s/sqrt(Hz))
        T sigma_gyro_bias; // gyro bias random walk std dev (rad/s^2/sqrt(Hz))
        T sigma_accel;     // accelerometer noise std dev (g)
        T sigma_mag;       // magnetometer noise std dev (uT or normalized)
    };

    RIEKF() : initialized_(false) {
        q_ = Vec4(T(0), T(0), T(0), T(1));
        bias_.setZero();
        omega_.setZero();
        P_.setIdentity();
        cfg_ = {T(0.01), T(0.001), T(0.1), T(0.1)};
    }

    void init(const Vec4& q0, const Vec3& bias0, const Mat6& P0) {
        q_ = q0.normalized();
        bias_ = bias0;
        P_ = P0;
        omega_.setZero();
        initialized_ = true;
    }

    void set_config(const Config& cfg) {
        cfg_ = cfg;
    }

    // --- Quaternion utilities (Hamilton, scalar-last [x,y,z,w]) ---

    static Vec4 quat_multiply(const Vec4& p, const Vec4& q) {
        T pw = p(3), qw = q(3);
        Vec3 pv = p.template head<3>(), qv = q.template head<3>();
        Vec4 result;
        result.template head<3>() = pw * qv + qw * pv + pv.cross(qv);
        result(3) = pw * qw - pv.dot(qv);
        return result;
    }

    static Vec4 quat_conjugate(const Vec4& q) {
        return Vec4(-q(0), -q(1), -q(2), q(3));
    }

    static Vec4 quat_exp(const Vec3& v) {
        T angle = v.norm();
        if (angle < T(1e-12)) {
            Vec4 result;
            result.template head<3>() = v * T(0.5);
            result(3) = T(1);
            return result.normalized();
        }
        T ha = angle * T(0.5);
        Vec4 result;
        result.template head<3>() = std::sin(ha) / angle * v;
        result(3) = std::cos(ha);
        return result;
    }

    static Mat3 quat_to_rotmat(const Vec4& q) {
        T x = q(0), y = q(1), z = q(2), w = q(3);
        Mat3 R;
        R << T(1) - T(2)*(y*y + z*z), T(2)*(x*y - w*z),       T(2)*(x*z + w*y),
             T(2)*(x*y + w*z),         T(1) - T(2)*(x*x + z*z), T(2)*(y*z - w*x),
             T(2)*(x*z - w*y),         T(2)*(y*z + w*x),         T(1) - T(2)*(x*x + y*y);
        return R;
    }

    static Vec3 quat_rotate_vec(const Vec4& q, const Vec3& v) {
        Vec4 v_quat(v(0), v(1), v(2), T(0));
        Vec4 result = quat_multiply(quat_multiply(q, v_quat), quat_conjugate(q));
        return result.template head<3>();
    }

    static Mat3 skew(const Vec3& v) {
        Mat3 S;
        S <<  T(0),  -v(2),  v(1),
              v(2),  T(0),  -v(0),
             -v(1),  v(0),   T(0);
        return S;
    }

    Mat3 rotation_matrix() const {
        return quat_to_rotmat(q_);
    }

    // --- Predict (propagation) ---

    void predict(const Vec3& gyro, T dt) {
        omega_ = gyro - bias_;

        Vec4 omega_quat(omega_(0), omega_(1), omega_(2), T(0));
        Vec4 q_dot = quat_multiply(q_, omega_quat);
        q_dot *= T(0.5);

        q_ = q_ + q_dot * dt;
        q_.normalize();

        // F matrix (Eq. 27)
        Mat3 AT = quat_to_rotmat(q_).transpose();
        Vec3 I_omega = AT * omega_;
        Mat6 F = Mat6::Zero();
        F.template block<3, 3>(0, 3) = -Mat3::Identity();
        F.template block<3, 3>(3, 3) = skew(I_omega);

        // G matrix (Eq. 27)
        Mat6 G = Mat6::Zero();
        G.template block<3, 3>(0, 0) = AT;
        G.template block<3, 3>(3, 3) = -AT;

        // Q matrix
        Mat6 Q = Mat6::Zero();
        Q.template block<3, 3>(0, 0) = cfg_.sigma_gyro * cfg_.sigma_gyro * Mat3::Identity();
        Q.template block<3, 3>(3, 3) = cfg_.sigma_gyro_bias * cfg_.sigma_gyro_bias * Mat3::Identity();

        // Covariance propagation
        Mat6 GQGt = G * Q * G.transpose();
        P_ = P_ + (F * P_ + P_ * F.transpose() + GQGt) * dt;
        P_ = (P_ + P_.transpose()) * T(0.5);
    }

    // --- Update ---

    void update(const Vec3& accel, const Vec3& mag,
                const Vec3& ref_accel, const Vec3& ref_mag) {
        Vec3 a = accel.normalized();
        Vec3 m = mag.normalized();
        Vec3 ra = ref_accel.normalized();
        Vec3 rm = ref_mag.normalized();

        Vec3 a_inertial = quat_rotate_vec(q_, a);
        Vec3 m_inertial = quat_rotate_vec(q_, m);

        Vec6 innovation;
        innovation.template head<3>() = ra - a_inertial;
        innovation.template tail<3>() = rm - m_inertial;

        // H matrix (Eq. 31)
        Mat6 H = Mat6::Zero();
        H.template block<3, 3>(0, 0) = skew(ra);
        H.template block<3, 3>(3, 0) = skew(rm);

        // Measurement noise R (Eq. 34)
        Mat3 AT = quat_to_rotmat(q_).transpose();
        Mat6 R_meas = Mat6::Zero();
        R_meas.template block<3, 3>(0, 0) = cfg_.sigma_accel * cfg_.sigma_accel * Mat3::Identity();
        R_meas.template block<3, 3>(3, 3) = cfg_.sigma_mag * cfg_.sigma_mag * Mat3::Identity();

        // Transform R: Rhat = blkdiag(A^T, A^T) * R * blkdiag(A, A)
        Mat6 AT_blk = Mat6::Zero();
        AT_blk.template block<3, 3>(0, 0) = AT;
        AT_blk.template block<3, 3>(3, 3) = AT;
        Mat6 R_hat = AT_blk * R_meas * AT_blk.transpose();

        // Kalman gain
        Mat6 S = H * P_ * H.transpose() + R_hat;
        Mat6 K = P_ * H.transpose() * S.inverse();

        // Correction
        Vec6 correction = K * innovation;
        Vec3 c_q    = correction.template head<3>();
        Vec3 c_beta = correction.template tail<3>();

        // Quaternion update: q+ = exp_q(-c_q/2) ⊗ q-
        q_ = quat_multiply(quat_exp(-c_q), q_);
        q_.normalize();

        // Bias update: beta+ = beta- - q+* ⊗ c_beta ⊗ q+
        bias_ = bias_ - quat_rotate_vec(quat_conjugate(q_), c_beta);

        // Covariance update
        P_ = (Mat6::Identity() - K * H) * P_;
        P_ = (P_ + P_.transpose()) * T(0.5);
    }

    // Accessors
    Vec4 quaternion() const { return q_; }
    Vec3 bias() const { return bias_; }
    Vec3 angular_velocity() const { return omega_; }
    Mat6 covariance() const { return P_; }

private:
    Vec4 q_;
    Vec3 bias_;
    Vec3 omega_;
    Mat6 P_;
    Config cfg_;
    bool initialized_;
};
