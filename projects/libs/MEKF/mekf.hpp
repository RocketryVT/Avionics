#pragma once

#include <Eigen/Dense>
#include <cmath>

/**
 * @brief Multiplicative Extended Kalman Filter (MEKF) for attitude estimation.
 *
 * Ported from the Python reference implementation by Matthew Hampsey (MIT License).
 * https://matthewhampsey.github.io/blog/2020/07/18/mekf
 * https://github.com/MatthewHampsey/mekf
 *
 * Fuses gyroscope, accelerometer, and magnetometer measurements to estimate
 * orientation, velocity, position, and sensor biases.
 *
 * Error state vector (18D):
 *   [0:3]   orientation error (rad)
 *   [3:6]   velocity error
 *   [6:9]   position error
 *   [9:12]  gyro bias
 *   [12:15] accelerometer bias
 *   [15:18] magnetometer bias
 *
 * Quaternion convention: Hamilton, scalar-last [x, y, z, w].
 *
 * @tparam T Scalar type (float or double)
 */
template <typename T = float>
class MEKF {
public:
    using Vec3  = Eigen::Matrix<T, 3, 1>;
    using Vec4  = Eigen::Matrix<T, 4, 1>;
    using Vec6  = Eigen::Matrix<T, 6, 1>;
    using Vec18 = Eigen::Matrix<T, 18, 1>;
    using Mat3  = Eigen::Matrix<T, 3, 3>;
    using Mat6  = Eigen::Matrix<T, 6, 6>;
    using Mat18 = Eigen::Matrix<T, 18, 18>;
    using Mat6x18  = Eigen::Matrix<T, 6, 18>;
    using Mat18x6  = Eigen::Matrix<T, 18, 6>;

    struct Config {
        T estimate_covariance;   // initial P diagonal scale
        T sigma_gyro;            // gyro noise covariance
        T sigma_gyro_bias;       // gyro bias random walk covariance
        T sigma_accel_proc;      // accelerometer process noise covariance
        T sigma_accel_bias;      // accelerometer bias random walk covariance
        T sigma_mag_proc;        // magnetometer process noise covariance
        T sigma_mag_bias;        // magnetometer bias random walk covariance
        T sigma_accel_obs;       // accelerometer observation noise covariance
        T sigma_mag_obs;         // magnetometer observation noise covariance
    };

    MEKF() {
        q_ = Vec4(T(0), T(0), T(0), T(1));
        gyro_bias_.setZero();
        accel_bias_.setZero();
        mag_bias_.setZero();
        P_ = Mat18::Identity();
        G_ = Mat18::Zero();
        R_obs_ = Mat6::Identity();
        cfg_ = {T(1), T(0.1), T(0.1), T(0.1), T(0.1), T(0.1), T(0.1), T(1), T(0.1)};
        apply_config();
    }

    void init(const Vec4& q0, const Config& cfg) {
        q_ = q0.normalized();
        gyro_bias_.setZero();
        accel_bias_.setZero();
        mag_bias_.setZero();
        cfg_ = cfg;
        apply_config();
    }

    void init(const Vec4& q0, const Vec3& gyro_bias0,
              const Vec3& accel_bias0, const Vec3& mag_bias0,
              const Config& cfg) {
        q_ = q0.normalized();
        gyro_bias_  = gyro_bias0;
        accel_bias_ = accel_bias0;
        mag_bias_   = mag_bias0;
        cfg_ = cfg;
        apply_config();
    }

    void set_config(const Config& cfg) {
        cfg_ = cfg;
        apply_config();
    }

    // ─── Quaternion utilities (Hamilton, scalar-last [x,y,z,w]) ───

    static Vec4 quat_multiply(const Vec4& p, const Vec4& q) {
        T pw = p(3), qw = q(3);
        Vec3 pv = p.template head<3>(), qv = q.template head<3>();
        Vec4 r;
        r.template head<3>() = pw * qv + qw * pv + pv.cross(qv);
        r(3) = pw * qw - pv.dot(qv);
        return r;
    }

    static Vec4 quat_conjugate(const Vec4& q) {
        return Vec4(-q(0), -q(1), -q(2), q(3));
    }

    static Vec3 quat_rotate_vec(const Vec4& q, const Vec3& v) {
        Vec4 vq(v(0), v(1), v(2), T(0));
        Vec4 r = quat_multiply(quat_multiply(q, vq), quat_conjugate(q));
        return r.template head<3>();
    }

    static Mat3 quat_to_rotmat(const Vec4& q) {
        T x = q(0), y = q(1), z = q(2), w = q(3);
        Mat3 R;
        R << T(1) - T(2)*(y*y + z*z), T(2)*(x*y - w*z),       T(2)*(x*z + w*y),
             T(2)*(x*y + w*z),         T(1) - T(2)*(x*x + z*z), T(2)*(y*z - w*x),
             T(2)*(x*z - w*y),         T(2)*(y*z + w*x),         T(1) - T(2)*(x*x + y*y);
        return R;
    }

    static Mat3 skew(const Vec3& v) {
        Mat3 S;
        S <<  T(0), -v(2),  v(1),
              v(2),  T(0), -v(0),
             -v(1),  v(0),  T(0);
        return S;
    }

    Mat3 rotation_matrix() const {
        return quat_to_rotmat(q_);
    }

    // --- Full update: gyro + accel + mag (kalman3.py) ---

    void update(const Vec3& gyro_meas, const Vec3& accel_meas, const Vec3& mag_meas, T dt) {
        // Bias-corrected measurements
        Vec3 gyro  = gyro_meas  - gyro_bias_;
        Vec3 accel = accel_meas - accel_bias_;
        Vec3 mag   = mag_meas   - mag_bias_;

        // Propagate quaternion via angular velocity
        Vec4 omega_q(gyro(0), gyro(1), gyro(2), T(0));
        Vec4 q_dot = quat_multiply(q_, omega_q);
        q_dot *= T(0.5);
        q_ = q_ + q_dot * dt;
        q_.normalize();

        // Build process model F = I + G·dt
        G_.template block<3, 3>(0, 0)  = -skew(gyro);
        G_.template block<3, 3>(3, 0)  = -quat_to_rotmat(q_) * skew(accel);
        G_.template block<3, 3>(3, 12) = -quat_to_rotmat(q_);
        Mat18 F = Mat18::Identity() + G_ * dt;

        // A priori covariance
        P_ = F * P_ * F.transpose() + process_covariance(dt);

        // Predicted body-frame observations
        Vec4 q_inv = quat_conjugate(q_);
        Vec3 pred_accel = quat_rotate_vec(q_inv, Vec3(T(0), T(0), T(-1)));
        Vec3 pred_mag   = quat_rotate_vec(q_inv, Vec3(T(1), T(0), T(0)));

        // Observation model H
        Mat6x18 H = Mat6x18::Zero();
        H.template block<3, 3>(0, 0)  = skew(pred_accel);
        H.template block<3, 3>(0, 12) = Mat3::Identity();
        H.template block<3, 3>(3, 0)  = skew(pred_mag);
        H.template block<3, 3>(3, 15) = Mat3::Identity();

        // Kalman gain
        Mat18x6 PH_T = P_ * H.transpose();
        Mat6 S = H * PH_T + R_obs_;
        Mat18x6 K = PH_T * S.inverse();

        // A posteriori covariance
        P_ = (Mat18::Identity() - K * H) * P_;
        P_ = (P_ + P_.transpose()) * T(0.5);

        // Innovation
        Vec6 obs;
        obs.template head<3>() = accel;
        obs.template tail<3>() = mag;
        Vec6 pred;
        pred.template head<3>() = pred_accel;
        pred.template tail<3>() = pred_mag;

        Vec18 dx = K * (obs - pred);

        // Fold error state back into full state
        Vec3 dtheta = dx.template segment<3>(0);
        Vec4 dq;
        dq.template head<3>() = T(0.5) * dtheta;
        dq(3) = T(1);
        q_ = quat_multiply(q_, dq);
        q_.normalize();

        gyro_bias_  += dx.template segment<3>(9);
        accel_bias_ += dx.template segment<3>(12);
        mag_bias_   += dx.template segment<3>(15);
    }

    // --- Accel-only update (kalman2.py, no magnetometer) ---

    void update(const Vec3& gyro_meas, const Vec3& accel_meas, T dt) {
        using Mat15   = Eigen::Matrix<T, 15, 15>;
        using Vec15   = Eigen::Matrix<T, 15, 1>;
        using Mat3x15 = Eigen::Matrix<T, 3, 15>;
        using Mat15x3 = Eigen::Matrix<T, 15, 3>;

        Vec3 gyro  = gyro_meas  - gyro_bias_;
        Vec3 accel = accel_meas - accel_bias_;

        // Propagate quaternion
        Vec4 omega_q(gyro(0), gyro(1), gyro(2), T(0));
        Vec4 q_dot = quat_multiply(q_, omega_q);
        q_dot *= T(0.5);
        q_ = q_ + q_dot * dt;
        q_.normalize();

        // Process model (15-state)
        Mat18 G_full = G_;
        G_full.template block<3, 3>(0, 0)  = -skew(gyro);
        G_full.template block<3, 3>(3, 0)  = -quat_to_rotmat(q_) * skew(accel);
        G_full.template block<3, 3>(3, 12) = -quat_to_rotmat(q_);

        Mat15 G15 = G_full.template topLeftCorner<15, 15>();
        Mat15 F15 = Mat15::Identity() + G15 * dt;

        Mat15 P15 = P_.template topLeftCorner<15, 15>();
        Mat18 Q18 = process_covariance(dt);
        P15 = F15 * P15 * F15.transpose() + Q18.template topLeftCorner<15, 15>();

        // Predicted body-frame gravity
        Vec4 q_inv = quat_conjugate(q_);
        Vec3 pred_accel = quat_rotate_vec(q_inv, Vec3(T(0), T(0), T(-1)));

        // Observation model H (3x15)
        Mat3x15 H = Mat3x15::Zero();
        H.template block<3, 3>(0, 0)  = skew(pred_accel);
        H.template block<3, 3>(0, 12) = Mat3::Identity();

        Mat3 R_accel = cfg_.sigma_accel_obs * Mat3::Identity();
        Mat15x3 PH_T = P15 * H.transpose();
        Mat3 S = H * PH_T + R_accel;
        Mat15x3 K = PH_T * S.inverse();

        P15 = (Mat15::Identity() - K * H) * P15;
        P15 = (P15 + P15.transpose()) * T(0.5);
        P_.template topLeftCorner<15, 15>() = P15;

        Vec3 innovation = accel - pred_accel;
        Vec15 dx = K * innovation;

        Vec3 dtheta = dx.template segment<3>(0);
        Vec4 dq;
        dq.template head<3>() = T(0.5) * dtheta;
        dq(3) = T(1);
        q_ = quat_multiply(q_, dq);
        q_.normalize();

        gyro_bias_  += dx.template segment<3>(9);
        accel_bias_ += dx.template segment<3>(12);
    }

    // --- Accessors ---

    Vec4 quaternion()      const { return q_; }
    Vec3 gyro_bias()       const { return gyro_bias_; }
    Vec3 accel_bias()      const { return accel_bias_; }
    Vec3 mag_bias()        const { return mag_bias_; }
    Mat18 covariance()     const { return P_; }

private:
    void apply_config() {
        P_ = cfg_.estimate_covariance * Mat18::Identity();

        G_ = Mat18::Zero();
        G_.template block<3, 3>(0, 9) = -Mat3::Identity();  // gyro bias coupling
        G_.template block<3, 3>(6, 3) =  Mat3::Identity();  // velocity -> position

        gyro_cov_       = cfg_.sigma_gyro       * Mat3::Identity();
        gyro_bias_cov_  = cfg_.sigma_gyro_bias  * Mat3::Identity();
        accel_cov_      = cfg_.sigma_accel_proc * Mat3::Identity();
        accel_bias_cov_ = cfg_.sigma_accel_bias * Mat3::Identity();
        mag_bias_cov_   = cfg_.sigma_mag_bias   * Mat3::Identity();

        R_obs_ = Mat6::Zero();
        R_obs_.template topLeftCorner<3, 3>()     = cfg_.sigma_accel_obs * Mat3::Identity();
        R_obs_.template bottomRightCorner<3, 3>() = cfg_.sigma_mag_obs   * Mat3::Identity();
    }

    Mat18 process_covariance(T dt) const {
        T dt2 = dt * dt;
        T dt3 = dt2 * dt;
        T dt4 = dt3 * dt;
        T dt5 = dt4 * dt;

        Mat18 Q = Mat18::Zero();

        // Orientation
        Q.template block<3, 3>(0, 0)   = gyro_cov_ * dt + gyro_bias_cov_ * (dt3 / T(3));
        Q.template block<3, 3>(0, 9)   = -gyro_bias_cov_ * (dt2 / T(2));

        // Velocity
        Q.template block<3, 3>(3, 3)   = accel_cov_ * dt + accel_bias_cov_ * (dt3 / T(3));
        Q.template block<3, 3>(3, 6)   = accel_bias_cov_ * (dt4 / T(8)) + accel_cov_ * (dt2 / T(2));
        Q.template block<3, 3>(3, 12)  = -accel_bias_cov_ * (dt2 / T(2));

        // Position
        Q.template block<3, 3>(6, 3)   = accel_cov_ * (dt2 / T(2)) + accel_bias_cov_ * (dt4 / T(8));
        Q.template block<3, 3>(6, 6)   = accel_cov_ * (dt3 / T(3)) + accel_bias_cov_ * (dt5 / T(20));
        Q.template block<3, 3>(6, 12)  = -accel_bias_cov_ * (dt3 / T(6));

        // Gyro bias
        Q.template block<3, 3>(9, 0)   = -gyro_bias_cov_ * (dt2 / T(2));
        Q.template block<3, 3>(9, 9)   = gyro_bias_cov_ * dt;

        // Accel bias
        Q.template block<3, 3>(12, 3)  = -accel_bias_cov_ * (dt2 / T(2));
        Q.template block<3, 3>(12, 6)  = -accel_bias_cov_ * (dt3 / T(6));
        Q.template block<3, 3>(12, 12) = accel_bias_cov_ * dt;

        // Mag bias
        Q.template block<3, 3>(15, 15) = mag_bias_cov_ * dt;

        return Q;
    }

    Vec4 q_;
    Vec3 gyro_bias_;
    Vec3 accel_bias_;
    Vec3 mag_bias_;
    Mat18 P_;
    Mat18 G_;
    Mat6 R_obs_;
    Config cfg_;

    Mat3 gyro_cov_;
    Mat3 gyro_bias_cov_;
    Mat3 accel_cov_;
    Mat3 accel_bias_cov_;
    Mat3 mag_bias_cov_;
};
