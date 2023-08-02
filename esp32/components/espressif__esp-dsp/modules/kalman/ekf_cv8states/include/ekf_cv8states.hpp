#ifndef EKF_CV8STATES_HPP
#define EKF_CV8STATES_HPP

#include "ekf.h"

/**
 * @brief This class is used to process and calculate 8 states (x,y,vx,vy,widght,height,vw,vh) from 4 measurements (x,y,w,h)
 *
 *   The class use state vector with 8 follows values
 *   X = [x, y, vx, vy, width, height, vw, vh]
 *  where
 *  x,y - position of object
 * vx,vy - velocity of object
 * width, height - size of object
 * vw, vh - velocity of size changing
//  */
// class ekf_cv8states : public ekf
// {
// public:
//       ekf_imu13states();
//       virtual ~ekf_imu13states();
//       virtual void Init();

//       // Method calculates Xdot values depends on U
//       // U - gyroscope values in radian per seconds (rad/sec)
//       virtual dspm::Mat StateXdot(dspm::Mat &x, float *u);
//       virtual void LinearizeFG(dspm::Mat &x, float *u);

//       /**
//        *     Method for development and tests only.
//        */
//       void Test();
//       /**
//        *     Method for development and tests only.
//        *
//        * @param[in] enable_att - enable attitude as input reference value
//        */
//       void TestFull(bool enable_att);

//       /**
//        *     Initial reference valie for magnetometer.
//        */
//       dspm::Mat mag0;
//       /**
//        *     Initial reference valie for accelerometer.
//        */
//       dspm::Mat accel0;

//       /**
//        * number of control measurements
//        */
//       int NUMU;

//       /**
//        * Update part of system state by reference measurements accelerometer and magnetometer.
//        * Only attitude and gyro bias will be updated.
//        * This method should be used as main method after calibration.
//        *
//        * @param[in] accel_data: accelerometer measurement vector XYZ in g, where 1 g ~ 9.81 m/s^2
//        * @param[in] magn_data: magnetometer measurement vector XYZ
//        * @param[in] R: measurement noise covariance values for diagonal covariance matrix. Then smaller value, then more you trust them.
//        */
//       void UpdateRefMeasurement(float *accel_data, float *magn_data, float R[6]);
//       /**
//        * Update full system state by reference measurements accelerometer and magnetometer.
//        * This method should be used at calibration phase.
//        *
//        * @param[in] accel_data: accelerometer measurement vector XYZ in g, where 1 g ~ 9.81 m/s^2
//        * @param[in] magn_data: magnetometer measurement vector XYZ
//        * @param[in] R: measurement noise covariance values for diagonal covariance matrix. Then smaller value, then more you trust them.
//        */
//       void UpdateRefMeasurementMagn(float *accel_data, float *magn_data, float R[6]);
//       /**
//        * Update system state by reference measurements accelerometer, magnetometer and attitude quaternion.
//        * This method could be used when system on constant state or in initialization phase.
//        * @param[in] accel_data: accelerometer measurement vector XYZ in g, where 1 g ~ 9.81 m/s^2
//        * @param[in] magn_data: magnetometer measurement vector XYZ
//        * @param[in] attitude: attitude quaternion
//        * @param[in] R: measurement noise covariance values for diagonal covariance matrix. Then smaller value, then more you trust them.
//        */
//       void UpdateRefMeasurement(float *accel_data, float *magn_data, float *attitude, float R[10]);
// };

// #endif // EKF_CV8STATES_HPP
