#pragma once
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>

struct EulerDeg { double pitch = 0, roll = 0, azimuth = 0; };   // benannte Felder: Reihenfolge kann nicht mehr verwechselt werden

struct CameraPose {
    cv::Matx33d R_wc = cv::Matx33d::eye();   // Welt -> Kamera (OpenCV-Frame)
    cv::Vec3d   C = cv::Vec3d(0, 0, 0);   // Projektionszentrum (lokal, um shift verschoben)

    static CameraPose fromEuler(const EulerDeg& e, const cv::Vec3d& C) {
        const double p = e.pitch * CV_PI / 180, r = e.roll * CV_PI / 180, a = e.azimuth * CV_PI / 180;
        const cv::Matx33d Rx(1, 0, 0, 0, std::cos(p), -std::sin(p), 0, std::sin(p), std::cos(p));
        const cv::Matx33d Ry(std::cos(r), 0, std::sin(r), 0, 1, 0, -std::sin(r), 0, std::cos(r));
        const cv::Matx33d Rz(std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1);
        CameraPose pose; pose.R_wc = Rx * Ry * Rz; pose.C = C;
        return pose;
    }

    /*
	* @brief Creates a CameraPose from OpenCV rotation and translation vectors.
	* @param rvec: Rotation vector (3x1) or rotation matrix (3x3) in OpenCV format.
	* @param tvec: Translation vector (3x1) in OpenCV format.
	* @return CameraPose: The camera pose with rotation matrix R_wc and camera center C.
	* @note The function converts the rotation vector to a rotation matrix if necessary, and computes the camera center C from the translation vector tvec.
    */
    static CameraPose fromOpenCV(const cv::Mat& rvec, const cv::Mat& tvec) {   // rvec 3x1 oder 3x3
        cv::Mat R = rvec, t64;
        if (rvec.total() == 3) cv::Rodrigues(rvec, R);
        cv::Mat R64; R.convertTo(R64, CV_64F);
        tvec.convertTo(t64, CV_64F);
        CameraPose pose;
        pose.R_wc = cv::Matx33d(R64);
        pose.C = -(pose.R_wc.t() * cv::Vec3d(t64.at<double>(0), t64.at<double>(1), t64.at<double>(2)));
        return pose;
    }

    /*
	* @brief Converts the CameraPose to OpenCV rotation and translation vectors.
	* @param[out] rvec: Output rotation vector (3x1) in OpenCV format.
	* @param[out] tvec: Output translation vector (3x1) in OpenCV format.
	* @note The function converts the rotation matrix R_wc to a rotation vector using Rodrigues' formula, and computes the translation vector tvec from the camera center C.
    */
    void toOpenCV(cv::Mat& rvec, cv::Mat& tvec) const {
        cv::Rodrigues(cv::Mat(R_wc), rvec);
        tvec = cv::Mat(cv::Vec3d(-(R_wc * C))).clone();
    }

    /*
	* @brief Converts the CameraPose to Euler angles (pitch, roll, azimuth) in degrees.
	* @return EulerDeg: The Euler angles corresponding to the camera pose.
	* @note The function computes the Euler angles from the rotation matrix R_wc, with a singularity at roll = ±90°. The angles are returned in degrees.
    */
    EulerDeg euler() const {   // exakte Umkehrung von fromEuler; Singularität bei roll = ±90°
        const double k = 180.0 / CV_PI;
        EulerDeg e;
        e.roll = std::asin(std::clamp(R_wc(0, 2), -1.0, 1.0)) * k;
        e.pitch = std::atan2(-R_wc(1, 2), R_wc(2, 2)) * k;
        e.azimuth = std::atan2(-R_wc(0, 1), R_wc(0, 0)) * k;
        return e;
    }

    cv::Vec3d toCamera(const cv::Vec3d& X) const { return R_wc * (X - C); }
};