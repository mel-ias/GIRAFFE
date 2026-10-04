#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

enum class LensModel { Pinhole, Fisheye };

struct CameraIntrinsics {
    cv::Mat   K;                      // 3x3, CV_64F, [px]
    cv::Mat   dist;                   // Pinhole: 5x1, Fisheye: 4x1, CV_64F
    LensModel model = LensModel::Pinhole;
    cv::Size  size;                   // Bildgröße [px]
    double    pixel_size_mm = 0.0;

    bool   isFisheye()   const { return model == LensModel::Fisheye; }
    double ck_mm()       const { return K.at<double>(0, 0) * pixel_size_mm; }
    double tanHalfFovH() const { return (size.width / 2.0) / K.at<double>(0, 0); }
    double tanHalfFovV() const { return (size.height / 2.0) / K.at<double>(1, 1); }

    static CameraIntrinsics fromNominal(double ck_mm, double pixel_size_mm, cv::Size size, LensModel model) {
        CameraIntrinsics c;
        const double f = ck_mm / pixel_size_mm;
        c.K = (cv::Mat_<double>(3, 3) << f, 0, size.width / 2, 0, f, size.height / 2, 0, 0, 1);
        c.dist = cv::Mat::zeros(model == LensModel::Fisheye ? 4 : 5, 1, CV_64FC1);
        c.model = model;
        c.size = size;
        c.pixel_size_mm = pixel_size_mm;
        return c;
    }

    // Gleiche Kamera mit anderem Linsenmodell (K bleibt exakt erhalten, dist = 0)
    CameraIntrinsics withModel(LensModel m) const {
        CameraIntrinsics c = *this;
        c.K = K.clone();
        c.dist = cv::Mat::zeros(m == LensModel::Fisheye ? 4 : 5, 1, CV_64FC1);
        c.model = m;
        return c;
    }

    // Intrinsics des entzerrten Bildes (Pinhole, dist = 0)
    CameraIntrinsics undistortedIntrinsics(double balance) const {
        const cv::Mat E = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat K_new;
        if (isFisheye())
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, dist, size, E, K_new, balance);
        else
            K_new = cv::getOptimalNewCameraMatrix(K, dist, size, balance);
        CameraIntrinsics c = *this;
        c.K = K_new.clone();
        c.dist = cv::Mat::zeros(5, 1, CV_64FC1);
        c.model = LensModel::Pinhole;
        return c;
    }

    // Entzerrt ein mit *this aufgenommenes Bild in die Geometrie von 'target' (= undistortedIntrinsics)
    cv::Mat undistortImage(const cv::Mat& img, const CameraIntrinsics& target) const {
        cv::Mat out;
        if (isFisheye()) {
            const cv::Mat E = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat map1, map2;
            cv::fisheye::initUndistortRectifyMap(K, dist, E, target.K, size, CV_16SC2, map1, map2);
            cv::remap(img, out, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else {
            cv::undistort(img, out, K, dist, target.K);   // Fix: neue Kameramatrix wird jetzt übergeben
        }
        return out;
    }
};