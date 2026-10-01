#pragma once

#include <cstdlib> // for std::system
#include <sys/stat.h>
#include <cerrno>
#include <fstream>
#include <string>
#include <stdexcept>
#include <filesystem> // for C++17 and newer
#include <iostream>

#ifdef _WIN32
#include <direct.h>  // For _getcwd on Windows
#else
#include <unistd.h>  // For getcwd on Unix-like systems
#endif


namespace fs = std::filesystem;

class Utils {

public:
	/**
	 * @brief Checks if a directory exists.
	 *
	 * @param dir Path to the directory as a string.
	 * @return True if the directory exists, false otherwise.
	 */
	static bool is_dir(const std::string& dir) {
		struct stat info;
		const char* path = dir.c_str();
		if (stat(path, &info) != 0) { // check if `stat` is successful
			return false; // `stat` fails if the directory is not existing
		}
		else if (info.st_mode & S_IFDIR) { // directory exists
			return true;
		}
		else { // Path exists but is not a directory		
			return false;
		}
	}

	/**
	 * @brief Calculates the size of a file in bytes.
	 *
	 * @param path Path to the file as a filesystem path.
	 * @return Size of the file in bytes, or -1 if the file could not be opened.
	 */
	static int calculate_file_size(const fs::path& path) {
		std::ifstream file(path, std::ios::binary | std::ios::ate); // Open file in binary mode at end
		if (!file.is_open()) { // Check if file was opened
			return -1; // File could not be opened, return error code (-1)
		}
		int size = static_cast<int>(file.tellg()); // Get file size by position at end
		file.close(); // close file
		return size;
	}

	/**
	 * @brief Checks if a file exists.
	 *
	 * @param name Path to the file as a filesystem path.
	 * @return True if the file exists, false otherwise.
	 */
	static bool is_file(const fs::path& name) {
		return fs::exists(name) && fs::is_regular_file(name); // Check if path is an existing file
	}

	/*
	* @brief Converts a rotation matrix to pitch, roll, and azimuth angles in degrees.
	*
	* @param R The rotation matrix (3x3) as a cv::Mat. 
	* @return A cv::Vec3d containing the angles: [pitch, roll, azimuth] in degrees.
	*/
	static cv::Vec3d rotMatToPitchRollAzimuth(const cv::Mat& R)
	{
		double roll = std::asin(R.at<double>(0, 2));
		double pitch = std::atan2(-R.at<double>(1, 2), R.at<double>(2, 2));
		double azimuth = std::atan2(-R.at<double>(0, 1), R.at<double>(0, 0));
		const double r2d = 180.0 / CV_PI;
		return cv::Vec3d(pitch * r2d, roll * r2d, azimuth * r2d);
	}


	/*
	* @brief Converts a rotation matrix to a quaternion representation (qw, qx, qy, qz).
	*
	* @param R The rotation matrix (3x3) as a cv::Mat.
	* @return A cv::Vec4d containing the quaternion components: [qw, qx, qy, qz], normed, qw >= 0
	*/
	static cv::Vec4d rotMatToQuaternion(const cv::Mat& R)
	{
		const double r00 = R.at<double>(0, 0), r01 = R.at<double>(0, 1), r02 = R.at<double>(0, 2);
		const double r10 = R.at<double>(1, 0), r11 = R.at<double>(1, 1), r12 = R.at<double>(1, 2);
		const double r20 = R.at<double>(2, 0), r21 = R.at<double>(2, 1), r22 = R.at<double>(2, 2);

		double qw, qx, qy, qz;
		const double tr = r00 + r11 + r22;
		if (tr > 0) {
			double s = std::sqrt(tr + 1.0) * 2.0;           // s = 4*qw
			qw = 0.25 * s;
			qx = (r21 - r12) / s;
			qy = (r02 - r20) / s;
			qz = (r10 - r01) / s;
		}
		else if (r00 > r11 && r00 > r22) {
			double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;  // s = 4*qx
			qw = (r21 - r12) / s;
			qx = 0.25 * s;
			qy = (r01 + r10) / s;
			qz = (r02 + r20) / s;
		}
		else if (r11 > r22) {
			double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;  // s = 4*qy
			qw = (r02 - r20) / s;
			qx = (r01 + r10) / s;
			qy = 0.25 * s;
			qz = (r12 + r21) / s;
		}
		else {
			double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;  // s = 4*qz
			qw = (r10 - r01) / s;
			qx = (r02 + r20) / s;
			qy = (r12 + r21) / s;
			qz = 0.25 * s;
		}

		cv::Vec4d q(qw, qx, qy, qz);
		q = q / cv::norm(q);
		if (q[0] < 0) q = -q;   // q and -q point to the same rotation; unify prefix
		return q;
	}

	/**
	 * @brief Retrieves the current working directory as a string.
	 *
	 * @return The current working directory.
	 * @throws std::runtime_error if the working directory cannot be retrieved.
	 */
	static std::string get_working_dir() {
		const size_t buffer_size = 260; // Initial buffer size
		char buff[buffer_size];


#ifdef _WIN32
		if (_getcwd(buff, sizeof(buff)) == nullptr) {
			throw std::runtime_error("Failed to get current working directory");
		}
		return std::string(buff);
#else
		char* cwd = getcwd(nullptr, 0); // System allocates buffer if nullptr and 0 are passed
		if (cwd == nullptr) {
			throw std::runtime_error("Failed to get current working directory");
		}
		std::string result(cwd);
		free(cwd); // Free the allocated buffer
		return result;
#endif
	}

private:

};