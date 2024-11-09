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