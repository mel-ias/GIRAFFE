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
	
	// run a batch file
	// input: path to file.bat
	// return: true to inform the main that task has been finished
	static bool run_batch_file(const std::string& path) {
		if (path.empty()) {
			throw std::invalid_argument("Path to batch file cannot be empty.");
		}
		int ret_code = std::system(path.c_str());
		if (ret_code != 0) {
			throw std::runtime_error("Failed to execute batch file. Return code: " + std::to_string(ret_code));
		}
		return true;
	}

	// delete all files within a directory
	// input: directory to be cleaned
	static void delete_all_files_dir(const std::string& dir) {
		namespace fs = std::filesystem;
		try {
			if (fs::exists(dir) && fs::is_directory(dir)) {
				for (const auto& entry : fs::directory_iterator(dir)) {
					fs::remove_all(entry.path());
				}
			}
		}
		catch (const fs::filesystem_error& e) {
			throw std::runtime_error("Error deleting files in directory: " + std::string(e.what()));
		}
	}

	// check if directory exists (returns either true or false)
	static bool is_dir(const std::string& dir) {
		struct stat info;
		const char* path = dir.c_str();
		if (stat(path, &info) != 0) { // check if `stat` ist successful
			return false; // `stat` fails if the directory is not existing
		}
		else if (info.st_mode & S_IFDIR) { // dir exists
			return true;
		}
		else { // not a file but something else			
			return false;
		}
	}

	static int calculateFileSize(const fs::path& path) {
		std::ifstream file(path, std::ios::binary | std::ios::ate); // open file as binary
		if (!file.is_open()) { // check file opening
			return -1; // file could not be opened, return error code (-1)
		}
		int size = static_cast<int>(file.tellg()); // `std::ios::ate` put file pointer to the end and allows for file calculation
		file.close(); // close file
		return size;
	}

	// check if file exists (returns either true or false)
	static bool is_file(const fs::path& name) {
		// Überprüfen, ob der Pfad eine existierende Datei ist
		return fs::exists(name) && fs::is_regular_file(name);
	}

	// get working directory
	static std::string get_working_dir() {
		const size_t buffer_size = 260; // Initial buffer size
		char buff[buffer_size];

#ifdef _WIN32
		if (_getcwd(buff, sizeof(buff)) == nullptr) {
			throw std::runtime_error("Failed to get current working directory");
		}
		return std::string(buff);
#else
		char* cwd = getcwd(nullptr, 0); // Passing nullptr and 0 lets the system allocate a buffer
		if (cwd == nullptr) {
			throw std::runtime_error("Failed to get current working directory");
		}
		std::string result(cwd);
		free(cwd); // Free the allocated buffer
		return result;
#endif
	}
	
	// helper function for output, append a string to a text file 
	// input: file path to txt.file (if file not exists, it will be created), line to be added
	static void append_line_to_file(const std::string& path, const std::string& line) {
		std::ofstream file;
		file.open(path, std::ios::app);
		if (!file.is_open()) { // check if file was opened
			throw std::ios_base::failure("Error opening file: " + std::string(std::strerror(errno)));
		}
		file.exceptions(file.exceptions() | std::ios::failbit | std::ios::badbit); // exception to catch writing errors
		try {
			file << line << std::endl;
		}
		catch (const std::ios_base::failure& e) {
			throw std::ios_base::failure("Error writing to file: " + std::string(e.what()));
		}
		file.close(); // close file (optional, d'tor should do this)
	}

private:



};