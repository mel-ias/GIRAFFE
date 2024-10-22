#pragma once



#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include"opencv2\opencv.hpp"
#include "json.hpp"
#include <windows.h>
#include "LogfilePrinter.h"

#include "DataManager.h"


#ifdef _WIN64
#include <direct.h>
#define GetCurrentDir _getcwd
#elif __linux__
#include <unistd.h>
#define GetCurrentDir getcwd
#endif


class Utils {

public:
	
	// run a batch file
	// input: path to file.bat
	// return: true to inform the main that task has been finished
	static bool run_batch_file(std::string path) {		
		system(path.c_str());
		return true;
	}

	// delete all files within a directory
	// input: directory to be cleaned
	static void delete_all_files_dir(std::string dir) {
		std::string task = "rm -rf " + dir;
		const char* c = task.c_str();
		system(c);
	}

	// check if directory exists (returns either true or false)
	static bool is_dir(std::string dir) {
		struct stat info;
		const char* path = dir.c_str();
		if (stat(path, &info) != 0)
			return true;
		else if (info.st_mode & S_IFDIR)
			return false;
		else
			return true;
	}

	// calculate and return size of a file 
	// input: path to file whose file size has to be calculated
	// note: requires #include <fstream>
	static int calculateFileSize(std::string path) {
		FILE* pFile = NULL;
		fopen_s(&pFile, path.c_str(), "rb"); // get the file stream
		fseek(pFile, 0, SEEK_END); // set the file pointer to end of file
		int Size = ftell(pFile); // get the file size
		// rewind( pFile ); // return the file pointer to begin of file if you want to read it		
		fclose(pFile); // close stream and release buffer
		return Size;
	}

	// check if file exists (returns either true or false)
	static bool is_file(const std::string& name) {
		struct stat buffer;
		return (stat(name.c_str(), &buffer) == 0);
	}

	// get path of dirctory where the tool is running
	static std::string get_working_dir() {
		char buff[FILENAME_MAX];
		GetCurrentDir(buff, FILENAME_MAX);
		std::string current_working_dir(buff);
		return current_working_dir;
	}

	
	// helper function for output, append a string to a text file 
	// input: file path to txt.file (if file not exists, it will be created), line to be added
	static void append_line_to_file(std::string path, std::string line) {
		std::ofstream file;
		file.open(path, std::ios::in | std::ios::out | std::ios::app);
		if (file.fail())
			throw std::ios_base::failure(std::strerror(errno));

		//make sure write fails with exception if something is wrong
		file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
		file << line << std::endl;
	}

private:



};