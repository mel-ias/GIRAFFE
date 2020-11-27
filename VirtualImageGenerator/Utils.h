#pragma once

#ifndef DATAMANAGER_H
#define DATAMANAGER_H

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


#ifdef _WIN64
#include <direct.h>
#define GetCurrentDir _getcwd
#elif __linux__
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

#endif

class Utils {

public:

	static bool executeBatch(std::string path, std::string tool_name) {		
		// path to the .bat to run
		system(path.c_str());

		// after finishing, return 'true' that the main knows that this task has been finished
		return true;
	}

	

	

	static void delete_all_files_dir(std::string _dir) {
		std::string task = "rm -rf " + _dir;
		const char* c = task.c_str();
		system(c);
	}

	static bool dirExists(std::string _dir)
	{
		struct stat info;
		const char* path = _dir.c_str();
		if (stat(path, &info) != 0)
			return true;
		else if (info.st_mode & S_IFDIR)
			return false;
		else
			return true;
	}

	// needs #include <fstream>
	static int calculateFileSize(std::string path) {
		FILE* pFile = NULL;
		fopen_s(&pFile, path.c_str(), "rb"); // get the file stream
		fseek(pFile, 0, SEEK_END); // set the file pointer to end of file
		int Size = ftell(pFile); // get the file size
		// rewind( pFile ); // return the file pointer to begin of file if you want to read it		
		fclose(pFile); // close stream and release buffer
		return Size;
	}




private:



};