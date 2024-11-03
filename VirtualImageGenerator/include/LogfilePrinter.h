#pragma once

#ifndef LOGFILEPRINTER_H
#define LOGFILEPRINTER_H

#include "stdafx.h"
#include <sstream>
#include <ostream>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>


// class to load a pointcloud
class LogFile {

public:

	LogFile() {	}

	~LogFile() { }

	// printer disk
	void print_content_disk(std::string _workingDir) {		
		std::ofstream datFile (_workingDir);
		datFile << logfile_all.rdbuf();	
		datFile.close();
	}

	// overloaded appending
	void append(const std::stringstream& logfile, int precision = 3) {
		logfile_all << std::setprecision(precision) << logfile.str() << std::endl;
		
		std::cout << std::fixed << logfile.str() << std::endl; //plot content that was appended
	}
	
	void append(const std::string& logfile, int precision = 3) {
		logfile_all << std::setprecision(precision) << logfile << std::endl;
	
		std::cout << std::fixed << logfile << std::endl; //plot content that was appended
	}

	void append(const char* logfile, int precision = 3) {
		logfile_all << std::setprecision(precision) << std::string(logfile) << std::endl;
		
		std::cout << std::fixed << std::string(logfile) << std::endl; //plot content that was appended
	}




private:
	std::stringstream logfile_all;
};

#endif