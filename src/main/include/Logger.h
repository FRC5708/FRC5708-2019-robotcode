#pragma once
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <sstream>
using namespace std;
class Logger{
public:
	string name;
	ofstream output_file;
	Logger(string name="NONE",string output_filepath="");
	~Logger();
	void log(char* message);
};
