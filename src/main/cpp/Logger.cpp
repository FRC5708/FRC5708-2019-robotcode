#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <sstream>
#include "Logger.h"

using namespace std;
Logger::Logger(string name,string output_filepath){
	this->name=name;
	time_t rawtime; time (&rawtime); struct tm *tm_struct = localtime(&rawtime);
	if(output_filepath==""){
		stringstream output;
		output <<  "/home/lvuser/Logging/Log_" << name <<  ':' << (tm_struct->tm_hour) << ':' << (tm_struct->tm_min) << ':' << (tm_struct->tm_sec) << ".txt";
		output_filepath=output.str();
	}
	this->output_file.open(output_filepath,ofstream::out | ofstream::trunc);
}
void Logger::log(const char* message){
	time_t rawtime; time (&rawtime); struct tm *tm_struct = localtime(&rawtime);
	output_file << tm_struct->tm_hour << ':' << tm_struct->tm_min << ':' << tm_struct->tm_sec << ' ' << message << endl;
}
Logger::~Logger(){
	output_file.close();
}

