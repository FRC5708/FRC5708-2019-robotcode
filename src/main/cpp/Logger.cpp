#include <iostream>
#include <fstream>
#include <time.h>
#include <string>

using namespace std;
class Logger{
    string name;
    ofstream output_file;
    Logger(string name="NONE",string output_file=""){
        this->name=name;
        time_t rawtime; time (&rawtime); struct tm *tm_struct = localtime(&rawtime);
        if(output_file==""){
            output_file = ("~/Log_" + name + ':' + (tm_struct->tm_hour) + ':' + (tm_struct->tm_min) + ':' + (tm_struct->tm_sec));
        }
        this->output_file.open(output_file,ofstream::out | ofstream::trunc);
    }
    void log(char* message){
        time_t rawtime; time (&rawtime); struct tm *tm_struct = localtime(&rawtime);
        output_file << tm_struct->tm_hour << ':' << tm_struct->tm_min << ':' << tm_struct->tm_sec << ' ' << message << endl;
    }
    ~Logger(){
        output_file.close();
    }
}