
#include <ros/ros.h>

#include <fstream>

#include "sys/types.h"
#include "sys/sysinfo.h"
#include <dirent.h>

using namespace std;

class CPUPercentage{
/** 
 * @file cpuusage.cpp
 * @brief print out the percent cpu usage at a given interval
 * @author Matthew McCormick (thewtex)
 * @version 
 * @date 2009-08-12
 */

private:
    ifstream m_stat_file;
    unsigned long long m_current_user;
    unsigned long long m_current_system;
    unsigned long long m_current_nice;
    unsigned long long m_current_idle;
    unsigned long long m_next_user;
    unsigned long long m_next_system;
    unsigned long long m_next_nice;
    unsigned long long m_next_idle;
    unsigned long long m_diff_user;
    unsigned long long m_diff_system;
    unsigned long long m_diff_nice;
    unsigned long long m_diff_idle;

    string m_stat_line;
    size_t m_line_start_pos;
    size_t m_line_end_pos;
    istringstream m_iss;

    float m_percentage;

public:
    CPUPercentage():
        m_current_user(0),
        m_current_system(0),
        m_current_nice(0),
        m_current_idle(0),
        m_next_user(0),
        m_next_system(0),
        m_next_nice(0),
        m_next_idle(0),
        m_diff_user(0),
        m_diff_system(0),
        m_diff_nice(0),
        m_diff_idle(0){
        m_stat_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);
    }

    ~CPUPercentage(){
        if (m_stat_file.is_open())
           m_stat_file.close();
    }
    float get_percentage(){
        m_stat_file.open("/proc/stat");
        getline(m_stat_file, m_stat_line);
        m_stat_file.close();

        // skip "cpu"
        m_line_start_pos = m_stat_line.find_first_not_of(" ", 3);
        m_line_end_pos = m_stat_line.find_first_of(' ', m_line_start_pos);
        m_line_end_pos = m_stat_line.find_first_of(' ', m_line_end_pos + 1);
        m_line_end_pos = m_stat_line.find_first_of(' ', m_line_end_pos + 1);
        m_line_end_pos = m_stat_line.find_first_of(' ', m_line_end_pos + 1);
        m_iss.str(m_stat_line.substr(m_line_start_pos, m_line_end_pos - m_line_start_pos));
        m_iss >> m_next_user >> m_next_nice >> m_next_system >> m_next_idle;
        m_iss.clear();

        m_diff_user   = m_next_user - m_current_user;
        m_diff_system = m_next_system - m_current_system;
        m_diff_nice   = m_next_nice - m_current_nice;
        m_diff_idle   = m_next_idle - m_current_idle;
        m_percentage = static_cast<float>(m_diff_user + m_diff_system + m_diff_nice)/static_cast<float>(m_diff_user + m_diff_system + m_diff_nice + m_diff_idle)*100.0;

        m_current_user = m_next_user;
        m_current_system = m_next_system;
        m_current_nice = m_next_nice;
        m_current_idle = m_next_idle;

        return m_percentage;
    }
};







class TraversabilityMonitor {

private:

    ros::NodeHandle nh;

    vector<string> processList;
    vector<string> processListShort;

    CPUPercentage cpupercent;
    float cpuTemperature;

    string greenColorStart; 
    string cyanColorStart;
    string colorEnd;

public:

    TraversabilityMonitor():
    nh("~")
    {
        // black - 30 red - 31 green - 32 brown - 33 blue - 34 magenta - 35 cyan - 36 lightgray - 37
        greenColorStart = "\033[1;32m"; 
        cyanColorStart = "\033[1;36m"; 
        colorEnd = "\033[0m";

        ros::Duration(3).sleep();

        // add process names
        // processList.push_back("traversability_filter");
        processList.push_back("traversability_map");
        // processList.push_back("traversability_prm");
        processList.push_back("traversability_cloud");

        // short names for display in terminal
        // processListShort.push_back("Filter");
        processListShort.push_back("Map");
        // processListShort.push_back("PRM");
        processListShort.push_back("Cloud");

        CPUPercentage cpupercent;

        // displayProcessName();

        cout << endl;
    }
    ~TraversabilityMonitor(){}

    void displayProcessName(){
        for (int i = 0; i < processListShort.size(); ++i){
            cout.width(10);
            cout << processListShort[i] << std::flush;
        }
        cout << endl;
    }

    void displaySystemInfo(){
        // cout settings
        cout.precision(1);
        cout.setf(ios::fixed | ios::right);
        
        cout << "\r" << std::flush << " ";

        // Display Node Memory Usage
        for (int i = 0; i < processList.size(); ++i){
            string thisName = processList[i];
            int thisPid = getPidByName(thisName);
            double thisMem = getMemoryByPid(thisPid);
            if (thisPid == -1)
                continue;
            cout << greenColorStart << processListShort[i] << ":" << colorEnd;
            cout << cyanColorStart << thisMem << "M  " << colorEnd;
        }

        // Display CPU Usage
        // cout << greenColorStart << "CPU Usage: " << colorEnd;
        // cout << cyanColorStart << cpupercent.get_percentage() << "%  " << colorEnd;
        // cout << greenColorStart << "CPU Temp: " << colorEnd;
        // cout << cyanColorStart << getCpuTemperature() << "°C  " << colorEnd;
        cout << greenColorStart << "CPU: " << colorEnd;
        cout << cyanColorStart << cpupercent.get_percentage() << "% " << getCpuTemperature() << "°C " << colorEnd;

        // Display Total Memory Usage
        cout << greenColorStart << "Mem Usage: " << colorEnd;
        cout << cyanColorStart << getTotalMemoryUsage() << "%  " << colorEnd;
    }

    int getPidByName(string procName){
        int pid = -1;
        // Open the /proc directory
        DIR *dp = opendir("/proc");
        if (dp != NULL){
            // Enumerate all entries in directory until process found
            struct dirent *dirp;
            while (pid < 0 && (dirp = readdir(dp))){
                // Skip non-numeric entries
                int id = atoi(dirp->d_name);
                if (id > 0){
                    // Read contents of virtual /proc/{pid}/cmdline file
                    string cmdPath = string("/proc/") + dirp->d_name + "/cmdline";
                    ifstream cmdFile(cmdPath.c_str());
                    string cmdLine;
                    getline(cmdFile, cmdLine);
                    if (!cmdLine.empty()){
                        // Keep first cmdline item which contains the program path
                        size_t pos = cmdLine.find('\0');
                        if (pos != string::npos)
                            cmdLine = cmdLine.substr(0, pos);
                        // Keep program name only, removing the path
                        pos = cmdLine.rfind('/');
                        if (pos != string::npos)
                            cmdLine = cmdLine.substr(pos + 1);
                        // Compare against requested process name
                        if (procName == cmdLine)
                            pid = id;
                    }
                }
            }
        }
        closedir(dp);
        return pid;
    }

    // Memory usage
    int parseLine(char* line){
        int i = strlen(line);
        const char* p = line;
        while (*p <'0' || *p > '9') p++; // skip string that is not between 0 ~ 9
        // '\0' is the null termination character. It marks the end of the string.
        line[i-3] = '\0'; // This assumes that a digit will be found and the line ends in " Kb".
        i = atoi(p); // Convert string to integer
        return i;
    }

    double getMemoryByPid(int pid){ //Note: this value is in KB!
        if (pid == -1)
            return -1;
        string pidName;
        pidName.append("/proc/");
        char pidString[10];
        sprintf(pidString, "%d", pid);
        pidName.append(pidString);
        pidName.append("/status");
        FILE* file = fopen(pidName.c_str(), "r");
        double result = -1;
        char line[128];

        while (fgets(line, 128, file) != NULL){ // Get string from stream
            if (strncmp(line, "VmRSS:", 6) == 0){ // Compare characters of two strings
                result = parseLine(line);
                break;
            }
        }
        fclose(file);
        return result / 1024.0;
    }
    // Read CPU Temperature
    float getCpuTemperature(){
        for (int i = 0; i < 10; ++i){
            ostringstream convert; 
            convert << i;
            string filePath = "/sys/bus/platform/devices/coretemp.0/hwmon/hwmon" + convert.str() + "/temp1_input";
            std::fstream myfile(filePath.c_str(), std::ios_base::in);
            if (myfile.fail())
                continue;
            myfile >> cpuTemperature;
            cpuTemperature = cpuTemperature / 1000.0;
            break;
        }
        return cpuTemperature;
    }
    // Read Total Memory Usage
    double getTotalMemoryUsage(){
        FILE* file = fopen("/proc/meminfo", "r");
        char line[128];
        double totalMem, availableMem;
        // total memory
        while (fgets(line, 128, file) != NULL){ // Get string from stream
            if (strncmp(line, "MemTotal:", 9) == 0){ // Compare characters of two strings
                totalMem = (double)parseLine(line);
                break;
            }
        }
        // available memory
        while (fgets(line, 128, file) != NULL){ // Get string from stream
            if (strncmp(line, "MemAvailable:", 13) == 0){ // Compare characters of two strings
                availableMem = (double)parseLine(line);
                break;
            }
        }
        return 100.0 - availableMem / totalMem * 100;
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_map");

    TraversabilityMonitor TM;

    ros::Rate r(0.5); // hz
    while (ros::ok()){
        TM.displaySystemInfo();
        r.sleep();
    }

    return 0;
}