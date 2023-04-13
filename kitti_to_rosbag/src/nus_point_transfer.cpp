#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <vector>

using namespace std;

void scan_one_dir( const char * dir_name, std::vector<std::string> &files_vector)
{  
    if( NULL == dir_name )
    {  
        cout<<" dir_name is null ! "<<endl;
        return;
    }
 
    struct stat s;  
    lstat( dir_name , &s );  
    if( ! S_ISDIR( s.st_mode ) )  
    {
        return;
    }
      
    struct dirent * filename;
    DIR * dir;
    dir = opendir( dir_name );  
    if( NULL == dir )  
    {  
        return;  
    }  
 
    int iName=0;
    while( ( filename = readdir(dir) ) != NULL )  
    {  
        if( strcmp( filename->d_name , "." ) == 0 ||
            strcmp( filename->d_name , "..") == 0)
            continue;
 
        char wholePath[128] = {0};
        sprintf(wholePath, "%s", filename->d_name);
        files_vector.push_back(wholePath);
    }
}


int main(int argc, char** argv) {
    std::unordered_map<string, string> origin_map;
    std::unordered_set<string> date_set;
    string out_dir = "/home/yihang/nuscenes/mini/samples_dyn_mini_6/LIDAR_TOP/";
    string file_dir = "/media/yihang/LYH/nuscences/v1.0-mini/samples/LIDAR_TOP/";
    std::vector<std::string> files_vector;
 
    scan_one_dir(file_dir.c_str(), files_vector);
    cout << "vector length: " << files_vector.size() << endl;
    cout << "first file name: " << files_vector[0] << endl;
    cout << "first file name date: " << files_vector[0].substr(0,42) << endl;
    cout << "first file name timestamp: " << files_vector[0] << endl;
    for(int i = 0; i < files_vector.size();i++)
    {
        origin_map[files_vector[i].substr(42,59)] = files_vector[i];
    }

    file_dir = "/home/yihang/nuscenes/mini/point_dyn_6/";
    std::vector<std::string> files_vector1;
    scan_one_dir(file_dir.c_str(), files_vector1);
    for(int i = 0; i < files_vector1.size(); i++)
    {   
        string sequence_dir = file_dir + files_vector1[i] + "/";
        std::vector<std::string> files_vector2;
        scan_one_dir(sequence_dir.c_str(), files_vector2);
        // cout << "sequence: " << sequence_dir << endl;
        // cout << "first sequence name date: " << files_vector2[0] << endl;
        cout << "sequence length: " << files_vector2.size() << endl;
        for (int j = 0; j < files_vector2.size(); j++)
        {   
            ifstream in;
            ofstream out;
            if(origin_map.count(files_vector2[j]))
            {   
                in.open((sequence_dir + files_vector2[j]).c_str(), std::ios::binary);
                out.open((out_dir + origin_map[files_vector2[j]]).c_str(), std::ios::binary);
                out << in.rdbuf();
                out.close();
                in.close();
            }
        }
    }
    cout << "finish transfer" << endl;

}
