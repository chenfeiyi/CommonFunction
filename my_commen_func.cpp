//读取文件路径下全部文件名称，并保存在vector中
#include "dirent.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <string.h>
#include <fstream>
using namespace std;


/*
  @para: file_list: 用来保存文件名称的vector
  		 folder_path:  文件夹的路径
  @return: None

*/
void getFileList(std::vector<std::string> &file_list,std::string folder_path)
{
    dirent *ptr;
    DIR *dir;
    file_list.clear();
    dir=opendir(folder_path.c_str());
    std::size_t pos;
    folder_path.find_last_of("/",pos);
    if(pos!=(folder_path.size()-1))
        folder_path.push_back('/');
    cout<<folder_path<<endl;

    while((ptr=readdir(dir))!=NULL)
    {
        if(strcmp(ptr->d_name,".")==0||strcmp(ptr->d_name,"..")==0)
            continue;
        std::stringstream ss;
        ss<<folder_path<<ptr->d_name;
        file_list.emplace_back(ss.str());
    }
    closedir(dir);
}

/**
读取txt文件夹中的数据,一行一行的读取
*/
void SplitString(std::string str,std::vector<std::string> &fields,char separators)
{
    std::istringstream sin(str);
    std::string field;
    while(std::getline(sin,field,separators))
    {
        fields.push_back(field);
    }
}

void Readtxt(std::string file_path,std::vector<std::string> &fields,char separator) {
    std::fstream fin;
    fin.open(file_path.c_str(),std::ios::out);
    if(!fin.is_open())
    {
        ROS_ERROR("can't open file %s",file_path.c_str());
        return;
    }
    std::string line;
    while (std::getline(fin, line)) {
        SplitString(line, fields, separator);//将line一整句string按照分号切割成多个string并存储到fields中vector中;
    }
}
