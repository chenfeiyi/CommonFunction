//读取文件路径下全部文件名称，并保存在vector中
//#include "dirent.h"

/*
  @para: file_stack: 用来保存文件名称的vector
  		 file_path:  文件夹的路径
  @return: None

*/
void read_file_path_and_stack(std::vector<std::string> &file_stack,std::string filepath)
{
    DIR *file_dir = opendir(filepath.c_str());
    struct dirent *filename;
    while((filename=readdir(file_dir))!=NULL)
    {
        if(strcmp(filename->d_name,".")==0||strcmp(filename->d_name,"..")==0)
            continue;
        std::stringstream ss;
        ss<<image_file_path<<filename->d_name;
        file_stack.push_back(ss.str());
    }
}


/**
读取txt文件夹中的数据
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
