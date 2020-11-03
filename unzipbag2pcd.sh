#!/bin/sh
#读取文件夹下的bag包，解压成pcd文件
echo "请输入文件夹路径"
read folder
if [ ! -d $folder ];then
echo "文件夹不存在，请重新运行脚本"
exit 1
fi

if [ ! -d "~/unZippedPcd" ];then
mkdir ~/unZippedPcd
fi

file_list=$(ls $folder)
echo "正在解压，请稍候..."

topic=/front/rslidar_points
target_frame=/rslidar
for file in $file_list
do 
    rosrun pcl_ros bag_to_pcd $folder$file $topic ~/unZippedPcd $target_frame
done
echo "完成解压！"
    
