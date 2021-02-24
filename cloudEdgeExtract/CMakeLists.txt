##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.11)
project(cloudEdgeExtract)# 在这里修改工程名字
add_compile_options(-std=c++14 )#添加c++11的编译选项
find_package(Eigen3 REQUIRED)#添加要查找的包
find_package(PCL REQUIRED)#添加要查找的包
find_package(Boost REQUIRED COMPONENTS system)
find_package(Ceres REQUIRED)#添加要查找的包
include_directories(
    include #文件夹
    ${EIGEN_INCLUDE_DIR} #根据导入的库进行头文件路径查找
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${ceres_INCLUDE_DIR}
)
include_directories("/usr/include/eigen3")
add_executable(${PROJECT_NAME}_node src/main.cpp src/extractEdge.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${PCL_LIBRARIES}
    ${EIGEN_LIBRARIES} 
    ${CERES_LIBRARIES} 
    ${Boost_LIBRARIES}
 )
################################################################################

# Install

################################################################################
#配置可执行文件
#install(TARGETS template_bin RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#配置包含的头文件（安装后可在/opt/ros/kinetic/include/${PROJECT_NAME}路径下找到该头文件）
#install(DIRECTORY include/${PROJECT_NAME}/

#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}

#)

#配置要开放的文件夹（安装后可在/opt/ros/kinetic/share/${PROJECT_NAME}路径下找到）
#install(DIRECTORY launch cfg resources 
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
