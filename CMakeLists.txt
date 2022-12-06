cmake_minimum_required(VERSION 3.24)
project(PCL_Learning)

#################################################################################
# 设置C++编译器选项
#################################################################################

set(CMAKE_CXX_STANDARD 11)              # 使用C++11标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)     # 使用C++
set(CMAKE_INCLUDE_CURRENT_DIR ON)       # 包含当前目录
set(CMAKE_BUILD_TYPE "Release")         # release模式去编译

if (WIN32)
    # 开启此选项,可以在release模式下进行debug调试
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
endif ()

#################################################################################
# 第三方库
#################################################################################

# 添加pcl
find_package(PCL REQUIRED)
if (PCL_FOUND)
    message(STATUS "The PCL lib is found!")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    message(STATUS PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS})
    message(STATUS PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS})
    message(STATUS PCL_DEFINITIONS: ${PCL_DEFINITIONS})
    message(STATUS PCL_VERSION: ${PCL_VERSION})

else ()
    message(STATUS "The PCL lib is no found!")
endif ()


#################################################################################
# 系统参数
#################################################################################

#################################################################################
# 添加工程路径
#################################################################################

add_subdirectory(00_common)
add_subdirectory(01_io)
add_subdirectory(02_kdtree)
