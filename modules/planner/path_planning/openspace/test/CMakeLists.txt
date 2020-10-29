# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (HybridAstar)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -mavx")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../../"
	"${PROJECT_SOURCE_DIR}/../../../../../common/math/pyclustering/ccore/include/"
	)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib"
	"${PROJECT_SOURCE_DIR}/../../../../../common/math/pyclustering/ccore/libs/"
   )

set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../../common/logging/src/easylogging++.cc" )


# thread库
find_package(Threads MODULE REQUIRED)
find_library(SQLITE3_LIBRARY sqlite3 HINTS ${LIBRARY_DIRECTORY})
find_library(CLUSTER_LIBRARY pyclustering HINTS ${LIBRARY_DIRECTORY})

# 指定生成目标
set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)

add_executable (HybridAstar_test HybridAstar_test.cc ${SOURCE_FILES} )
target_include_directories(HybridAstar_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(HybridAstar_test PUBLIC ${RARE_LIBRARIES} )
target_link_libraries(HybridAstar_test PUBLIC ${CLUSTER_LIBRARY})


add_executable (CollisionChecking_test CollisionChecking_test.cc ${SOURCE_FILES} )
target_include_directories(CollisionChecking_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(CollisionChecking_test PUBLIC ${CLUSTER_LIBRARY})


add_executable (OpenSpace_test OpenSpace_test.cc ${SOURCE_FILES} )
target_include_directories(OpenSpace_test PRIVATE ${HEADER_DIRECTORY})


add_executable (Smoothing_test Smoothing_test.cc ${SOURCE_FILES} )
target_include_directories(Smoothing_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(Smoothing_test PUBLIC ${CLUSTER_LIBRARY})
