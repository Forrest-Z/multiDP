# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (Geomentry)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
   	"${PROJECT_SOURCE_DIR}/../../../../"
    )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )

# 指定生成目标

add_executable (miniball_example miniball_example.cc)

add_executable (miniball_example_containers miniball_example_containers.cc)


add_executable (simple_voronoi simple_voronoi.cc)
target_include_directories(simple_voronoi PRIVATE ${HEADER_DIRECTORY})


set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)
add_executable (plot_voronoi plot_voronoi.cc)
target_include_directories(plot_voronoi PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(plot_voronoi ${RARE_LIBRARIES})


add_executable (test_RScurve test_RScurve.cc)
target_include_directories(test_RScurve PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(test_RScurve ${RARE_LIBRARIES})


ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MODULE) 
add_executable (linesegment2d_test linesegment2d_test.cc)
target_include_directories(linesegment2d_test PRIVATE ${HEADER_DIRECTORY})

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MODULE) 
add_executable (aabox2d_test aabox2d_test.cc)
target_include_directories(aabox2d_test PRIVATE ${HEADER_DIRECTORY})

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MODULE) 
add_executable (box2d_test box2d_test.cc)
target_include_directories(box2d_test PRIVATE ${HEADER_DIRECTORY})