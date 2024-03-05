cmake_minimum_required(VERSION 3.3)
project(resilient_team_planner)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

find_package(GUROBI REQUIRED)

if(APPLE)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        set(YAML_CPP_LIBRARIES /opt/homebrew/lib/libyaml-cpp.dylib) # Or libyaml-cpp.a for static library
        include_directories(include src ${GUROBI_INCLUDE_DIRS} /usr/local/include)
    else()
        set(YAML_CPP_LIBRARIES /usr/local/lib/libyaml-cpp.dylib) # Or libyaml-cpp.a for static library
        include_directories(include src ${GUROBI_INCLUDE_DIRS} /usr/local/include)
    endif()
else()
    set(YAML_CPP_LIBRARIES yaml-cpp)
    include_directories(include src ${GUROBI_INCLUDE_DIRS})
endif()

set(TEAMPLANNER_PYTOOL OFF)
if(TEAMPLANNER_PYTOOL)
    find_package(Python COMPONENTS Interpreter NumPy)
    find_package(Boost REQUIRED COMPONENTS python)
    include(FindPythonLibs)
    include_directories(${PYTHON_INCLUDE_DIRS})
    set(CMAKE_SHARED_MODULE_PREFIX "")
endif()

# list source files here
add_library(LocalMath src/LocalMath.cpp)

add_library(TeamPlannerParam src/TaskParam.cpp src/VehicleParam.cpp src/TeamPlannerParam.cpp)
target_link_libraries(TeamPlannerParam ${YAML_CPP_LIBRARIES})

add_library(TeamPlannerVar src/VehicleVar.cpp src/TaskVar.cpp)
target_link_libraries(TeamPlannerVar TeamPlannerParam ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

add_library(TeamPlannerGraph src/Graph.cpp)
target_link_libraries(TeamPlannerGraph ${YAML_CPP_LIBRARIES})

add_library(FlowConverter src/FlowConverter.cpp)
target_link_libraries(FlowConverter TeamPlannerGraph LocalMath ${YAML_CPP_LIBRARIES} ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

add_library(TeamPlannerCallback src/TeamPlannerCallback.cpp)

add_library(TeamPlanner src/TeamPlanner.cpp)
target_link_libraries(TeamPlanner TeamPlannerGraph TeamPlannerParam TeamPlannerVar TeamPlannerCallback FlowConverter LocalMath ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

add_executable(main src/main.cpp)
target_link_libraries(main TeamPlanner  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# add_executable(multi_main test/multi_main.cpp)
# target_link_libraries(multi_main TeamPlanner  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# add_executable(explore_main test/explore_main.cpp)
# target_link_libraries(explore_main TeamPlanner  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# add_executable(medical_main test/medical_main.cpp)
# target_link_libraries(medical_main TeamPlanner  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# add_executable(modify_main test/modify_main.cpp)
# target_link_libraries(modify_main TeamPlanner  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# add_executable(mip1_c++ mip1_c++.cpp)
# target_link_libraries(mip1_c++ optimized ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})

# add_executable(facility_c++ facility_c++.cpp)
# target_link_libraries(facility_c++ optimized ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})

# target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY})

# Test files
# add_executable(graph_main test/graph_main.cpp)
# target_link_libraries(graph_main TeamPlannerGraph  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# add_executable(inverse_norm_cdf_main test/inverse_norm_cdf_main.cpp)
# target_link_libraries(inverse_norm_cdf_main LocalMath  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

# yibin add #

# add_executable(flow_main test/flow_main.cpp)
# target_link_libraries(flow_main ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY} FlowConverter)

# add_executable(yaml_main test/yaml_main.cpp)
# target_link_libraries(yaml_main ${YAML_CPP_LIBRARIES})

# add_executable(param_main test/param_main.cpp)
# target_link_libraries(param_main TeamPlannerParam  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

#add_executable(var_main test/var_main.cpp)
#target_link_libraries(var_main TeamPlannerVar  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    include(FeatureSummary)
    feature_summary(WHAT ALL)
endif()


# if(TEAMPLANNER_PYTOOL)
#     #env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -DCMAKE_BUILD_TYPE=Release ..
#     add_library(TeamPlannerPy MODULE src/TeamPlannerPy.cpp)
#     target_link_libraries(TeamPlannerPy TeamPlanner ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} Python::NumPy)
#     target_include_directories(TeamPlannerPy PRIVATE ${PYTHON_INCLUDE_DIRS})
# endif()

message(STATUS "GUROBI_LIBRARY: ${GUROBI_LIBRARY}")
message(STATUS "GUROBI_CXX_LIBRARY: ${GUROBI_CXX_LIBRARY}")
message(STATUS "GUROBI_INCLUDE_DIRS: ${GUROBI_INCLUDE_DIRS}")

target_include_directories(TeamPlanner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${GUROBI_INCLUDE_DIRS})
