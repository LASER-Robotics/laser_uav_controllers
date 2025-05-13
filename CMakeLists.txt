cmake_minimum_required(VERSION 3.8)
project(laser_uav_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=0")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(acados_include "$ENV{ACADOS_SOURCE_DIR}/include")
set(acados_lib "$ENV{ACADOS_SOURCE_DIR}/lib")
set(full_model_build ${CMAKE_SOURCE_DIR}/scripts/c_generated_code)

set(dependencies
Eigen3
std_srvs 
nav_msgs
std_msgs
laser_msgs
geometry_msgs
)

include_directories(
  include
  ${full_model_build}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${full_model_build})

add_library(laser_uav_controllers SHARED
  src/nmpc_controller.cpp
)

target_link_libraries(laser_uav_controllers
  ${full_model_build}/libacados_ocp_solver_quadrotor_ode.so
	${acados_lib}/libacados.so
)

target_include_directories(laser_uav_controllers PRIVATE src/include)

ament_target_dependencies(laser_uav_controllers
  ${dependencies}
)

install(TARGETS laser_uav_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  scripts
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
