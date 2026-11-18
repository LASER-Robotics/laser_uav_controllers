cmake_minimum_required(VERSION 3.8)
project(laser_uav_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
find_package(Boost REQUIRED) 

set(acados_include "$ENV{ACADOS_SOURCE_DIR}/include")
set(acados_lib "$ENV{ACADOS_SOURCE_DIR}/lib")
set(full_model_build ${CMAKE_CURRENT_SOURCE_DIR}/scripts/multirotor_ode/)

if(NOT DEFINED ENV{ACADOS_SOURCE_DIR} OR "$ENV{ACADOS_SOURCE_DIR}" STREQUAL "")
  message(FATAL_ERROR "ACADOS_SOURCE_DIR environment variable is not set.")
endif()

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
  ${Boost_INCLUDE_DIR}
)

add_library(acados_ocp_solver_multirotor_ode SHARED
  ${full_model_build}/acados_solver_multirotor_ode.c
  ${full_model_build}/multirotor_ode_model/multirotor_ode_expl_ode_fun.c
  ${full_model_build}/multirotor_ode_model/multirotor_ode_expl_vde_forw.c
  ${full_model_build}/multirotor_ode_model/multirotor_ode_expl_vde_adj.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_0_fun.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_0_fun_jac_ut_xt.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_0_hess.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_fun.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_fun_jac_ut_xt.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_hess.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_e_fun.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_e_fun_jac_ut_xt.c
  ${full_model_build}/multirotor_ode_cost/multirotor_ode_cost_y_e_hess.c
)

set_target_properties(acados_ocp_solver_multirotor_ode PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED ON
)
target_compile_options(acados_ocp_solver_multirotor_ode PRIVATE -O2 -w)

target_link_libraries(acados_ocp_solver_multirotor_ode
  ${acados_lib}/libacados.so
  ${acados_lib}/libhpipm.so
  ${acados_lib}/libblasfeo.so
  m
)

add_library(laser_uav_controllers SHARED
  src/common.cpp
  src/nmpc_controller.cpp
  src/indi_controller.cpp
)

target_link_libraries(laser_uav_controllers
  acados_ocp_solver_multirotor_ode
	${acados_lib}/libacados.so
)

target_include_directories(laser_uav_controllers PRIVATE src/include)

ament_target_dependencies(laser_uav_controllers
  ${dependencies}
)

install(TARGETS laser_uav_controllers acados_ocp_solver_multirotor_ode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  scripts
  DESTINATION include/
  PATTERN "*.o" EXCLUDE
  PATTERN "*.so" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(laser_uav_controllers acados_ocp_solver_multirotor_ode)
ament_export_dependencies(${dependencies})
ament_package()
