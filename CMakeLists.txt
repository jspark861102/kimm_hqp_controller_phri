cmake_minimum_required(VERSION 3.0.2)
SET(PROJECT_NAME kimm_hqp_controller)
SET(PROJECT_DESCRIPTION "HQP Controller for KIMM")
SET(PROJECT_URL "https://github.com/ggory15/kimm_hqp_controller")
add_compile_options(-std=c++17 -Ofast)

SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

ADD_PROJECT_DEPENDENCY(pinocchio 2.3.1 REQUIRED)
ADD_PROJECT_DEPENDENCY(eiquadprog 1.1.3 REQUIRED)
ADD_PROJECT_DEPENDENCY(kimm_qpoases REQUIRED)
ADD_PROJECT_DEPENDENCY(kimm_trajectory_smoother REQUIRED)

FINDPYTHON(3.6 EXACT REQUIRED)
SET(PYWRAP ${PROJECT_NAME}_pywrap)
ADD_PROJECT_DEPENDENCY(eigenpy REQUIRED)
SEARCH_FOR_BOOST_PYTHON(REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(kimm_qpoases REQUIRED)
find_package(kimm_trajectory_smoother REQUIRED)

# Main Library
SET(${PROJECT_NAME}_MATH_HEADERS
  include/kimm_hqp_controller/math/fwd.hpp
  include/kimm_hqp_controller/math/util.hpp
  include/kimm_hqp_controller/math/constraint_base.hpp
  include/kimm_hqp_controller/math/constraint_equality.hpp
  include/kimm_hqp_controller/math/constraint_inequality.hpp
  include/kimm_hqp_controller/math/constraint_bound.hpp
  )

SET(${PROJECT_NAME}_TASKS_HEADERS
  include/kimm_hqp_controller/tasks/fwd.hpp
  include/kimm_hqp_controller/tasks/task_base.hpp
  include/kimm_hqp_controller/tasks/task_motion.hpp
  include/kimm_hqp_controller/tasks/task_se3_equality.hpp
  include/kimm_hqp_controller/tasks/task_joint_bound.hpp
  include/kimm_hqp_controller/tasks/task_joint_posture.hpp
  include/kimm_hqp_controller/tasks/task_mobile_base.hpp
  include/kimm_hqp_controller/tasks/task_contact_force.hpp
  )

SET(${PROJECT_NAME}_CONTACTS_HEADERS
  include/kimm_hqp_controller/contact/fwd.hpp
  include/kimm_hqp_controller/contact/contact_base.hpp
  # include/kimm_hqp_controller/contacts/contact-6d.hpp
  include/kimm_hqp_controller/contact/contact_point.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
  include/kimm_hqp_controller/trajectory/fwd.hpp
  include/kimm_hqp_controller/trajectory/trajectory_base.hpp
  include/kimm_hqp_controller/trajectory/trajectory_se3.hpp
  include/kimm_hqp_controller/trajectory/trajectory_euclidian.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
  include/kimm_hqp_controller/solver/fwd.hpp
  include/kimm_hqp_controller/solver/util.hpp
  include/kimm_hqp_controller/solver/solver_HQP_output.hpp
  include/kimm_hqp_controller/solver/solver_HQP_base.hpp
  include/kimm_hqp_controller/solver/solver_HQP_factory.hpp
  include/kimm_hqp_controller/solver/solver_HQP_factory.hxx
  include/kimm_hqp_controller/solver/solver_HQP_eiquadprog.hpp
  include/kimm_hqp_controller/solver/solver_HQP_eiquadprog_rt.hpp
  include/kimm_hqp_controller/solver/solver_HQP_eiquadprog_rt.hxx
  include/kimm_hqp_controller/solver/solver_HQP_eiquadprog_fast.hpp
  include/kimm_hqp_controller/solver/solver_HQP_qpoases.hpp
  )

SET(${PROJECT_NAME}_ROBOTS_HEADERS
  include/kimm_hqp_controller/robot/fwd.hpp
  include/kimm_hqp_controller/robot/robot_wrapper.hpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_HEADERS
  include/kimm_hqp_controller/formulation/contact_level.hpp
  include/kimm_hqp_controller/formulation/inverse_dynamics_formulation_base.hpp
  include/kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp
  )

FILE(GLOB ${PYWRAP}_HEADERS
  include/kimm_hqp_controller/bindings/python/fwd.hpp
  include/kimm_hqp_controller/bindings/python/constraint/*.hpp
  # include/kimm_hqp_controller/bindings/python/contacts/*.hpp
  include/kimm_hqp_controller/bindings/python/formulation/*.hpp
  include/kimm_hqp_controller/bindings/python/robot/*.hpp
  include/kimm_hqp_controller/bindings/python/solver/*.hpp
  include/kimm_hqp_controller/bindings/python/tasks/*.hpp
  include/kimm_hqp_controller/bindings/python/trajectory/*.hpp
  # include/kimm_hqp_controller/bindings/python/utils/*.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  include/kimm_hqp_controller/config.hpp
  include/kimm_hqp_controller/util/statistics.hpp
  include/kimm_hqp_controller/util/stop_watch.hpp
  include/kimm_hqp_controller/util/Stdafx.hh
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_TASKS_HEADERS}
  ${${PROJECT_NAME}_CONTACTS_HEADERS}
  ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
  ${${PROJECT_NAME}_SOLVERS_HEADERS}
  ${${PROJECT_NAME}_ROBOTS_HEADERS}
  ${${PROJECT_NAME}_FORMULATIONS_HEADERS}
  )

LIST(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_MATH_SOURCES
  src/math/constraint_base.cpp
  src/math/constraint_equality.cpp
  src/math/constraint_inequality.cpp
  src/math/constraint_bound.cpp
  src/math/util.cpp
  )

SET(${PROJECT_NAME}_TASKS_SOURCES
  src/tasks/task_base.cpp
  src/tasks/task_joint_bound.cpp
  src/tasks/task_joint_posture.cpp
  src/tasks/task_motion.cpp
  src/tasks/task_se3_equality.cpp
  src/tasks/task_mobile_base.cpp
  )

SET(${PROJECT_NAME}_CONTACTS_SOURCES
  src/contact/contact_base.cpp
  src/contact/contact_point.cpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_SOURCES
  src/trajectory/trajectory_se3.cpp
  src/trajectory/trajectory_euclidian.cpp
  )

SET(${PROJECT_NAME}_SOLVERS_SOURCES
  src/solver/solver_HQP_base.cpp
  src/solver/solver_HQP_factory.cpp
  src/solver/solver_HQP_eiquadprog.cpp
  src/solver/solver_HQP_eiquadprog_fast.cpp
  src/solver/solver_HQP_qpoases.cpp
  src/solver/util.cpp
  )

SET(${PROJECT_NAME}_ROBOTS_SOURCES
  src/robot/robot_wrapper.cpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_SOURCES
  src/formulation/contact_level.cpp
  src/formulation/inverse_dynamics_formulation_base.cpp
  src/formulation/inverse_dynamics_formulation_acc.cpp
  )

SET(${PROJECT_NAME}_SOURCES
  src/util/statistics.cpp
  src/util/stop_watch.cpp
  ${${PROJECT_NAME}_MATH_SOURCES}
  ${${PROJECT_NAME}_TASKS_SOURCES}
  ${${PROJECT_NAME}_CONTACTS_SOURCES}
  ${${PROJECT_NAME}_TRAJECTORIES_SOURCES}
  ${${PROJECT_NAME}_SOLVERS_SOURCES}
  ${${PROJECT_NAME}_ROBOTS_SOURCES}
  ${${PROJECT_NAME}_FORMULATIONS_SOURCES}
  )

ADD_HEADER_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> ${kimm_trajectory_smoother_INCLUDE_DIRS}  ${kimm_qpoases_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio eiquadprog::eiquadprog ${kimm_trajectory_smoother_libs} ${kimm_qpoases_smoother_libs})

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(bindings)

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})

