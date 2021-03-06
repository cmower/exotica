cmake_minimum_required(VERSION 2.8.3)
project(exotica_ik_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica
)

find_package(LAPACK REQUIRED)

AddInitializer(IKSolver)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica
  DEPENDS LAPACK
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ik_solver.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LAPACK_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
