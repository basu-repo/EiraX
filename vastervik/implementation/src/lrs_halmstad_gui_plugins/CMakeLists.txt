cmake_minimum_required(VERSION 3.16)
project(lrs_halmstad_gui_plugins)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Qml Quick Widgets)

add_library(UavSpawner SHARED
  src/UavSpawner.cc
  include/lrs_halmstad_gui_plugins/UavSpawner.hh
  qml/UavSpawner.qrc
)

target_include_directories(UavSpawner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(UavSpawner
  PRIVATE
    ament_index_cpp::ament_index_cpp
    gz-gui8::gz-gui8
    gz-msgs10::gz-msgs10
    gz-plugin2::core
    gz-transport13::gz-transport13
    Qt5::Core
    Qt5::Qml
    Qt5::Quick
    Qt5::Widgets
)

install(
  TARGETS UavSpawner
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
