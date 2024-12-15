cmake_minimum_required(VERSION 3.0.2)
project(cream_sooo)

## C++ 표준 설정
add_compile_options(-std=c++14)

## Catkin 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  jsk_recognition_msgs
  visualization_msgs
  tf2
  tf2_ros
)

## Catkin 패키지 선언
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs jsk_recognition_msgs tf2 tf2_ros
)

## include 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 실행 파일 추가
add_executable(hazard_detection src/hazard_detection.cpp)

## Catkin 라이브러리 링크
target_link_libraries(hazard_detection
  ${catkin_LIBRARIES}
)
