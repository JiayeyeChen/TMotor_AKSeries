cmake_minimum_required(VERSION 3.0.2)
project(serial_asio)

find_package(Boost REQUIRED COMPONENTS system thread regex)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
)

set(SRC
  main.cpp
)

set(LIBS
  ${Boost_LIBRARIES}
)

add_executable(example ${SRC})
target_link_libraries(example ${LIBS})