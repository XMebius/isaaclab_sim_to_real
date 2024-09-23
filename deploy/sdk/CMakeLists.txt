cmake_minimum_required(VERSION 2.8.3)
project(SimToReal)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

link_directories(lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

add_executable(aliengo_lcm ${CMAKE_CURRENT_SOURCE_DIR}/aliengo_lcm.cpp)
target_link_libraries(aliengo_lcm ${EXTRA_LIBS})