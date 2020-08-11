cmake_minimum_required(VERSION 3.16)
project(DijkstraSearch)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS “${CMAKE_CXX_FLAGS}  -pthread”)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)

find_package(Threads REQUIRED)

add_executable(DijkstraSearch  main.cpp DijkstraSearch.h DijkstraSearch.cpp Server.cpp Server.h EditDataFromClients.cpp EditDataFromClients.h NodeAllocation.cpp NodeAllocation.h)

target_link_libraries(DijkstraSearch ${OpenCV_LIBS})

target_link_libraries(DijkstraSearch Threads::Threads)

