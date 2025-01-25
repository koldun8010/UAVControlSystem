cmake_minimum_required(VERSION 3.26)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra -Werror")

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

add_library(myServer SHARED
    server.cpp
    server.h
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(myServer
        MAVSDK::mavsdk
)
