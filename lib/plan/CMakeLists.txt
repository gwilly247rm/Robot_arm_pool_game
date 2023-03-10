cmake_minimum_required(VERSION 2.8)

# Set variables
set(PROJECT_NAME plan)
set(PROJECT_LANGUAGE C CXX)
#set(PROJECT_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/demo.h)
file(GLOB PROJECT_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
#set(PROJECT_DEPS ${DARKNET_LIBRARIES})

# Compile setting
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set project
project(${PROJECT_NAME} ${PROJECT_LANGUAGE})

# Add definition
if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif()

# Include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Set file list
file(GLOB PROJECT_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    )

# Build library
add_library(${PROJECT_NAME} STATIC ${PROJECT_SRCS})
#target_link_libraries(${PROJECT_NAME} ${PROJECT_DEPS})
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    OUTPUT_NAME         ${PROJECT_NAME}
        PREFIX          "lib"
    )

# Install
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
    LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include"
    )
install(FILES ${PROJECT_HEADERS}
    DESTINATION "${CMAKE_INSTALL_PREFIX}/include"
    )
