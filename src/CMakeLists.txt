cmake_minimum_required(VERSION 3.12)
include_directories(${PROJECT_SOURCE_DIR})
file(GLOB helloworld_SRC CONFIGURE_DEPENDS "*.h" "*.hh" "*.cpp")

idf_component_register(SRCS "main.cpp")