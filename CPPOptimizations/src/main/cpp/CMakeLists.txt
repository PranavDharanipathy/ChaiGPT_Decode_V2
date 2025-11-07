cmake_minimum_required(VERSION 18.0.1)

add_library(
        native-lib
        SHARED
        native-lib.cpp
)

find_library(
        log-lib
        log
)

find_library(
        android
)

target_link_libraries(
        native-lib
        ${log_lib}
)