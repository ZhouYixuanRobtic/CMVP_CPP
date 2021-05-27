add_library(libexample SHARED "")

file(
        GLOB_RECURSE all_files
        Chapter1/*.cpp
        Chapter1/*.h
)

target_sources(
        libexample
        PRIVATE
        ${all_files}
)

target_link_directories(
        libexample
        PUBLIC
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
)