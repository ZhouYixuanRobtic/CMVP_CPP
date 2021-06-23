add_library(libexample SHARED "")

file(
        GLOB_RECURSE all_files
        Chapter1/*.cpp
        Chapter1/*.h
        Chapter2/*.cpp
        Chapter2/*.h
)

target_sources(
        libexample
        PRIVATE
        ${all_files}
)

target_link_libraries(
        libexample
        PUBLIC
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        ${VTK_LIBRARIES}
)