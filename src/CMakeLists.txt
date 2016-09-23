rock_library(depth_map_preprocessing
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(depth_map_preprocessing_bin Main.cpp
    DEPS depth_map_preprocessing)

