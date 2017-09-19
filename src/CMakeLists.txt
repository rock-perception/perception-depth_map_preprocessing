rock_library(depth_map_preprocessing
    SOURCES Filters.cpp
            PointCloudConversion.cpp
    HEADERS Filters.hpp
            PointCloudConversion.hpp
            Config.hpp
    DEPS_PKGCONFIG base-types)

