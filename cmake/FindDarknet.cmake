find_path(DARKNET_INCLUDE_DIRS yolo_v2_class.hpp
    "/usr/include"
    "/usr/local/include"
    "/home/lijie/lab/darknet/include"
    )
find_library(DARKNET_LIBRARIES libdarknet.so
    "/usr/lib"
    "/usr/local/lib"
    "/home/liijie/darknet"
    )
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(darknet
    FOUND_VAR darknet_FOUND
    REQUIRED_VARS DARKNET_INCLUDE_DIRS DARKNET_LIBRARIES
    )

