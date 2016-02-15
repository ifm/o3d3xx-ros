add_compile_options(-std=c++11)

find_path(libo3d3xx_INCLUDE_DIRS
  NAMES o3d3xx.h
  PATHS /opt/libo3d3xx/include /usr/include
  NO_DEFAULT_PATH
  DOC "libo3d3xx Include directory"
  )

find_library(libo3d3xx_LIBRARIES
  NAMES o3d3xx
  PATHS /opt/libo3d3xx/lib /usr/lib
  NO_DEFAULT_PATH
  DOC "libo3d3xx shared object file"
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(o3d3xx
  DEFAULT_MSG
  libo3d3xx_LIBRARIES libo3d3xx_INCLUDE_DIRS
  )

mark_as_advanced(libo3d3xx_LIBRARIES libo3d3xx_INCLUDE_DIRS)
