#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kiss_icp_core::kiss_icp_core" for configuration ""
set_property(TARGET kiss_icp_core::kiss_icp_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kiss_icp_core::kiss_icp_core PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkiss_icp_core.so"
  IMPORTED_SONAME_NOCONFIG "libkiss_icp_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS kiss_icp_core::kiss_icp_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_kiss_icp_core::kiss_icp_core "${_IMPORT_PREFIX}/lib/libkiss_icp_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
