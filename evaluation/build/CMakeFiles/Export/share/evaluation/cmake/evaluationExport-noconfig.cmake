#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "evaluation::evaluation" for configuration ""
set_property(TARGET evaluation::evaluation APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(evaluation::evaluation PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libevaluation.so"
  IMPORTED_SONAME_NOCONFIG "libevaluation.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS evaluation::evaluation )
list(APPEND _IMPORT_CHECK_FILES_FOR_evaluation::evaluation "${_IMPORT_PREFIX}/lib/libevaluation.so" )

# Import target "evaluation::complete_kitti_evaluation" for configuration ""
set_property(TARGET evaluation::complete_kitti_evaluation APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(evaluation::complete_kitti_evaluation PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/complete_kitti_evaluation"
  )

list(APPEND _IMPORT_CHECK_TARGETS evaluation::complete_kitti_evaluation )
list(APPEND _IMPORT_CHECK_FILES_FOR_evaluation::complete_kitti_evaluation "${_IMPORT_PREFIX}/bin/complete_kitti_evaluation" )

# Import target "evaluation::test_evaluation" for configuration ""
set_property(TARGET evaluation::test_evaluation APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(evaluation::test_evaluation PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/test_evaluation"
  )

list(APPEND _IMPORT_CHECK_TARGETS evaluation::test_evaluation )
list(APPEND _IMPORT_CHECK_FILES_FOR_evaluation::test_evaluation "${_IMPORT_PREFIX}/bin/test_evaluation" )

# Import target "evaluation::simple_test" for configuration ""
set_property(TARGET evaluation::simple_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(evaluation::simple_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/simple_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS evaluation::simple_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_evaluation::simple_test "${_IMPORT_PREFIX}/bin/simple_test" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
