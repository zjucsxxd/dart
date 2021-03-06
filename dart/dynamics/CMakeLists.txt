# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
dart_add_library(dart_dynamics ${srcs} ${hdrs})
target_link_libraries(
  dart_dynamics
  dart_common
  dart_math
  ${DART_CORE_DEPENDENCIES}
)

# Generate header for this namespace
dart_get_filename_components(header_names "dynamics headers" ${hdrs})
dart_generate_include_header_list(
  dynamics_headers
  "dart/dynamics/"
  "dynamics headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/dynamics.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dynamics.h
  DESTINATION include/dart/dynamics
  COMPONENT headers
)
install(TARGETS dart_dynamics EXPORT DARTCoreTargets DESTINATION lib)
install(TARGETS dart_dynamics EXPORT DARTTargets DESTINATION lib)

