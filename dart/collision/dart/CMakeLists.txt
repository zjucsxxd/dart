# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
dart_add_library(dart_collision_dart ${srcs} ${hdrs})
target_link_libraries(dart_collision_dart ${DART_CORE_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_dart headers" ${hdrs})
dart_generate_include_header_list(
  collision_dart_headers
  "dart/collision/dart/"
  "collision_dart headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/dart.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/dart.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dart.h
  DESTINATION include/dart/collision/dart
  COMPONENT headers
)
install(TARGETS dart_collision_dart EXPORT DARTCoreTargets DESTINATION lib)
install(TARGETS dart_collision_dart EXPORT DARTTargets DESTINATION lib)

