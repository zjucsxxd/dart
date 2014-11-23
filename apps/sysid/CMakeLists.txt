###############################################
# apps/sysid
file(GLOB sysid_src "*.cpp" "*.h")
add_executable(sysid ${sysid_src})
target_link_libraries(sysid dart)
set_target_properties(sysid PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
