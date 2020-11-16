option(BUILD_JAXON2_SAMPLES "Building JAXON2 sampels" OFF)
if(NOT BUILD_JAXON2_SAMPLES)
  return()
endif()

if(UNIX)
  add_custom_target(
    symlink_to_JAXON2 ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CNOID_BINARY_SHARE_DIR}/JAXON2)
endif()

install(DIRECTORY model DESTINATION ${CHOREONOID_SHARE_SUBDIR}/JAXON2)
