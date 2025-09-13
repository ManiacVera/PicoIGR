set(PICOIGR_PINOUT_FILE "${CMAKE_CURRENT_LIST_DIR}/${PICOIGR_VARIANT}.cmake")
if (EXISTS ${PICOIGR_PINOUT_FILE})
    include(${PICOIGR_PINOUT_FILE})
else()
    message(FATAL_ERROR "Pinout file not found: ${PICOIGR_PINOUT_FILE}")
endif()

message(STATUS "Building for ${PICOIGR_VARIANT}.")

configure_file(${CMAKE_CURRENT_LIST_DIR}/picoigr_pinout.h.in picoigr_pinout.h @ONLY)

set(PROJECT_NAME "${PICOIGR_VARIANT}")
