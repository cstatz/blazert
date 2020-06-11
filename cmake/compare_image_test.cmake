execute_process(COMMAND ${RENDER_EXECUTABLE} RESULT_VARIABLE status)
if (NOT status STREQUAL "0")
    message(FATAL_ERROR "Error running ${RENDER_EXECUTABLE}.")
endif ()
if (NOT COMPARE_EXECUTABLE STREQUAL "")
    execute_process(
        COMMAND
            ${COMPARE_EXECUTABLE}
            -metric MSE -compose Src -highlight-color White -lowlight-color Black
            ${OUTPUT}
            ${REFERENCE}
            ${DIFFERENCE}
        RESULT_VARIABLE status)
    if (NOT status STREQUAL "0")
        message(FATAL_ERROR "Compare failed: '${OUTPUT}' and '${REFERENCE}' differ.")
    else ()
        message(STATUS "Compare succeeded: '${OUTPUT}' and '${REFERENCE}' are identical.")
    endif ()
    file(REMOVE ${DIFFERENCE})
    file(REMOVE ${OUTPUT})
endif ()
