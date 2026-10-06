# Run at every build: records the source tree state the binaries are built from.
execute_process(COMMAND sh "${SOURCE_DIR}/cmake/source_state.sh" "${SOURCE_DIR}"
                OUTPUT_VARIABLE state OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc)
if(NOT rc EQUAL 0 OR state STREQUAL "")
  set(state "unknown 1 unknown")
endif()
string(REPLACE " " ";" parts "${state}")
list(GET parts 0 head)
list(GET parts 1 dirty)
list(GET parts 2 digest)
if(IMAGING)
  set(imaging 1)
else()
  set(imaging 0)
endif()
set(content "#pragma once
#define SSB_BUILD_GIT_HEAD \"${head}\"
#define SSB_BUILD_GIT_DIRTY ${dirty}
#define SSB_BUILD_GIT_DIFF_SHA256 \"${digest}\"
#define SSB_BUILD_TYPE \"${BUILD_TYPE}\"
#define SSB_BUILD_IMAGING ${imaging}
")
if(EXISTS "${OUTPUT}")
  file(READ "${OUTPUT}" old)
endif()
if(NOT old STREQUAL content)
  file(WRITE "${OUTPUT}" "${content}")
endif()
