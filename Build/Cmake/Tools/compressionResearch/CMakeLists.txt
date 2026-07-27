#
# compressionResearch CMakeLists.txt | iccDEV Project
# Copyright (c) 2026 The International Color Consortium. All rights reserved.
#

set(TARGET_NAME compressionResearch)
set(SRC_PATH ../../../.. )
set(SRC_ROOT ${SRC_PATH}/Tools/CmdLine/compressionResearch )
set(SOURCES
	"${SRC_ROOT}/compressionResearch.cpp"
)

add_executable(${TARGET_NAME} ${SOURCES})

set_target_properties(${TARGET_NAME} PROPERTIES
  OUTPUT_NAME "compressionResearch"
)

target_link_libraries(${TARGET_NAME}
  PRIVATE
    ${TARGET_LIB_ICCPROFLIB}
)

target_include_directories(${TARGET_NAME}
  PRIVATE
    ${PROJECT_ROOT_DIR}/IccProfLib
)

install(TARGETS ${TARGET_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

# Emscripten WASM support
if(EMSCRIPTEN)
  set_target_properties(${TARGET_NAME} PROPERTIES
    SUFFIX ".js"
    LINK_FLAGS "-sMODULARIZE=1 -sINVOKE_RUN=0 -sEXPORTED_RUNTIME_METHODS=callMain,FS -sALLOW_MEMORY_GROWTH=1"
  )
endif()
