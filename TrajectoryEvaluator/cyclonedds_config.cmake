# Copyright (c) 2022-2023, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

macro(create_cdds_lib CYCLONEDDS_ROOT)
  include(ExternalProject)

  # Get compile options from Zephyr to be passed to the application components.
  zephyr_get_system_include_directories_for_lang_as_string(C ext_system_includes)
  zephyr_get_include_directories_for_lang_as_string(C ext_includes)
  zephyr_get_compile_definitions_for_lang_as_string(C ext_defs)
  zephyr_get_compile_options_for_lang_as_String(C ext_Copts)
  zephyr_get_compile_options_for_lang_as_String(CXX ext_CXXopts)

  if(CONFIG_POSIX_API)
    get_target_property(posix_incs posix_subsys INTERFACE_INCLUDE_DIRECTORIES)
  else()
    message(FATAL_ERROR "CycloneDDS requires POSIX API support (CONFIG_POSIX_API=y)")
  endif()

  set(ext_cflags
      "${ext_defs} -D_POSIX_C_SOURCE=200809L ${ext_system_includes} ${ext_includes} -I${posix_incs} ${ext_Copts} -Wno-error -DDDSRT_HAVE_INET_PTON=1 -Wno-type-limits"
  )

  set(CDDS_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds-prefix/lib)
  set(CDDS_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds-prefix/include)

  ExternalProject_Add(cyclonedds
    SOURCE_DIR ${CYCLONEDDS_ROOT}
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    CMAKE_ARGS
      -DBUILD_SHARED_LIBS=0 -DENABLE_SECURITY=0 -DENABLE_SSL=0 -DENABLE_SOURCE_SPECIFIC_MULTICAST=0 -DENABLE_IPV6=0 -DWITH_ZEPHYR=1 -DENABLE_SHM=0 -DBUILD_IDLC=0
      -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
      -DCMAKE_VERBOSE_MAKEFILE=1
      -DCMAKE_C_FLAGS=${ext_cflags}
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_SYSTEM_NAME=Generic
      -DCMAKE_BUILD_TYPE=Debug
    DEPENDS zephyr_interface
    BUILD_BYPRODUCTS ${CDDS_LIB_DIR}/libddsc.a
  )

  add_library(cdds_lib STATIC IMPORTED GLOBAL)
  add_dependencies(cdds_lib cyclonedds)
  set_target_properties(cdds_lib PROPERTIES IMPORTED_LOCATION ${CDDS_LIB_DIR}/libddsc.a)

  target_include_directories(app PUBLIC ${CDDS_INCLUDE_DIR})
endmacro()

# Populate the include_path argument with the include directory.
function(cyclonedds_include include_path)
  # CycloneDDS is a host package, and doesn't care about the architecture of the target platform.
  # Prevent cmake from checking the architecture by temporarily unsetting CMAKE_SIZEOF_VOID_P.
  set(TMP_CMAKE_SIZEOF_VOID_P ${CMAKE_SIZEOF_VOID_P})
  unset(CMAKE_SIZEOF_VOID_P)
  find_package(CycloneDDS REQUIRED COMPONENTS ddsc)
  set(CMAKE_SIZEOF_VOID_P ${TMP_CMAKE_SIZEOF_VOID_P})

  get_target_property(CycloneDDS_INCLUDE_DIR CycloneDDS::ddsc INTERFACE_INCLUDE_DIRECTORIES)
  set(${include_path} ${CycloneDDS_INCLUDE_DIR} PARENT_SCOPE)
endfunction()
