function(enable_project_cpp_diagnostics target_name)
  if(NOT TARGET "${target_name}")
    message(FATAL_ERROR "Cannot enable project diagnostics for missing target '${target_name}'")
  endif()

  set(warning_flags
    -Wall
    -Wextra
    -Wpedantic
    -pedantic-errors
    -Werror
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wformat=2
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wold-style-cast
    -Woverloaded-virtual
    -Wnon-virtual-dtor
    -Wcast-align
    -Wcast-qual
    -Wundef
    -Wswitch-enum
    -Wzero-as-null-pointer-constant
  )

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    list(APPEND warning_flags
      -Wduplicated-branches
      -Wduplicated-cond
      -Wlogical-op
      -Wuseless-cast
    )
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(APPEND warning_flags
      -Wcomma
      -Wextra-semi
      -Wimplicit-int-conversion
      -Wloop-analysis
      -Wmove
      -Wthread-safety
    )
  else()
    message(FATAL_ERROR
      "Project C++ diagnostics require GCC or Clang; found '${CMAKE_CXX_COMPILER_ID}'")
  endif()

  set_target_properties("${target_name}" PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options("${target_name}" PRIVATE ${warning_flags})
endfunction()
