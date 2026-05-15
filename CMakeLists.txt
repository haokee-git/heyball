cmake_minimum_required(VERSION 3.20)

set(MSYS2_UCRT64 "D:/msys64/ucrt64" CACHE PATH "MSYS2 UCRT64 root")
if(WIN32)
  cmake_path(CONVERT "$ENV{PATH}" TO_CMAKE_PATH_LIST _HEYBALL_PATH NORMALIZE)
  list(PREPEND _HEYBALL_PATH "${MSYS2_UCRT64}/bin")
  list(REMOVE_DUPLICATES _HEYBALL_PATH)
  cmake_path(CONVERT "${_HEYBALL_PATH}" TO_NATIVE_PATH_LIST _HEYBALL_NATIVE_PATH)
  set(ENV{PATH} "${_HEYBALL_NATIVE_PATH}")
  set(CMAKE_RC_COMPILER "${MSYS2_UCRT64}/bin/windres.exe" CACHE FILEPATH "MSYS2 windres")
endif()

project(HeyballRaylib LANGUAGES CXX RC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
if(NOT CMAKE_CONFIGURATION_TYPES)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

set(UCRT64_RULE_LAUNCHER "${CMAKE_CURRENT_SOURCE_DIR}/tools/ucrt64-env.cmd")
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${UCRT64_RULE_LAUNCHER}")
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${UCRT64_RULE_LAUNCHER}")

add_library(heyball_core
  src/core.cpp
)
target_include_directories(heyball_core PUBLIC src)
target_compile_options(heyball_core PRIVATE -Wall -Wextra -Wpedantic -finput-charset=UTF-8 -fexec-charset=UTF-8)

add_executable(heyball
  src/main.cpp
  src/async_runtime.cpp
  src/network.cpp
  src/text_utils.cpp
)
if(WIN32)
  set_target_properties(heyball PROPERTIES WIN32_EXECUTABLE TRUE)
endif()
target_include_directories(heyball PRIVATE "${MSYS2_UCRT64}/include")
target_link_libraries(heyball PRIVATE
  heyball_core
  "${MSYS2_UCRT64}/lib/libraylib.a"
  "${MSYS2_UCRT64}/lib/libglfw3.dll.a"
  -lgdi32
  -luser32
  -lwinmm
  -lopengl32
  -lkernel32
  -lws2_32
)
target_compile_options(heyball PRIVATE -Wall -Wextra -Wpedantic -finput-charset=UTF-8 -fexec-charset=UTF-8)
target_link_options(heyball PRIVATE
  -mwindows
  -static
)
add_custom_command(TARGET heyball POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${MSYS2_UCRT64}/bin/glfw3.dll"
    "$<TARGET_FILE_DIR:heyball>/glfw3.dll"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/assets/PingFangSC.otf"
    "$<TARGET_FILE_DIR:heyball>/PingFangSC.otf"
)

add_executable(heyball_tests
  tests/test_runner.cpp
)
target_link_libraries(heyball_tests PRIVATE heyball_core)
target_compile_options(heyball_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_options(heyball_tests PRIVATE -static-libgcc -static-libstdc++)

add_executable(heyball_network_tests
  tests/test_network.cpp
  src/network.cpp
)
target_include_directories(heyball_network_tests PRIVATE src tests)
target_link_libraries(heyball_network_tests PRIVATE heyball_core -lws2_32)
target_compile_options(heyball_network_tests PRIVATE -Wall -Wextra -Wpedantic -DHEYBALL_TEST_BUILD)
target_link_options(heyball_network_tests PRIVATE -static-libgcc -static-libstdc++)

enable_testing()
add_test(NAME heyball_tests COMMAND heyball_tests)
add_test(NAME heyball_network_tests COMMAND heyball_network_tests)
