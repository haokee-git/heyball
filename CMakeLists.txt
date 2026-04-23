cmake_minimum_required(VERSION 3.20)

project(HeyballRaylib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MSYS2_UCRT64 "D:/msys64/ucrt64" CACHE PATH "MSYS2 UCRT64 root")
set(UCRT64_RULE_LAUNCHER "${CMAKE_CURRENT_SOURCE_DIR}/tools/ucrt64-env.cmd")
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${UCRT64_RULE_LAUNCHER}")
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${UCRT64_RULE_LAUNCHER}")

add_library(heyball_core
  src/core.cpp
)
target_include_directories(heyball_core PUBLIC src)
target_compile_options(heyball_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(heyball
  src/main.cpp
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
  -lwinmm
  -lopengl32
  -lkernel32
)
target_compile_options(heyball PRIVATE -Wall -Wextra -Wpedantic)
target_link_options(heyball PRIVATE -mwindows -static-libgcc -static-libstdc++)
add_custom_command(TARGET heyball POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${MSYS2_UCRT64}/bin/glfw3.dll"
    "$<TARGET_FILE_DIR:heyball>/glfw3.dll"
)

add_executable(heyball_tests
  tests/test_runner.cpp
)
target_link_libraries(heyball_tests PRIVATE heyball_core)
target_compile_options(heyball_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_options(heyball_tests PRIVATE -static-libgcc -static-libstdc++)

enable_testing()
add_test(NAME heyball_tests COMMAND heyball_tests)
