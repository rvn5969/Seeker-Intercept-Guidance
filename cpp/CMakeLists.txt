find_package(Threads REQUIRED)

add_library(intercept_guidance
  src/config.cpp
  src/estimator.cpp
  src/guidance.cpp
  src/guidance_stack.cpp)
target_include_directories(intercept_guidance PUBLIC include)
target_compile_options(intercept_guidance PRIVATE -Wall -Wextra -Wpedantic)

add_executable(validation_benchmark apps/validation_benchmark.cpp)
target_link_libraries(validation_benchmark PRIVATE intercept_guidance Threads::Threads)

if(UNIX)
  add_executable(hil_controller apps/hil_controller.cpp)
  target_link_libraries(hil_controller PRIVATE intercept_guidance)
endif()

function(add_guidance_test name)
  add_executable(${name} tests/${name}.cpp)
  target_link_libraries(${name} PRIVATE intercept_guidance)
  add_test(NAME ${name} COMMAND ${name} ${PROJECT_SOURCE_DIR}/config/experiment.json)
endfunction()

add_guidance_test(test_math)
add_guidance_test(test_estimator)
add_guidance_test(test_guidance)
add_guidance_test(test_closed_loop)
