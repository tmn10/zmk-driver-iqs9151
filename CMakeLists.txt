zephyr_include_directories(include)

add_subdirectory(drivers)

if(EXISTS "${APPLICATION_SOURCE_DIR}/include/drivers/behavior.h")
  add_subdirectory(behaviors)
endif()

if(EXISTS "${APPLICATION_SOURCE_DIR}/include/drivers/input_processor.h")
  add_subdirectory(input_processors)
endif()
