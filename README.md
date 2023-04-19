# pico-task: Cooperative Task Scheduler

To use these drivers with `pico-sdk`, modify your `CMakeLists.txt`:

```cmake
add_subdirectory(vendor/pico-task)
target_link_libraries(your_target pico_task ...)
add_definitions(-I${CMAKE_CURRENT_LIST_DIR}/vendor/pico-task/include)
list(APPEND PICO_CONFIG_HEADER_FILES task_hooks.h)
```

If you do not have your board header file, create `include/boards/myboard.h` and use it like this:

```cmake
set(PICO_BOARD myboard)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/boards)
```

Your header file may include the original board file, but it must define following:

```c
/* Maximum number of tasks on a single core. */
#define MAX_TASKS 8
```

See `include/task.h` for interface.
