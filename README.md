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

Your header file may include the original board file, but it should define following:

```c
/* Maximum number of tasks on a single core. */
#define MAX_TASKS 8

/* Individual task stack size. All tasks have the same stack size. */
#define TASK_STACK_SIZE 1024
```

See `include/task.h` for interface.

Example usage (in your `main.c`):

```c
void power_task(void);
void input_task(void);
void sd_task(void);
void tft_task(void);

void stats_task(void)
{
        while (true) {
                task_sleep_ms(10 * 1000);
                for (unsigned i = 0; i < NUM_CORES; i++)
                        task_stats_report_reset(i);
        }
}

task_t task_avail[NUM_CORES][MAX_TASKS] = {
        {
                /* On the first core: */
                MAKE_TASK(4, "stats", stats_task),
                MAKE_TASK(3, "power", power_task),
                MAKE_TASK(2, "input", input_task),
                NULL,
        },
        {
                /* On the second core: */
                MAKE_TASK(2, "sd", sd_task),
                MAKE_TASK(1, "tft", tft_task),
                NULL,
        },
};

int main()
{
        /* Non-concurrent initialization steps: */
        adc_init();
        tft_init();

        /* Initialize the task scheduler. */
        task_init();

        /* Run tasks on the secondary core. */
        multicore_reset_core1();
        multicore_launch_core1(task_run_loop);

        /* Run tasks on the primary core. */
        task_run_loop();
}
```
