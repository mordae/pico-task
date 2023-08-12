if (NOT TARGET pico_task)
    pico_add_library(pico_task)

    target_include_directories(
      pico_task

      INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include
    )

    target_sources(
      pico_task

      INTERFACE ${CMAKE_CURRENT_LIST_DIR}/task.c
                ${CMAKE_CURRENT_LIST_DIR}/task.S
    )

    target_link_libraries(
      pico_task

      INTERFACE pico_stdlib
                pico_util
    )

endif()
