add_library(robot INTERFACE)

target_sources(robot INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/robot.cpp
)

target_include_directories(robot INTERFACE ${CMAKE_CURRENT_LIST_DIR})


target_link_libraries(robot INTERFACE
        pico_stdlib
        button
        motor
        encoder
        pid
        plasma
)