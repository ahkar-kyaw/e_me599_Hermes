cmake_minimum_required(VERSION 3.16)

project(app_common_tests C)

enable_testing()

add_executable(test_est_collinear_mecanum
    test_est_collinear_mecanum.c
    ../common/src/estimation/est_collinear_mecanum.c
)

target_include_directories(test_est_collinear_mecanum PRIVATE
    ../common/include
)

target_compile_features(test_est_collinear_mecanum PRIVATE c_std_11)
target_compile_options(test_est_collinear_mecanum PRIVATE
    -Wall
    -Wextra
    -Werror
)
target_link_libraries(test_est_collinear_mecanum PRIVATE m)

add_test(NAME est_collinear_mecanum COMMAND test_est_collinear_mecanum)

add_executable(test_ctrl_torque_input
    test_ctrl_torque_input.c
    ../common/src/control/ctrl_torque_input.c
)

target_include_directories(test_ctrl_torque_input PRIVATE
    ../common/include
)

target_compile_features(test_ctrl_torque_input PRIVATE c_std_11)
target_compile_options(test_ctrl_torque_input PRIVATE
    -Wall
    -Wextra
    -Werror
)
target_link_libraries(test_ctrl_torque_input PRIVATE m)

add_test(NAME ctrl_torque_input COMMAND test_ctrl_torque_input)

add_executable(test_safety_torque_control
    test_safety_torque_control.c
    ../common/src/safety/safety_torque_control.c
)

target_include_directories(test_safety_torque_control PRIVATE
    ../common/include
)

target_compile_features(test_safety_torque_control PRIVATE c_std_11)
target_compile_options(test_safety_torque_control PRIVATE
    -Wall
    -Wextra
    -Werror
)
target_link_libraries(test_safety_torque_control PRIVATE m)

add_test(NAME safety_torque_control COMMAND test_safety_torque_control)
