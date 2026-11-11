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

add_executable(test_ctrl_balance_controller
    test_ctrl_balance_controller.c
    ../common/src/control/ctrl_balance_controller.c
)

target_include_directories(test_ctrl_balance_controller PRIVATE
    ../common/include
)

target_compile_features(test_ctrl_balance_controller PRIVATE c_std_11)
target_compile_options(test_ctrl_balance_controller PRIVATE
    -Wall
    -Wextra
    -Werror
)
target_link_libraries(test_ctrl_balance_controller PRIVATE m)

add_test(NAME ctrl_balance_controller COMMAND test_ctrl_balance_controller)

add_executable(test_ctrl_collinear_mecanum_allocator
    test_ctrl_collinear_mecanum_allocator.c
    ../common/src/control/ctrl_collinear_mecanum_allocator.c
)

target_include_directories(test_ctrl_collinear_mecanum_allocator PRIVATE
    ../common/include
)

target_compile_features(test_ctrl_collinear_mecanum_allocator PRIVATE c_std_11)
target_compile_options(test_ctrl_collinear_mecanum_allocator PRIVATE
    -Wall
    -Wextra
    -Werror
)
target_link_libraries(test_ctrl_collinear_mecanum_allocator PRIVATE m)

add_test(NAME ctrl_collinear_mecanum_allocator
         COMMAND test_ctrl_collinear_mecanum_allocator)

add_executable(test_safety_balance_control
    test_safety_balance_control.c
    ../common/src/safety/safety_balance_control.c
)

target_include_directories(test_safety_balance_control PRIVATE
    ../common/include
)

target_compile_features(test_safety_balance_control PRIVATE c_std_11)
target_compile_options(test_safety_balance_control PRIVATE
    -Wall
    -Wextra
    -Werror
)
target_link_libraries(test_safety_balance_control PRIVATE m)

add_test(NAME safety_balance_control COMMAND test_safety_balance_control)

add_executable(test_app_operator_controls
    test_app_operator_controls.c
    ../common/src/app/app_operator_controls.c
)

target_include_directories(test_app_operator_controls PRIVATE
    ../common/include
)

target_compile_features(test_app_operator_controls PRIVATE c_std_11)
target_compile_options(test_app_operator_controls PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME app_operator_controls COMMAND test_app_operator_controls)
