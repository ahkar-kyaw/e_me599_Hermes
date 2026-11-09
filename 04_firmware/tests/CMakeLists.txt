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
