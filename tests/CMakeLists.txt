set(TARGET unittest)

add_executable(${TARGET}
    test.cc
)

if(UNIX)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra -Werror)
elseif(MSVC)
    target_compile_options(${TARGET} PRIVATE /W4 /WX)
endif()

target_include_directories(${TARGET} PRIVATE
    ${PROJECT_SOURCE_DIR}
)

target_include_directories(${TARGET} SYSTEM PRIVATE
    ${PROJECT_SOURCE_DIR}/third_party/manif/include
)

target_link_libraries(${TARGET} PRIVATE
    Eigen3::Eigen
    GTest::gtest
    GTest::gtest_main
    MANIF::manif
)

add_test(NAME ${TARGET} COMMAND $<TARGET_FILE:${TARGET}>)
