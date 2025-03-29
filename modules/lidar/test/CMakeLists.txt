cmake_minimum_required(VERSION 3.14)
project(lidar_tests)

# C++20 を有効化
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# モックディレクトリをインクルードパスの先頭に追加
# これにより、実際のPico-SDKの代わりにモックが使用される
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

# テストソースファイル
set(TEST_SOURCES
    main.cpp
    task_test.cpp
    mocks/pico_mock_impl.cpp
)

# テスト実行ファイルの作成
add_executable(task_tests ${TEST_SOURCES})

# テスト実行コマンド
add_custom_target(run_tests
    COMMAND ./task_tests
    DEPENDS task_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
