cmake_minimum_required(VERSION 3.10)
project(RULA CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)   # apt install nlohmann-json3-dev

# cppzmq is header-only but ships a CMake config on most distros.
# If find_package fails, we fall back to plain pkg-config for libzmq.

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED) # <-- AGGIUNGI QUESTA RIGA
find_package(cppzmq QUIET)
if (NOT cppzmq_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZMQ REQUIRED libzmq)
endif()

# Aggiungi le directory degli header
include_directories(${ZMQ_INCLUDE_DIRS}
    include
)

# Definisci l'eseguibile con i suoi file sorgente
add_executable(rula_evaluation
    src/rula_evaluation.cpp
    src/rula_score_computation.cpp
)

# Collega le librerie all'eseguibile
target_link_libraries(rula_evaluation PRIVATE
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
    pthread
    rt
)




# # ── Executable ───────────────────────────────────────────────────────────────
# # data_transmitter.hpp is header-only, so just point at its directory and
# # include it from your .cpp source (e.g. main.cpp).
# add_executable(data_transmitter_demo main.cpp)
# 
# target_include_directories(data_transmitter_demo PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}
# )
# 
# target_link_libraries(data_transmitter_demo PRIVATE
#     ${OpenCV_LIBS}
#     nlohmann_json::nlohmann_json
#     pthread
#     rt
# )
# 
# if (cppzmq_FOUND)
#     target_link_libraries(data_transmitter_demo PRIVATE cppzmq)
# else()
#     target_include_directories(data_transmitter_demo PRIVATE ${ZMQ_INCLUDE_DIRS})
#     target_link_libraries(data_transmitter_demo PRIVATE ${ZMQ_LIBRARIES})
# endif()