cmake_minimum_required(VERSION 3.5)
project(RULA CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trova le dipendenze necessarie
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED) # <-- AGGIUNGI QUESTA RIGA

# Trova ZMQ (ZeroMQ) usando pkg-config, un modo comune per trovarlo
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

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
    Threads::Threads # <-- E AGGIUNGI QUESTA
)