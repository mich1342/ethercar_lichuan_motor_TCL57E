project(EthercatLichuanTCL57E)
cmake_minimum_required(VERSION 3.2)

find_package(EtherCAT REQUIRED)

add_executable(tcl57e
    main.cpp
)

target_link_libraries(tcl57e PUBLIC EtherLab::ethercat)