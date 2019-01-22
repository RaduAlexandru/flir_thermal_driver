cmake_minimum_required(VERSION 2.6)
project ( flir_thermal_driver )

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread" )
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -pthread " )

find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  image_transport
  roscpp
  roslib
  cv_bridge
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS cmake_modules roscpp roslib
)

include_directories (
   libuvc/build/include
   libuvc/include
   boson_sdk
   lepton_sdk
   ${LIBUSB_INCLUDE_DIRS}
   include
   ${catkin_INCLUDE_DIRS}
)

set( boson_srcs
    src/boson_main.cpp
    src/boson_node.cpp
    src/boson_interface.cpp
    boson_sdk/Client_API.c
    boson_sdk/Client_Dispatcher.c
    boson_sdk/Client_Packager.c
    boson_sdk/Example.c
    boson_sdk/Serializer_BuiltIn.c
    boson_sdk/Serializer_Struct.c
    boson_sdk/UART_Connector.c
    boson_sdk/flirChannels.c
    boson_sdk/flirCRC.c
    boson_sdk/libusb_binary_protocol.c
)

set( lepton_srcs
    src/lepton_main.cpp
    src/lepton_node.cpp
    src/lepton_interface.cpp
    lepton_sdk/LEPTON_AGC.c
    lepton_sdk/LEPTON_OEM.c
    lepton_sdk/LEPTON_RAD.c
    lepton_sdk/LEPTON_SDK.c
    lepton_sdk/LEPTON_SYS.c
    lepton_sdk/LEPTON_VID.c
    lepton_sdk/crc16fast.c
)

find_library (  UVC_LOCATION NAMES libuvcstatic.a uvcstatic HINTS ${PROJECT_SOURCE_DIR}/libuvc/build ) 
message ( ERROR " ${UVC_LOCATION}" )
message ( ERROR " ${PROJECT_SOURCE_DIR}")
message ( ERROR " ${LIBUSB_LIBRARIES}")

add_executable( ${PROJECT_NAME}_boson ${boson_srcs} )
target_link_libraries( ${PROJECT_NAME}_boson ${UVC_LOCATION} ${LIBUSB_LIBRARIES}  ${catkin_LIBRARIES} )

add_executable( ${PROJECT_NAME}_lepton ${lepton_srcs} )
target_link_libraries( ${PROJECT_NAME}_lepton ${UVC_LOCATION} ${LIBUSB_LIBRARIES} ${catkin_LIBRARIES} )

