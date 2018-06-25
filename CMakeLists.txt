cmake_minimum_required(VERSION 2.6)
project ( flir_boson_driver )

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)


set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread" )
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -pthread " )

# Find the QtWidgets library
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Multimedia CONFIG REQUIRED)
find_package(Qt5Qml CONFIG REQUIRED)

find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)

include_directories ( 
   libuvc/build/include
   libuvc/include
   boson_sdk
    ${LIBUSB_INCLUDE_DIRS}
   include 
)

QT5_WRAP_CPP ( moc_uvcacq include/uvcacquisition.hpp )
QT5_WRAP_CPP ( moc_acc include/abstractccinterface.hpp )
QT5_WRAP_CPP ( moc_bv include/bosonvariation.hpp )
QT5_WRAP_CPP ( moc_uvcvp include/uvcvideoproducer.hpp )

# Populate a CMake variable with the sources
set( srcs
    #mainwindow.ui
    #mainwindow.cpp
    src/main.cpp
    ${moc_uvcacq}
    ${moc_uvcvp}
    ${moc_acc}
    ${moc_bv}
    src/uvcvideoproducer.cpp
    src/uvcacquisition.cpp
    src/abstractccinterface.cpp
    src/uvcbuffer.cpp
    src/abstractccinterface.cpp
    src/bosonvariation.cpp
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

find_library (  UVC_LOCATION NAMES libuvcstatic.a uvcstatic HINTS ${PROJECT_SOURCE_DIR}/libuvc/build ) 
message ( STATUS "${UVC_LOCATION}" )
message ( STATUS "${PROJECT_SOURCE_DIR}")

# Tell CMake to create the helloworld executable
add_executable( ${PROJECT_NAME} ${srcs} )
qt5_use_modules ( ${PROJECT_NAME} Widgets Core Multimedia Qml)
# Use the Widgets module from Qt 5
target_link_libraries( ${PROJECT_NAME} ${LIBUSB_LIBRARIES} ${UVC_LOCATION} Qt5::Core Qt5::Widgets  Qt5::Multimedia Qt5::Qml  )

