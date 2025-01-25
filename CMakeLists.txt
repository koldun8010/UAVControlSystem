cmake_minimum_required(VERSION 3.5)

project(ControlSystemGUI VERSION 0.1 LANGUAGES CXX)

add_subdirectory(lib)
add_subdirectory(api)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS
     Core
     Gui
     Widgets
     Network
     GeographicLib
)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS
     Core
     Gui
     Widgets
     Network
)
find_package(MAVSDK REQUIRED)
find_package(SDL2 REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories (${GeographicLib_INCLUDE_DIRS})

file(GLOB SRC
     "*.h"
     "*.cpp"
     "*.ui"
)

set(PROJECT_SOURCES
    ${SRC}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ControlSystemGUI
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        commandbuttons.h commandbuttons.cpp commandbuttons.ui
        placemark.h placemark.cpp
        mainwindow.h mainwindow.cpp
        images/checkedToggle.png  images/fpv_screenshot.png images/uncheckedToggle.png

        indicators.h indicators.cpp indicators.ui
        routes.h routes.cpp routes.ui
        approvaldialog.h approvaldialog.cpp approvaldialog.ui
        images/pointerVer2.png
        images/horizonFg.png

        images/horizonView.png
        notification.h notification.cpp notification.ui

        images/horizonPlane.png
        images/horizonAltScale.png images/horizonSpeedScale.png
        images/horizonHor.png
        images/horizonAngle.png images/horizonCompasScale.png
        images/droneIcon.png
        images/deleteIcon.png
        pointmark.h pointmark.cpp
        routeline.h routeline.cpp



    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET ControlSystemGUI APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(ControlSystemGUI SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(ControlSystemGUI
            ${PROJECT_SOURCES}
        )
    endif()
endif()


target_link_libraries(ControlSystemGUI
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Network
        QGeoView
        MAVSDK::mavsdk
        SDL2::SDL2
        #MAVSDK::mavsdk_mission
        #MAVSDK::mavsdk_action
        myServer
        ${GeographicLib_LIBRARIES}
        GeographicLib::GeographicLib
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.ControlSystemGUI)
endif()
set_target_properties(ControlSystemGUI PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS ControlSystemGUI
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ControlSystemGUI)
endif()
