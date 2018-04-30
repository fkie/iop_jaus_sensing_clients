cmake_minimum_required(VERSION 2.8.3)
project(iop_client_still_image_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                cv_bridge
                image_transport
                iop_client_visual_sensor_fkie
                iop_component_fkie
                iop_ocu_slavelib_fkie
                roscpp
                sensor_msgs
                )

iop_init(COMPONENT_ID 0)
# export this service for index in eclipse
iop_export_service(urn_jaus_jss_environmentSensing_StillImageClient)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        cv_bridge
        image_transport
        iop_client_visual_sensor_fkie
        iop_component_fkie
        iop_ocu_slavelib_fkie
        sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
iop_code_generator(
# IDLS: the service definition files located in iop_builder_fkie/jsidl
# OWN_IDLS: the service definition files located in PROJECT_DIR/jsidl
# OVERRIDES: files which override the generated files while build. The generated files are copied to the same location with .gen extension
# EXTERN_SERVICES: the generated folder name of a service which is used from other library. This folder must be deleted while build.
  IDLS
    urn.jaus.jss.core-v1.0/AccessControlClient.xml
    urn.jaus.jss.core-v1.0/EventsClient.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.environmentSensing/VisualSensorClient.xml
    urn.jaus.jss.environmentSensing/StillImageClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_environmentSensing_StillImageClient/StillImageClient_ReceiveFSM.h
    src/urn_jaus_jss_environmentSensing_StillImageClient/StillImageClient_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_environmentSensing_VisualSensorClient
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${OpenCV_INCLUDES_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_library(${PROJECT_NAME}
            src/StillImageClientPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBS}
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

