CMAKE_MINIMUM_REQUIRED(VERSION 3.4.0)

PROJECT(RobotCalibration)

CMAKE_POLICY(SET CMP0020 NEW)
CMAKE_POLICY(SET CMP0028 NEW)
CMAKE_POLICY(SET CMP0053 NEW)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

#---�����Զ�����moc�ļ�,����ȱ��---------
SET(CMAKE_AUTOMOC ON)

#------------------------------------------
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5SerialPort)
FIND_PACKAGE(Qt5SerialBus)
FIND_PACKAGE(Qt5Network)



#--------���ҵ�ǰ�ļ��е�����Դ��----------
INCLUDE_DIRECTORIES("UrAPI")

INCLUDE_DIRECTORIES("libmodbus/windows64/includes")
LINK_DIRECTORIES("libmodbus/windows64/lib")
INCLUDE_DIRECTORIES("D:/source/eigen3")
INCLUDE_DIRECTORIES("D:/source/NdiApi/include")
LINK_DIRECTORIES("D:/source/NdiApi/bin/win64")

FILE(GLOB SRC_FILES "./*.cpp")
FILE(GLOB HEAD_FILES "./*.h")
FILE(GLOB UI_FILES "./*.ui")
FILE(GLOB UR_SRC "./UrAPI/*.cpp")
FILE(GLOB UR_HDR "./UrAPI/*.h")


SET(PROJECT_RESOURCE RobotCalibration.qrc)

#--------����ui�ļ���ͷ�ļ�----------------
QT5_WRAP_UI(FORMS_HEADERS ${UI_FILES})
#QT5_WRAP_CPP(HEADERS_MOC ${HEAD_FILES})   #�ƺ����Բ���д��仰����Ϊ��ʹmoc�ļ�������CMakeListsĿ¼��
QT5_ADD_RESOURCES(PROJECT_RESOURCE_RCC ${PROJECT_RESOURCE})


#--------���������ļ�-----------------------
ADD_EXECUTABLE(${PROJECT_NAME}
               ${HEAD_FILES}
               ${SRC_FILES} 			   
               ${UI_FILES}
			   ${UR_SRC}
			   ${UR_HDR}
			   ${FORMS_HEADERS}
               #${HEADERS_MOC}
			   ${PROJECT_RESOURCE_RCC}
			   ${PROJECT_RESOURCE}
               )

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      Qt5::Widgets
                      Qt5::Core
                      Qt5::Gui
                      Qt5::Network
					  Qt5::SerialPort
					  Qt5::SerialBus)
					  
					  
#---------����ɸѡ��----------------				  
source_group("UrAPI\\Source Files" FILES ${UR_SRC})
source_group("UrAPI\\Header Files" FILES ${UR_HDR})
source_group("Forms" FILES ${UI_FILES})
source_group("Forms\\WRAP_UI" FILES ${FORMS_HEADERS})
#source_group("Forms\\WRAP_CPP" FILES ${HEADERS_MOC})



#---------����DLL�ļ�----------------	
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to project
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/libmodbus/windows64/lib/libmodbus.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to project
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "D:/source/NdiApi/bin/win64/library.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path