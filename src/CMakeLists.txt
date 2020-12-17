CMAKE_MINIMUM_REQUIRED(VERSION 3.4.0)

PROJECT(RobotCalibration)

CMAKE_POLICY(SET CMP0020 NEW)
CMAKE_POLICY(SET CMP0028 NEW)
CMAKE_POLICY(SET CMP0053 NEW)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

#---设置自动生成moc文件,不可缺少---------
SET(CMAKE_AUTOMOC ON)

#------------------------------------------
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5SerialPort)
FIND_PACKAGE(Qt5SerialBus)
FIND_PACKAGE(Qt5Network)



#--------查找当前文件中的所有源码----------
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

#--------生成ui文件的头文件----------------
QT5_WRAP_UI(FORMS_HEADERS ${UI_FILES})
#QT5_WRAP_CPP(HEADERS_MOC ${HEAD_FILES})   #似乎可以不用写这句话，因为会使moc文件生成在CMakeLists目录下
QT5_ADD_RESOURCES(PROJECT_RESOURCE_RCC ${PROJECT_RESOURCE})


#--------创建工程文件-----------------------
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
					  
					  
#---------添加筛选器----------------				  
source_group("UrAPI\\Source Files" FILES ${UR_SRC})
source_group("UrAPI\\Header Files" FILES ${UR_HDR})
source_group("Forms" FILES ${UI_FILES})
source_group("Forms\\WRAP_UI" FILES ${FORMS_HEADERS})
#source_group("Forms\\WRAP_CPP" FILES ${HEADERS_MOC})



#---------拷贝DLL文件----------------	
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to project
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/libmodbus/windows64/lib/libmodbus.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to project
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "D:/source/NdiApi/bin/win64/library.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path
