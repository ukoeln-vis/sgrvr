find_package(Boost COMPONENTS chrono date_time filesystem iostreams regex serialization system thread REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(OPENGL REQUIRED)
find_package(Virvo REQUIRED)
find_package(Visionaray REQUIRED)

include_directories(${VIRVO_INCLUDE_DIR})
include_directories(${VISIONARAY_INCLUDE_DIR})

set(CLIENT_SOURCES
    ${CMD_LINE_DIR}/src/CmdLine.cpp
    ${CMD_LINE_DIR}/src/CmdLineUtil.cpp

    ${COMMON_DIR}/manip/arcball_manipulator.cpp
    ${COMMON_DIR}/manip/camera_manipulator.cpp
    ${COMMON_DIR}/manip/pan_manipulator.cpp
    ${COMMON_DIR}/manip/zoom_manipulator.cpp

    ${COMMON_DIR}/viewer_glut.cpp

    async/connection.h
    async/connection.cpp
    async/connection_manager.h
    async/connection_manager.cpp
    async/message.h
    async/message.cpp

    client.cpp
)

set(SERVER_SOURCES
    ${CMD_LINE_DIR}/src/CmdLine.cpp
    ${CMD_LINE_DIR}/src/CmdLineUtil.cpp

    async/connection.h
    async/connection.cpp
    async/connection_manager.h
    async/connection_manager.cpp
    async/message.h
    async/message.cpp

    server.cpp
)

add_executable(client
    ${CLIENT_SOURCES}
)

add_executable(server
    ${SERVER_SOURCES}
)

target_link_libraries(client
    ${Boost_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${VISIONARAY_LIBRARY}
)

target_link_libraries(server
    ${Boost_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${VIRVO_LIBRARY}
    ${VISIONARAY_LIBRARY}
)
