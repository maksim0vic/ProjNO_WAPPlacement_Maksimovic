# --- Application name
set(NOPROJECT_NAME ${SOLUTION_NAME})

# --- Gather sources explicitly
set(NOPROJECT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/FloorPlan.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Gradient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Objective.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
)

set(NOPROJECT_INCS
    ${CMAKE_CURRENT_LIST_DIR}/src/FloorPlan.h
    ${CMAKE_CURRENT_LIST_DIR}/src/Gradient.h
    ${CMAKE_CURRENT_LIST_DIR}/src/Objective.h
)

# Sparse/Dense Matrices
file(GLOB NOPROJECT_INC_DENSE ${NATID_SDK_INC}/dense/*.h)
file(GLOB NOPROJECT_INC_DENSE_PRIV ${NATID_SDK_INC}/dense/priv/*.h)
file(GLOB NOPROJECT_INC_SPARSE ${NATID_SDK_INC}/sparse/*.h)
file(GLOB NOPROJECT_INC_SPARSE_PRIV ${NATID_SDK_INC}/sparse/priv/*.h)

# Math and Matrix utilities
file(GLOB NOPROJECT_INC_MATH ${NATID_SDK_INC}/math/*.h)
file(GLOB NOPROJECT_INC_MATRIX ${NATID_SDK_INC}/matrix/*.h)
file(GLOB NOPROJECT_INC_MTX ${NATID_SDK_INC}/mtx/*.h)
file(GLOB NOPROJECT_INC_MU ${NATID_SDK_INC}/mu/*.h)
file(GLOB NOPROJECT_INC_MEM ${NATID_SDK_INC}/mem/*.h)

# GUI and other utilities
file(GLOB NOPROJECT_INC_TD ${NATID_SDK_INC}/td/*.h)
file(GLOB NOPROJECT_INC_GUI ${NATID_SDK_INC}/gui/*.h)

file(GLOB NOPROJECT_INC_THREAD ${NATID_SDK_INC}/thread/*.h)
file(GLOB NOPROJECT_INC_CNT ${NATID_SDK_INC}/cnt/*.h)
file(GLOB NOPROJECT_INC_FO ${NATID_SDK_INC}/fo/*.h)
file(GLOB NOPROJECT_INC_XML ${NATID_SDK_INC}/xml/*.h)

# --- Application icon / plist configuration
set(NOPROJECT_PLIST ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/AppIcon.plist)
if(WIN32)
    set(NOPROJECT_WINAPP_ICON ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/winAppIcon.rc)
else()
    set(NOPROJECT_WINAPP_ICON ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/winAppIcon.cpp)
endif()

add_executable(${NOPROJECT_NAME}
    ${NOPROJECT_INCS}
    ${NOPROJECT_SOURCES}
    ${NOPROJECT_INC_DENSE}
    ${NOPROJECT_INC_DENSE_PRIV}
    ${NOPROJECT_INC_SPARSE}
    ${NOPROJECT_INC_SPARSE_PRIV}
    ${NOPROJECT_INC_MATH}
    ${NOPROJECT_INC_MATRIX}
    ${NOPROJECT_INC_MTX}
    ${NOPROJECT_INC_MU}
    ${NOPROJECT_INC_MEM}
    ${NOPROJECT_INC_TD}
    ${NOPROJECT_INC_GUI}
    ${NOPROJECT_INC_FO}
    ${NOPROJECT_INC_CNT}
    ${NOPROJECT_INC_XML}
    ${NOPROJECT_INC_THREAD}
    ${NOPROJECT_WINAPP_ICON}
)

source_group("src" FILES ${NOPROJECT_SOURCES})
source_group("inc" FILES ${NOPROJECT_INCS})

target_compile_definitions(${NOPROJECT_NAME} PUBLIC MU_USETIMER)

target_link_libraries(${NOPROJECT_NAME}
    debug ${MU_LIB_DEBUG}
    debug ${MATRIX_LIB_DEBUG}
    debug ${NATGUI_LIB_DEBUG}
    optimized ${MU_LIB_RELEASE}
    optimized ${MATRIX_LIB_RELEASE}
    optimized ${NATGUI_LIB_RELEASE}
)

# --- Apply SDK macros
setIDEPropertiesForExecutable(${NOPROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR})
setAppIcon(${NOPROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR})
setIDEPropertiesForGUIExecutable(${NOPROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR})
setTargetPropertiesForGUIApp(${NOPROJECT_NAME} ${NOPROJECT_PLIST})
set_target_properties(${NOPROJECT_NAME} PROPERTIES
    VS_DEBUGGER_COMMAND_ARGUMENTS "-devResPath=${CMAKE_CURRENT_LIST_DIR}")
setPlatformDLLPath(${NOPROJECT_NAME})

# --- Linux icon installation
if(UNIX AND NOT APPLE)
    set(ICON_SIZES 16 32 48 128 256)
    foreach(SIZE ${ICON_SIZES})
        if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/lnxApp${SIZE}.png)
            install(FILES ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/lnxApp${SIZE}.png
                    DESTINATION share/icons/hicolor/${SIZE}x${SIZE}/apps
                    RENAME wapProject.png)
        endif()
    endforeach()

    if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/no_project.desktop)
        install(FILES ${CMAKE_CURRENT_LIST_DIR}/no_project.desktop
                DESTINATION share/applications)
    endif()
endif()
