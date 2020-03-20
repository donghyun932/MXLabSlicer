cmake_minimum_required(VERSION 3.8)
project(MXLabSlicer-native)

add_subdirectory(build-utils)
add_subdirectory(admesh)
add_subdirectory(avrdude)
# boost/nowide
add_subdirectory(boost)
add_subdirectory(clipper)
add_subdirectory(miniz)
add_subdirectory(glu-libtess)
add_subdirectory(polypartition)
add_subdirectory(poly2tri)
add_subdirectory(qhull)
add_subdirectory(Shiny)
add_subdirectory(semver)
add_subdirectory(libigl)

# Adding libnest2d project for bin packing...
add_subdirectory(libnest2d)

add_subdirectory(libslic3r)

if (SLIC3R_GUI)
    add_subdirectory(imgui)
    add_subdirectory(hidapi)
    include_directories(hidapi/include)

    if(WIN32)
        message(STATUS "WXWIN environment set to: $ENV{WXWIN}")
    elseif(UNIX)
        set(wxWidgets_USE_UNICODE ON)
        if(SLIC3R_STATIC)
            set(wxWidgets_USE_STATIC ON)
        else()
            set(wxWidgets_USE_STATIC OFF)
        endif()
    endif()

    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set (wxWidgets_CONFIG_OPTIONS "--toolkit=gtk${SLIC3R_GTK}")
        if (SLIC3R_WX_STABLE)
            find_package(wxWidgets 3.0 REQUIRED COMPONENTS base core adv html gl)
        else ()
            find_package(wxWidgets 3.1 QUIET COMPONENTS base core adv html gl)

            if (NOT wxWidgets_FOUND)
                message(FATAL_ERROR "\nCould not find wxWidgets 3.1.\n"
                    "Hint: On Linux you can set -DSLIC3R_WX_STABLE=1 to use wxWidgets 3.0\n")
            endif ()
        endif ()
    else ()
        find_package(wxWidgets 3.1 REQUIRED COMPONENTS html adv gl core base)
    endif ()

    if(UNIX)
        message(STATUS "wx-config path: ${wxWidgets_CONFIG_EXECUTABLE}")
    endif()

    include(${wxWidgets_USE_FILE})

#    list(REMOVE_ITEM wxWidgets_LIBRARIES oleacc)
    message(STATUS "wx libs: ${wxWidgets_LIBRARIES}")

    add_subdirectory(slic3r)
endif()


# Create a slic3r executable
# Process mainfests for various platforms.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/msw/MXLabSlicer.rc.in ${CMAKE_CURRENT_BINARY_DIR}/MXLabSlicer.rc @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/msw/MXLabSlicer.manifest.in ${CMAKE_CURRENT_BINARY_DIR}/MXLabSlicer.manifest @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/osx/Info.plist.in ${CMAKE_CURRENT_BINARY_DIR}/Info.plist @ONLY)
if (WIN32)
    add_library(MXLabSlicer SHARED MXLabSlicer.cpp MXLabSlicer.hpp)
else ()
    add_executable(MXLabSlicer MXLabSlicer.cpp MXLabSlicer.hpp)
endif ()

if (MINGW)
    target_link_options(MXLabSlicer PUBLIC "-Wl,-allow-multiple-definition")
    set_target_properties(MXLabSlicer PROPERTIES PREFIX "")
endif (MINGW)

if (NOT WIN32)
    # Binary name on unix like systems (OSX, Linux)
    set_target_properties(MXLabSlicer PROPERTIES OUTPUT_NAME "mxlab-slicer")
endif ()

target_link_libraries(MXLabSlicer libslic3r cereal)
if (APPLE)
#    add_compile_options(-stdlib=libc++)
#    add_definitions(-DBOOST_THREAD_DONT_USE_CHRONO -DBOOST_NO_CXX11_RVALUE_REFERENCES -DBOOST_THREAD_USES_MOVE)
    # -liconv: boost links to libiconv by default
    target_link_libraries(MXLabSlicer "-liconv -framework IOKit" "-framework CoreFoundation" -lc++)
elseif (MSVC)
    # Manifest is provided through MXLabSlicer.rc, don't generate your own.
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /MANIFEST:NO")
else ()
    # Boost on Raspberry-Pi does not link to pthreads explicitely.
    target_link_libraries(MXLabSlicer ${CMAKE_DL_LIBS} -lstdc++ Threads::Threads)
endif ()

# Add the Slic3r GUI library, libcurl, OpenGL and GLU libraries.
if (SLIC3R_GUI)
#    target_link_libraries(MXLabSlicer ws2_32 uxtheme setupapi libslic3r_gui ${wxWidgets_LIBRARIES})
target_link_libraries(MXLabSlicer libslic3r_gui ${wxWidgets_LIBRARIES})

    # Configure libcurl and its dependencies OpenSSL & zlib
    find_package(CURL REQUIRED)
    if (NOT WIN32)
        # Required by libcurl
        find_package(ZLIB REQUIRED)
    endif()
    target_include_directories(MXLabSlicer PRIVATE ${CURL_INCLUDE_DIRS})
    target_link_libraries(MXLabSlicer ${CURL_LIBRARIES} ${ZLIB_LIBRARIES})
    if (SLIC3R_STATIC)
        if (NOT APPLE)
            # libcurl is always linked dynamically to the system libcurl on OSX.
            # On other systems, libcurl is linked statically if SLIC3R_STATIC is set.
            target_compile_definitions(MXLabSlicer PRIVATE CURL_STATICLIB)
        endif()
        if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
            # As of now, our build system produces a statically linked libcurl,
            # which links the OpenSSL library dynamically.
            find_package(OpenSSL REQUIRED)
            message("OpenSSL include dir: ${OPENSSL_INCLUDE_DIR}")
            message("OpenSSL libraries: ${OPENSSL_LIBRARIES}")
            target_include_directories(MXLabSlicer PRIVATE ${OPENSSL_INCLUDE_DIR})
            target_link_libraries(MXLabSlicer ${OPENSSL_LIBRARIES})
        endif()
    endif()

    if (MSVC)
        # Generate debug symbols even in release mode.
        target_link_options(MXLabSlicer PUBLIC "$<$<CONFIG:RELEASE>:/DEBUG>")
        target_link_libraries(MXLabSlicer user32.lib Setupapi.lib)
    elseif (MINGW)
        target_link_libraries(MXLabSlicer ws2_32 uxtheme setupapi)
    elseif (APPLE)
        target_link_libraries(MXLabSlicer "-framework OpenGL")
    else ()
        target_link_libraries(MXLabSlicer -ldl)
    endif ()
endif ()

# On Windows, a shim application is required to produce a console / non console version of the Slic3r application.
# Also the shim may load the Mesa software OpenGL renderer if the default renderer does not support OpenGL 2.0 and higher.
if (WIN32)
    if (MINGW)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -municode")
    endif()

    add_executable(MXLabSlicer_app_gui WIN32 MXLabSlicer_app_msvc.cpp ${CMAKE_CURRENT_BINARY_DIR}/MXLabSlicer.rc)
    # Generate debug symbols even in release mode.
    if(MSVC)
        target_link_options(MXLabSlicer_app_gui PUBLIC "$<$<CONFIG:RELEASE>:/DEBUG>")
    endif()
    target_compile_definitions(MXLabSlicer_app_gui PRIVATE -DSLIC3R_WRAPPER_NOCONSOLE)
    add_dependencies(MXLabSlicer_app_gui MXLabSlicer)
    set_target_properties(MXLabSlicer_app_gui PROPERTIES OUTPUT_NAME "mxlab-slicer")
    target_link_libraries(MXLabSlicer_app_gui PRIVATE boost_headeronly)

    add_executable(MXLabSlicer_app_console MXLabSlicer_app_msvc.cpp ${CMAKE_CURRENT_BINARY_DIR}/MXLabSlicer.rc)
    # Generate debug symbols even in release mode.
    if (MSVC)
    target_link_options(MXLabSlicer_app_console PUBLIC "$<$<CONFIG:RELEASE>:/DEBUG>")
    endif ()
    target_compile_definitions(MXLabSlicer_app_console PRIVATE -DSLIC3R_WRAPPER_CONSOLE)
    add_dependencies(MXLabSlicer_app_console MXLabSlicer)
    set_target_properties(MXLabSlicer_app_console PROPERTIES OUTPUT_NAME "mxlab-slicer-console")
    target_link_libraries(MXLabSlicer_app_console PRIVATE boost_headeronly)
endif ()

# Link the resources dir to where Slic3r GUI expects it
if (WIN32)
    if (CMAKE_CONFIGURATION_TYPES)
        foreach (CONF ${CMAKE_CONFIGURATION_TYPES})
            file(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/${CONF}" WIN_CONF_OUTPUT_DIR)
            file(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/${CONF}/resources" WIN_RESOURCES_SYMLINK)
            add_custom_command(TARGET MXLabSlicer POST_BUILD
                COMMAND if exist "${WIN_CONF_OUTPUT_DIR}" "("
                        if not exist "${WIN_RESOURCES_SYMLINK}" "("
                            mklink /J "${WIN_RESOURCES_SYMLINK}" "${SLIC3R_RESOURCES_DIR_WIN}"
                        ")"
                    ")"
                COMMENT "Symlinking the resources directory into the build tree"
                VERBATIM
            )
        endforeach ()
    else ()
        file(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/resources" WIN_RESOURCES_SYMLINK)
        add_custom_command(TARGET MXLabSlicer POST_BUILD
            COMMAND if not exist "${WIN_RESOURCES_SYMLINK}" "(" mklink /J "${WIN_RESOURCES_SYMLINK}" "${SLIC3R_RESOURCES_DIR_WIN}" ")"
            COMMENT "Symlinking the resources directory into the build tree"
            VERBATIM
        )
    endif ()
    
    # This has to be a separate target due to the windows command line lenght limits
    add_custom_target(MXLabSlicerDllsCopy ALL DEPENDS MXLabSlicer)
    mxlabslicer_copy_dlls(MXLabSlicerDllsCopy)
    
elseif (XCODE)
    # Because of Debug/Release/etc. configurations (similar to MSVC) the slic3r binary is located in an extra level
    add_custom_command(TARGET MXLabSlicer POST_BUILD
        COMMAND ln -sf "${SLIC3R_RESOURCES_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/resources"
        COMMENT "Symlinking the resources directory into the build tree"
        VERBATIM
    )
else ()
    add_custom_command(TARGET MXLabSlicer POST_BUILD
        COMMAND ln -sf "${SLIC3R_RESOURCES_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/../resources"
        COMMENT "Symlinking the resources directory into the build tree"
        VERBATIM
    )
endif()

# Slic3r binary install target
if (WIN32)
    install(TARGETS MXLabSlicer RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}")
    if (MSVC)
        install(TARGETS MXLabSlicer_app_gui RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}")
        install(TARGETS MXLabSlicer_app_console RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}")
    endif ()
else ()
    install(TARGETS MXLabSlicer RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif ()
