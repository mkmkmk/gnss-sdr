
find_package(Boost COMPONENTS iostreams serialization QUIET)
if(CMAKE_VERSION VERSION_LESS 3.5)
    if(NOT TARGET Boost::iostreams)
        add_library(Boost::iostreams IMPORTED SHARED)
        set_property(TARGET Boost::iostreams PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
        set_property(TARGET Boost::iostreams PROPERTY
            INTERFACE_LINK_LIBRARIES ${Boost_IOSTREAMS_LIBRARIES})
        set_property(TARGET Boost::iostreams PROPERTY
            IMPORTED_LOCATION ${Boost_IOSTREAMS_LIBRARIES})
    endif()
    if(NOT TARGET Boost::serialization)
        add_library(Boost::serialization IMPORTED SHARED)
        set_property(TARGET Boost::serialization PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
        set_property(TARGET Boost::serialization PROPERTY
            INTERFACE_LINK_LIBRARIES ${Boost_SERIALIZARION_LIBRARIES})
        set_property(TARGET Boost::serialization PROPERTY
            IMPORTED_LOCATION ${Boost_SERIALIZARION_LIBRARIES})
    endif()
endif()


if(Boost_FOUND)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
    add_executable(obs2rinex ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

    target_link_libraries(obs2rinex
        PUBLIC
            Boost::iostreams
            Boost::serialization
            signal_processing_testing_lib
            core_receiver
    )

#    target_include_directories(obs2rinex
#        PUBLIC
#            #${GPSTK_INCLUDE_DIR}/gpstk
#    )

else()
    message(STATUS "Boost Iostreams library not found.")
    message(STATUS " obs2rinex will not be built.")
endif()

set(Boost_FOUND TRUE) # trick for summary report
