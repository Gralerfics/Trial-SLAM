add_library(TrialSLAM SHARED
    camera.cpp
    config.cpp
    dashboard.cpp
    feature.cpp
    frame.cpp
    frontend.cpp
    g2olib.cpp
    landmark.cpp
    map.cpp
    visual_odometry.cpp
)

target_link_libraries(TrialSLAM ${THIRD_PARTY_LIBS})
