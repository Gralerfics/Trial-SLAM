add_library(TrialSLAM SHARED
    config.cpp
    visual_odometry.cpp
)

target_link_libraries(TrialSLAM ${THIRD_PARTY_LIBS})
