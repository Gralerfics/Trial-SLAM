add_library(TrialSLAM SHARED
    visual_odometry.cpp
)

target_link_libraries(TrialSLAM ${THIRD_PARTY_LIBS})
