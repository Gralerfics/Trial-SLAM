add_library(TrialSLAM SHARED
    camera.cpp
    frame.cpp
    vo.cpp
)

target_link_libraries(TrialSLAM ${THIRD_PARTY_LIBS})
