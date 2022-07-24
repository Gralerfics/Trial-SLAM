add_library(TrialSLAM SHARED
    backend.cpp
    camera.cpp
    dashboard.cpp
    frame.cpp
    frontend.cpp
    landmark.cpp
    map.cpp
    vo.cpp
)

target_link_libraries(TrialSLAM ${THIRD_PARTY_LIBS})
