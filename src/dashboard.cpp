#include "trialSlam/dashboard.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Dashboard::initialize() {
    if (_camera == nullptr) return false;
    _dashboard_running = true;
    _dashboard_thread = std::thread(std::bind(&Dashboard::loop, this));
    return true;
}

void Dashboard::stop() {
    _dashboard_running = false;
    _dashboard_thread.join();
}

void Dashboard::loop() {
    pangolin::CreateWindowAndBind("Dashboard", DASHBOARD_WIDTH, DASHBOARD_HEIGHT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // TOFIX: uncertain parameters
    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(DASHBOARD_WIDTH, DASHBOARD_HEIGHT, 400, 400, DASHBOARD_WIDTH / 2, DASHBOARD_HEIGHT / 2, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View& vis_display = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1.0f * DASHBOARD_WIDTH / DASHBOARD_HEIGHT)
        .SetHandler(new pangolin::Handler3D(vis_camera));

    const float COLOR_CURRENT_FRAME[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && _dashboard_running) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> ulock(_mutex);
        if (_cur_frame) {
            drawFrame(_cur_frame, COLOR_CURRENT_FRAME);
            moveToCurrentFrame(vis_camera);

            // TODO: Camera Window
        }

        if (_map) drawLandmarks();

        pangolin::FinishFrame();
        usleep(1000);
    }
}

void Dashboard::update() {
    std::unique_lock<std::mutex> ulock(_mutex);
    if (_map) {
        _active_keyframes = _map -> getActiveKeyFrames();
        _active_landmarks = _map -> getActiveLandmarks();
    }
}

void Dashboard::drawFrame(Frame::Ptr frame, const float* color) {
    SE3 T_wc = frame -> getTcw().inverse();

    const float sz = 1.0;
    const float fx = _camera -> _fx, fy = _camera -> _fy;
    const float cx = _camera -> _cx, cy = _camera -> _cy;
    const float width = _camera -> _width, height = _camera -> _height;

    glPushMatrix();

    Sophus::Matrix4f m = T_wc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    glColor3f(color[0], color[1], color[2]);

    glLineWidth(2);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glEnd();

    glPopMatrix();
}

void Dashboard::drawLandmarks() {
    const float COLOR_FRAME[3] = {0, 0, 1}, COLOR_LANDMARK[3] = {0, 0, 1};

    for (auto& keyframe : _active_keyframes)
        drawFrame(keyframe.second, COLOR_FRAME);

    glPointSize(2);
    glBegin(GL_POINTS);
        for (auto& landmark : _active_landmarks) {
            auto position = landmark.second -> getPosition();
            glColor3f(COLOR_LANDMARK[0], COLOR_LANDMARK[1], COLOR_LANDMARK[2]);
            glVertex3d(position[0], position[1], position[2]);
        }
    glEnd();
}

void Dashboard::moveToCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 T_wc = _cur_frame -> getTcw().inverse();
    vis_camera.Follow(pangolin::OpenGlMatrix(T_wc.matrix()), true);
}

TRIAL_SLAM_NAMESPACE_END
