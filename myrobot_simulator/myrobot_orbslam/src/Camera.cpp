//
// Created by jl on 2019/12/13.
//

#include "Camera.h"

myslam::Camera::Camera() {

}

Vector3d myslam::Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w) {
    return T_c_w*p_w;
}

Vector3d myslam::Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w) {
    return NULL;
}

Vector2d myslam::Camera::camera2pixel(const Vector3d &p_c) {
    return NULL;
}

Vector3d myslam::Camera::pixel2camera(const Vector2d &p_p, double depth) {
    return NULL;
}

Vector3d myslam::Camera::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth) {
    return NULL;
}

vector2d myslam::Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w) {
    return NULL;
}


