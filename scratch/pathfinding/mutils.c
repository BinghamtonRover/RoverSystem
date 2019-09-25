#include <string.h>
#include <math.h>

#include "mutils.h"

void mat3f_zero_inplace(Mat3f* mat) {
    memset(mat->data, 0, sizeof(float) * 9);
}

Mat3f mat3f_zero_copy() {
    Mat3f mat;
    mat3f_zero_inplace(&mat);
    return mat;
}

void mat3f_identity_inplace(Mat3f* mat) {
    mat3f_zero_inplace(mat);

    mat->data[0] = 1.0f;
    mat->data[4] = 1.0f;
    mat->data[8] = 1.0f;
}

Mat3f mat3f_identity_copy() {
    Mat3f mat;
    mat3f_identity_inplace(&mat);
    return mat;
}

void mat3f_projection_inplace(Mat3f* mat, float left, float right, float top, float bottom) {
     mat3f_identity_inplace(mat);

     mat->data[0] = 2.0f / (right - left);
     mat->data[2] = - (right + left) / (right - left);
     mat->data[4] = 2.0f / (top - bottom);
     mat->data[5] = - (top + bottom) / (top - bottom);
}

Mat3f mat3f_projection_copy(float left, float right, float top, float bottom) {
    Mat3f mat;
    mat3f_projection_inplace(&mat, left, right, top, bottom);
    return mat;
}

static float dtr(float d) {
    return d * M_PI / 180.0f;
}

void mat3f_transformation_inplace(Mat3f* mat, float scale, float angle, float transx, float transy) {
    float theta = dtr(angle);

    mat->data[0] = scale * cosf(theta);
    mat->data[1] = scale * -sinf(theta);
    mat->data[2] = transx;
    mat->data[3] = scale * sinf(theta);
    mat->data[4] = scale * cosf(theta);
    mat->data[5] = transy;
    mat->data[6] = 0;
    mat->data[7] = 0;
    mat->data[8] = 1;
}

Mat3f mat3f_transformation_copy(float scale, float angle, float transx, float transy) {
    Mat3f mat;
    mat3f_transformation_inplace(&mat, scale, angle, transx, transy);
    return mat;
}

void mat3f_camera_inplace(Mat3f* mat, float scalex, float scaley, float angle, float transx, float transy) {
    float theta = dtr(angle);

    mat->data[0] = scalex* cosf(theta);
    mat->data[1] = scalex* -sinf(theta);
    mat->data[2] = transx;
    mat->data[3] = scaley * sinf(theta);
    mat->data[4] = scaley * cosf(theta);
    mat->data[5] = transy;
    mat->data[6] = 0;
    mat->data[7] = 0;
    mat->data[8] = 1;
}

Mat3f mat3f_camera_copy(float scalex, float scaley, float angle, float transx, float transy) {
    Mat3f mat;
    mat3f_camera_inplace(&mat, scalex, scaley, angle, transx, transy);
    return mat;
}
