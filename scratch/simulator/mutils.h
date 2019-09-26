#ifndef MUTILS_H
#define MUTILS_H

#ifndef M_PI
#define M_PI 3.1415926535
#endif

typedef struct {
    float data[9];
} Mat3f;

void mat3f_zero_inplace(Mat3f* mat);
Mat3f mat3f_zero_copy();

void mat3f_identity_inplace(Mat3f* mat);
Mat3f mat3f_identity_copy();

void mat3f_projection_inplace(Mat3f* mat, float left, float right, float top, float bottom);
Mat3f mat3f_projection_copy(float left, float right, float top, float bottom);

void mat3f_transformation_inplace(Mat3f* mat, float scale, float angle, float transx, float transy);
Mat3f mat3f_transformation_copy(float scale, float angle, float transx, float transy);

Mat3f mat3f_camera_copy(float scalex, float scaley, float angle, float transx, float transy);
void mat3f_camera_inplace(Mat3f* mat, float scalex, float scaley, float angle, float transx, float transy);

#endif
