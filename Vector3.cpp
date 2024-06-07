#include "Vector3.h"

/*----------単項演算子----------*/
Vector3 Vector3::operator-() const {
    return { -x, -y, -z };
}

Vector3 Vector3::operator+() const {
    return *this;
}

/*----------複合代入演算子----------*/
Vector3& Vector3::operator*=(float s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

Vector3& Vector3::operator+=(const Vector3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3& Vector3::operator/=(float s) {
    x /= s;
    y /= s;
    z /= s;
    return *this;
}

/*----------二項演算子----------*/
Vector3 operator+(const Vector3& v1, const Vector3& v2) {
    return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

Vector3 operator-(const Vector3& v1, const Vector3& v2) {
    return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

Vector3 operator*(const Vector3& v1, const Vector3& v2) {
    return { v1.x * v2.x, v1.y * v2.y, v1.z * v2.z };
}

Vector3 operator*(const Vector3& v, float s) {
    return { v.x * s, v.y * s, v.z * s };
}

Vector3 operator*(float s, const Vector3& v) {
    return { v.x * s, v.y * s, v.z * s };
}

Vector3 operator/(const Vector3& v, float s) {
    return { v.x / s, v.y / s, v.z / s };
}
