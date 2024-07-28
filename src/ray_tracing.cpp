#include "ray_tracing.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

float determinant(glm::vec3& col1, glm::vec3 col2, glm::vec3 col3) {
    // Sarrus Rule
    return +(col1.x * col2.y * col3.z)
        + (col1.y * col2.z * col3.x)
        + (col1.z * col2.x * col3.y)
        - (col1.z * col2.y * col3.x)
        - (col1.x * col2.z * col3.y)
        - (col1.y * col2.x * col3.z);
}



glm::vec3 barycentricCoordinates(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p) {
    float a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3;
    a1 = v0.x - v1.x;
    a2 = v0.y - v1.y;
    a3 = v0.z - v1.z;

    b1 = v0.x - v2.x;
    b2 = v0.y - v2.y;
    b3 = v0.z - v2.z;

    c1 = 1.0f;
    c2 = 1.0f;
    c3 = 1.0f;

    d1 = v0.x - p.x;
    d2 = v0.y - p.y;
    d3 = v0.z - p.z;

    glm::vec3 acol = { a1, a2, a3 };
    glm::vec3 bcol = { b1, b2, b3 };
    glm::vec3 ccol = { c1, c2, c3 };
    glm::vec3 dcol = { d1, d2, d3 };

    float m_det = determinant(acol, bcol, ccol);
    float beta = determinant(dcol, bcol, ccol) / m_det;
    float gamma = determinant(acol, dcol, ccol) / m_det;
    float alpha = 1 - beta - gamma;
    return glm::vec3{ alpha, beta, gamma };
}

bool pointInTriangle(float beta, float gamma)
{
    return (gamma > -1e-6 && beta > -1e-6 && (beta + gamma) < (1.0f + 1e-6));
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    auto debug = glm::dot(ray.direction, plane.normal);
    if (-1e-6 < debug && debug < 1e-6) {
        auto originDistance = glm::dot(ray.origin, plane.normal);
        if (glm::abs(plane.D - glm::dot(ray.origin, plane.normal)) < 1e-6) {
            return false;
        }
        return false;
    }
    else {
        auto t = (plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal);
        ray.t = t;
    }
    return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::normalize(glm::cross((v0 - v2), (v1 - v2)));
    plane.D = glm::dot(v0, plane.normal);
    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const Vertex& v0,const Vertex& v1,const Vertex& v2, Ray& ray, HitInfo& hitInfo)
{
    auto previousRayT = ray.t;
    auto vc0 = v0.position;
    auto vc1 = v1.position;
    auto vc2 = v2.position;
    auto plane = trianglePlane(vc0, vc1, vc2);
    if (intersectRayWithPlane(plane, ray) && (ray.t > 0 && ray.t < previousRayT)){
        glm::vec3 barycentric = barycentricCoordinates(vc0, vc1, vc2, ray.origin + ray.direction * ray.t);
        if (pointInTriangle(barycentric.y, barycentric.z)) {
            hitInfo.barycentric = barycentric;
            hitInfo.normal = glm::normalize(v0.normal * barycentric.x + v1.normal * barycentric.y + v2.normal * barycentric.z);
            //hitInfo.normal = plane.normal;
            hitInfo.v0 = v0;
            hitInfo.v1 = v1;
            hitInfo.v2 = v2;
            return true;
        }
    }
    ray.t = previousRayT;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    auto newOrigin = ray.origin - sphere.center;
    auto a = glm::pow(ray.direction.x, 2) + glm::pow(ray.direction.y, 2) + glm::pow(ray.direction.z, 2);
    auto b = 2 * (ray.direction.x * newOrigin.x + ray.direction.y * newOrigin.y + ray.direction.z * newOrigin.z);
    auto c = glm::pow(newOrigin.x, 2) + glm::pow(newOrigin.y, 2) + glm::pow(newOrigin.z, 2) - glm::pow(sphere.radius, 2);

    auto d = glm::pow(b, 2) - 4 * a * c;
    if (d < 0.0f) return false;
    if (-1e-6 < d && d < 1e-6) {
        auto t = (-b / 2 * a);
        if (t < ray.t) {
            ray.t = t;
            return true;
        }
    }
    else {
        auto t0 = (-b - glm::sqrt(d)) / (2 * a);
        auto t1 = (-b + glm::sqrt(d)) / (2 * a);

        if (t0 < 0.0f) {
            if (t1 < 0.0f) {
                // sphere is behind?
                return false;
            }
            else {
                // inside the shpere
                if (t1 < ray.t) {
                    ray.t = t1;
                    return true;
                }
            }
        }
        else {
            if (t0 < ray.t) {
                ray.t = t0;
                return true;
            }
        }
    }
    return false;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray, float& t)
{
    auto txmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    auto tymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    auto tzmin = (box.lower.z - ray.origin.z) / ray.direction.z;

    auto txmax = (box.upper.x - ray.origin.x) / ray.direction.x;
    auto tymax = (box.upper.y - ray.origin.y) / ray.direction.y;
    auto tzmax = (box.upper.z - ray.origin.z) / ray.direction.z;

    auto txin = glm::min(txmin, txmax);
    auto tyin = glm::min(tymin, tymax);
    auto tzin = glm::min(tzmin, tzmax);

    auto tin = glm::max(txin, glm::max(tyin, tzin));
    if (tin > ray.t) return false;

    auto txout = glm::max(txmin, txmax);
    auto tyout = glm::max(tymin, tymax);
    auto tzout = glm::max(tzmin, tzmax);

    auto tout = glm::min(txout, glm::min(tyout, tzout));
    if (tout < tin || tout < 1e-6) return false;

    if (tin < 0.0f)
        t = tout;
    else
        t = tin;

    return true;
}
