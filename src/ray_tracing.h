#pragma once
#include "scene.h"

struct HitInfo {
    glm::vec3 normal;
    Material material;
    glm::vec3 barycentric;
    Vertex v0;
    Vertex v1;
    Vertex v2;
};

bool intersectRayWithPlane(const Plane& plane, Ray& ray);

// Returns true if the point p is inside the triangle spanned by v0, v1, v2 with normal n.
bool pointInTriangle(float beta, float gamma);

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2);

bool intersectRayWithTriangle(const Vertex&,const Vertex& v1, const Vertex& v2, Ray& ray, HitInfo& hitInfo);
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo);
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray, float &t);
glm::vec3 barycentricCoordintes(const Vertex&, const Vertex&, const Vertex&, const glm::vec3&);