#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <span>
#include <queue>

struct Node
{
    bool leaf;
    std::vector<int> childIndices; // if leaf, contains indices to triangles, else contains indices to nodes
    int level;
    AxisAlignedBox aabb;
};


struct NodeProps {
    int nodeInt;
    float t;
    bool operator<(const NodeProps& other) {
        return t < other.t;
    }
};

struct TriangleWithMesh
{
    glm::uvec3 triangle;
    int meshIndex;
};

class BoundingVolumeHierarchy
{
public:
    BoundingVolumeHierarchy(Scene *pScene);
    // Implement these two functions for the Visual Debug.
    // The first function should return how many levels there are in the tree that you have constructed.
    // The second function should draw the bounding boxes of the nodes at the selected level.
    int numLevels() const;
    void debugDraw(int level);
    void debugTraversal(bool on);
    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray &ray, HitInfo &hitInfo) const;

    std::vector<Vertex> vertices;
    std::vector<TriangleWithMesh> triangles;
private:
    void copySceneData();
    void computeBVH();
    void computeBVH(int);
    bool intersectNodes(Ray& ray, HitInfo& hitInfo, int nodeInxed) const;
    bool intersectTriangles(Ray& ray, HitInfo& hitInfo, int nodeIndex) const;
    void computeAABBForNode(int);
    void collectIntersectedBoxes(Ray&, HitInfo&, int, std::priority_queue<NodeProps>&) const;

    Scene *m_pScene;
    AxisAlignedBox root;
    std::vector<Node> nodes;
    int maxLevel = 0;
    bool visualDebugTraversal = false;
};
