#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/geometric.hpp>
#include <functional>
#include <queue>
const int triangleLimit = 3;


void BoundingVolumeHierarchy::computeAABBForNode(int nodeIndex)
{
	auto& node = this->nodes[nodeIndex];


	float minx = INFINITY, miny = INFINITY, minz = INFINITY,
		maxx = -INFINITY, maxy = -INFINITY, maxz = -INFINITY;
	for (int i = 0; i < node.childIndices.size(); i++)
	{
		const auto& triangle = this->triangles[node.childIndices[i]].triangle;

		const auto& tx = vertices[triangle.x];
		minx = glm::min(tx.position.x, minx);
		miny = glm::min(tx.position.y, miny);
		minz = glm::min(tx.position.z, minz);
		maxx = glm::max(tx.position.x, maxx);
		maxy = glm::max(tx.position.y, maxy);
		maxz = glm::max(tx.position.z, maxz);

		const auto& ty = vertices[triangle.y];
		minx = glm::min(ty.position.x, minx);
		miny = glm::min(ty.position.y, miny);
		minz = glm::min(ty.position.z, minz);
		maxx = glm::max(ty.position.x, maxx);
		maxy = glm::max(ty.position.y, maxy);
		maxz = glm::max(ty.position.z, maxz);

		const auto& tz = vertices[triangle.z];
		minx = glm::min(tz.position.x, minx);
		miny = glm::min(tz.position.y, miny);
		minz = glm::min(tz.position.z, minz);
		maxx = glm::max(tz.position.x, maxx);
		maxy = glm::max(tz.position.y, maxy);
		maxz = glm::max(tz.position.z, maxz);
	}
	node.aabb = AxisAlignedBox{ {minx, miny, minz}, {maxx, maxy, maxz} };
}

void BoundingVolumeHierarchy::copySceneData()
{
	const auto& scene = (*this->m_pScene);
	unsigned int startingVertexPos = 0;
	for (int i = 0; i < scene.meshes.size(); i++)
	{
		const auto& mesh = scene.meshes[i];
		for (int j = 0; j < mesh.vertices.size(); j++)
		{
			this->vertices.push_back(mesh.vertices[j]);
		}
		for (int j = 0; j < mesh.triangles.size(); j++)
		{
			TriangleWithMesh t;
			t.meshIndex = i;
			t.triangle = mesh.triangles[j] + startingVertexPos;
			this->triangles.push_back(t);
		}
		startingVertexPos += mesh.vertices.size();
	}
}

void BoundingVolumeHierarchy::computeBVH()
{
	this->computeBVH(0);
}


// source: https://www.geeksforgeeks.org/program-to-find-the-centroid-of-the-triangle/ //
float centroidPerTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, const int splitAxis)
{
	if (splitAxis == 0)
	{
		return (v0.position.x + v1.position.x + v2.position.x) / 3.0;
	}
	else if (splitAxis == 1)
	{
		return (v0.position.y + v1.position.y + v2.position.y) / 3.0;
	}
	return (v0.position.z + v1.position.z + v2.position.z) / 3.0;
}

bool operator<(const NodeProps& l, const NodeProps& r) {
	return l.t > r.t;
}

// source: https://www.geeksforgeeks.org/comparator-class-in-c-with-examples/ //
class TriangleComparator {
public:
	int splitAxis;
	BoundingVolumeHierarchy* bvh;
	TriangleComparator(int splitAxis, BoundingVolumeHierarchy* bvh) {
		this->splitAxis = splitAxis;
		this->bvh = bvh;
	}
	bool operator()(int leftTriangle, int rightTriangle) {

		const auto& t0v0 = bvh->vertices[bvh->triangles[leftTriangle].triangle.x];
		const auto& t0v1 = bvh->vertices[bvh->triangles[leftTriangle].triangle.y];
		const auto& t0v2 = bvh->vertices[bvh->triangles[leftTriangle].triangle.z];

		const auto& t1v0 = bvh->vertices[bvh->triangles[rightTriangle].triangle.x];
		const auto& t1v1 = bvh->vertices[bvh->triangles[rightTriangle].triangle.y];
		const auto& t1v2 = bvh->vertices[bvh->triangles[rightTriangle].triangle.z];

		return centroidPerTriangle(t0v0, t0v1, t0v2, splitAxis) < centroidPerTriangle(t1v0, t1v1, t1v2, splitAxis);
	}

};


void BoundingVolumeHierarchy::computeBVH(int nodeIndex)
{
	auto& node = this->nodes[nodeIndex];

	// Return back to "<" if doesnt work
	if (node.childIndices.size() <= triangleLimit)
	{
		node.leaf = true;
		return;
	}

	int splitAxis = node.level % 3;

	auto comp = new TriangleComparator(splitAxis, this);

	std::sort(node.childIndices.begin(), node.childIndices.end(), *comp);
	int medianPoint = node.childIndices.size() / 2;
	
	std::vector<int> leftChildren;
	std::vector<int> rightChildren;

	for (int i = 0; i < medianPoint; i++) {
		leftChildren.push_back(node.childIndices[i]);
	}
	for (int i = medianPoint; i < node.childIndices.size(); i++) {
		rightChildren.push_back(node.childIndices[i]);
	}


	Node left = Node{ false, leftChildren, node.level + 1 };
	Node right = Node{ false, rightChildren, node.level + 1 };

	this->maxLevel = glm::max(left.level, this->maxLevel);

	this->nodes.push_back(left);
	this->nodes.push_back(right);

	auto& reRefNode = this->nodes[nodeIndex];

	int leftNodeIndex = this->nodes.size() - 2;
	int rightNodeIndex = this->nodes.size() - 1;

	this->computeAABBForNode(leftNodeIndex);
	this->computeAABBForNode(rightNodeIndex);

	reRefNode.childIndices.clear(); // problem //
	reRefNode.childIndices.push_back(leftNodeIndex);
	reRefNode.childIndices.push_back(rightNodeIndex);

	this->computeBVH(leftNodeIndex);
	this->computeBVH(rightNodeIndex);
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene) : m_pScene(pScene)
{
	this->copySceneData();
	std::vector<int> triangleIndices;
	for (auto i = 0; i < this->triangles.size(); i++)
	{
		triangleIndices.push_back(i);
	}
	// this is adding the root node
	this->nodes.push_back(Node{ false, triangleIndices, 0 });
	this->computeAABBForNode(0);
	this->computeBVH();
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{
	return maxLevel;
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
	for (int i = 0; i < this->nodes.size(); i++) {
		if (this->nodes[i].level == level) {
			drawAABB(this->nodes[i].aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);
			drawAABB(this->nodes[i].aabb, DrawMode::Wireframe);
		}
	}
}

void BoundingVolumeHierarchy::debugTraversal(bool on) {
	if (on) this->visualDebugTraversal = true;
	else this->visualDebugTraversal = false;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
	bool hit = false;
	// if (nodes.size() == 0) return false;
	// collecting all the leaf nodes that the ray interesects
	// std::priority_queue<NodeProps> onlyIntersectedNodes;
	// float t;
	// auto& root = nodes[0];
	// if (intersectRayWithShape(root.aabb, ray, t)) {
		// onlyIntersectedNodes.push(NodeProps{ 0, t });
		// while (!onlyIntersectedNodes.empty()) {
			// auto& props = onlyIntersectedNodes.top();
			// onlyIntersectedNodes.pop();
			// if (props.t > ray.t) break;
			// auto& node = nodes[props.nodeInt];
			// if (node.leaf) {
				// hit |= intersectTriangles(ray, hitInfo, props.nodeInt);
			// }
			// else {
				// if (intersectRayWithShape(nodes[node.childIndices[0]].aabb, ray, t)) {
					// onlyIntersectedNodes.push(NodeProps{node.childIndices[0], t});
				// } 
				// if (intersectRayWithShape(nodes[node.childIndices[1]].aabb, ray, t)) {
					// onlyIntersectedNodes.push(NodeProps{ node.childIndices[1], t });
				// }
			// }
		// }
	// }
	//collectIntersectedBoxes(ray, hitInfo, 0, onlyIntersectedNodes);

	//// sort by the distance from the origin of the ray to the intersection point with AABB
	//std::sort(onlyIntersectedNodes.begin(), onlyIntersectedNodes.end(), [](const NodeProps& l, const NodeProps& r)->bool {
	//	return std::get<1>(l) < std::get<1>(r);
	//	});

	hit = this->intersectNodes(ray, hitInfo, 0);

	// Intersect with all triangles of all meshes.
	//for (const auto &mesh : m_pScene->meshes)
	//{
	//    for (const auto &tri : mesh.triangles)
	//    {
	//        const auto v0 = mesh.vertices[tri[0]];
	//        const auto v1 = mesh.vertices[tri[1]];
	//        const auto v2 = mesh.vertices[tri[2]];
	//        if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo))
	//        {
	//            hitInfo.material = mesh.material;
	//            hit = true;
	//        }
	//    }
	//}

	// Intersect with spheres.
	
	
	for (const auto& sphere : m_pScene->spheres)
		hit |= intersectRayWithShape(sphere, ray, hitInfo);

	if (hit && this->visualDebugTraversal) {
		drawTriangle(hitInfo.v0, hitInfo.v1, hitInfo.v2);
	}
	return hit;
}

void BoundingVolumeHierarchy::collectIntersectedBoxes(Ray& ray, HitInfo& hitInfo, int nodeIndex, 
	std::priority_queue<NodeProps>& collection) const {
	bool hit = false;
	auto& node = this->nodes[nodeIndex];
	float t = 0.0f;
	hit = intersectRayWithShape(node.aabb, ray, t);

	if (hit) {
		if (node.leaf)
			collection.push(NodeProps{nodeIndex, t});
		else {
			collectIntersectedBoxes(ray, hitInfo, node.childIndices[0], collection);
			collectIntersectedBoxes(ray, hitInfo, node.childIndices[1], collection);
		}
	}

	return;
}

bool BoundingVolumeHierarchy::intersectNodes(Ray& ray, HitInfo& hitInfo, int nodeIndex) const {
	bool hit = false;
	auto& node = this->nodes[nodeIndex];
	// add a reference to the book
	float t;
	hit = intersectRayWithShape(node.aabb, ray,t);
	// debug traversal
	if (hit && this->visualDebugTraversal) {
		drawAABB(node.aabb, DrawMode::Wireframe, glm::vec3(1.0f, 0.0f, 0.0f));
	}
	if (hit && !node.leaf) {
		bool left = this->intersectNodes(ray, hitInfo, node.childIndices[0]);
		bool right = this->intersectNodes(ray, hitInfo, node.childIndices[1]);
		hit = (left | right);
	}
	else if (hit && node.leaf) {
		return this->intersectTriangles(ray, hitInfo, nodeIndex);
	}

	return hit;
}

bool BoundingVolumeHierarchy::intersectTriangles(Ray& ray, HitInfo& hitInfo, int nodeIndex) const {
	bool hit = false;

	// Intersect with all triangles of all meshes.
	for (const auto& triangleIndex : this->nodes[nodeIndex].childIndices)
	{
		auto meshIndex = this->triangles[triangleIndex].meshIndex;
		auto tri = this->triangles[triangleIndex].triangle;
		const auto v0 = this->vertices[tri[0]];
		const auto v1 = this->vertices[tri[1]];
		const auto v2 = this->vertices[tri[2]];
		if (intersectRayWithTriangle(v0, v1, v2, ray, hitInfo))
		{
			hitInfo.material = this->m_pScene->meshes[meshIndex].material;
			hit = true;
		}

	}
	return hit;
}
