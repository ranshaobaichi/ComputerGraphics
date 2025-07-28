#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if (splitMethod == SplitMethod::NAIVE)
    {
        printf(" - Generating BVH using NAIVE method...\n\n");
        root = recursiveBuild(primitives);
    }
    else if (splitMethod == SplitMethod::SAH)
    {
        printf(" - Generating BVH using SAH method...\n\n");
        root = BuildSAH(primitives);
    }
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::BuildSAH(std::vector<Object*> objects)
{
    // Base case: create leaf node
    if (objects.size() <= 2) {
        return BVHAccel::recursiveBuild(objects); // 回退为递归构建
    }

    // 计算包围盒和质心包围盒
    Bounds3 bounds, centroidBounds;
    for (auto obj : objects) {
        bounds = Union(bounds, obj->getBounds());
        centroidBounds = Union(centroidBounds, obj->getBounds().Centroid());
    }

    int dim = centroidBounds.maxExtent(); // 选择最长轴

    // 如果所有物体质心都在一个点上，退化为 leaf
    if (centroidBounds.Diagonal()[dim] == 0) {
        return BVHAccel::recursiveBuild(objects);
    }

    // Step 1: 创建桶
    const int nBuckets = 8;
    struct BucketInfo {
        int count = 0;
        Bounds3 bounds;
    };
    BucketInfo buckets[nBuckets];

    for (auto obj : objects) {
        Vector3f centroid = obj->getBounds().Centroid();
        float offset = centroidBounds.Offset(centroid)[dim]; // 在该轴上的[0,1]归一化位置
        int b = std::min((int)(nBuckets * offset), nBuckets - 1);
        buckets[b].count++;
        buckets[b].bounds = Union(buckets[b].bounds, obj->getBounds());
    }

    // Step 2: 遍历所有可能的划分位置，记录最小代价
    float minCost = std::numeric_limits<float>::infinity();
    int minCostSplitBucket = -1;

    Bounds3 leftBounds[nBuckets - 1], rightBounds[nBuckets - 1];
    int leftCount[nBuckets - 1] = {}, rightCount[nBuckets - 1] = {};

    for (int i = 0; i < nBuckets - 1; ++i) {
        Bounds3 b;
        int count = 0;
        for (int j = 0; j <= i; ++j) {
            b = Union(b, buckets[j].bounds);
            count += buckets[j].count;
        }
        leftBounds[i] = b;
        leftCount[i] = count;

        b = Bounds3();
        count = 0;
        for (int j = i + 1; j < nBuckets; ++j) {
            b = Union(b, buckets[j].bounds);
            count += buckets[j].count;
        }
        rightBounds[i] = b;
        rightCount[i] = count;

        float cost = 0.125f + 
                     (leftCount[i] * leftBounds[i].SurfaceArea() +
                      rightCount[i] * rightBounds[i].SurfaceArea()) /
                     bounds.SurfaceArea();

        if (cost < minCost) {
            minCost = cost;
            minCostSplitBucket = i;
        }
    }

    // Step 3: 根据 minCostSplitBucket 划分 objects
    std::vector<Object*> leftShapes, rightShapes;
    for (auto obj : objects) {
        Vector3f centroid = obj->getBounds().Centroid();
        float offset = centroidBounds.Offset(centroid)[dim];
        int b = std::min((int)(nBuckets * offset), nBuckets - 1);
        if (b <= minCostSplitBucket) {
            leftShapes.push_back(obj);
        } else {
            rightShapes.push_back(obj);
        }
    }

    if (leftShapes.empty() || rightShapes.empty()) {
        // fallback 防止一边为空，避免无限递归
        return BVHAccel::recursiveBuild(objects);
    }

    BVHBuildNode* node = new BVHBuildNode();
    node->left = BuildSAH(leftShapes);
    node->right = BuildSAH(rightShapes);
    node->bounds = Union(node->left->bounds, node->right->bounds);

    return node;
}


Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if (node == nullptr) {
        return Intersection();
    }

    if (node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0})) 
    {
        if (node->left == nullptr && node->right == nullptr && node->object != nullptr)
        {
            return node->object->getIntersection(ray);
        }  

        Intersection l_intersection, r_intersection;
        if (node->left) {
            l_intersection = getIntersection(node->left, ray);
        }
        if (node->right) {
            r_intersection = getIntersection(node->right, ray);
        }
        
        if (l_intersection.happened && r_intersection.happened) {
            if (l_intersection.distance < r_intersection.distance) {
                return l_intersection;
            } else {
                return r_intersection;
            }
        } else if (l_intersection.happened) {
            return l_intersection;
        } else if (r_intersection.happened) {
            return r_intersection;
        }
    }

    return Intersection();
}