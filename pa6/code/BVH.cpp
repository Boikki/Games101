#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Intersection.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

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
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        // 看样子因该是合并传入的两个 Box
        // 将传入的两个Bounds（姑且称为包围盒）
        // 将两个包围盒中最小的顶点和最大的顶点找出来
        // 将他们作为新的包围盒边界
        // 从而达成了边界合并的效果，生成新的包围盒
        // 应用于三角形片元中
        // 这是对上述提到过的object中所有三角形片元进行包围盒的合并

        // Triangle 重载了obj的方法
        // Bounds3 getBounds() { return bounding_box; }
        // return Bounds3 
        bounds = Union(bounds, objects[i]->getBounds());
    // //如果仅有一个片元，则创建一个叶子结点
    if (objects.size() == 1) {
        // Create leaf _SAHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    //如果有两个片元，则生成的节点分别记录指向的节点，且该节点实际记录的是两个子节点共同的大包装盒
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    // SAH 对于一个节点和两个节点
    // 的处理方式和BVH一样的
    // 不同的是下面
    else {
        // 获取质心的Box
        // 以质心作为主要点，生成所有三角片元质心的大包装盒
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        // Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());

        // Diagonal 获取Box对角向量
        // Vector3f Diagonal() const { return pMax - pMin; }
        // int maxExtent() const
        // {
        //     Vector3f d = Diagonal();
        //     if (d.x > d.y && d.x > d.z) //x分量最大
        //         return 0;
        //     else if (d.y > d.z) //y分量最大
        //         return 1;
        //     else //z分量最大
        //         return 2;
        // }
        // 当对角向量x分量最大，则返回0；y分量最大，则返回1；z分量最大，则返回2。
        //
        // 三个轴分别求最优解
        int dim = centroidBounds.maxExtent();
        switch (dim) {
            // x分量最大
            // 按中心点的x坐标升序
            // 下面的以此类推
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

        // 二分Mesh Tri 数组
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        // 递归构建BVH树
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

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
    Intersection intersection;
    if (node == nullptr || !(node->bounds.IntersectP(ray, ray.direction_inv, {0, 0, 0}))) {
        return intersection;
    }
    // 如果碰撞盒不再继续细分，测试碰撞盒内的所有物体是否与光线相交，返回最早相交的
    if (node->left == nullptr && node->right == nullptr) {
         // 这里调用的是obj子类：Triangle的
         // getIntersection，即到叶子节点的包围盒开始与三角形求交
         // 纯虚函数 在加载进Triangle类进来会load三角形的方法
        return node->object->getIntersection(ray);
    }
    // 递归划分碰撞盒并且遍历测试
    Intersection hitLeft = getIntersection(node->left, ray);
    Intersection hitRight = getIntersection(node->right, ray);

    return hitLeft.distance < hitRight.distance ? hitLeft : hitRight;
}
