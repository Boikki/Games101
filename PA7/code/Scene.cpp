//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Vector.hpp"
#include "global.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    // 在场景的所有
    // 光源上按面积 uniform 地 sample 一个点
    // 并计算该 sample 的概率密度
    // 对所有光源随机采样
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            // 自发光area总面积
            // 为什么要求这个
            // 因为下面要随机取一个光源面
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                // 在第k个光源面上按pdf选一个pos点出来
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
        // 也就是取面是随机的 取点也是随机的
        // 并且pdf都可以概括为1 / area
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// 这里传入参数 depth 自然没什么用了
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);

    if (!intersection.happened) return Vector3f();

    if (intersection.m->hasEmission()) return intersection.m->getEmission();

    // 按照伪代码来
    // sampleLight(inter, pdf_light)
    Vector3f L_dir, L_indir;
    Intersection inter_light;
    float pdf_light = 0.f;
    sampleLight(inter_light, pdf_light);
    // Get x, ws, NN, emit from inter

    // Shoot a ray from p to x
    Vector3f obj2light = inter_light.coords - intersection.coords;
    // 物体->光源的方向
    Vector3f ws = obj2light.normalized();
    // 物体->光源的距离
    float dis = obj2light.norm();

    Ray _obj2light(intersection.coords, ws);
    // If the ray is not blocked in the middle
    Intersection scene2light = Scene::intersect(_obj2light);
    // dis > _obj2light 就说明有遮挡 因为新发射出的逆向光线小于原来的了
    Vector3f wo = ray.direction;
    Vector3f N = intersection.normal.normalized();
    Vector3f NN = inter_light.normal.normalized();
    if (scene2light.happened && intersect(_obj2light).distance - dis >= -EPSILON) {
        //
        // L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
        Vector3f _emit = inter_light.emit;
        Vector3f Fr = intersection.m->eval(wo, ws, N);
        L_dir = _emit * Fr * dotProduct(ws, N) * dotProduct(-ws, NN) / (dis * dis) / pdf_light;
    }
    // R_PP
    // 俄罗斯轮盘赌
    // Manually specify a probability P_RR
    // P_RR has defined in Scene.hpp = 0.8
    // Randomly select ksi in a uniform dist. in [0, 1]
    // If (ksi > P_RR) return 0.0;
    if (get_random_float() > RussianRoulette) return L_dir;

    // calcuate non-direct illumation
    // 计算间接光照
    Vector3f wi = intersection.m->sample(wo, N);
    Ray nextObjRay(intersection.coords, wi);

    Intersection nextInter = Scene::intersect(nextObjRay);
    if (nextInter.happened && !nextInter.m->hasEmission())  {
        Vector3f Fr = nextInter.m->eval(wo, wi, N);
        L_indir = castRay(nextObjRay, depth + 1) * Fr * dotProduct(wi, N) / intersection.m->pdf(wo, wi, N) / RussianRoulette;
    }

            
    return L_dir + L_indir;

}
