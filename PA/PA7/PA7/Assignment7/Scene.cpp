//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <random>

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
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
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

/*
1 shade(p, wo)
2 sampleLight(inter , pdf_light)
3 Get x, ws, NN, emit from inter
4 Shoot a ray from p to x
5 If the ray is not blocked in the middle
6 L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
7
8 L_indir = 0.0
9 Test Russian Roulette with probability RussianRoulette
10 wi = sample(wo, N)
11 Trace a ray r(p, wi)
12 If ray r hit a non-emitting object at q
13 L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
14
15 Return L_dir + L_indir
*/
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f();
    }

    // If the intersection point is an emitting object, return its emission
    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }

    Vector3f L_dir, L_indir;
    Vector3f wo = normalize(ray.direction);
    Vector3f p = inter.coords;
    Vector3f N = inter.normal;

    // Sample the light
    Intersection light_inter;
    float pdf_light = 0.f;
    sampleLight(light_inter, pdf_light);
    Vector3f x = light_inter.coords, ws = normalize(x - p), NN = light_inter.normal;
    Ray detect_ray(p, ws);
    Intersection p2x = intersect(detect_ray);
    if (p2x.happened && p2x.distance - (x - p).norm() > -0.005)
    {
        L_dir = light_inter.emit * inter.m->eval(wo, ws, N) 
                    * dotProduct(ws, N) * dotProduct(-ws, NN) 
                    / dotProduct(x - p, x - p) / pdf_light;
    }

    // Russian Roulette
    float rr_prob = get_random_float();
    if (rr_prob < RussianRoulette) {
        auto wi = normalize(inter.m->sample(wo, N));
        Ray r(p, wi);
        Intersection q = intersect(r);
        if (q.happened && !q.m->hasEmission())
        {
            L_indir = castRay(r, depth + 1) * inter.m->eval(wo, wi, N) * dotProduct(wi, N)
                      / inter.m->pdf(wo, wi, N) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}