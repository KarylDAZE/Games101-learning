//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
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
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{

    // TODO Implement Path Tracing Algorithm here
    Vector3f L_dir, L_indir, wi, wo, ws;
    Intersection hitPoint, inter, inter_fact;
    float pdf_light;
    hitPoint = intersect(ray);
    if (!hitPoint.happened)
        return Vector3f();
    // std::cout << " p:" << hitPoint.coords << std::endl;
    wo = -ray.direction;
    if (hitPoint.obj->hasEmit())
    // hitPoint on light
    {
        L_dir = hitPoint.m->getEmission();
    }
    else
    {
        sampleLight(inter, pdf_light);
        ws = normalize(inter.coords - hitPoint.coords);
        inter_fact = intersect(Ray(hitPoint.coords, ws));
        if (inter_fact.happened && inter_fact.obj->hasEmit())
        {
            // If the ray is not blocked in the middle
            // std::cout << "directory" << std::endl;
            // std::cout << inter.emit * hitPoint.m->eval(wo, ws, hitPoint.normal) << std::endl;
            L_dir = inter.emit * hitPoint.m->eval(wo, ws, hitPoint.normal) * dotProduct(ws, hitPoint.normal) * dotProduct(-ws, inter.normal) / pow((inter.coords - hitPoint.coords).norm(), 2) / pdf_light;
        }

        if ((float)rand() / RAND_MAX < RussianRoulette)
        // Test Russian Roulette with probability RussianRoulette
        {
            // std::cout << "p coords:" << hitPoint.coords << " p emit:" << hitPoint.obj->hasEmit() << std::endl;
            wi = hitPoint.m->sample(wo, hitPoint.normal);

            inter_fact = intersect(Ray(hitPoint.coords, wi));

            if (inter_fact.happened && !inter_fact.obj->hasEmit())
            {
                L_indir = castRay(Ray(hitPoint.coords, wi), depth + 1) * hitPoint.m->eval(wo, wi, hitPoint.normal) * dotProduct(wi, hitPoint.normal) / hitPoint.m->pdf(wo, wi, hitPoint.normal) / RussianRoulette;
            }
        }
        // std::cout << L_dir + L_indir << std::endl;
    }
    return L_dir + L_indir;
}