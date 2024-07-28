#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "ray_tracing.h"
#include "screen.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>

DISABLE_WARNINGS_PUSH()

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
#include <nfd.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

DISABLE_WARNINGS_POP()

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <framework/image.h>
#include <framework/imguizmo.h>
#include <framework/trackball.h>
#include <framework/variant_helper.h>
#include <framework/window.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#include <variant>

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2
windowResolution {
800, 800 };
const std::filesystem::path dataPath{DATA_DIR};

const int sampleSegmentLineLimit = 6;
const int sampleParallelogramWidthLimit = 5; 
const int glossyReflectionCount = 3;
const int randomUpperLimit = 100;

const bool disableGlossy = true;

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};

static glm::vec3 getReflectiveColor(const Scene &, Ray, HitInfo, const BoundingVolumeHierarchy &, int);
static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray);

static glm::vec3 diffuseOnly(glm::vec3 kd, glm::vec3 normal, glm::vec3 lightDirection, glm::vec3 color) {
    return (kd * glm::dot(normal, glm::normalize(lightDirection))) * color;
}

static glm::vec3 specularOnly(glm::vec3 ks, glm::vec3 normal, float shininess, glm::vec3 lightPos, glm::vec3 vertexPos,
                              glm::vec3 cameraPos) {
    glm::vec3 result{0, 0, 0};

    glm::vec3 lightDirection = glm::normalize(lightPos - vertexPos);

    if (glm::max(glm::dot(lightDirection, normal), (float) 0.0) > 0.0) {
        glm::vec3 reflectionVector = glm::reflect(-lightDirection, glm::normalize(normal));
        glm::vec3 viewVector = glm::normalize(cameraPos - vertexPos);
        float dotP = glm::max(glm::dot(reflectionVector, viewVector), (float) 0.0);
        result = ks * glm::pow(dotP, shininess);
    }

    return result;
}

static glm::vec3 clamp(glm::vec3 toClamp) {
    if (toClamp.x > 1) toClamp.x = 1;
    if (toClamp.y > 1) toClamp.y = 1;
    if (toClamp.z > 1) toClamp.z = 1;
    return toClamp;
}

static glm::vec2 getTextureCoordinate(HitInfo hitInfo) {
    glm::vec2 v0 = hitInfo.v0.texCoord;
    glm::vec2 v1 = hitInfo.v1.texCoord;
    glm::vec2 v2 = hitInfo.v2.texCoord;
    glm::vec3 bary = hitInfo.barycentric;

    glm::vec2 point = v0 * bary.x + v1 * bary.y + v2 * bary.z;
    return point;
}

static float computeHardShadows(Ray shadowRay, HitInfo hitInfoShadows, glm::vec3 vertexPos, glm::vec3 avgPos, const BoundingVolumeHierarchy& bvh, glm::vec3 pointLightColor) {
    float result = 1;
    bool f = bvh.intersect(shadowRay, hitInfoShadows);
    if (f && glm::length(avgPos - vertexPos) > glm::length((shadowRay.origin + shadowRay.t * shadowRay.direction) - vertexPos)) {
        // check if object is transparent
        float transparencyShadow = hitInfoShadows.material.transparency;
        if (transparencyShadow == 1) {
            result = 0;
            // debug hard shadow (red ray)
            drawRay(shadowRay, glm::vec3{ 1, 0, 0 });
        }
        else {
            // debug transparent hard shadow (yellow ray)
            drawRay(shadowRay, glm::vec3{ 1,1,0 });
            shadowRay.origin = shadowRay.origin + shadowRay.t * shadowRay.direction + shadowRay.direction * 0.00001f;
            shadowRay.t = std::numeric_limits<float>::max();
            result = result * (1 - transparencyShadow) * computeHardShadows(shadowRay, hitInfoShadows, vertexPos, avgPos, bvh, pointLightColor);
        }
    }
    else {
        drawRay(shadowRay, pointLightColor);
        result = 1;
    }

    return result;
}



static glm::vec3 calculatePointLight(const Scene &scene, const BoundingVolumeHierarchy &bvh, Ray ray, HitInfo hitInfo,
                                     PointLight &pointLight, int recursiveLevel) {
    glm::vec3 diffuse;
    glm::vec3 specular;
    Material material = hitInfo.material;
    glm::vec3 kd = material.kd;
    
    // texture variable
    std::optional<Image> kdTexture = material.kdTexture;
    if (material.kdTexture) {
        kd = material.kdTexture->getTexel(getTextureCoordinate(hitInfo));
    }
    
    glm::vec3 ks = material.ks;
    float shininess = material.shininess;
    float transparency = material.transparency;
    glm::vec3 normal = hitInfo.normal;
    glm::vec3 vertexPos = ray.origin + ray.t * ray.direction;

    Ray debugNormal;
    debugNormal.direction = normal;
    debugNormal.origin = vertexPos;
    debugNormal.t = .3;
    drawRay(debugNormal, { 0, 0, 1 });

    Ray v0d;
    v0d.origin = hitInfo.v0.position;
    v0d.direction = hitInfo.v0.normal;
    v0d.t = .3;
    drawRay(v0d, { 0, 1, 0 });

    Ray v1d;
    v1d.origin = hitInfo.v1.position;
    v1d.direction = hitInfo.v1.normal;
    v1d.t = .3;
    drawRay(v1d, { 0, 1, 0 });

    Ray v2d;
    v2d.origin = hitInfo.v2.position;
    v2d.direction = hitInfo.v2.normal;
    v2d.t = .3;
    drawRay(v2d, { 0, 1, 0 });

    glm::vec3 lightDirection;
    glm::vec3 avgPos;
    glm::vec3 avgColor;
    glm::vec3 result = glm::vec3{0, 0, 0};
    auto originalHitInfo = hitInfo;

    // Perform your calculations for a point light.
    avgPos = pointLight.position;
    lightDirection = avgPos - vertexPos;
    diffuse = diffuseOnly(kd, normal, lightDirection, pointLight.color);
    specular = specularOnly(ks, normal, shininess, avgPos, vertexPos, ray.origin);
    
    // calculate color background for transparency
    Ray transparencyRay;
    transparencyRay.origin = vertexPos + ray.direction * 0.00001f;
    transparencyRay.direction = ray.direction;
    glm::vec3 transparentColor = glm::vec3{ 0,0,0 };
    if (transparency != 1) {
        transparentColor = getFinalColor(scene, bvh, transparencyRay);
    }
    
    // check hard shadows 
    Ray shadowRay;
    shadowRay.direction = glm::normalize(avgPos - vertexPos);
    if (glm::length(vertexPos - normal * 0.1f - ray.origin) < glm::length(vertexPos + normal * 0.1f - ray.origin)) {
        shadowRay.origin = vertexPos - normal * 0.0001f - ray.direction * 0.0001f;
    }
    else {
        shadowRay.origin = vertexPos + normal * 0.0001f - ray.direction * 0.0001f;
    }

    HitInfo hitInfoShadows;
    glm::vec3 pointLightColor = pointLight.color;
    float hardShadows = computeHardShadows(shadowRay, hitInfoShadows, vertexPos, avgPos, bvh, pointLightColor);

    

    result = result + hardShadows * transparency * (diffuse + specular) + (1 - transparency) * transparentColor 
        + ks * getReflectiveColor(scene, ray, originalHitInfo, bvh, recursiveLevel);

    return result;

}

static glm::vec3 calculateSegmentLight(const Scene &scene, const BoundingVolumeHierarchy &bvh, Ray ray, HitInfo hitInfo,
                                       const SegmentLight &segmentLight, int recursiveLevel) {
    glm::vec3 currentSumColor{0.0f};

    for (int i = 0; i < sampleSegmentLineLimit; i++) {
        float deviation = (((float)(rand() % randomUpperLimit)) / randomUpperLimit) * (1.0/sampleSegmentLineLimit);
        float interpI = i + deviation;
        PointLight pointLight;

        pointLight.position = glm::mix(segmentLight.endpoint0, segmentLight.endpoint1, interpI /
                                                                                       (sampleSegmentLineLimit -
                                                                                        1));
        pointLight.color = glm::mix(segmentLight.color0, segmentLight.color1, interpI / (sampleSegmentLineLimit -
                                                                                             1));
        currentSumColor += calculatePointLight(scene, bvh, ray, hitInfo, pointLight, recursiveLevel);
    }

    currentSumColor /= (float) (sampleSegmentLineLimit);
    return currentSumColor;
}

static glm::vec3
calculateParallelogramLight(const Scene &scene, const BoundingVolumeHierarchy bvh, Ray ray, HitInfo hitInfo,
                          const ParallelogramLight &light, int recursiveLevel) {
    auto res = glm::vec3{0};
    // sample segments from the parallelogram, and add them to the result
    for (int i = 0; i < sampleParallelogramWidthLimit; i++) {
        SegmentLight segmentLight;
        float deviation = (((float)(rand() % randomUpperLimit)) / randomUpperLimit) * (1.0/sampleParallelogramWidthLimit);
        float interpI = i + deviation;
        // interp colors
        segmentLight.color0 = glm::mix(light.color0, light.color1, interpI / (sampleParallelogramWidthLimit - 1));
        segmentLight.color1 = glm::mix(light.color2, light.color3, interpI / (sampleParallelogramWidthLimit - 1));
        // interp coords
        segmentLight.endpoint0 = glm::mix(light.v0, light.v0 + light.edge01, interpI /(sampleParallelogramWidthLimit - 1));
        segmentLight.endpoint1 = glm::mix(light.v0 + light.edge02, light.v0 + light.edge02 + light.edge01, interpI /(sampleParallelogramWidthLimit - 1));

        float interpJ = ((float)(rand() % ((sampleSegmentLineLimit - 1) * 1000))) / 1000;
        
        //PointLight pointLight;
        //pointLight.color = glm::mix(segmentLight.color0, segmentLight.color1, interpJ / (sampleSegmentLineLimit - 1));
        //pointLight.position = glm::mix(segmentLight.endpoint0, segmentLight.endpoint1, interpJ / (sampleSegmentLineLimit - 1));
        
        // interpolate the color over the segment that was created
        res += calculateSegmentLight(scene, bvh, ray, hitInfo, segmentLight, recursiveLevel);
    }

    return res / (float) (sampleParallelogramWidthLimit);
}


static glm::vec3
finalColorHelper(const Scene &scene, const BoundingVolumeHierarchy &bvh, Ray ray, HitInfo hitInfo, int recursiveLevel) {
    glm::vec3 result = glm::vec3{0, 0, 0};
    HitInfo originalHitInfo = hitInfo;
    // The code to iterate over the lights thus looks like this:
    for (const auto& light : scene.lights) {
        if (std::holds_alternative<PointLight>(light)) {
            PointLight pointLight = std::get<PointLight>(light);
            result += calculatePointLight(scene, bvh, ray, hitInfo, pointLight, recursiveLevel);
        } else if (std::holds_alternative<SegmentLight>(light)) {
            const SegmentLight segmentLight = std::get<SegmentLight>(light);
            result += calculateSegmentLight(scene, bvh, ray, hitInfo, segmentLight, recursiveLevel);
        } else if (std::holds_alternative<ParallelogramLight>(light)) {
            const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
            //// Perform your calculations for a parallelogram light.
            result += calculateParallelogramLight(scene, bvh, ray, hitInfo, parallelogramLight, recursiveLevel);
        }
    }
    return clamp(result);
}

static void debugGlossyRay(Ray& ray, float shininess) {
    // compute the basis for the square: (bu, bv)
    float square_size = .3;
    auto t = ray.direction;
    t.z += .2;
    t = glm::normalize(t);
    auto bu = glm::normalize(glm::cross(ray.direction, t));
    auto bv = glm::normalize(glm::cross(ray.direction, bu));

    float llu = -(square_size / 2);
    float llv = -(square_size / 2);
    float ulu = +(square_size / 2);
    float ulv = +(square_size / 2);

    Ray llr;
    llr.origin = ray.origin;
    llr.direction = glm::normalize(ray.direction + llu * bu + llv * bv);
    drawRay(llr, { 1, 1, 1 });

    Ray ulr;
    ulr.origin = ray.origin;
    ulr.direction = glm::normalize(ray.direction + ulu * bu + llv * bv);
    drawRay(ulr, { 1, 1, 1 });

    Ray lrr;
    lrr.origin = ray.origin;
    lrr.direction = glm::normalize(ray.direction + llu * bu + ulv * bv);
    drawRay(lrr, { 1, 1, 1 });

    Ray urr;
    urr.origin = ray.origin;
    urr.direction = glm::normalize(ray.direction + ulu * bu + ulv * bv);
    drawRay(urr, { 1, 1, 1 });
}

static void perturbRay(Ray& ray, float shininess, float& cosine) {
    /*
    Source: Fundamentals of computer graphics by Steve Marschner(Author) Peter Shirley (Author)
    Chapter 13.4.4
    Algorithm description:

    Simulate the effect of a glossy reflection (some image is visible, but blurred in the reflection)
    by randomly perturbing the ideal specular reflection. 

    To perturb the specular reflection, sample a random square which is perpendicular to r. 
    Set up the square's orientation by creating an orthonormal basis to r. (bu, bv)
    Take two random points ru, rv from [0, 1]
    Compute the analogous points su, sv
    Compute the perturb ray
    */
    // compute the basis for the square: (bu, bv)
    auto t = ray.direction;
    if (glm::abs(t.z) < glm::abs(t.y) && glm::abs(t.z) < glm::abs(t.x)) {
        t.z = 1;
    } else if (glm::abs(t.y) < glm::abs(t.z) && glm::abs(t.y) < glm::abs(t.x)) {
        t.y = 1;
    }
    else {
        t.x = 1;
    }
    t = glm::normalize(t);
    auto bu = glm::normalize(glm::cross(ray.direction, t));
    auto bv = glm::normalize(glm::cross(ray.direction, bu));

    // compute the analogous points su, sv 
    float ru = ((float)(rand() % randomUpperLimit)); 
    ru = ru / randomUpperLimit;
    float rv = ((float)(rand() % randomUpperLimit)); 
    rv = rv / randomUpperLimit;

    float square_size = 1/shininess;

    auto su = -(square_size / 2) + ru * square_size;
    auto sv = -(square_size / 2) + rv * square_size;
    
    // perturb the ray in the new direction
    auto oldDir = ray.direction;
    ray.direction = glm::normalize(ray.direction + su * bu + sv * bv);
    cosine = glm::dot(oldDir, ray.direction);
}

static glm::vec3 getReflectiveColor(const Scene& scene, Ray ray, HitInfo hitInfo, const BoundingVolumeHierarchy& bvh,
    int recursiveLevel) {
    // Base case
    if (recursiveLevel <= 0)
        return glm::vec3{ 0 };

    glm::vec3 finalColor{ 0, 0, 0 };
    //int sampleRays = ((hitInfo.material.shininess != 1) ? (glossyReflectionCount) : 1);

    // Base case point of intersection does not have specular component
    // or is black
    if (hitInfo.material.ks.x < 1e-6 && hitInfo.material.ks.y < 1e-6 && hitInfo.material.ks.z < 1e-6)
        return glm::vec3{ 0 };

    int sampleRays;
    bool glossy = (hitInfo.material.shininess > 1e-6 && !disableGlossy);
    if (glossy)  {
        sampleRays = glossyReflectionCount;
    }
    else {
        sampleRays = 1;
    }

    if (glossy) {
        Ray glossyNewRay;
        glossyNewRay.origin = ray.origin + ray.t * ray.direction;
        glossyNewRay.direction = ray.direction - (2.0f * glm::dot(hitInfo.normal, ray.direction)) * hitInfo.normal;
        debugGlossyRay(glossyNewRay, hitInfo.material.shininess);
    }
    float cosineWithOriginal = 0.0;

    for (auto ki = 0; ki < sampleRays; ki++) {
        // Intersection point formula: ray.origin + ray.t * ray.direction
        Ray newRay;
        newRay.origin = ray.origin + ray.t * ray.direction;
        newRay.direction = ray.direction - (2.0f * glm::dot(hitInfo.normal, ray.direction)) * hitInfo.normal;
        if (glossy) perturbRay(newRay, hitInfo.material.shininess, cosineWithOriginal);
        
        newRay.origin = newRay.origin + (float)1e-4 * newRay.direction;
        HitInfo hitInfoNewRay;
        bool newRayHasIntersected = bvh.intersect(newRay, hitInfoNewRay);

        if (newRayHasIntersected) {
            Ray drawableRay = newRay;

            drawRay(drawableRay, glm::vec3{ 1, 1, 0 });
            try {
                finalColor += finalColorHelper(scene, bvh, drawableRay, hitInfoNewRay, recursiveLevel - 1);
            }
            catch (std::exception) {
                auto debug = true;
            }
        }
        else {
            drawRay(newRay, glm::vec3{ 1, 0, 0 });
        }
    }


    return finalColor / (float) sampleRays;
}

static glm::vec3 getFinalColor(const Scene &scene, const BoundingVolumeHierarchy &bvh, Ray ray) {
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo)) {
        // Draw a white debug ray if the ray hits.
        glm::vec3 cachedColor = finalColorHelper(scene, bvh, ray, hitInfo, 3);
        drawRay(ray, cachedColor);
        // Set the color of the pixel to white if the ray hits.
        return cachedColor;
        // return glm::vec3{ 1,1,1 };
    } else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }

    // Lights are stored in a single array (scene.lights) where each item can be either a PointLight, SegmentLight or ParallelogramLight.
    // You can check whether a light at index i is a PointLight using std::holds_alternative:
    // std::holds_alternative<PointLight>(scene.lights[i])
    //
    // If it is indeed a point light, you can "convert" it to the correct type using std::get:
    // PointLight pointLight = std::get<PointLight>(scene.lights[i]);
    //
    //
    // Regarding the soft shadows for **other** light sources **extra** feature:
    // To add a new light source, define your new light struct in scene.h and modify the Scene struct (also in scene.h)
    // by adding your new custom light type to the lights std::variant. For example:
    // std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight, MyCustomLightType>> lights;
    //
    // You can add the light sources programmatically by creating a custom scene (modify the Custom case in the
    // loadScene function in scene.cpp). Custom lights will not be visible in rasterization view.
}

static void setOpenGLMatrices(const Trackball &camera);

static void drawLightsOpenGL(const Scene &scene, const Trackball &camera, int selectedLight);

static void drawSceneOpenGL(const Scene &scene);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void
renderRayTracing(const Scene &scene, const Trackball &camera, const BoundingVolumeHierarchy &bvh, Screen &screen) {
#ifndef NDEBUG
    // Single threaded in debug mode
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos{
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay));
        }
    }
#else
    // Multi-threaded in release mode
    const tbb::blocked_range2d<int, int> windowRange { 0, windowResolution.y, 0, windowResolution.x };
    tbb::parallel_for(windowRange, [&](tbb::blocked_range2d<int, int> localRange) {
        for (int y = std::begin(localRange.rows()); y != std::end(localRange.rows()); y++) {
            for (int x = std::begin(localRange.cols()); x != std::end(localRange.cols()); x++) {
                // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                const glm::vec2 normalizedPixelPos {
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
                };
                const Ray cameraRay = camera.generateRay(normalizedPixelPos);
                screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay));
            }
        }
    });
#endif
}

int main(int argc, char **argv) {
    srand(time(NULL));
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
              << std::endl;

    Window window{"Final Project", windowResolution, OpenGLVersion::GL2};
    Screen screen{windowResolution};
    Trackball camera{&window, glm::radians(50.0f), 3.0f};
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType{SceneType::SingleTriangle};
    std::optional <Ray> optDebugRay;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh{&scene};

    int bvhDebugLevel = 0;
    bool debugBVH{false};
    bool debugTraversal{ false };
    ViewMode viewMode{ViewMode::Rasterization};

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_R: {
                    // Shoot a ray. Produce a ray from camera to the far plane.
                    const auto tmp = window.getNormalizedCursorPos();
                    optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
                }
                    break;
                case GLFW_KEY_ESCAPE: {
                    window.close();
                }
                    break;
            };
        }
    });

    int selectedLightIdx = scene.lights.empty() ? -1 : 0;
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project");
        {
            constexpr
            std::array items{"SingleTriangle", "Cube (segment light)", "Cornell Box (with mirror)",
                             "Cornell Box (parallelogram light and mirror)", "Monkey", "Teapot", "Dragon", /* "AABBs",*/
                             "Spheres", /*"Mixed",*/ "Custom", "TextureVisualDebug"};
            if (ImGui::Combo("Scenes", reinterpret_cast<int *>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                selectedLightIdx = scene.lights.empty() ? -1 : 0;
                bvh = BoundingVolumeHierarchy(&scene);
                if (optDebugRay) {
                    HitInfo dummy{};
                    bvh.intersect(*optDebugRay, dummy);
                }
            }
        }
        {
            constexpr
            std::array items{"Rasterization", "Ray Traced"};
            ImGui::Combo("View mode", reinterpret_cast<int *>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            // Show a file picker.
            nfdchar_t *pOutPath = nullptr;
            const nfdresult_t result = NFD_SaveDialog("bmp", nullptr, &pOutPath);
            if (result == NFD_OKAY) {
                std::filesystem::path outPath{pOutPath};
                free(pOutPath); // NFD is a C API so we have to manually free the memory it allocated.
                outPath.replace_extension("bmp"); // Make sure that the file extension is *.bmp

                // Perform a new render and measure the time it took to generate the image.
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                renderRayTracing(scene, camera, bvh, screen);
                screen.filterImage(3, 0.9f, 2.0f); // comment this line for no filter to be applied
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count()
                          << " milliseconds" << std::endl;

                // Store the new image.
                screen.writeBitmapToFile(outPath);
            }
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH)
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
        }

        ImGui::Spacing();
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Debug BVH traversal", &debugTraversal);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        {
            std::vector <std::string> options;
            options.push_back("None");
            for (size_t i = 0; i < scene.lights.size(); i++) {
                options.push_back("Light " + std::to_string(i));
            }
            std::vector<const char *> optionsPointers;
            std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers),
                           [](const auto &str) { return str.c_str(); });

            // Offset such that selectedLightIdx=-1 becomes item 0 (None).
            ++selectedLightIdx;
            ImGui::Combo("Selected light", &selectedLightIdx, optionsPointers.data(),
                         static_cast<int>(optionsPointers.size()));
            --selectedLightIdx;

            if (selectedLightIdx >= 0) {
                setOpenGLMatrices(camera);
                std::visit(
                        make_visitor(
                                [&](PointLight &light) {
                                    showImGuizmoTranslation(window, camera,
                                                            light.position); // 3D controls to translate light source.
                                    ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f,
                                                      3.0f);
                                    ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                                },
                                [&](SegmentLight &light) {
                                    static int selectedEndpoint = 0;
                                    // 3D controls to translate light source.
                                    if (selectedEndpoint == 0)
                                        showImGuizmoTranslation(window, camera, light.endpoint0);
                                    else
                                        showImGuizmoTranslation(window, camera, light.endpoint1);

                                    const std::array<const char *, 2> endpointOptions{"Endpoint 0", "Endpoint 1"};
                                    ImGui::Combo("Selected endpoint", &selectedEndpoint, endpointOptions.data(),
                                                 (int) endpointOptions.size());
                                    ImGui::DragFloat3("Endpoint 0", glm::value_ptr(light.endpoint0), 0.01f, -3.0f,
                                                      3.0f);
                                    ImGui::DragFloat3("Endpoint 1", glm::value_ptr(light.endpoint1), 0.01f, -3.0f,
                                                      3.0f);
                                    ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                                    ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                                },
                                [&](ParallelogramLight &light) {
                                    glm::vec3 vertex1 = light.v0 + light.edge01;
                                    glm::vec3 vertex2 = light.v0 + light.edge02;

                                    static int selectedVertex = 0;
                                    // 3D controls to translate light source.
                                    if (selectedVertex == 0)
                                        showImGuizmoTranslation(window, camera, light.v0);
                                    else if (selectedVertex == 1)
                                        showImGuizmoTranslation(window, camera, vertex1);
                                    else
                                        showImGuizmoTranslation(window, camera, vertex2);

                                    const std::array<const char *, 3> vertexOptions{"Vertex 0", "Vertex 1", "Vertex 2"};
                                    ImGui::Combo("Selected vertex", &selectedVertex, vertexOptions.data(),
                                                 (int) vertexOptions.size());
                                    ImGui::DragFloat3("Vertex 0", glm::value_ptr(light.v0), 0.01f, -3.0f, 3.0f);
                                    ImGui::DragFloat3("Vertex 1", glm::value_ptr(vertex1), 0.01f, -3.0f, 3.0f);
                                    light.edge01 = vertex1 - light.v0;
                                    ImGui::DragFloat3("Vertex 2", glm::value_ptr(vertex2), 0.01f, -3.0f, 3.0f);
                                    light.edge02 = vertex2 - light.v0;

                                    ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                                    ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                                    ImGui::ColorEdit3("Color 2", glm::value_ptr(light.color2));
                                    ImGui::ColorEdit3("Color 3", glm::value_ptr(light.color3));
                                },
                                [](auto) { /* any other type of light */ }),
                        scene.lights[selectedLightIdx]);
            }
        }

        if (ImGui::Button("Add point light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(PointLight{.position = glm::vec3(0.0f), .color = glm::vec3(1.0f)});
        }
        if (ImGui::Button("Add segment light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(
                    SegmentLight{.endpoint0 = glm::vec3(0.0f), .endpoint1 = glm::vec3(1.0f), .color0 = glm::vec3(1, 0,
                                                                                                                 0), .color1 = glm::vec3(
                            0, 0, 1)});
        }
        if (ImGui::Button("Add parallelogram light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(ParallelogramLight{
                    .v0 = glm::vec3(0.0f),
                    .edge01 = glm::vec3(1, 0, 0),
                    .edge02 = glm::vec3(0, 1, 0),
                    .color0 = glm::vec3(1, 0, 0), // red
                    .color1 = glm::vec3(0, 1, 0), // green
                    .color2 = glm::vec3(0, 0, 1), // blue
                    .color3 = glm::vec3(1, 1, 1) // white
            });
        }
        if (selectedLightIdx >= 0 && ImGui::Button("Remove selected light")) {
            scene.lights.erase(std::begin(scene.lights) + selectedLightIdx);
            selectedLightIdx = -1;
        }

        // Clear screen.
        glViewport(0, 0, window.getFrameBufferSize().x, window.getFrameBufferSize().y);
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setOpenGLMatrices(camera);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
            case ViewMode::Rasterization: {
                glPushAttrib(GL_ALL_ATTRIB_BITS);
                drawSceneOpenGL(scene);
                if (optDebugRay) {
                    // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                    // draw the rays instead.
                    enableDrawRay = true;
                    glDepthFunc(GL_LEQUAL);
                    glDisable(GL_LIGHTING);
                    (void) getFinalColor(scene, bvh, *optDebugRay);
                    enableDrawRay = false;
                }
                glPopAttrib();
            }
                break;
            case ViewMode::RayTracing: {
                screen.clear(glm::vec3(0.0f));
                renderRayTracing(scene, camera, bvh, screen);
                screen.setPixel(0, 0, glm::vec3(1.0f));
                screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
            }
                break;
            default:
                break;
        };

        drawLightsOpenGL(scene, camera, selectedLightIdx);

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.debugDraw(bvhDebugLevel);
            enableDrawRay = false;
            glPopAttrib();
        }

        if (debugTraversal || !debugTraversal) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.debugTraversal(debugTraversal);
            enableDrawRay = false;
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0;
}

static void setOpenGLMatrices(const Trackball &camera) {
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void drawLightsOpenGL(const Scene &scene, const Trackball &camera, int selectedLight) {
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);

    glDisable(GL_LIGHTING);
    // Draw all non-selected lights.
    for (size_t i = 0; i < scene.lights.size(); i++) {
        std::visit(
                make_visitor(
                        [](const PointLight &light) { drawSphere(light.position, 0.01f, light.color); },
                        [](const SegmentLight &light) {
                            glPushAttrib(GL_ALL_ATTRIB_BITS);
                            glBegin(GL_LINES);
                            glColor3fv(glm::value_ptr(light.color0));
                            glVertex3fv(glm::value_ptr(light.endpoint0));
                            glColor3fv(glm::value_ptr(light.color1));
                            glVertex3fv(glm::value_ptr(light.endpoint1));
                            glEnd();
                            glPopAttrib();
                            drawSphere(light.endpoint0, 0.01f, light.color0);
                            drawSphere(light.endpoint1, 0.01f, light.color1);
                        },
                        [](const ParallelogramLight &light) {
                            glPushAttrib(GL_ALL_ATTRIB_BITS);
                            glBegin(GL_QUADS);
                            glColor3fv(glm::value_ptr(light.color0));
                            glVertex3fv(glm::value_ptr(light.v0));
                            glColor3fv(glm::value_ptr(light.color1));
                            glVertex3fv(glm::value_ptr(light.v0 + light.edge01));
                            glColor3fv(glm::value_ptr(light.color3));
                            glVertex3fv(glm::value_ptr(light.v0 + light.edge01 + light.edge02));
                            glColor3fv(glm::value_ptr(light.color2));
                            glVertex3fv(glm::value_ptr(light.v0 + light.edge02));
                            glEnd();
                            glPopAttrib();
                        },
                        [](auto) { /* any other type of light */ }),
                scene.lights[i]);
    }

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

void drawSceneOpenGL(const Scene &scene) {
    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    // Tell OpenGL where the lights are (so it nows how to shade surfaces in the scene).
    // This is only used in the rasterization view. OpenGL only supports point lights so
    // we replace segment/parallelogram lights by point lights.
    int i = 0;
    const auto enableLight = [&](const glm::vec3 &position, const glm::vec3 color) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4{position, 1};
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4{glm::clamp(color, 0.0f, 1.0f), 1.0f};
        const glm::vec4 zero4{0.0f, 0.0f, 0.0f, 1.0f};
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto &light: scene.lights) {
        std::visit(
                make_visitor(
                        [&](const PointLight &light) {
                            enableLight(light.position, light.color);
                        },
                        [&](const SegmentLight &light) {
                            // Approximate with two point lights: one at each endpoint.
                            enableLight(light.endpoint0, 0.5f * light.color0);
                            enableLight(light.endpoint1, 0.5f * light.color1);
                        },
                        [&](const ParallelogramLight &light) {
                            enableLight(light.v0, 0.25f * light.color0);
                            enableLight(light.v0 + light.edge01, 0.25f * light.color1);
                            enableLight(light.v0 + light.edge02, 0.25f * light.color2);
                            enableLight(light.v0 + light.edge01 + light.edge02, 0.25f * light.color3);
                        },
                        [](auto) { /* any other type of light */ }),
                light);
    }

    // Draw the scene and the ray (if any).
    drawScene(scene);
}
