#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <SDL3/SDL.h>

struct Vec3 {
    float x;
    float y;
    float z;
};

struct CubeMesh {
    std::array<Vec3, 8> vertices;
    std::array<std::uint16_t, 36> indices;
    std::array<std::uint16_t, 24> edges;
    std::array<std::uint16_t, 72> triangleEdges{};
};

struct CubePlacement {
    Vec3 position;
    float yawDegrees;
    float scale;
};

struct GroundPlaneMesh {
    std::array<Vec3, 4> vertices;
    std::array<std::uint16_t, 6> triangles;
    std::array<std::uint16_t, 8> edges;
    std::array<std::uint16_t, 12> triangleEdges{};
};

enum class DrawMode {
    Filled,
    Outline,
    Triangles
};

struct PlayerCamera {
    Vec3 position{0.0f, 1.6f, 6.0f};
    float yawRadians = 0.0f;
    float pitchRadians = 0.0f;
};

CubeMesh MakeCubeGeometry(float halfExtent) {
    CubeMesh mesh{
        {{
            {-halfExtent, -halfExtent, -halfExtent},
            { halfExtent, -halfExtent, -halfExtent},
            { halfExtent,  halfExtent, -halfExtent},
            {-halfExtent,  halfExtent, -halfExtent},
            {-halfExtent, -halfExtent,  halfExtent},
            { halfExtent, -halfExtent,  halfExtent},
            { halfExtent,  halfExtent,  halfExtent},
            {-halfExtent,  halfExtent,  halfExtent}
        }},
        {{
            4, 5, 6, 4, 6, 7,
            1, 0, 3, 1, 3, 2,
            0, 4, 7, 0, 7, 3,
            5, 1, 2, 5, 2, 6,
            3, 7, 6, 3, 6, 2,
            0, 1, 5, 0, 5, 4
        }},
        {{
            0, 1, 1, 2, 2, 3, 3, 0,
            4, 5, 5, 6, 6, 7, 7, 4,
            0, 4, 1, 5, 2, 6, 3, 7
        }}
    };
    for (size_t triangle = 0; triangle < mesh.indices.size(); triangle += 3) {
        const size_t edge = triangle * 2;
        const std::uint16_t a = mesh.indices[triangle];
        const std::uint16_t b = mesh.indices[triangle + 1];
        const std::uint16_t c = mesh.indices[triangle + 2];
        mesh.triangleEdges[edge] = a;
        mesh.triangleEdges[edge + 1] = b;
        mesh.triangleEdges[edge + 2] = b;
        mesh.triangleEdges[edge + 3] = c;
        mesh.triangleEdges[edge + 4] = c;
        mesh.triangleEdges[edge + 5] = a;
    }
    return mesh;
}

GroundPlaneMesh MakeGroundPlane(float halfExtent) {
    GroundPlaneMesh mesh{
        {{
            {-halfExtent, 0.0f, -halfExtent},
            { halfExtent, 0.0f, -halfExtent},
            { halfExtent, 0.0f,  halfExtent},
            {-halfExtent, 0.0f,  halfExtent}
        }},
        {{0, 2, 1, 0, 3, 2}},
        {{0, 1, 1, 2, 2, 3, 3, 0}}
    };
    for (size_t triangle = 0; triangle < mesh.triangles.size(); triangle += 3) {
        const size_t edge = triangle * 2;
        const std::uint16_t a = mesh.triangles[triangle];
        const std::uint16_t b = mesh.triangles[triangle + 1];
        const std::uint16_t c = mesh.triangles[triangle + 2];
        mesh.triangleEdges[edge] = a;
        mesh.triangleEdges[edge + 1] = b;
        mesh.triangleEdges[edge + 2] = b;
        mesh.triangleEdges[edge + 3] = c;
        mesh.triangleEdges[edge + 4] = c;
        mesh.triangleEdges[edge + 5] = a;
    }
    return mesh;
}

bool LoadMap(const std::string& path, std::vector<CubePlacement>& placements,
    float& groundHeight, float& groundHalfExtent, std::string& error) {
    std::ifstream mapFile(path);
    if (!mapFile) {
        error = "could not open map file: " + path;
        return false;
    }

    std::vector<CubePlacement> loadedPlacements;
    float loadedGroundHeight = -1.0f;
    float loadedGroundHalfExtent = 50.0f;
    bool groundDefined = false;
    std::string line;
    size_t lineNumber = 0;
    while (std::getline(mapFile, line)) {
        ++lineNumber;
        std::istringstream parser(line);
        std::string objectType;
        if (!(parser >> objectType) || objectType.front() == '#') {
            continue;
        }
        if (objectType == "ground") {
            if (groundDefined) {
                error = "line " + std::to_string(lineNumber) + ": ground is already defined";
                return false;
            }
            if (!(parser >> loadedGroundHeight >> loadedGroundHalfExtent)) {
                error = "line " + std::to_string(lineNumber) + ": expected ground y halfExtent";
                return false;
            }
            std::string extra;
            if (parser >> extra && extra.front() != '#') {
                error = "line " + std::to_string(lineNumber) + ": unexpected value '" + extra + "'";
                return false;
            }
            if (!std::isfinite(loadedGroundHeight) || !std::isfinite(loadedGroundHalfExtent) ||
                loadedGroundHalfExtent <= 0.0f) {
                error = "line " + std::to_string(lineNumber) + ": ground height must be finite and halfExtent must be positive";
                return false;
            }
            groundDefined = true;
            continue;
        }
        if (objectType != "cube") {
            error = "line " + std::to_string(lineNumber) + ": unsupported object '" + objectType + "'";
            return false;
        }

        CubePlacement placement{};
        if (!(parser >> placement.position.x >> placement.position.y >> placement.position.z
                     >> placement.yawDegrees >> placement.scale)) {
            error = "line " + std::to_string(lineNumber) + ": expected cube x y z yawDegrees scale";
            return false;
        }

        std::string extra;
        if (parser >> extra && extra.front() != '#') {
            error = "line " + std::to_string(lineNumber) + ": unexpected value '" + extra + "'";
            return false;
        }
        if (!std::isfinite(placement.position.x) || !std::isfinite(placement.position.y) ||
            !std::isfinite(placement.position.z) || !std::isfinite(placement.yawDegrees) ||
            !std::isfinite(placement.scale) || placement.scale <= 0.0f) {
            error = "line " + std::to_string(lineNumber) + ": coordinates and yaw must be finite; scale must be positive";
            return false;
        }
        loadedPlacements.push_back(placement);
    }

    if (loadedPlacements.empty()) {
        error = "map contains no cube placements: " + path;
        return false;
    }
    placements = loadedPlacements;
    groundHeight = loadedGroundHeight;
    groundHalfExtent = loadedGroundHalfExtent;
    return true;
}

struct Mat4 {
    std::array<float, 16> values{};
};

Mat4 Multiply(const Mat4& left, const Mat4& right) {
    Mat4 result{};
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int inner = 0; inner < 4; ++inner) {
                result.values[column * 4 + row] +=
                    left.values[inner * 4 + row] * right.values[column * 4 + inner];
            }
        }
    }
    return result;
}

Mat4 MakeRotationX(float angleRadians) {
    Mat4 result{};
    const float cosine = std::cos(angleRadians);
    const float sine = std::sin(angleRadians);
    result.values[0] = 1.0f;
    result.values[5] = cosine;
    result.values[6] = sine;
    result.values[9] = -sine;
    result.values[10] = cosine;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakeRotationY(float angleRadians) {
    Mat4 result{};
    const float cosine = std::cos(angleRadians);
    const float sine = std::sin(angleRadians);
    result.values[0] = cosine;
    result.values[2] = -sine;
    result.values[5] = 1.0f;
    result.values[8] = sine;
    result.values[10] = cosine;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakeScale(float uniformScale) {
    Mat4 result{};
    result.values[0] = uniformScale;
    result.values[5] = uniformScale;
    result.values[10] = uniformScale;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakeTranslation(float x, float y, float z) {
    Mat4 result{};
    result.values[0] = 1.0f;
    result.values[5] = 1.0f;
    result.values[10] = 1.0f;
    result.values[12] = x;
    result.values[13] = y;
    result.values[14] = z;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakeViewTranslation(float x, float y, float z) {
    Mat4 result{};
    result.values[0] = 1.0f;
    result.values[5] = 1.0f;
    result.values[10] = 1.0f;
    result.values[12] = x;
    result.values[13] = y;
    result.values[14] = z;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakePerspective(float verticalFovRadians, float aspectRatio, float nearPlane, float farPlane) {
    const float focalLength = 1.0f / std::tan(verticalFovRadians * 0.5f);
    Mat4 result{};
    result.values[0] = focalLength / aspectRatio;
    result.values[5] = focalLength;
    result.values[10] = farPlane / (nearPlane - farPlane);
    result.values[11] = -1.0f;
    result.values[14] = (nearPlane * farPlane) / (nearPlane - farPlane);
    return result;
}

Mat4 MakeCameraView(const PlayerCamera& camera) {
    const Mat4 inverseYaw = MakeRotationY(camera.yawRadians);
    const Mat4 inversePitch = MakeRotationX(-camera.pitchRadians);
    const Mat4 inversePosition = MakeViewTranslation(
        -camera.position.x, -camera.position.y, -camera.position.z);
    return Multiply(Multiply(inversePitch, inverseYaw), inversePosition);
}

Mat4 MakeViewProjection(const PlayerCamera& camera, float aspectRatio) {
    constexpr float pi = 3.14159265358979323846f;
    const Mat4 view = MakeCameraView(camera);
    const Mat4 projection = MakePerspective(pi / 3.0f, aspectRatio, 0.1f, 100.0f);
    return Multiply(projection, view);
}

void ApplyMouseLook(PlayerCamera& camera, float deltaX, float deltaY) {
    constexpr float sensitivity = 0.0025f;
    constexpr float maxPitch = 1.55334306f;
    camera.yawRadians += deltaX * sensitivity;
    camera.pitchRadians = std::clamp(
        camera.pitchRadians - deltaY * sensitivity, -maxPitch, maxPitch);
}

void MovePlayer(PlayerCamera& camera, const bool* keyboardState, float deltaSeconds) {
    const float forwardX = std::sin(camera.yawRadians);
    const float forwardZ = -std::cos(camera.yawRadians);
    const float rightX = std::cos(camera.yawRadians);
    const float rightZ = std::sin(camera.yawRadians);
    float moveX = 0.0f;
    float moveZ = 0.0f;

    if (keyboardState[SDL_SCANCODE_W]) {
        moveX += forwardX;
        moveZ += forwardZ;
    }
    if (keyboardState[SDL_SCANCODE_S]) {
        moveX -= forwardX;
        moveZ -= forwardZ;
    }
    if (keyboardState[SDL_SCANCODE_D]) {
        moveX += rightX;
        moveZ += rightZ;
    }
    if (keyboardState[SDL_SCANCODE_A]) {
        moveX -= rightX;
        moveZ -= rightZ;
    }

    const float moveLength = std::sqrt(moveX * moveX + moveZ * moveZ);
    if (moveLength > 0.0f) {
        const float baseSpeed = keyboardState[SDL_SCANCODE_LSHIFT] ? 9.0f : 4.5f;
        const float distance = baseSpeed * deltaSeconds / moveLength;
        camera.position.x += moveX * distance;
        camera.position.z += moveZ * distance;
    }
}

Mat4 MakeModelMatrix(const CubePlacement& placement) {
    constexpr float pi = 3.14159265358979323846f;
    const float yawRadians = placement.yawDegrees * (pi / 180.0f);
    const Mat4 scale = MakeScale(placement.scale);
    const Mat4 rotation = MakeRotationY(yawRadians);
    const Mat4 translation = MakeTranslation(
        placement.position.x, placement.position.y, placement.position.z);
    return Multiply(translation, Multiply(rotation, scale));
}

SDL_GPUShader* LoadShader(SDL_GPUDevice* device, const char* path, SDL_GPUShaderStage stage) {
    size_t codeSize = 0;
    void* code = SDL_LoadFile(path, &codeSize);
    if (!code) {
        std::cerr << "SDL_LoadFile failed for " << path << ": " << SDL_GetError() << '\n';
        return nullptr;
    }

    SDL_GPUShaderCreateInfo createInfo{};
    createInfo.code_size = codeSize;
    createInfo.code = static_cast<const Uint8*>(code);
    createInfo.entrypoint = "main";
    createInfo.format = SDL_GPU_SHADERFORMAT_SPIRV;
    createInfo.stage = stage;
    createInfo.num_uniform_buffers = stage == SDL_GPU_SHADERSTAGE_VERTEX ? 1 : 0;
    SDL_GPUShader* shader = SDL_CreateGPUShader(device, &createInfo);
    SDL_free(code);
    if (!shader) {
        std::cerr << "SDL_CreateGPUShader failed: " << SDL_GetError() << '\n';
    }
    return shader;
}

SDL_GPUBuffer* CreateBufferFromData(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
        const void* data, Uint32 size) {
    SDL_GPUBufferCreateInfo bufferInfo{};
    bufferInfo.usage = usage;
    bufferInfo.size = size;
    SDL_GPUBuffer* buffer = SDL_CreateGPUBuffer(device, &bufferInfo);
    if (!buffer) {
        std::cerr << "SDL_CreateGPUBuffer failed: " << SDL_GetError() << '\n';
        return nullptr;
    }

    SDL_GPUTransferBufferCreateInfo transferInfo{};
    transferInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    transferInfo.size = size;
    SDL_GPUTransferBuffer* transferBuffer = SDL_CreateGPUTransferBuffer(device, &transferInfo);
    if (!transferBuffer) {
        std::cerr << "SDL_CreateGPUTransferBuffer failed: " << SDL_GetError() << '\n';
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }

    void* mappedData = SDL_MapGPUTransferBuffer(device, transferBuffer, false);
    if (!mappedData) {
        std::cerr << "SDL_MapGPUTransferBuffer failed: " << SDL_GetError() << '\n';
        SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }
    std::memcpy(mappedData, data, size);
    SDL_UnmapGPUTransferBuffer(device, transferBuffer);

    SDL_GPUCommandBuffer* commandBuffer = SDL_AcquireGPUCommandBuffer(device);
    if (!commandBuffer) {
        std::cerr << "SDL_AcquireGPUCommandBuffer failed: " << SDL_GetError() << '\n';
        SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }

    SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(commandBuffer);
    const SDL_GPUTransferBufferLocation source{transferBuffer, 0};
    const SDL_GPUBufferRegion destination{buffer, 0, size};
    SDL_UploadToGPUBuffer(copyPass, &source, &destination, false);
    SDL_EndGPUCopyPass(copyPass);
    if (!SDL_SubmitGPUCommandBuffer(commandBuffer)) {
        std::cerr << "SDL_SubmitGPUCommandBuffer failed: " << SDL_GetError() << '\n';
        SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }
    SDL_WaitForGPUIdle(device);
    SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
    return buffer;
}

SDL_GPUTexture* CreateDepthTexture(SDL_GPUDevice* device, Uint32 width, Uint32 height) {
    SDL_GPUTextureCreateInfo textureInfo{};
    textureInfo.type = SDL_GPU_TEXTURETYPE_2D;
    textureInfo.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    textureInfo.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    textureInfo.width = width;
    textureInfo.height = height;
    textureInfo.layer_count_or_depth = 1;
    textureInfo.num_levels = 1;
    textureInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
    SDL_GPUTexture* texture = SDL_CreateGPUTexture(device, &textureInfo);
    if (!texture) {
        std::cerr << "SDL_CreateGPUTexture failed: " << SDL_GetError() << '\n';
    }
    return texture;
}

SDL_GPUGraphicsPipeline* CreateCubePipeline(SDL_GPUDevice* device, SDL_Window* window,
    SDL_GPUShader* vertexShader, SDL_GPUShader* fragmentShader,
    SDL_GPUPrimitiveType primitiveType) {
    SDL_GPUVertexBufferDescription vertexBufferDescription{};
    vertexBufferDescription.slot = 0;
    vertexBufferDescription.pitch = sizeof(Vec3);
    vertexBufferDescription.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    const SDL_GPUVertexAttribute vertexAttribute{
        0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, 0};
    SDL_GPUColorTargetDescription colorTargetDescription{};
    colorTargetDescription.format = SDL_GetGPUSwapchainTextureFormat(device, window);

    SDL_GPUGraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo.vertex_shader = vertexShader;
    pipelineInfo.fragment_shader = fragmentShader;
    pipelineInfo.vertex_input_state = {&vertexBufferDescription, 1, &vertexAttribute, 1};
    pipelineInfo.primitive_type = primitiveType;
    pipelineInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    pipelineInfo.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
    pipelineInfo.rasterizer_state.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    pipelineInfo.rasterizer_state.enable_depth_clip = true;
    pipelineInfo.multisample_state.sample_count = SDL_GPU_SAMPLECOUNT_1;
    pipelineInfo.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_LESS;
    pipelineInfo.depth_stencil_state.enable_depth_test = true;
    pipelineInfo.depth_stencil_state.enable_depth_write = true;
    pipelineInfo.target_info.color_target_descriptions = &colorTargetDescription;
    pipelineInfo.target_info.num_color_targets = 1;
    pipelineInfo.target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    pipelineInfo.target_info.has_depth_stencil_target = true;

    SDL_GPUGraphicsPipeline* pipeline = SDL_CreateGPUGraphicsPipeline(device, &pipelineInfo);
    if (!pipeline) {
        std::cerr << "SDL_CreateGPUGraphicsPipeline failed: " << SDL_GetError() << '\n';
    }
    return pipeline;
}

void DrawCube(SDL_GPUCommandBuffer* commandBuffer, SDL_GPUTexture* swapchainTexture,
        SDL_GPUTexture* depthTexture, Uint32 width, Uint32 height,
        SDL_GPUGraphicsPipeline* pipeline, SDL_GPUBuffer* vertexBuffer,
    SDL_GPUBuffer* indexBuffer, Uint32 indexCount,
    const std::vector<CubePlacement>& placements, const PlayerCamera& camera,
    SDL_GPUBuffer* groundVertexBuffer, SDL_GPUBuffer* groundIndexBuffer,
    Uint32 groundIndexCount, float groundHeight,
    SDL_GPUGraphicsPipeline* groundPipeline) {
    SDL_GPUColorTargetInfo colorTarget{};
    colorTarget.texture = swapchainTexture;
    colorTarget.clear_color = {0.20f, 0.55f, 1.00f, 1.0f};
    colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;

    SDL_GPUDepthStencilTargetInfo depthTarget{};
    depthTarget.texture = depthTexture;
    depthTarget.clear_depth = 1.0f;
    depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass* renderPass = SDL_BeginGPURenderPass(
        commandBuffer, &colorTarget, 1, &depthTarget);
    const SDL_GPUViewport viewport{
        0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height), 0.0f, 1.0f};
    SDL_SetGPUViewport(renderPass, &viewport);
    SDL_BindGPUGraphicsPipeline(renderPass, pipeline);
    const SDL_GPUBufferBinding vertexBinding{vertexBuffer, 0};
    const SDL_GPUBufferBinding indexBinding{indexBuffer, 0};
    SDL_BindGPUVertexBuffers(renderPass, 0, &vertexBinding, 1);
    SDL_BindGPUIndexBuffer(renderPass, &indexBinding, SDL_GPU_INDEXELEMENTSIZE_16BIT);

    const Mat4 viewProjection = MakeViewProjection(camera,
        static_cast<float>(width) / static_cast<float>(height));
    for (const CubePlacement& placement : placements) {
        const Mat4 modelViewProjection = Multiply(viewProjection, MakeModelMatrix(placement));
        SDL_PushGPUVertexUniformData(commandBuffer, 0, modelViewProjection.values.data(),
            sizeof(modelViewProjection.values));
        SDL_DrawGPUIndexedPrimitives(renderPass, indexCount, 1, 0, 0, 0);
    }

    const SDL_GPUBufferBinding groundVertexBinding{groundVertexBuffer, 0};
    const SDL_GPUBufferBinding groundIndexBinding{groundIndexBuffer, 0};
    SDL_BindGPUGraphicsPipeline(renderPass, groundPipeline);
    SDL_BindGPUVertexBuffers(renderPass, 0, &groundVertexBinding, 1);
    SDL_BindGPUIndexBuffer(renderPass, &groundIndexBinding, SDL_GPU_INDEXELEMENTSIZE_16BIT);
    const Mat4 groundModel = MakeTranslation(0.0f, groundHeight, 0.0f);
    const Mat4 groundMvp = Multiply(viewProjection, groundModel);
    SDL_PushGPUVertexUniformData(commandBuffer, 0, groundMvp.values.data(),
        sizeof(groundMvp.values));
    SDL_DrawGPUIndexedPrimitives(renderPass, groundIndexCount, 1, 0, 0, 0);
    SDL_EndGPURenderPass(renderPass);
}

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_GPUDevice* device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, false, nullptr);
    if (!device) {
        std::cerr << "SDL_CreateGPUDevice failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "SDL3D Engine", 1280, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    bool windowClaimed = false;
    SDL_GPUShader* vertexShader = nullptr;
    SDL_GPUShader* fragmentShader = nullptr;
    SDL_GPUShader* groundFragmentShader = nullptr;
    SDL_GPUGraphicsPipeline* filledPipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlinePipeline = nullptr;
    SDL_GPUGraphicsPipeline* filledGroundPipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlineGroundPipeline = nullptr;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUBuffer* indexBuffer = nullptr;
    SDL_GPUBuffer* edgeBuffer = nullptr;
    SDL_GPUBuffer* triangleEdgeBuffer = nullptr;
    SDL_GPUBuffer* groundVertexBuffer = nullptr;
    SDL_GPUBuffer* groundTriangleBuffer = nullptr;
    SDL_GPUBuffer* groundEdgeBuffer = nullptr;
    SDL_GPUBuffer* groundTriangleEdgeBuffer = nullptr;
    SDL_GPUTexture* depthTexture = nullptr;
    auto cleanup = [&]() {
        SDL_WaitForGPUIdle(device);
        if (depthTexture) SDL_ReleaseGPUTexture(device, depthTexture);
        if (vertexBuffer) SDL_ReleaseGPUBuffer(device, vertexBuffer);
        if (indexBuffer) SDL_ReleaseGPUBuffer(device, indexBuffer);
        if (edgeBuffer) SDL_ReleaseGPUBuffer(device, edgeBuffer);
        if (triangleEdgeBuffer) SDL_ReleaseGPUBuffer(device, triangleEdgeBuffer);
        if (groundVertexBuffer) SDL_ReleaseGPUBuffer(device, groundVertexBuffer);
        if (groundTriangleBuffer) SDL_ReleaseGPUBuffer(device, groundTriangleBuffer);
        if (groundEdgeBuffer) SDL_ReleaseGPUBuffer(device, groundEdgeBuffer);
        if (groundTriangleEdgeBuffer) SDL_ReleaseGPUBuffer(device, groundTriangleEdgeBuffer);
        if (filledPipeline) SDL_ReleaseGPUGraphicsPipeline(device, filledPipeline);
        if (outlinePipeline) SDL_ReleaseGPUGraphicsPipeline(device, outlinePipeline);
        if (filledGroundPipeline) SDL_ReleaseGPUGraphicsPipeline(device, filledGroundPipeline);
        if (outlineGroundPipeline) SDL_ReleaseGPUGraphicsPipeline(device, outlineGroundPipeline);
        if (vertexShader) SDL_ReleaseGPUShader(device, vertexShader);
        if (fragmentShader) SDL_ReleaseGPUShader(device, fragmentShader);
        if (groundFragmentShader) SDL_ReleaseGPUShader(device, groundFragmentShader);
        if (windowClaimed) SDL_ReleaseWindowFromGPUDevice(device, window);
        if (window) SDL_DestroyWindow(window);
        SDL_DestroyGPUDevice(device);
        SDL_Quit();
    };

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        cleanup();
        return 1;
    }
    if (!SDL_ClaimWindowForGPUDevice(device, window)) {
        std::cerr << "SDL_ClaimWindowForGPUDevice failed: " << SDL_GetError() << '\n';
        cleanup();
        return 1;
    }
    windowClaimed = true;

    std::vector<CubePlacement> placements;
    float groundHeight = -1.0f;
    float groundHalfExtent = 50.0f;
    std::string mapError;
    if (!LoadMap("maps/test.map", placements, groundHeight, groundHalfExtent, mapError)) {
        std::cerr << "Map load failed: " << mapError << '\n';
        cleanup();
        return 1;
    }
    std::cout << "Loaded " << placements.size() << " cube(s) from maps/test.map\n";

    vertexShader = LoadShader(device, "shaders/cube.vert.spv", SDL_GPU_SHADERSTAGE_VERTEX);
    fragmentShader = LoadShader(device, "shaders/cube.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT);
    groundFragmentShader = LoadShader(device, "shaders/ground.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT);
    const CubeMesh cube = MakeCubeGeometry(1.0f);
    const GroundPlaneMesh ground = MakeGroundPlane(groundHalfExtent);
    vertexBuffer = CreateBufferFromData(
        device, SDL_GPU_BUFFERUSAGE_VERTEX, cube.vertices.data(), sizeof(cube.vertices));
    indexBuffer = CreateBufferFromData(
        device, SDL_GPU_BUFFERUSAGE_INDEX, cube.indices.data(), sizeof(cube.indices));
    edgeBuffer = CreateBufferFromData(
        device, SDL_GPU_BUFFERUSAGE_INDEX, cube.edges.data(), sizeof(cube.edges));
    triangleEdgeBuffer = CreateBufferFromData(
        device, SDL_GPU_BUFFERUSAGE_INDEX, cube.triangleEdges.data(), sizeof(cube.triangleEdges));
    groundVertexBuffer = CreateBufferFromData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
        ground.vertices.data(), sizeof(ground.vertices));
    groundTriangleBuffer = CreateBufferFromData(device, SDL_GPU_BUFFERUSAGE_INDEX,
        ground.triangles.data(), sizeof(ground.triangles));
    groundEdgeBuffer = CreateBufferFromData(device, SDL_GPU_BUFFERUSAGE_INDEX,
        ground.edges.data(), sizeof(ground.edges));
    groundTriangleEdgeBuffer = CreateBufferFromData(device, SDL_GPU_BUFFERUSAGE_INDEX,
        ground.triangleEdges.data(), sizeof(ground.triangleEdges));
    if (!vertexShader || !fragmentShader || !vertexBuffer || !indexBuffer || !edgeBuffer ||
        !triangleEdgeBuffer || !groundVertexBuffer || !groundTriangleBuffer ||
        !groundEdgeBuffer || !groundTriangleEdgeBuffer) {
        cleanup();
        return 1;
    }

    filledPipeline = CreateCubePipeline(device, window, vertexShader, fragmentShader,
        SDL_GPU_PRIMITIVETYPE_TRIANGLELIST);
    outlinePipeline = CreateCubePipeline(device, window, vertexShader, fragmentShader,
        SDL_GPU_PRIMITIVETYPE_LINELIST);
    filledGroundPipeline = CreateCubePipeline(device, window, vertexShader, groundFragmentShader,
        SDL_GPU_PRIMITIVETYPE_TRIANGLELIST);
    outlineGroundPipeline = CreateCubePipeline(device, window, vertexShader, groundFragmentShader,
        SDL_GPU_PRIMITIVETYPE_LINELIST);
    if (!filledPipeline || !outlinePipeline || !filledGroundPipeline || !outlineGroundPipeline) {
        cleanup();
        return 1;
    }

    bool running = true;
    DrawMode drawMode = DrawMode::Filled;
    bool mouseCaptured = SDL_SetWindowRelativeMouseMode(window, true);
    if (!mouseCaptured) {
        std::cerr << "Mouse capture unavailable: " << SDL_GetError() << '\n';
    }
    PlayerCamera camera{};
    Uint32 depthWidth = 0;
    Uint32 depthHeight = 0;
    Uint64 previousCounter = SDL_GetPerformanceCounter();
    const Uint64 counterFrequency = SDL_GetPerformanceFrequency();
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT ||
                (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                running = false;
            } else if (event.type == SDL_EVENT_MOUSE_MOTION && mouseCaptured) {
                ApplyMouseLook(camera, static_cast<float>(event.motion.xrel),
                    static_cast<float>(event.motion.yrel));
            } else if (event.type == SDL_EVENT_KEY_DOWN &&
                       event.key.key == SDLK_X && !event.key.repeat) {
                if (drawMode == DrawMode::Filled) {
                    drawMode = DrawMode::Outline;
                } else if (drawMode == DrawMode::Outline) {
                    drawMode = DrawMode::Triangles;
                } else {
                    drawMode = DrawMode::Filled;
                }
                const char* modeName = drawMode == DrawMode::Filled ? "filled" :
                    drawMode == DrawMode::Outline ? "outline" : "triangles";
                std::cout << "Drawing mode: " << modeName << '\n';
            } else if (event.type == SDL_EVENT_KEY_DOWN &&
                       event.key.key == SDLK_TAB && !event.key.repeat) {
                mouseCaptured = !mouseCaptured;
                if (!SDL_SetWindowRelativeMouseMode(window, mouseCaptured)) {
                    std::cerr << "Mouse capture change failed: " << SDL_GetError() << '\n';
                    mouseCaptured = !mouseCaptured;
                }
            }
        }
        if (!running) break;

        const Uint64 currentCounter = SDL_GetPerformanceCounter();
        const float deltaSeconds = std::min(
            static_cast<float>(currentCounter - previousCounter) /
                static_cast<float>(counterFrequency),
            0.05f);
        previousCounter = currentCounter;
        int keyCount = 0;
        const bool* keyboardState = SDL_GetKeyboardState(&keyCount);
        if (keyboardState) {
            MovePlayer(camera, keyboardState, deltaSeconds);
        }

        int pixelWidth = 0;
        int pixelHeight = 0;
        SDL_GetWindowSizeInPixels(window, &pixelWidth, &pixelHeight);
        if (pixelWidth <= 0 || pixelHeight <= 0) {
            SDL_Delay(16);
            continue;
        }
        if (!depthTexture || depthWidth != static_cast<Uint32>(pixelWidth) ||
            depthHeight != static_cast<Uint32>(pixelHeight)) {
            SDL_GPUTexture* resizedDepthTexture = CreateDepthTexture(
                device, static_cast<Uint32>(pixelWidth), static_cast<Uint32>(pixelHeight));
            if (!resizedDepthTexture) {
                running = false;
                continue;
            }
            if (depthTexture) {
                SDL_WaitForGPUIdle(device);
                SDL_ReleaseGPUTexture(device, depthTexture);
            }
            depthTexture = resizedDepthTexture;
            depthWidth = static_cast<Uint32>(pixelWidth);
            depthHeight = static_cast<Uint32>(pixelHeight);
        }

        SDL_GPUCommandBuffer* commandBuffer = SDL_AcquireGPUCommandBuffer(device);
        if (!commandBuffer) {
            std::cerr << "SDL_AcquireGPUCommandBuffer failed: " << SDL_GetError() << '\n';
            running = false;
            continue;
        }

        SDL_GPUTexture* swapchainTexture = nullptr;
        Uint32 swapchainWidth = 0;
        Uint32 swapchainHeight = 0;
        if (!SDL_WaitAndAcquireGPUSwapchainTexture(
                commandBuffer, window, &swapchainTexture, &swapchainWidth, &swapchainHeight)) {
            std::cerr << "SDL_WaitAndAcquireGPUSwapchainTexture failed: " << SDL_GetError() << '\n';
            SDL_CancelGPUCommandBuffer(commandBuffer);
            running = false;
            continue;
        }

        if (swapchainTexture && swapchainWidth == depthWidth && swapchainHeight == depthHeight) {
            const bool filled = drawMode == DrawMode::Filled;
            const bool triangles = drawMode == DrawMode::Triangles;
            SDL_GPUBuffer* selectedIndexBuffer = filled ? indexBuffer :
                triangles ? triangleEdgeBuffer : edgeBuffer;
            const Uint32 selectedIndexCount = filled
                ? static_cast<Uint32>(cube.indices.size())
                : triangles ? static_cast<Uint32>(cube.triangleEdges.size())
                    : static_cast<Uint32>(cube.edges.size());
            SDL_GPUBuffer* selectedGroundIndexBuffer = filled ? groundTriangleBuffer :
                triangles ? groundTriangleEdgeBuffer : groundEdgeBuffer;
            const Uint32 selectedGroundIndexCount = filled
                ? static_cast<Uint32>(ground.triangles.size())
                : triangles ? static_cast<Uint32>(ground.triangleEdges.size())
                    : static_cast<Uint32>(ground.edges.size());
            DrawCube(commandBuffer, swapchainTexture, depthTexture, swapchainWidth,
                swapchainHeight, filled ? filledPipeline : outlinePipeline, vertexBuffer,
                selectedIndexBuffer, selectedIndexCount, placements, camera, groundVertexBuffer,
                selectedGroundIndexBuffer, selectedGroundIndexCount, groundHeight,
                filled ? filledGroundPipeline : outlineGroundPipeline);
        }

        if (!SDL_SubmitGPUCommandBuffer(commandBuffer)) {
            std::cerr << "SDL_SubmitGPUCommandBuffer failed: " << SDL_GetError() << '\n';
            running = false;
        }
    }

    cleanup();
    return 0;
}
