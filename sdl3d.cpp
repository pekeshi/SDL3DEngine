// SDL3D Engine - Step-by-Step Training Build
//
// Each section builds on the previous one:
//   1. Data structures and meshes
//   2. Text-file map loading
//   3. Matrix math and camera movement
//   4. SDL GPU resource setup
//   5. Main rendering loop

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

struct Vec3 {
    float x;
    float y;
    float z;
};

struct CubePlacement {
    Vec3 position;
    float yawDegrees;
    float scale;
};

struct PlayerCamera {
    Vec3 position{0.0f, 1.6f, 6.0f};
    float yawRadians = 0.0f;
    float pitchRadians = 0.0f;
    float eyeHeight = 1.6f;
    float radius = 0.35f;
    float height = 1.8f;
    float verticalVelocity = 0.0f;
    bool grounded = true;
    bool jumpRequested = false;
};

struct CubeMesh {
    std::array<Vec3, 8> vertices;
    std::array<std::uint16_t, 36> triangles;
};

struct GroundMesh {
    std::array<Vec3, 4> vertices;
    std::array<std::uint16_t, 6> triangles;
};

// -----------------------------------------------------------------------------
// Step 1: Build mesh data.
// -----------------------------------------------------------------------------
CubeMesh MakeCubeMesh() {
    CubeMesh mesh{
        {{
            {-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f},
            {1.0f,  1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f},
            {-1.0f, -1.0f,  1.0f}, {1.0f, -1.0f,  1.0f},
            {1.0f,  1.0f,  1.0f}, {-1.0f, 1.0f,  1.0f}
        }},
        {{
            4, 5, 6, 4, 6, 7,
            1, 0, 3, 1, 3, 2,
            0, 4, 7, 0, 7, 3,
            5, 1, 2, 5, 2, 6,
            3, 7, 6, 3, 6, 2,
            0, 1, 5, 0, 5, 4
        }}
    };
    return mesh;
}

GroundMesh MakeGroundMesh(float halfExtent) {
    GroundMesh mesh{
        {{
            {-halfExtent, 0.0f, -halfExtent},
            { halfExtent, 0.0f, -halfExtent},
            { halfExtent, 0.0f,  halfExtent},
            {-halfExtent, 0.0f,  halfExtent}
        }},
        {{0, 2, 1, 0, 3, 2}}
    };
    return mesh;
}

// -----------------------------------------------------------------------------
// Step 2: Read map/test.map into cube placements.
// -----------------------------------------------------------------------------
bool LoadMap(const std::string& path,
             std::vector<CubePlacement>& placements,
             float& groundHeight,
             float& groundHalfExtent,
             std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "could not open " + path;
        return false;
    }

    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        std::istringstream parser(line);
        std::string type;
        if (!(parser >> type) || type.front() == '#') continue;

        if (type == "ground") {
            if (!(parser >> groundHeight >> groundHalfExtent)) {
                error = "invalid ground on line " + std::to_string(lineNumber);
                return false;
            }
        } else if (type == "cube") {
            CubePlacement cube{};
            if (!(parser >> cube.position.x >> cube.position.y >> cube.position.z
                         >> cube.yawDegrees >> cube.scale)) {
                error = "invalid cube on line " + std::to_string(lineNumber);
                return false;
            }
            placements.push_back(cube);
        } else {
            error = "unknown object '" + type + "' on line " +
                    std::to_string(lineNumber);
            return false;
        }
    }

    if (placements.empty()) {
        error = "map has no cubes";
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// Step 3: Matrix math.  Matrices are stored in column-major float arrays.
// -----------------------------------------------------------------------------
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

Mat4 MakeScale(float scale) {
    Mat4 result{};
    result.values[0] = scale;
    result.values[5] = scale;
    result.values[10] = scale;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakeRotationY(float radians) {
    Mat4 result{};
    const float cosine = std::cos(radians);
    const float sine = std::sin(radians);
    result.values[0] = cosine;
    result.values[2] = -sine;
    result.values[5] = 1.0f;
    result.values[8] = sine;
    result.values[10] = cosine;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakeRotationX(float radians) {
    Mat4 result{};
    const float cosine = std::cos(radians);
    const float sine = std::sin(radians);
    result.values[0] = 1.0f;
    result.values[5] = cosine;
    result.values[6] = sine;
    result.values[9] = -sine;
    result.values[10] = cosine;
    result.values[15] = 1.0f;
    return result;
}

Mat4 MakePerspective(float fovRadians, float aspect, float nearPlane, float farPlane) {
    Mat4 result{};
    const float focalLength = 1.0f / std::tan(fovRadians * 0.5f);
    result.values[0] = focalLength / aspect;
    result.values[5] = focalLength;
    result.values[10] = farPlane / (nearPlane - farPlane);
    result.values[11] = -1.0f;
    result.values[14] = (nearPlane * farPlane) / (nearPlane - farPlane);
    return result;
}

Mat4 MakeView(const PlayerCamera& camera) {
    const Mat4 yaw = MakeRotationY(camera.yawRadians);
    const Mat4 pitch = MakeRotationX(-camera.pitchRadians);
    const Mat4 position = MakeTranslation(
        -camera.position.x, -camera.position.y, -camera.position.z);
    return Multiply(Multiply(pitch, yaw), position);
}

Mat4 MakeModel(const CubePlacement& cube) {
    constexpr float pi = 3.14159265358979323846f;
    const Mat4 scale = MakeScale(cube.scale);
    const Mat4 rotation = MakeRotationY(cube.yawDegrees * pi / 180.0f);
    const Mat4 position = MakeTranslation(
        cube.position.x, cube.position.y, cube.position.z);
    return Multiply(position, Multiply(rotation, scale));
}

// -----------------------------------------------------------------------------
// Step 4: Camera input and simple jumping/ground support.
// -----------------------------------------------------------------------------
void ApplyMouseLook(PlayerCamera& camera, float deltaX, float deltaY) {
    constexpr float sensitivity = 0.0025f;
    constexpr float pitchLimit = 1.55334306f;
    camera.yawRadians += deltaX * sensitivity;
    camera.pitchRadians = std::clamp(
        camera.pitchRadians - deltaY * sensitivity, -pitchLimit, pitchLimit);
}

bool PointInsideExpandedCubeXZ(const Vec3& point,
                              const CubePlacement& cube) {
    constexpr float pi = 3.14159265358979323846f;
    const float angle = cube.yawDegrees * pi / 180.0f;
    const float cosine = std::cos(-angle);
    const float sine = std::sin(-angle);
    const float deltaX = point.x - cube.position.x;
    const float deltaZ = point.z - cube.position.z;
    const float localX = cosine * deltaX - sine * deltaZ;
    const float localZ = sine * deltaX + cosine * deltaZ;
    return std::abs(localX) <= cube.scale + 0.35f &&
           std::abs(localZ) <= cube.scale + 0.35f;
}

bool HorizontalCubeCollision(const PlayerCamera& camera,
                             const std::vector<CubePlacement>& cubes) {
    const float playerBottom = camera.position.y - camera.eyeHeight;
    const float playerTop = playerBottom + camera.height;

    for (const CubePlacement& cube : cubes) {
        const float cubeBottom = cube.position.y - cube.scale;
        const float cubeTop = cube.position.y + cube.scale;
        if (playerTop <= cubeBottom || playerBottom >= cubeTop) continue;

        constexpr float pi = 3.14159265358979323846f;
        const float angle = cube.yawDegrees * pi / 180.0f;
        const float cosine = std::cos(-angle);
        const float sine = std::sin(-angle);
        const float deltaX = camera.position.x - cube.position.x;
        const float deltaZ = camera.position.z - cube.position.z;
        const float localX = cosine * deltaX - sine * deltaZ;
        const float localZ = sine * deltaX + cosine * deltaZ;

        const float closestX = std::clamp(localX,
            -cube.scale, cube.scale);
        const float closestZ = std::clamp(localZ,
            -cube.scale, cube.scale);
        const float differenceX = localX - closestX;
        const float differenceZ = localZ - closestZ;
        if (differenceX * differenceX + differenceZ * differenceZ
            < camera.radius * camera.radius) {
            return true;
        }
    }
    return false;
}

void MovePlayer(PlayerCamera& camera, const bool* keys, float dt,
                const std::vector<CubePlacement>& cubes,
                float groundHeight, float groundHalfExtent) {
    const float forwardX = std::sin(camera.yawRadians);
    const float forwardZ = -std::cos(camera.yawRadians);
    const float rightX = std::cos(camera.yawRadians);
    const float rightZ = std::sin(camera.yawRadians);

    float moveX = 0.0f;
    float moveZ = 0.0f;
    if (keys[SDL_SCANCODE_W]) { moveX += forwardX; moveZ += forwardZ; }
    if (keys[SDL_SCANCODE_S]) { moveX -= forwardX; moveZ -= forwardZ; }
    if (keys[SDL_SCANCODE_D]) { moveX += rightX; moveZ += rightZ; }
    if (keys[SDL_SCANCODE_A]) { moveX -= rightX; moveZ -= rightZ; }

    const float length = std::sqrt(moveX * moveX + moveZ * moveZ);
    if (length > 0.0f) {
        const float speed = keys[SDL_SCANCODE_LSHIFT] ? 9.0f : 4.5f;
        const float normalizedX = moveX / length;
        const float normalizedZ = moveZ / length;
        const float distance = speed * dt;

        camera.position.x += normalizedX * distance;
        if (HorizontalCubeCollision(camera, cubes)) {
            camera.position.x -= normalizedX * distance;
        }

        camera.position.z += normalizedZ * distance;
        if (HorizontalCubeCollision(camera, cubes)) {
            camera.position.z -= normalizedZ * distance;
        }
    }

    if (camera.jumpRequested && camera.grounded) {
        camera.verticalVelocity = 7.5f;
        camera.grounded = false;
    }
    camera.jumpRequested = false;

    camera.verticalVelocity -= 18.0f * dt;
    camera.position.y += camera.verticalVelocity * dt;

    const bool onGroundPlane =
        std::abs(camera.position.x) <= groundHalfExtent &&
        std::abs(camera.position.z) <= groundHalfExtent;

    float lowestSupportedHeight = onGroundPlane
        ? groundHeight + camera.eyeHeight
        : -std::numeric_limits<float>::infinity();

    for (const CubePlacement& cube : cubes) {
        const float cubeBottom = cube.position.y - cube.scale;
        const float cubeTop = cube.position.y + cube.scale;
        const float playerBase = camera.position.y - camera.eyeHeight;

        if (PointInsideExpandedCubeXZ(camera.position, cube) &&
            playerBase >= cubeBottom - 0.15f &&
            playerBase <= cubeTop + 0.15f) {
            lowestSupportedHeight = std::max(
                lowestSupportedHeight,
                cubeTop + camera.eyeHeight);
        }
    }

    if (camera.position.y <= lowestSupportedHeight) {
        camera.position.y = lowestSupportedHeight;
        camera.verticalVelocity = 0.0f;
        camera.grounded = true;
    } else {
        camera.grounded = false;
    }
}

// -----------------------------------------------------------------------------
// Step 5: SDL GPU resource helpers.
// -----------------------------------------------------------------------------
SDL_GPUShader* LoadShader(SDL_GPUDevice* device,
                          const char* path,
                          SDL_GPUShaderStage stage) {
    std::size_t codeSize = 0;
    void* code = SDL_LoadFile(path, &codeSize);
    if (!code) {
        std::cerr << "Could not load shader: " << path << '\n';
        return nullptr;
    }

    SDL_GPUShaderCreateInfo info{};
    info.code_size = codeSize;
    info.code = static_cast<const Uint8*>(code);
    info.entrypoint = "main";
    info.format = SDL_GPU_SHADERFORMAT_SPIRV;
    info.stage = stage;
    info.num_uniform_buffers = stage == SDL_GPU_SHADERSTAGE_VERTEX ? 1 : 0;

    SDL_GPUShader* shader = SDL_CreateGPUShader(device, &info);
    SDL_free(code);
    if (!shader) std::cerr << "Shader creation failed: " << SDL_GetError() << '\n';
    return shader;
}

SDL_GPUBuffer* UploadBuffer(SDL_GPUDevice* device,
                            SDL_GPUBufferUsageFlags usage,
                            const void* data,
                            Uint32 size) {
    SDL_GPUBufferCreateInfo bufferInfo{};
    bufferInfo.usage = usage;
    bufferInfo.size = size;
    SDL_GPUBuffer* buffer = SDL_CreateGPUBuffer(device, &bufferInfo);
    if (!buffer) return nullptr;

    SDL_GPUTransferBufferCreateInfo transferInfo{};
    transferInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    transferInfo.size = size;
    SDL_GPUTransferBuffer* transfer =
        SDL_CreateGPUTransferBuffer(device, &transferInfo);
    if (!transfer) {
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }

    void* mapped = SDL_MapGPUTransferBuffer(device, transfer, false);
    if (!mapped) {
        SDL_ReleaseGPUTransferBuffer(device, transfer);
        SDL_ReleaseGPUBuffer(device, buffer);
        return nullptr;
    }
    std::memcpy(mapped, data, size);
    SDL_UnmapGPUTransferBuffer(device, transfer);

    SDL_GPUCommandBuffer* commands = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(commands);
    SDL_GPUTransferBufferLocation source{transfer, 0};
    SDL_GPUBufferRegion destination{buffer, 0, size};
    SDL_UploadToGPUBuffer(copy, &source, &destination, false);
    SDL_EndGPUCopyPass(copy);
    SDL_SubmitGPUCommandBuffer(commands);
    SDL_WaitForGPUIdle(device);
    SDL_ReleaseGPUTransferBuffer(device, transfer);
    return buffer;
}

SDL_GPUTexture* CreateDepthTexture(SDL_GPUDevice* device,
                                   Uint32 width,
                                   Uint32 height) {
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    info.width = width;
    info.height = height;
    info.layer_count_or_depth = 1;
    info.num_levels = 1;
    info.sample_count = SDL_GPU_SAMPLECOUNT_1;
    return SDL_CreateGPUTexture(device, &info);
}

SDL_GPUGraphicsPipeline* CreatePipeline(SDL_GPUDevice* device,
                                        SDL_Window* window,
                                        SDL_GPUShader* vertex,
                                        SDL_GPUShader* fragment) {
    SDL_GPUVertexBufferDescription bufferDescription{};
    bufferDescription.slot = 0;
    bufferDescription.pitch = sizeof(Vec3);
    bufferDescription.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

    SDL_GPUVertexAttribute attribute{
        0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, 0};
    SDL_GPUColorTargetDescription colorTarget{};
    colorTarget.format = SDL_GetGPUSwapchainTextureFormat(device, window);

    SDL_GPUGraphicsPipelineCreateInfo info{};
    info.vertex_shader = vertex;
    info.fragment_shader = fragment;
    info.vertex_input_state = {&bufferDescription, 1, &attribute, 1};
    info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    info.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    info.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
    info.rasterizer_state.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    info.rasterizer_state.enable_depth_clip = true;
    info.multisample_state.sample_count = SDL_GPU_SAMPLECOUNT_1;
    info.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_LESS;
    info.depth_stencil_state.enable_depth_test = true;
    info.depth_stencil_state.enable_depth_write = true;
    info.target_info.color_target_descriptions = &colorTarget;
    info.target_info.num_color_targets = 1;
    info.target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    info.target_info.has_depth_stencil_target = true;

    return SDL_CreateGPUGraphicsPipeline(device, &info);
}

// -----------------------------------------------------------------------------
// Step 6: Program setup, event loop, and rendering.
// -----------------------------------------------------------------------------
int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_GPUDevice* device =
        SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, false, nullptr);
    SDL_Window* window = SDL_CreateWindow(
        "SDL3D Training Engine", 1280, 720,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);

    if (!device || !window ||
        !SDL_ClaimWindowForGPUDevice(device, window)) {
        std::cerr << "GPU/window setup failed: " << SDL_GetError() << '\n';
        if (window) SDL_DestroyWindow(window);
        if (device) SDL_DestroyGPUDevice(device);
        SDL_Quit();
        return 1;
    }

    float groundHeight = -1.0f;
    float groundHalfExtent = 50.0f;
    std::vector<CubePlacement> cubes;
    std::string mapError;
    if (!LoadMap("maps/test.map", cubes, groundHeight,
                 groundHalfExtent, mapError)) {
        std::cerr << "Map load failed: " << mapError << '\n';
        SDL_DestroyWindow(window);
        SDL_DestroyGPUDevice(device);
        SDL_Quit();
        return 1;
    }

    SDL_GPUShader* vertexShader =
        LoadShader(device, "shaders/cube.vert.spv", SDL_GPU_SHADERSTAGE_VERTEX);
    SDL_GPUShader* cubeFragment =
        LoadShader(device, "shaders/cube.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT);
    SDL_GPUShader* groundFragment =
        LoadShader(device, "shaders/ground.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT);

    const CubeMesh cube = MakeCubeMesh();
    const GroundMesh ground = MakeGroundMesh(groundHalfExtent);
    SDL_GPUBuffer* cubeVertices = UploadBuffer(
        device, SDL_GPU_BUFFERUSAGE_VERTEX,
        cube.vertices.data(), sizeof(cube.vertices));
    SDL_GPUBuffer* cubeIndices = UploadBuffer(
        device, SDL_GPU_BUFFERUSAGE_INDEX,
        cube.triangles.data(), sizeof(cube.triangles));
    SDL_GPUBuffer* groundVertices = UploadBuffer(
        device, SDL_GPU_BUFFERUSAGE_VERTEX,
        ground.vertices.data(), sizeof(ground.vertices));
    SDL_GPUBuffer* groundIndices = UploadBuffer(
        device, SDL_GPU_BUFFERUSAGE_INDEX,
        ground.triangles.data(), sizeof(ground.triangles));

    SDL_GPUGraphicsPipeline* cubePipeline = CreatePipeline(
        device, window, vertexShader, cubeFragment);
    SDL_GPUGraphicsPipeline* groundPipeline = CreatePipeline(
        device, window, vertexShader, groundFragment);

    if (!vertexShader || !cubeFragment || !groundFragment || !cubeVertices ||
        !cubeIndices || !groundVertices || !groundIndices || !cubePipeline ||
        !groundPipeline) {
        std::cerr << "Engine resource setup failed\n";
        SDL_DestroyWindow(window);
        SDL_DestroyGPUDevice(device);
        SDL_Quit();
        return 1;
    }

    PlayerCamera camera;
    bool mouseCaptured = SDL_SetWindowRelativeMouseMode(window, true);
    bool running = true;
    Uint64 previousCounter = SDL_GetPerformanceCounter();
    const Uint64 frequency = SDL_GetPerformanceFrequency();
    SDL_GPUTexture* depthTexture = nullptr;
    Uint32 depthWidth = 0;
    Uint32 depthHeight = 0;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT ||
                (event.type == SDL_EVENT_KEY_DOWN &&
                 event.key.key == SDLK_ESCAPE)) {
                running = false;
            } else if (event.type == SDL_EVENT_MOUSE_MOTION && mouseCaptured) {
                ApplyMouseLook(camera,
                    static_cast<float>(event.motion.xrel),
                    static_cast<float>(event.motion.yrel));
            } else if (event.type == SDL_EVENT_KEY_DOWN &&
                       event.key.key == SDLK_SPACE && !event.key.repeat) {
                camera.jumpRequested = true;
            } else if (event.type == SDL_EVENT_KEY_DOWN &&
                       event.key.key == SDLK_TAB && !event.key.repeat) {
                mouseCaptured = !mouseCaptured;
                SDL_SetWindowRelativeMouseMode(window, mouseCaptured);
            }
        }

        const Uint64 currentCounter = SDL_GetPerformanceCounter();
        float dt = static_cast<float>(currentCounter - previousCounter) /
                   static_cast<float>(frequency);
        dt = std::min(dt, 0.05f);
        previousCounter = currentCounter;

        int keyCount = 0;
        const bool* keys = SDL_GetKeyboardState(&keyCount);
        if (keys) MovePlayer(camera, keys, dt, cubes,
                             groundHeight, groundHalfExtent);

        int pixelWidth = 0;
        int pixelHeight = 0;
        SDL_GetWindowSizeInPixels(window, &pixelWidth, &pixelHeight);
        if (pixelWidth <= 0 || pixelHeight <= 0) {
            SDL_Delay(16);
            continue;
        }

        if (!depthTexture || depthWidth != static_cast<Uint32>(pixelWidth) ||
            depthHeight != static_cast<Uint32>(pixelHeight)) {
            SDL_GPUTexture* newDepth = CreateDepthTexture(
                device, static_cast<Uint32>(pixelWidth),
                static_cast<Uint32>(pixelHeight));
            if (!newDepth) {
                running = false;
                break;
            }
            if (depthTexture) {
                SDL_WaitForGPUIdle(device);
                SDL_ReleaseGPUTexture(device, depthTexture);
            }
            depthTexture = newDepth;
            depthWidth = static_cast<Uint32>(pixelWidth);
            depthHeight = static_cast<Uint32>(pixelHeight);
        }

        SDL_GPUCommandBuffer* commands = SDL_AcquireGPUCommandBuffer(device);
        SDL_GPUTexture* swapchain = nullptr;
        Uint32 swapWidth = 0;
        Uint32 swapHeight = 0;
        if (!SDL_WaitAndAcquireGPUSwapchainTexture(
                commands, window, &swapchain, &swapWidth, &swapHeight)) {
            SDL_CancelGPUCommandBuffer(commands);
            running = false;
            break;
        }

        if (swapchain) {
            SDL_GPUColorTargetInfo color{};
            color.texture = swapchain;
            color.clear_color = {0.20f, 0.55f, 1.00f, 1.0f};
            color.load_op = SDL_GPU_LOADOP_CLEAR;
            color.store_op = SDL_GPU_STOREOP_STORE;

            SDL_GPUDepthStencilTargetInfo depth{};
            depth.texture = depthTexture;
            depth.clear_depth = 1.0f;
            depth.load_op = SDL_GPU_LOADOP_CLEAR;
            depth.store_op = SDL_GPU_STOREOP_DONT_CARE;
            depth.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
            depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

            SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(
                commands, &color, 1, &depth);
            SDL_GPUViewport viewport{
                0.0f, 0.0f,
                static_cast<float>(swapWidth),
                static_cast<float>(swapHeight),
                0.0f, 1.0f};
            SDL_SetGPUViewport(pass, &viewport);

            constexpr float pi = 3.14159265358979323846f;
            const Mat4 projection = MakePerspective(
                pi / 3.0f,
                static_cast<float>(swapWidth) /
                    static_cast<float>(swapHeight),
                0.1f, 100.0f);
            const Mat4 viewProjection = Multiply(projection, MakeView(camera));

            SDL_GPUBufferBinding cubeVertexBinding{cubeVertices, 0};
            SDL_GPUBufferBinding cubeIndexBinding{cubeIndices, 0};
            SDL_BindGPUGraphicsPipeline(pass, cubePipeline);
            SDL_BindGPUVertexBuffers(pass, 0, &cubeVertexBinding, 1);
            SDL_BindGPUIndexBuffer(pass, &cubeIndexBinding,
                                   SDL_GPU_INDEXELEMENTSIZE_16BIT);

            for (const CubePlacement& cubePlacement : cubes) {
                const Mat4 mvp = Multiply(viewProjection,
                                          MakeModel(cubePlacement));
                SDL_PushGPUVertexUniformData(
                    commands, 0, mvp.values.data(),
                    sizeof(mvp.values));
                SDL_DrawGPUIndexedPrimitives(pass,
                    static_cast<Uint32>(cube.triangles.size()), 1, 0, 0, 0);
            }

            SDL_GPUBufferBinding groundVertexBinding{groundVertices, 0};
            SDL_GPUBufferBinding groundIndexBinding{groundIndices, 0};
            SDL_BindGPUGraphicsPipeline(pass, groundPipeline);
            SDL_BindGPUVertexBuffers(pass, 0, &groundVertexBinding, 1);
            SDL_BindGPUIndexBuffer(pass, &groundIndexBinding,
                                   SDL_GPU_INDEXELEMENTSIZE_16BIT);
            const Mat4 groundMvp = Multiply(
                viewProjection,
                MakeTranslation(0.0f, groundHeight, 0.0f));
            SDL_PushGPUVertexUniformData(
                commands, 0, groundMvp.values.data(),
                sizeof(groundMvp.values));
            SDL_DrawGPUIndexedPrimitives(pass,
                static_cast<Uint32>(ground.triangles.size()), 1, 0, 0, 0);

            SDL_EndGPURenderPass(pass);
        }

        if (!SDL_SubmitGPUCommandBuffer(commands)) {
            running = false;
        }
    }

    SDL_WaitForGPUIdle(device);
    if (depthTexture) SDL_ReleaseGPUTexture(device, depthTexture);
    if (cubeVertices) SDL_ReleaseGPUBuffer(device, cubeVertices);
    if (cubeIndices) SDL_ReleaseGPUBuffer(device, cubeIndices);
    if (groundVertices) SDL_ReleaseGPUBuffer(device, groundVertices);
    if (groundIndices) SDL_ReleaseGPUBuffer(device, groundIndices);
    if (cubePipeline) SDL_ReleaseGPUGraphicsPipeline(device, cubePipeline);
    if (groundPipeline) SDL_ReleaseGPUGraphicsPipeline(device, groundPipeline);
    if (vertexShader) SDL_ReleaseGPUShader(device, vertexShader);
    if (cubeFragment) SDL_ReleaseGPUShader(device, cubeFragment);
    if (groundFragment) SDL_ReleaseGPUShader(device, groundFragment);
    SDL_ReleaseWindowFromGPUDevice(device, window);
    SDL_DestroyWindow(window);
    SDL_DestroyGPUDevice(device);
    SDL_Quit();
    return 0;
}
