#include "gpu_reachability_renderer.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstring>

// OpenGL headers
#ifdef __APPLE__
    #include <OpenGL/gl3.h>
#else
    #include <GL/glew.h>
    #include <GL/gl.h>
#endif

#include <OgreRoot.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>

namespace reachability_map_visualizer {

// Cube vertices (unit cube centered at origin)
static const float CUBE_VERTEX_DATA[CUBE_VERTICES * 3] = {
    -0.5f, -0.5f, -0.5f,  // 0
     0.5f, -0.5f, -0.5f,  // 1
     0.5f,  0.5f, -0.5f,  // 2
    -0.5f,  0.5f, -0.5f,  // 3
    -0.5f, -0.5f,  0.5f,  // 4
     0.5f, -0.5f,  0.5f,  // 5
     0.5f,  0.5f,  0.5f,  // 6
    -0.5f,  0.5f,  0.5f   // 7
};

// Cube normals (face normals for simple lighting)
static const float CUBE_NORMAL_DATA[CUBE_VERTICES * 3] = {
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
     1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f
};

// Cube indices (12 triangles, 36 indices)
static const uint16_t CUBE_INDEX_DATA[CUBE_INDICES] = {
    // Front
    0, 1, 2,  2, 3, 0,
    // Back
    5, 4, 7,  7, 6, 5,
    // Left
    4, 0, 3,  3, 7, 4,
    // Right
    1, 5, 6,  6, 2, 1,
    // Top
    3, 2, 6,  6, 7, 3,
    // Bottom
    4, 5, 1,  1, 0, 4
};

GPUReachabilityRenderer::GPUReachabilityRenderer(Ogre::SceneManager* scene_manager,
                                                  Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager),
      scene_node_(parent_node->createChildSceneNode()),
      manual_object_(nullptr),
      compute_program_(0),
      render_program_(0),
      vao_(0),
      vbo_vertices_(0),
      vbo_normals_(0),
      ibo_(0),
      ssbo_ri_(0),
      ssbo_indices_(0),
      ssbo_counter_(0),
      size_x_(0), size_y_(0), size_z_(0),
      resolution_(0.0f),
      origin_x_(0.0f), origin_y_(0.0f), origin_z_(0.0f),
      total_voxels_(0),
      visible_count_(0),
      enabled_(true),
      initialized_(false),
      data_uploaded_(false)
{
    if (isSupported()) {
        try {
            initializeGL();
            RCLCPP_INFO(rclcpp::get_logger("GPUReachabilityRenderer"),
                        "GPU renderer initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("GPUReachabilityRenderer"),
                         "Failed to initialize GPU renderer: %s", e.what());
            enabled_ = false;
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("GPUReachabilityRenderer"),
                    "GPU compute shaders not supported. Falling back to CPU rendering.");
        enabled_ = false;
    }
}

GPUReachabilityRenderer::~GPUReachabilityRenderer() {
    // Cleanup OpenGL resources
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vbo_vertices_) glDeleteBuffers(1, &vbo_vertices_);
    if (vbo_normals_) glDeleteBuffers(1, &vbo_normals_);
    if (ibo_) glDeleteBuffers(1, &ibo_);
    if (ssbo_ri_) glDeleteBuffers(1, &ssbo_ri_);
    if (ssbo_indices_) glDeleteBuffers(1, &ssbo_indices_);
    if (ssbo_counter_) glDeleteBuffers(1, &ssbo_counter_);
    if (compute_program_) glDeleteProgram(compute_program_);
    if (render_program_) glDeleteProgram(render_program_);

    if (scene_node_) {
        scene_manager_->destroySceneNode(scene_node_);
    }
}

bool GPUReachabilityRenderer::isSupported() {
#ifndef __APPLE__
    // Check OpenGL version and extensions
    // Requires OpenGL 4.3+ for compute shaders
    const GLubyte* version = glGetString(GL_VERSION);
    if (!version) return false;

    // Parse version
    int major = 0, minor = 0;
    sscanf(reinterpret_cast<const char*>(version), "%d.%d", &major, &minor);

    if (major < 4 || (major == 4 && minor < 3)) {
        return false;
    }

    // Check for SSBO support
    GLint max_ssbo_bindings = 0;
    glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &max_ssbo_bindings);
    if (max_ssbo_bindings < 3) {
        return false;
    }

    return true;
#else
    // macOS doesn't support OpenGL 4.3+ compute shaders
    return false;
#endif
}

void GPUReachabilityRenderer::initializeGL() {
#ifndef __APPLE__
    // Initialize GLEW if needed
    static bool glew_initialized = false;
    if (!glew_initialized) {
        glewExperimental = GL_TRUE;
        GLenum err = glewInit();
        if (err != GLEW_OK) {
            throw std::runtime_error("GLEW initialization failed");
        }
        glew_initialized = true;
    }

    createComputeShader();
    createRenderShaders();
    createCubeMesh();
    createSSBOs();

    initialized_ = true;
#endif
}

std::string GPUReachabilityRenderer::loadShaderFile(const std::string& filename) {
    // Try to find shader file in package share directory
    std::string shader_path = "/home/user/reachability_map_visualizer/shaders/" + filename;

    std::ifstream file(shader_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open shader file: " + shader_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

uint32_t GPUReachabilityRenderer::compileShader(const std::string& source, uint32_t type) {
    uint32_t shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    // Check compilation
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("Shader compilation failed: ") + log);
    }

    return shader;
}

uint32_t GPUReachabilityRenderer::linkProgram(const std::vector<uint32_t>& shaders) {
    uint32_t program = glCreateProgram();

    for (uint32_t shader : shaders) {
        glAttachShader(program, shader);
    }

    glLinkProgram(program);

    // Check linking
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        glDeleteProgram(program);
        throw std::runtime_error(std::string("Program linking failed: ") + log);
    }

    // Cleanup shaders (no longer needed after linking)
    for (uint32_t shader : shaders) {
        glDeleteShader(shader);
    }

    return program;
}

void GPUReachabilityRenderer::createComputeShader() {
    std::string compute_source = loadShaderFile("reachability_filter.comp");
    uint32_t compute_shader = compileShader(compute_source, GL_COMPUTE_SHADER);
    compute_program_ = linkProgram({compute_shader});
}

void GPUReachabilityRenderer::createRenderShaders() {
    std::string vert_source = loadShaderFile("reachability_instanced.vert");
    std::string frag_source = loadShaderFile("reachability_instanced.frag");

    uint32_t vert_shader = compileShader(vert_source, GL_VERTEX_SHADER);
    uint32_t frag_shader = compileShader(frag_source, GL_FRAGMENT_SHADER);

    render_program_ = linkProgram({vert_shader, frag_shader});
}

void GPUReachabilityRenderer::createCubeMesh() {
    // Create VAO
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    // Vertex positions
    glGenBuffers(1, &vbo_vertices_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_VERTEX_DATA), CUBE_VERTEX_DATA, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    // Normals
    glGenBuffers(1, &vbo_normals_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_normals_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_NORMAL_DATA), CUBE_NORMAL_DATA, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(1);

    // Indices
    glGenBuffers(1, &ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(CUBE_INDEX_DATA), CUBE_INDEX_DATA, GL_STATIC_DRAW);

    glBindVertexArray(0);
}

void GPUReachabilityRenderer::createSSBOs() {
    // SSBO for RI values (binding 0)
    glGenBuffers(1, &ssbo_ri_);

    // SSBO for visible indices (binding 1)
    glGenBuffers(1, &ssbo_indices_);

    // SSBO for counter (binding 2)
    glGenBuffers(1, &ssbo_counter_);
}

void GPUReachabilityRenderer::uploadRIData(const std::vector<float>& ri_values,
                                            int size_x, int size_y, int size_z,
                                            float resolution,
                                            float origin_x, float origin_y, float origin_z) {
    if (!enabled_ || !initialized_) return;

    size_x_ = size_x;
    size_y_ = size_y;
    size_z_ = size_z;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    origin_z_ = origin_z;
    total_voxels_ = ri_values.size();

    // Upload RI values to GPU
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_ri_);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
                 ri_values.size() * sizeof(float),
                 ri_values.data(),
                 GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_ri_);

    // Allocate space for visible indices (worst case: all voxels visible)
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_indices_);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
                 total_voxels_ * sizeof(uint32_t),
                 nullptr,
                 GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo_indices_);

    // Counter buffer (atomic)
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_counter_);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint32_t), nullptr, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, ssbo_counter_);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    data_uploaded_ = true;

    RCLCPP_INFO(rclcpp::get_logger("GPUReachabilityRenderer"),
                "Uploaded %zu voxels to GPU (%dx%dx%d)",
                total_voxels_, size_x, size_y, size_z);
}

void GPUReachabilityRenderer::render(int low_ri, int high_ri,
                                      int disect_choice, int disect_min, int disect_max) {
    if (!enabled_ || !initialized_ || !data_uploaded_) return;

    // Step 1: Reset counter
    uint32_t zero = 0;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_counter_);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(uint32_t), &zero);

    // Step 2: Run compute shader to filter voxels
    glUseProgram(compute_program_);

    // Set uniforms
    GLint loc_grid_size = glGetUniformLocation(compute_program_, "grid_size");
    GLint loc_low_ri = glGetUniformLocation(compute_program_, "low_ri");
    GLint loc_high_ri = glGetUniformLocation(compute_program_, "high_ri");
    GLint loc_disect_choice = glGetUniformLocation(compute_program_, "disect_choice");
    GLint loc_disect_min = glGetUniformLocation(compute_program_, "disect_min");
    GLint loc_disect_max = glGetUniformLocation(compute_program_, "disect_max");

    glUniform3i(loc_grid_size, size_x_, size_y_, size_z_);
    glUniform1i(loc_low_ri, low_ri);
    glUniform1i(loc_high_ri, high_ri);
    glUniform1i(loc_disect_choice, disect_choice);
    glUniform1i(loc_disect_min, disect_min);
    glUniform1i(loc_disect_max, disect_max);

    // Dispatch compute shader
    GLuint num_workgroups = (total_voxels_ + 255) / 256;
    glDispatchCompute(num_workgroups, 1, 1);

    // Wait for compute shader to finish
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Step 3: Read back visible count
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_counter_);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(uint32_t), &visible_count_);

    // Step 4: Render using instanced drawing
    if (visible_count_ > 0) {
        glUseProgram(render_program_);

        // Set uniforms (would need camera matrices from Ogre)
        // For now, this is a placeholder - full integration requires Ogre camera

        glBindVertexArray(vao_);
        glDrawElementsInstanced(GL_TRIANGLES, CUBE_INDICES, GL_UNSIGNED_SHORT, nullptr, visible_count_);
        glBindVertexArray(0);
    }

    glUseProgram(0);
}

} // namespace reachability_map_visualizer
