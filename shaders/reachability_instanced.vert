#version 430 core

// Vertex Shader: Rendu instancié de cubes
// Input: Vertices d'UN cube + indices voxels visibles
// Output: Position finale de chaque instance

layout(location = 0) in vec3 in_Position;    // Vertex du cube (8 vertices)
layout(location = 1) in vec3 in_Normal;      // Normale pour lighting

// SSBO: Indices des voxels visibles
layout(std430, binding = 1) readonly buffer VisibleIndices {
    uint visible_indices[];
};

// SSBO: RI values (pour couleur)
layout(std430, binding = 0) readonly buffer RIValues {
    float ri_values[];
};

// Uniforms
uniform mat4 u_ViewProjection;
uniform vec3 u_GridOrigin;
uniform vec3 u_GridSize;       // size_x, size_y, size_z
uniform float u_VoxelResolution;

// Outputs vers fragment shader
out vec3 v_Normal;
out float v_RI;
out vec3 v_WorldPos;

void main() {
    // Récupérer l'index du voxel visible pour cette instance
    uint voxel_flat_idx = visible_indices[gl_InstanceID];

    // Convertir index flat → coordonnées 3D
    uint x = voxel_flat_idx / uint(u_GridSize.y * u_GridSize.z);
    uint y = (voxel_flat_idx / uint(u_GridSize.z)) % uint(u_GridSize.y);
    uint z = voxel_flat_idx % uint(u_GridSize.z);

    // Calculer position du voxel dans le monde
    vec3 voxel_center = u_GridOrigin + vec3(x, y, z) * u_VoxelResolution;

    // Échelle du cube = résolution voxel
    vec3 scaled_position = in_Position * u_VoxelResolution;

    // Position finale = centre voxel + offset vertex
    vec3 world_position = voxel_center + scaled_position;

    // Projection
    gl_Position = u_ViewProjection * vec4(world_position, 1.0);

    // Pass data au fragment shader
    v_Normal = in_Normal;
    v_RI = ri_values[voxel_flat_idx];
    v_WorldPos = world_position;
}
