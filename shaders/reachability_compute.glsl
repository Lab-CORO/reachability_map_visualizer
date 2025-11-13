#version 430 core

// Compute shader pour le calcul parallèle des couleurs et filtrage
// Workgroup size: 256 threads per workgroup
layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;

// Structure d'entrée : positions + RI des voxels
struct VoxelInput {
    vec3 position;
    float ri;
};

// Structure de sortie : positions + couleurs RGBA
struct VoxelOutput {
    vec3 position;
    float padding;  // Alignment
    vec4 color;     // RGBA
};

// Shader Storage Buffer Objects (SSBO)
layout(std430, binding = 0) buffer InputBuffer {
    VoxelInput inputs[];
};

layout(std430, binding = 1) buffer OutputBuffer {
    VoxelOutput outputs[];
};

// Uniforms pour les filtres
uniform float low_ri;
uniform float high_ri;
uniform vec3 disect_min;
uniform vec3 disect_max;
uniform int disect_axis;  // 0=None, 1=X, 2=Y, 3=Z

// Fonction de mapping couleur selon RI
vec3 getColorForRI(float ri) {
    if (ri >= 90.0) {
        return vec3(0.0, 0.0, 1.0);  // Bleu
    } else if (ri >= 50.0) {
        return vec3(0.0, 1.0, 1.0);  // Cyan
    } else if (ri >= 30.0) {
        return vec3(0.0, 1.0, 0.0);  // Vert
    } else if (ri >= 5.0) {
        return vec3(1.0, 1.0, 0.0);  // Jaune
    } else {
        return vec3(1.0, 0.0, 0.0);  // Rouge
    }
}

void main() {
    // Index global du thread
    uint idx = gl_GlobalInvocationID.x;

    // Bounds check
    if (idx >= inputs.length()) {
        return;
    }

    // Lire les données d'entrée
    VoxelInput voxel = inputs[idx];

    // Initialiser la sortie avec la position
    outputs[idx].position = voxel.position;
    outputs[idx].padding = 0.0;

    // Filtrage par Reachability Index
    if (voxel.ri < low_ri || voxel.ri > high_ri) {
        outputs[idx].color = vec4(0.0, 0.0, 0.0, 0.0);  // Alpha = 0 (invisible)
        return;
    }

    // Filtrage par dissection
    bool filtered = false;

    if (disect_axis == 1) {  // X axis
        if (voxel.position.x < disect_min.x || voxel.position.x > disect_max.x) {
            filtered = true;
        }
    } else if (disect_axis == 2) {  // Y axis
        if (voxel.position.y < disect_min.y || voxel.position.y > disect_max.y) {
            filtered = true;
        }
    } else if (disect_axis == 3) {  // Z axis
        if (voxel.position.z < disect_min.z || voxel.position.z > disect_max.z) {
            filtered = true;
        }
    }

    if (filtered) {
        outputs[idx].color = vec4(0.0, 0.0, 0.0, 0.0);  // Alpha = 0 (invisible)
        return;
    }

    // Voxel visible : calculer la couleur
    vec3 color = getColorForRI(voxel.ri);
    outputs[idx].color = vec4(color, 1.0);  // Alpha = 1 (visible)
}
