#version 430 core

// Fragment Shader: Calcul couleur selon RI + lighting simple

in vec3 v_Normal;
in float v_RI;
in vec3 v_WorldPos;

out vec4 FragColor;

// Uniforms pour lighting
uniform vec3 u_LightDir;
uniform vec3 u_CameraPos;

// Fonction couleur selon RI (identique à la LUT CPU)
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
    // Couleur de base selon RI
    vec3 base_color = getColorForRI(v_RI);

    // Lighting simple (diffuse + ambient)
    vec3 N = normalize(v_Normal);
    vec3 L = normalize(u_LightDir);

    float diffuse = max(dot(N, L), 0.0);
    float ambient = 0.3;

    vec3 lighting = vec3(ambient + diffuse * 0.7);

    // Couleur finale
    vec3 final_color = base_color * lighting;

    FragColor = vec4(final_color, 1.0);
}
