#version 330 core

// Per-vertex inputs, fed from the VAO/VBO. location = the attribute slot the
// renderer binds when it uploads the mesh.
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

// Uniforms: the same for every vertex in a draw call. These are the MVP matrices.
uniform mat4 uModel;        // local -> world  (built from a body's position + orientation)
uniform mat4 uView;         // world -> camera
uniform mat4 uProjection;   // camera -> clip (perspective)

out vec3 vNormalWorld;      // passed to the fragment shader for lighting

void main()
{
	// The MVP chain: this single line is what places a 3D point on the 2D screen.
	gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);

	// Rotate the normal into world space (good enough for uniform scaling).
	vNormalWorld = mat3(uModel) * aNormal;
}
