#version 330 core

// Model, view and projection place each vertex on screen; the normal turns with the model and is renormalised per fragment.

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vNormalWorld;

void main()
{
	gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);
	vNormalWorld = mat3(uModel) * aNormal;
}
