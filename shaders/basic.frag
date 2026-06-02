#version 330 core

in vec3 vNormalWorld;
out vec4 FragColor;

uniform vec3 uColor;   // base color of the object
uniform int  uLit;     // 1 = simple diffuse lighting, 0 = flat (wireframe/debug)

void main()
{
	if (uLit == 0)
	{
		FragColor = vec4(uColor, 1.0);
		return;
	}

	// Cheap directional (sun) light so 3D shapes read as solid.
	vec3  lightDir = normalize(vec3(-0.4, -1.0, -0.3));
	float diffuse  = max(dot(normalize(vNormalWorld), -lightDir), 0.0);
	float ambient  = 0.25;

	FragColor = vec4(uColor * (ambient + diffuse * 0.85), 1.0);
}
