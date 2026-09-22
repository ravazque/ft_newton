#version 330 core

in vec3 vNormalWorld;
out vec4 FragColor;

uniform vec3 uColor;   // base color of the object
uniform int  uLit;     // 2 = per-vertex color (2D overlay), 1 = diffuse lighting, 0 = flat uColor

void main()
{
	if (uLit == 2)
	{
		// The 2D overlay packs a color per vertex in the normal slot, so one
		// draw call can mix panels and text of different colors.
		FragColor = vec4(vNormalWorld, 1.0);
		return;
	}

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
