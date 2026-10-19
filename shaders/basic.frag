#version 330 core

// uLit: 1 lit by a directional light, 0 flat uColor, 2 the overlay's per-vertex colour carried in the normal slot.

in vec3 vNormalWorld;
out vec4 FragColor;

uniform vec3 uColor;
uniform int  uLit;

void main()
{
	if (uLit == 2)
	{
		FragColor = vec4(vNormalWorld, 1.0);
		return;
	}
	if (uLit == 0)
	{
		FragColor = vec4(uColor, 1.0);
		return;
	}
	vec3  lightDirection = normalize(vec3(-0.4, -1.0, -0.3));
	float diffuse = max(dot(normalize(vNormalWorld), -lightDirection), 0.0);
	float ambient = 0.25;

	FragColor = vec4(uColor * (ambient + diffuse * 0.85), 1.0);
}
