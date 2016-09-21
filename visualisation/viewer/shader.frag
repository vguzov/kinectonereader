#version 120

uniform vec4 vertex_color;

varying vec3 fragmentWorldPos;
varying vec3 fragmentNormal;
varying vec3 fragmentSunView;
varying vec3 fragmentNormalView;
uniform float ambientLight, spotLightIntensity;
float lambert, light_mul;
void main(void)
{	
	lambert = max(dot(normalize(fragmentNormal),normalize(fragmentSunView)),0.0)*spotLightIntensity;
	light_mul = min(lambert+ambientLight,1.0);
	gl_FragColor = vec4(vertex_color.xyz*light_mul,1.0);
}