#version 120

/*uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 objectMatrix;

uniform vec3 g_sunDir;
uniform float normalMul;

in vec4 vertex;
in vec3 normal;
in vec2 texCoord;*/
uniform mat4 objectMatrix;
uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;

uniform vec3 sunDir;


attribute vec3 vertex;
attribute vec4 color;
attribute vec3 normal;

varying vec3 fragmentNormal;
varying vec3 fragmentWorldPos;
varying vec3 fragmentViewVec;
varying vec3 fragmentSunView;
varying vec3 fragmentNormalView;
float normSign;
void main() {
	vec4 worldPos  = objectMatrix*vec4(vertex,1);
	vec4 viewPos   = modelViewMatrix*worldPos;
	
    fragmentWorldPos = worldPos.xyz;
    fragmentNormal   = normalize(mat3(objectMatrix)*normal);
    fragmentViewVec = normalize(viewPos.xyz-fragmentWorldPos);
    fragmentSunView    = sunDir-fragmentWorldPos;
    fragmentNormalView = mat3(modelViewMatrix)*fragmentNormal;
    normSign = sign(dot(fragmentNormalView,normalize(-viewPos.xyz)));
    fragmentNormal *= normSign+(1-abs(normSign));
	
	gl_Position    = projectionMatrix*viewPos;
}
