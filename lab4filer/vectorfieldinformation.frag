#include "utils/structs.glsl"

uniform sampler2D vfColor;
uniform ImageParameters vfParameters;
in vec3 texCoord_;

float passThrough(vec2 coord){
    return texture(vfColor,coord).x;
}

float magnitude( vec2 coord ){
    //TASK 1: find the magnitude of the vectorfield at the position coords
    vec2 velo = texture(vfColor, coord.xy).xy;

    float m = sqrt(pow(velo.x,2.f) + pow(velo.y,2.f));
    return abs(m);
}
// How can vector magnitude be extended to 3D?
// -> adding a z coordinate to the texture, calculating magnitude of all 3 coordinate points

float divergence(vec2 coord){
    //TASK 2: find the divergence of the vectorfield at the position coords
    vec2 pixelSize = vfParameters.reciprocalDimensions;

    vec2 velo = texture(vfColor, coord.xy).xy;

    // derivative x
    vec2 velo_x_pos = texture(vfColor, (coord + vec2(pixelSize.x,0)) ).xy;
    vec2 velo_x_neg = texture(vfColor, (coord - vec2(pixelSize.x,0)) ).xy;

    vec2 devX = (velo_x_pos - velo_x_neg) / (2.0 * pixelSize.x);

    // derivative y
    
    vec2 velo_y_pos = texture(vfColor, (coord + vec2(0, pixelSize.y)) ).xy;
    vec2 velo_y_neg = texture(vfColor, (coord - vec2(0, pixelSize.y)) ).xy;

    vec2 devY = (velo_y_pos - velo_y_neg) / (2.0 * pixelSize.y);

    float V = devX.x + devY.y; // ekvation 3
    return V;
}

float rotation(vec2 coord){
    //TASK 3: find the curl of the vectorfield at the position coords
    vec2 pixelSize = vfParameters.reciprocalDimensions;

    vec2 velo = texture(vfColor, coord.xy).xy;

    // derivative y by x
    vec2 velo_x_pos = texture(vfColor, (coord + vec2(pixelSize.x,0)) ).xy;
    vec2 velo_x_neg = texture(vfColor, (coord - vec2(pixelSize.x,0)) ).xy;

    vec2 devX = (velo_x_pos - velo_x_neg) / (2.0 * pixelSize.x);

    // derivative x by y
    
    vec2 velo_y_pos = texture(vfColor, (coord + vec2(0, pixelSize.y)) ).xy;
    vec2 velo_y_neg = texture(vfColor, (coord - vec2(0, pixelSize.y)) ).xy;

    vec2 devY = (velo_y_pos - velo_y_neg) / (2.0 * pixelSize.y);

    float V = devX.y - devY.x; // ekvation 4

    return V;
}

void main(void) {
    float v = OUTPUT(texCoord_.xy);
    FragData0 = vec4(v);
}
