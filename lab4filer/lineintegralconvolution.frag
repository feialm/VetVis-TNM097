uniform sampler2D vfColor; // vector field color
uniform sampler2D noiseColor; // noise color texture

uniform int nSteps;
uniform float stepSize;

in vec3 texCoord_;

/*
* Traverse the vector field and sample the noise image
* @param posF Starting position
* @param stepSize length of each step
* @param nSteps the number of steps to traverse
* @param accVal the accumulated value from sampling the noise image
* @param nSamples the number of samples used for v
*/
void traverse(vec2 posF, float stepSize, int nSteps, inout float accVal, inout int nSamples){
    // traverse the vectorfield staring at `posF` for `nSteps` using `stepSize` and sample the noiseColor texture for each position
    // store the accumulated value in `accVal` and the amount of samples in `nSamples`


    vec2 currentPosition = posF;

    vec2 currentDirection = vec2(0.0, 0.0);

    for(int i = 0; i < nSteps; i++){
        
        currentDirection = normalize(texture(vfColor, currentPosition).xy);

        currentPosition = (currentPosition + currentDirection * stepSize);

        accVal += texture(noiseColor, currentPosition).r;

        nSamples++;
    }

}

void main(void) {
    float accVal = texture(noiseColor, texCoord_.xy).r;
    int nSamples = 1;
    
    //traverse the vector field both forward and backwards to calculate the output color

    traverse(texCoord_.xy, stepSize, nSteps, accVal, nSamples); //forward

    // alt 2, sätt nSamples till noll här så vi kan följa box-filter grejjen bostavligen
    nSamples=0;

    traverse(texCoord_.xy, -1.0*stepSize, nSteps, accVal, nSamples); //backwards


    accVal = accVal / (2*nSamples + 1); // box filter, gångrar ej med 2 för att vi räknar upp nSamples  två gånger med först forward och sen backward

    FragData0 = vec4(accVal, accVal, accVal, 1);
}
