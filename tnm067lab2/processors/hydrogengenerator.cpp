#include <modules/tnm067lab2/processors/hydrogengenerator.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/util/volumeramutils.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/base/algorithm/dataminmax.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

namespace inviwo {

const ProcessorInfo HydrogenGenerator::processorInfo_{
    "org.inviwo.HydrogenGenerator",  // Class identifier
    "Hydrogen Generator",            // Display name
    "TNM067",                        // Category
    CodeState::Stable,               // Code state
    Tags::CPU,                       // Tags
};

const ProcessorInfo HydrogenGenerator::getProcessorInfo() const { return processorInfo_; }

HydrogenGenerator::HydrogenGenerator()
    : Processor(), volume_("volume"), size_("size_", "Volume Size", 16, 4, 256) {
    addPort(volume_);
    addProperty(size_);
}

void HydrogenGenerator::process() {
    auto vol = std::make_shared<Volume>(size3_t(size_), DataFloat32::get());

    auto ram = vol->getEditableRepresentation<VolumeRAM>();
    auto data = static_cast<float*>(ram->getData());
    util::IndexMapper3D index(ram->getDimensions());

    util::forEachVoxel(*ram, [&](const size3_t& pos) {
        vec3 cartesian = idTOCartesian(pos);
        data[index(pos)] = static_cast<float>(eval(cartesian));
    });

    auto minMax = util::volumeMinMax(ram);
    vol->dataMap_.dataRange = vol->dataMap_.valueRange = dvec2(minMax.first.x, minMax.second.x);

    volume_.setData(vol);
}

vec3 HydrogenGenerator::cartesianToSpherical(vec3 cartesian) {
    vec3 sph{cartesian};
    //vec3=(x,y,z) [1,2,3]
    // TASK 1: implement conversion using the equations in the lab script
    sph.r = std::sqrt(std::pow(cartesian.x,2.0) + std::pow(cartesian.y,2.0) + std::pow(cartesian.z,2.0));

    if (sph.r < 1.0e-20) { // Så vi inte delar med 0
        sph.t = 0.0;
        sph.p = 0.0;
        
        return sph;
    }
    sph.t = std::acos(cartesian.z / sph.r);
    sph.p = std::atan2(cartesian.y, cartesian.x); // kan ge lite olikt resultat


    return sph;
}

double HydrogenGenerator::eval(vec3 cartesian) {
    const double density = cartesian.x;

    // TASK 2: Evaluate wave function
    float Z = 1.f;
    float a0 = 1.f;

    vec3 sph = cartesianToSpherical(cartesian);

    float yellow = 1.f / (81.f * std::sqrt(6.f * M_PI));
    float red = pow((Z/a0),(3.f /2.f));
    float blue = (pow(Z,2.f)*pow(sph.r, 2.f))/pow(a0,2.f);
    float green = exp((-Z*sph.r)/(3.f*a0));
    float pink = 3.f*pow(cos(sph.t),2.f)-1.f;

    // QUESTION : SHOULD BE USE DNESITY? WHY CONST?


    return pow(yellow * red * blue * green * pink,2.f);
}

vec3 HydrogenGenerator::idTOCartesian(size3_t pos) {
    vec3 p(pos);
    p /= size_ - 1;
    return p * (36.0f) - 18.0f;
}

}  // namespace inviwo
