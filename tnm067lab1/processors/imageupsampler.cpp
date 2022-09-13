#include <inviwo/core/util/logcentral.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/tnm067lab1/processors/imageupsampler.h>
#include <modules/tnm067lab1/utils/interpolationmethods.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/imageramutils.h>

namespace inviwo {

namespace detail {

template <typename T>
void upsample(ImageUpsampler::IntepolationMethod method, const LayerRAMPrecision<T>& inputImage,
              LayerRAMPrecision<T>& outputImage) {
    using F = typename float_type<T>::type;

    const size2_t inputSize = inputImage.getDimensions();
    const size2_t outputSize = outputImage.getDimensions();

    const T* inPixels = inputImage.getDataTyped();
    T* outPixels = outputImage.getDataTyped();

    auto inIndex = [&inputSize](auto pos) -> size_t {
        pos = glm::clamp(pos, decltype(pos)(0), decltype(pos)(inputSize - size2_t(1)));
        return pos.x + pos.y * inputSize.x;
    };
    auto outIndex = [&outputSize](auto pos) -> size_t {
        pos = glm::clamp(pos, decltype(pos)(0), decltype(pos)(outputSize - size2_t(1)));
        return pos.x + pos.y * outputSize.x;
    };

    util::forEachPixel(outputImage, [&](ivec2 outImageCoords) {
        // outImageCoords: Exact pixel coordinates in the output image currently writing to
        // inImageCoords: Relative coordinates of outImageCoords in the input image, might be
        // between pixels
        dvec2 inImageCoords =
            ImageUpsampler::convertCoordinate(outImageCoords, inputSize, outputSize);

        T finalColor(0);

        // DUMMY COLOR, remove or overwrite this bellow
        finalColor = inPixels[inIndex(
            glm::clamp(size2_t(outImageCoords), size2_t(0), size2_t(inputSize - size2_t(1))))];

        switch (method) {
            case ImageUpsampler::IntepolationMethod::PiecewiseConstant: {
                // Task 6
                // Update finalColor


                // inPixels = färgen för punkten inIndex, inIndex = pixeln i mitten, inImageCord är koordinaterna närmast (granne grejset)
                // inIndex = calculate pixel index from image coordinates
                // inPixels = get pixel by index

                finalColor = inPixels[inIndex(floor(inImageCoords))];

                break;
            }
            case ImageUpsampler::IntepolationMethod::Bilinear: {
                // Update finalColor

                ivec2 position0{ floor(inImageCoords.x),floor(inImageCoords.y) }; // ivec2 = 2 dimensional integer vector, storar x och y koordinaterna
                ivec2 position1{ ceil(inImageCoords.x),floor(inImageCoords.y) };
                ivec2 position2{ floor(inImageCoords.x),ceil(inImageCoords.y) };
                ivec2 position3{ ceil(inImageCoords.x),ceil(inImageCoords.y) };
                   
                std::array<T, 4> v{
                    inPixels[inIndex(position0)],
                    inPixels[inIndex(position1)],
                    inPixels[inIndex(position2)],
                    inPixels[inIndex(position3)]
                };

                double x = inImageCoords.x - position0.x; // Get domain value in x direction
                double y = inImageCoords.y - position0.y; // Get domain value in y direction

                finalColor = TNM067::Interpolation::bilinear(v, x, y);
                
                break;
            }
            case ImageUpsampler::IntepolationMethod::Biquadratic: {
                // Update finalColor

                ivec2 position0{ floor(inImageCoords.x),floor(inImageCoords.y) }; // ivec2 = 2 dimensional integer vector, storar x och y koordinaterna
                ivec2 position1{ ceil(inImageCoords.x),floor(inImageCoords.y) };
                ivec2 position2{ ceil(inImageCoords.x)+1,floor(inImageCoords.y) };


                ivec2 position3{ floor(inImageCoords.x),ceil(inImageCoords.y) };
                ivec2 position4{ ceil(inImageCoords.x),ceil(inImageCoords.y) }; // ivec2 = 2 dimensional integer vector, storar x och y koordinaterna
                ivec2 position5{ ceil(inImageCoords.x)+1,ceil(inImageCoords.y) };

                ivec2 position6{ floor(inImageCoords.x),ceil(inImageCoords.y)+1 };
                ivec2 position7{ ceil(inImageCoords.x),ceil(inImageCoords.y)+1 };
                ivec2 position8{ ceil(inImageCoords.x)+1,ceil(inImageCoords.y) +1};

                std::array<T, 9> v{
                    inPixels[inIndex(position0)],
                    inPixels[inIndex(position1)],
                    inPixels[inIndex(position2)],
                    inPixels[inIndex(position3)],
                    inPixels[inIndex(position4)],
                    inPixels[inIndex(position5)],
                    inPixels[inIndex(position6)],
                    inPixels[inIndex(position7)],
                    inPixels[inIndex(position8)]
                };

                double x = inImageCoords.x - position0.x; // Get parametrization in x direction
                double y = inImageCoords.y - position0.y; // Get parametrization in y direction


                finalColor = TNM067::Interpolation::biQuadratic(v, x/2.0, y/2.0); // Divide with 2 as we want between 0 and 1, NOT 0 and 2
                //pixels found are double the actual length and need to half the parametrization (quadratic fit)

                break;
            }
            case ImageUpsampler::IntepolationMethod::Barycentric: {

                ivec2 position0{ floor(inImageCoords.x),floor(inImageCoords.y) }; // ivec2 = 2 dimensional integer vector, storar x och y koordinaterna
                ivec2 position1{ ceil(inImageCoords.x),floor(inImageCoords.y) };
                ivec2 position2{ floor(inImageCoords.x),ceil(inImageCoords.y) };
                ivec2 position3{ ceil(inImageCoords.x),ceil(inImageCoords.y) };

                std::array<T, 4> v{
                    inPixels[inIndex(position0)],
                    inPixels[inIndex(position1)],
                    inPixels[inIndex(position2)],
                    inPixels[inIndex(position3)]
                };

                double x = inImageCoords.x - position0.x; // Get domain value in x direction
                double y = inImageCoords.y - position0.y; // Get domain value in y direction

                finalColor = TNM067::Interpolation::barycentric(v, x, y);
                break;
            }
            default:
                break;
        }

        outPixels[outIndex(outImageCoords)] = finalColor;
    });
}

}  // namespace detail

const ProcessorInfo ImageUpsampler::processorInfo_{
    "org.inviwo.imageupsampler",  // Class identifier
    "Image Upsampler",            // Display name
    "TNM067",                     // Category
    CodeState::Experimental,      // Code state
    Tags::None,                   // Tags
};
const ProcessorInfo ImageUpsampler::getProcessorInfo() const { return processorInfo_; }

ImageUpsampler::ImageUpsampler()
    : Processor()
    , inport_("inport", true)
    , outport_("outport", true)
    , interpolationMethod_("interpolationMethod", "Interpolation Method",
                           {
                               {"piecewiseconstant", "Piecewise Constant (Nearest Neighbor)",
                                IntepolationMethod::PiecewiseConstant},
                               {"bilinear", "Bilinear", IntepolationMethod::Bilinear},
                               {"biquadratic", "Biquadratic", IntepolationMethod::Biquadratic},
                               {"barycentric", "Barycentric", IntepolationMethod::Barycentric},
                           }) {
    addPort(inport_);
    addPort(outport_);
    addProperty(interpolationMethod_);
}

void ImageUpsampler::process() {
    auto inputImage = inport_.getData();
    if (inputImage->getDataFormat()->getComponents() != 1) {
        LogError("The ImageUpsampler processor does only support single channel images");
    }

    auto inSize = inport_.getData()->getDimensions();
    auto outDim = outport_.getDimensions();

    auto outputImage = std::make_shared<Image>(outDim, inputImage->getDataFormat());
    outputImage->getColorLayer()->setSwizzleMask(inputImage->getColorLayer()->getSwizzleMask());
    outputImage->getColorLayer()
        ->getEditableRepresentation<LayerRAM>()
        ->dispatch<void, dispatching::filter::Scalars>([&](auto outRep) {
            auto inRep = inputImage->getColorLayer()->getRepresentation<LayerRAM>();
            detail::upsample(interpolationMethod_.get(), *(const decltype(outRep))(inRep), *outRep);
        });

    outport_.setData(outputImage);
}

dvec2 ImageUpsampler::convertCoordinate(ivec2 outImageCoords, size2_t inputSize, size2_t outputSize) {
    // TODO implement
    dvec2 c(outImageCoords);

    // TASK 5: Convert the outImageCoords to its coordinates in the input image

    dvec2 factor = dvec2(inputSize) / dvec2(outputSize); // factor behöver definieras innan

    c = c * factor;

    return c;
}

}  // namespace inviwo
