#include <inviwo/tnm067lab3/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/network/networklock.h>
#include <modules/tnm067lab1/utils/interpolationmethods.h>
#include <iostream>
#include <fstream>

namespace inviwo {

size_t MarchingTetrahedra::HashFunc::max = 1;

const ProcessorInfo MarchingTetrahedra::processorInfo_{
    "org.inviwo.MarchingTetrahedra",  // Class identifier
    "Marching Tetrahedra",            // Display name
    "TNM067",                         // Category
    CodeState::Stable,                // Code state
    Tags::None,                       // Tags
};
const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

MarchingTetrahedra::MarchingTetrahedra()
    : Processor()
    , volume_("volume")
    , mesh_("mesh")
    , isoValue_("isoValue", "ISO value", 0.5f, 0.0f, 1.0f) {

    addPort(volume_);
    addPort(mesh_);

    addProperty(isoValue_);

    isoValue_.setSerializationMode(PropertySerializationMode::All);

    volume_.onChange([&]() {
        if (!volume_.hasData()) {
            return;
        }
        NetworkLock lock(getNetwork());
        float iso = (isoValue_.get() - isoValue_.getMinValue()) /
                    (isoValue_.getMaxValue() - isoValue_.getMinValue());
        const auto vr = volume_.getData()->dataMap_.valueRange;
        isoValue_.setMinValue(static_cast<float>(vr.x));
        isoValue_.setMaxValue(static_cast<float>(vr.y));
        isoValue_.setIncrement(static_cast<float>(glm::abs(vr.y - vr.x) / 50.0));
        isoValue_.set(static_cast<float>(iso * (vr.y - vr.x) + vr.x));
        isoValue_.setCurrentStateAsDefault();
    });
}

void MarchingTetrahedra::process() {
    auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
    MeshHelper mesh(volume_.getData());

    const auto& dims = volume->getDimensions();
    MarchingTetrahedra::HashFunc::max = dims.x * dims.y * dims.z;

    const float iso = isoValue_.get();

    util::IndexMapper3D mapVolPosToIndex(dims); //used for setting 1D index of each data point

    const static size_t tetrahedraIds[6][4] = {{0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7},
                                               {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2}}; // 6 tetrahedras, each tetrahedra has 4 points

    size3_t pos{};
    for (pos.z = 0; pos.z < dims.z - 1; ++pos.z) { // 0 -> 6
        for (pos.y = 0; pos.y < dims.y - 1; ++pos.y) {
            for (pos.x = 0; pos.x < dims.x - 1; ++pos.x) {
                // The DataPoint index should be the 1D-index for the DataPoint in the cell
                // Use volume->getAsDouble to query values from the volume
                // Spatial position should be between 0 and 1

                // TODO: TASK 2: create a nested for loop to construct the cell
                Cell c;
                size_t indexForPoint = 0;

                for (int z = 0; z < 2; z++) { // 0 -> 1, 
                    for (int y = 0; y < 2; y++) {
                        for (int x = 0; x < 2; x++) {
                            vec3 cellPos = vec3{ x,y,z }; // position for cell
                            vec3 scaledCellPos = calculateDataPointPos(pos, cellPos, dims); //pos = posVolume, position of volume

                            vec3 cellPosInVol(pos.x + x, pos.y + y, pos.z + z);// cell position in volume

                            size_t ind1D = calculateDataPointIndexInCell(cellPos); //index value from 3D to 1D

                            float funcValue = volume->getAsDouble(cellPosInVol); //query values from volume

                            //Set final cell values
                            c.dataPoints[indexForPoint].pos = scaledCellPos;
                            c.dataPoints[indexForPoint].indexInVolume = ind1D;
                            c.dataPoints[indexForPoint].value = funcValue;

                            indexForPoint++;
                        }
                    }
                }


                // TODO: TASK 3: Subdivide cell into 6 tetrahedra (hint: use tetrahedraIds)
                std::vector<Tetrahedra> tetrahedras;


                Tetrahedra temp;
                for (int i = 0; i < 6; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        size_t getId = tetrahedraIds[i][j];
                        DataPoint dPoint = c.dataPoints[getId];
                        temp.dataPoints[j] = dPoint;
                    }
                    tetrahedras.push_back(temp);
                }



                for (const Tetrahedra& tetrahedra : tetrahedras) {
                    // TODO: TASK 4: Calculate case id for each tetrahedra, and add triangles for
                    // each case (use MeshHelper)

                    // Calculate for tetra case index
                    int caseId = 0;

                    // T XX = a * (1 - x) + b * x;
                    
                    DataPoint v0 = tetrahedra.dataPoints[0];
                    DataPoint v1 = tetrahedra.dataPoints[1];
                    DataPoint v2 = tetrahedra.dataPoints[2];
                    DataPoint v3 = tetrahedra.dataPoints[3];

                    // if value in vertex < iso -> negative vertex
                    // if value in vertex > iso -> positive vertex
                    for (auto t : tetrahedra.dataPoints) {
                        if (t.value < iso) { // negativ vertex ? positiv? hur veta?
                            // caseId = ...;
                        }
                    }

                    // Extract triangles
                    switch (caseId) {
                        case 0:
                        case 15:
                            break;
                        case 1:
                            // calculate location of the vertices of the triangle(s)
                            // veta edge ->  interpolation?
                            // add vertex: idOfVertex = mesh.addVertex
                            // add triangel: mesh.addTriangle(idOfVertex1, idOfvertex2, idofvertex3)
                        case 14: {

                            break;
                        }
                        case 2:
                        case 13: {

                            break;
                        }
                        case 3:
                        case 12: {

                            break;
                        }
                        case 4:
                        case 11: {

                            break;
                        }
                        case 5:
                        case 10: {

                            break;
                        }
                        case 6:
                        case 9: {

                            break;
                        }
                        case 7:
                        case 8: {

                            break;
                        }
                    }
                }
            }
        }
    }

    mesh_.setData(mesh.toBasicMesh());
}

int MarchingTetrahedra::calculateDataPointIndexInCell(ivec3 index3D) {
    // ivec3 = ({x,y,z})
    // TODO: TASK 1: map 3D index to 1D
    int oneDIndex = 1 * index3D.x + 2 * index3D.y + 4 * index3D.z; // index = z * index4 + y * index2 + x
    
    return oneDIndex;
}

vec3 MarchingTetrahedra::calculateDataPointPos(size3_t posVolume, ivec3 posCell, ivec3 dims) {
    // TODO: TASK 1: scale DataPoint position with dimensions to be between 0 and 1
    //posVolume: position of the cell in the volume, 
    //posCell: the 3D index of the data point within the cell
    //dims: the dimension of the volume.

    vec3 scaledPoint = vec3{ 0.f,0.f,0.f };
    //std::cout << "\n";
    //std::cout << "posvolume: " << posVolume[0] << " " << posVolume[1] << " " << posVolume[2] << "\n";
    //std::cout << "poscell: " << posCell[0] << " " << posCell[1] << " " << posCell[2] << "\n";
    //std::cout << "dims: " << dims[0] << " " << dims[1] << " " << dims[2] << "\n";
    //std::cout << "\n";

    scaledPoint[0] = (posVolume[0] + posCell[0]) / (dims[0] - 1.f); // dims = antalet celler i volymen/kuben, dims - 1 = 6 ????
    scaledPoint[1] = (posVolume[1] + posCell[1]) / (dims[1] - 1.f);
    scaledPoint[2] = (posVolume[2] + posCell[2]) / (dims[2] - 1.f);

    return scaledPoint;
}

MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
    : edgeToVertex_()
    , vertices_()
    , mesh_(std::make_shared<BasicMesh>())
    , indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)) {
    mesh_->setModelMatrix(vol->getModelMatrix());
    mesh_->setWorldMatrix(vol->getWorldMatrix());
}

void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
    IVW_ASSERT(i0 != i1, "i0 and i1 should not be the same value");
    IVW_ASSERT(i0 != i2, "i0 and i2 should not be the same value");
    IVW_ASSERT(i1 != i2, "i1 and i2 should not be the same value");

    indexBuffer_->add(static_cast<glm::uint32_t>(i0));
    indexBuffer_->add(static_cast<glm::uint32_t>(i1));
    indexBuffer_->add(static_cast<glm::uint32_t>(i2));

    const auto a = std::get<0>(vertices_[i0]);
    const auto b = std::get<0>(vertices_[i1]);
    const auto c = std::get<0>(vertices_[i2]);

    const vec3 n = glm::normalize(glm::cross(b - a, c - a));
    std::get<1>(vertices_[i0]) += n;
    std::get<1>(vertices_[i1]) += n;
    std::get<1>(vertices_[i2]) += n;
}

std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
    for (auto& vertex : vertices_) {
        // Normalize the normal of the vertex
        std::get<1>(vertex) = glm::normalize(std::get<1>(vertex));
    }
    mesh_->addVertices(vertices_);
    return mesh_;
}

std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
    IVW_ASSERT(i != j, "i and j should not be the same value");
    if (j < i) std::swap(i, j);

    auto [edgeIt, inserted] = edgeToVertex_.try_emplace(std::make_pair(i, j), vertices_.size());
    if (inserted) {
        vertices_.push_back({pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f)});
    }
    return static_cast<std::uint32_t>(edgeIt->second);
}

}  // namespace inviwo
