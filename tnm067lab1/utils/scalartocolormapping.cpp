#include <modules/tnm067lab1/utils/scalartocolormapping.h>

namespace inviwo {

void ScalarToColorMapping::clearColors() { baseColors_.clear(); }
void ScalarToColorMapping::addBaseColors(vec4 color) { baseColors_.push_back(color); }

vec4 ScalarToColorMapping::sample(float t) const {
    if (baseColors_.size() == 0) return vec4(t);
    if (baseColors_.size() == 1) return vec4(baseColors_[0]);
    if (t <= 0) return vec4(baseColors_.front());
    if (t >= 1) return vec4(baseColors_.back());

    // TODO: use t to select which two base colors to interpolate in-between
    // plocka ut två färger från basecolors, med t
    // få hela storleken av baseColors_
    // t som index

    // RGB
    // 5 färger men size är 4

    float new_t = t * (baseColors_.size() - 1.f); // 0.4 * ( 5 - 1) = 1.6
    int index = new_t; // 1, avrundar neråt

    
    // interpolate those two values
   // int rightIndex = ceil(dummy);
   // int leftIndex = floor(dummy);

    vec4 col_L = baseColors_[index];
    vec4 col_R = baseColors_[index + 1];

    // linear interpolate
    // 1 - något

    // TODO: Interpolate colors in baseColors_ and set dummy color to result

    //vec4 finalColor(t, t, t, 1);  // dummy color
    vec4 finalColor = (col_R - col_L) * (new_t - index) + col_L;
       
    return finalColor;
}

}  // namespace inviwo
