#pragma once

#include "ByteTrack/Rect.h"
#include <vector>
namespace byte_track
{
struct Object
{
    Rect<float> rect;
    int label;
    float prob;
    



    //目标框特征：长度为80 ，40，20
    std::vector<float> rect_feature;



    Object(){}
    Object(const Rect<float> &_rect,
           const int &_label,
           const float &_prob);
};
}