#include "PointCloudConversion.hpp"

using namespace depth_map_preprocessing;

void PointCloudConversion::computeLocalTransfromations(const PointCloudConversion::TransformationVector& transformations,
                                                       PointCloudConversion::TransformationVector& laserLinesToLatestLine)
{
   // computes the poses of t_i expressed in t_latest;
   laserLinesToLatestLine.resize(transformations.size());
   for(unsigned i = 0; i < transformations.size(); i++)
       laserLinesToLatestLine[i] = transformations.back().inverse() * transformations[i];
}