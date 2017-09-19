#ifndef _DEPTH_MAP_PREPROCESSING_POINTCLOUD_CONVERSION_HPP_
#define _DEPTH_MAP_PREPROCESSING_POINTCLOUD_CONVERSION_HPP_

#include <base-logging/Logging.hpp>
#include <base/samples/DepthMap.hpp>
#include <list>
#include <vector>
#include "Config.hpp"

namespace depth_map_preprocessing
{

class PointCloudConversion
{
public:
    typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> TransformationVector;

    /**
     * Converts the given sample to a pointcloud while applying the selected motion compensation option.
     * @param depth_map_sample input
     * @param depth_map_transforms Either two for interpolation or |horizontal| or |vertical| transforms
     * @param motion_compensation type of compensation
     * @param pointcloud output
     * @returns false on invalid input
     */
    template<class T>
    static bool convertToPointCloud(const base::samples::DepthMap &depth_map_sample,
                             const TransformationVector& depth_map_transforms,
                             MotionCompensation motion_compensation, std::vector<T>& pointcloud);


    /**
     * Helper method to compute the relative transformations inside of one scan.
     * Computes transformations[i] expressed in transformations[last].
     * It ensures |transformations| == |laserLinesInLatestLine|, therefore the last transformation is allways the Identity.
     * @param transformations laser lines in reference frame
     * @param laserLinesInLatestLine laser lines in frame of last line
     */
    static void computeLocalTransfromations(const TransformationVector& transformations, TransformationVector& laserLinesInLatestLine);
};



template<class T>
bool PointCloudConversion::convertToPointCloud(const base::samples::DepthMap& depth_map_sample,
                                               const TransformationVector& depth_map_transforms,
                                               MotionCompensation motion_compensation, std::vector< T >& pointcloud)
{
    TransformationVector laserLinesToLatestLine;
    computeLocalTransfromations(depth_map_transforms, laserLinesToLatestLine);

    if((motion_compensation == HorizontalInterpolation || motion_compensation == VerticalInterpolation) && laserLinesToLatestLine.size() == 2)
    {
        if(depth_map_sample.timestamps.front() <= depth_map_sample.timestamps.back())
            depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine.front(), laserLinesToLatestLine.back(),
                                                        true, true, motion_compensation == HorizontalInterpolation ? false : true);
        else
            depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine.back(), laserLinesToLatestLine.front(),
                                                        true, true, motion_compensation == HorizontalInterpolation ? false : true);
        return true;
    }
    else if((motion_compensation == Horizontal && laserLinesToLatestLine.size() == depth_map_sample.horizontal_size) ||
            (motion_compensation == Vertical && laserLinesToLatestLine.size() == depth_map_sample.vertical_size))
    {
        if(depth_map_sample.timestamps.front() <= depth_map_sample.timestamps.back())
            depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine,
                                                        true, true, motion_compensation == Horizontal ? false : true);
        else
        {
            TransformationVector laserLinesToLatestLine_reverse(laserLinesToLatestLine.size());
            for(unsigned i = 1; i <= laserLinesToLatestLine.size(); i++)
                laserLinesToLatestLine_reverse[laserLinesToLatestLine.size()-i] = laserLinesToLatestLine[i-1];
            depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine_reverse,
                                                        true, true, motion_compensation == Horizontal ? false : true);
        }
        return true;
    }
    return false;
}

}

#endif