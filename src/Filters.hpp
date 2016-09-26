#ifndef _DEPTH_MAP_PREPROCESSING_FILTERS_HPP_
#define _DEPTH_MAP_PREPROCESSING_FILTERS_HPP_

#include <base/samples/DepthMap.hpp>

namespace depth_map_preprocessing
{

class Filters
{
public:

   /**
    * This method filters outliers according to a given minimum distance from the scan origin.
    * @param laser_scan input scan
    * @param min_range min range each measurement must have
    */
    static void filterMinDistance(base::samples::DepthMap &laser_scan, float min_range);

   /**
    * This method filters outliers according to the maximum angle to each of their four direct neighbors.
    * It aims to remove outliers which occure separated and therefore have high angles to their next neighbors.
    * The angle is allways from the plain described by the unit vector of the current measurement and the vector
    * from the current measurement to the neighbor measurement.
    * @param laser_scan input scan
    * @param max_deviation_angle the angle to a valid neighbor must be lower than this angle. Values in range [0,0.5*PI]
    * @param min_neighbors minimum number of valid neighbors each measurement should have
    */
    static void filterOutliers(base::samples::DepthMap &laser_scan, double max_deviation_angle, unsigned min_neighbors = 1);

protected:

    /**
     * Computes the angle between the plain described by the unit vector of the origin and the vector
     * from the origin to neighbor.
     * If the angle is smaller or equal to max_angle returns 1, 0 otherwiese.
     */
    static unsigned checkPoints(const Eigen::Vector3f& origin, const Eigen::Vector3f& neighbor, double max_angle);
};

} // end namespace depth_map_preprocessing

#endif // _DEPTH_MAP_PREPROCESSING_FILTERS_HPP_
