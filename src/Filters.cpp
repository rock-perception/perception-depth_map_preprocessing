#include "Filters.hpp"
#include <base/Float.hpp>

using namespace depth_map_preprocessing;

void Filters::filterMinDistance(base::samples::DepthMap& laser_scan, float min_range)
{
    for(unsigned i = 0; i < laser_scan.distances.size(); i++)
    {
        // check min range of measurement
        if(laser_scan.distances[i] < min_range)
            laser_scan.distances[i] = 0.0;
    }
}

void Filters::filterOutliers(base::samples::DepthMap& laser_scan, double max_deviation_angle, unsigned int min_neighbors)
{
    if(laser_scan.horizontal_size == 0 || laser_scan.vertical_size == 0)
        return;

    std::vector<Eigen::Vector3f> points;
    laser_scan.convertDepthMapToPointCloud(points, true, false);
    assert(points.size() == laser_scan.distances.size());

    base::samples::DepthMap::DepthMatrixMap distances = laser_scan.getDistanceMatrixMap();
    unsigned h_max = laser_scan.horizontal_size - 1;
    unsigned v_max = laser_scan.vertical_size - 1;
    unsigned valid_neighbors = 0;
    for(unsigned c = 0; c < laser_scan.vertical_size; c++)
    {
        for(unsigned r = 0; r < laser_scan.horizontal_size; r++)
        {
            const Eigen::Vector3f& origin = points[laser_scan.getIndex(c,r)];
            valid_neighbors = (r > 0 ? checkPoints(origin, points[laser_scan.getIndex(c,r-1)], max_deviation_angle) : 1) +
                              (c > 0 ? checkPoints(origin, points[laser_scan.getIndex(c-1,r)], max_deviation_angle) : 1) +
                              (c < v_max ? checkPoints(origin, points[laser_scan.getIndex(c+1,r)], max_deviation_angle) : 1) +
                              (r < h_max ? checkPoints(origin, points[laser_scan.getIndex(c,r+1)], max_deviation_angle) : 1);

            // invalidate distance measurement
            if(valid_neighbors < min_neighbors)
                distances(c,r) = base::NaN<float>();
        }
    }
}

unsigned Filters::checkPoints(const Eigen::Vector3f& origin, const Eigen::Vector3f& neighbor, double max_angle)
{
    if(!origin.allFinite() || !neighbor.allFinite())
        return 0;
    Eigen::Vector3f from_origin = neighbor - origin;
    if(std::abs(asin(origin.normalized().dot(from_origin) / from_origin.norm())) <= max_angle)
        return 1;
    return 0;
}