#ifndef _DEPTH_MAP_PREPROCESSING_CONFIG_HPP_
#define _DEPTH_MAP_PREPROCESSING_CONFIG_HPP_

namespace depth_map_preprocessing
{

/**
 * :NoCompensation
 *    No motion compensation will be applieed.
 * :HorizontalInterpolation and :VerticalInterpolation
 *    Will interpolate the local movement by the first and the last timestamp
 *    and will apply the transformations either horizontal (column-wise) or
 *    vertical (row-wise) in the frame of the newest scan.
 *    Note that |depth_map.timestamps| >= 2.
 * :Horizontal and :Vertical
 *    Will get one transformation for each timestamp in depth_map.timestamps
 *    and will apply them in the frame of the newest scan.
 *    Note that |depth_map.timestamps| == |depth_map.horizontal_size| for the
 *    horizontal case and |depth_map.timestamps| == |depth_map.vertical_size|
 *    for the vertical case.
 */
enum MotionCompensation
{
    NoCompensation = 0,
    HorizontalInterpolation,
    Horizontal,
    VerticalInterpolation,
    Vertical
};

}

#endif