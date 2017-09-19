#include <boost/test/unit_test.hpp>

#include <depth_map_preprocessing/PointCloudConversion.hpp>

using namespace depth_map_preprocessing;

BOOST_AUTO_TEST_CASE(test_computeLocalTransfromations)
{
    Eigen::Affine3d t0 = Eigen::Affine3d::Identity();
    t0.translation() = Eigen::Vector3d(1,2,3);
    Eigen::Affine3d t1 = Eigen::Affine3d::Identity();
    t1.translation() = t0.translation() + Eigen::Vector3d(1,0,0);
    Eigen::Affine3d t2 = Eigen::Affine3d::Identity();
    t2.translation() = t1.translation() + Eigen::Vector3d(1,0,0);
    PointCloudConversion::TransformationVector transforms;
    transforms.push_back(t0);
    transforms.push_back(t1);
    transforms.push_back(t2);
    PointCloudConversion::TransformationVector delta_transforms;

    PointCloudConversion::computeLocalTransfromations(transforms, delta_transforms);

    BOOST_CHECK(delta_transforms.size() == 3);
    BOOST_CHECK(delta_transforms[0].translation().isApprox(Eigen::Vector3d(-2,0,0)));
    BOOST_CHECK(delta_transforms[1].translation().isApprox(Eigen::Vector3d(-1,0,0)));
    BOOST_CHECK(delta_transforms[2].translation().isApprox(Eigen::Vector3d(0,0,0)));
}

BOOST_AUTO_TEST_CASE(test_convertToPointCloud_left_to_right)
{
    base::samples::DepthMap depth_map;
    depth_map.distances.resize(3,1.f);
    depth_map.horizontal_size = 3;
    depth_map.vertical_size = 1;
    depth_map.horizontal_interval.push_back(M_PI_2);
    depth_map.horizontal_interval.push_back(-M_PI_2);
    depth_map.vertical_interval.push_back(0.f);
    depth_map.vertical_projection = base::samples::DepthMap::POLAR;
    depth_map.horizontal_projection = base::samples::DepthMap::POLAR;
    depth_map.timestamps.push_back(base::Time::fromSeconds(0.0));
    depth_map.timestamps.push_back(base::Time::fromSeconds(1.0));
    depth_map.timestamps.push_back(base::Time::fromSeconds(2.0));

    Eigen::Affine3d t0 = Eigen::Affine3d::Identity();
    t0.translation() = Eigen::Vector3d(1,2,3);
    Eigen::Affine3d t1 = Eigen::Affine3d::Identity();
    t1.translation() = t0.translation() + Eigen::Vector3d(1,0,0);
    Eigen::Affine3d t2 = Eigen::Affine3d::Identity();
    t2.translation() = t1.translation() + Eigen::Vector3d(1,0,0);

    PointCloudConversion::TransformationVector transforms;
    transforms.push_back(t0);
    transforms.push_back(t1);
    transforms.push_back(t2);
    std::vector<Eigen::Vector3d> pc;

    // test horizontal compensation
    BOOST_CHECK(PointCloudConversion::convertToPointCloud(depth_map, transforms, Horizontal, pc));
    BOOST_CHECK(pc.size() == 3);
    BOOST_CHECK(pc[0].isApprox(Eigen::Vector3d(-2,1,0)));
    BOOST_CHECK(pc[1].isApprox(Eigen::Vector3d(0,0,0)));
    BOOST_CHECK(pc[2].isApprox(Eigen::Vector3d(0,-1,0)));

    transforms.clear();
    pc.clear();
    transforms.push_back(t0);
    transforms.push_back(t2);

    // test horizontal interpolated compensation
    BOOST_CHECK(PointCloudConversion::convertToPointCloud(depth_map, transforms, HorizontalInterpolation, pc));
    BOOST_CHECK(pc.size() == 3);
    BOOST_CHECK(pc[0].isApprox(Eigen::Vector3d(-2,1,0)));
    BOOST_CHECK(pc[1].isApprox(Eigen::Vector3d(0,0,0)));
    BOOST_CHECK(pc[2].isApprox(Eigen::Vector3d(0,-1,0)));
}

BOOST_AUTO_TEST_CASE(test_convertToPointCloud_right_to_left)
{
    base::samples::DepthMap depth_map;
    depth_map.distances.resize(3,1.f);
    depth_map.horizontal_size = 3;
    depth_map.vertical_size = 1;
    depth_map.horizontal_interval.push_back(M_PI_2);
    depth_map.horizontal_interval.push_back(-M_PI_2);
    depth_map.vertical_interval.push_back(0.f);
    depth_map.vertical_projection = base::samples::DepthMap::POLAR;
    depth_map.horizontal_projection = base::samples::DepthMap::POLAR;
    depth_map.timestamps.push_back(base::Time::fromSeconds(2.0));
    depth_map.timestamps.push_back(base::Time::fromSeconds(1.0));
    depth_map.timestamps.push_back(base::Time::fromSeconds(0.0));

    Eigen::Affine3d t0 = Eigen::Affine3d::Identity();
    t0.translation() = Eigen::Vector3d(1,2,3);
    Eigen::Affine3d t1 = Eigen::Affine3d::Identity();
    t1.translation() = t0.translation() + Eigen::Vector3d(1,0,0);
    Eigen::Affine3d t2 = Eigen::Affine3d::Identity();
    t2.translation() = t1.translation() + Eigen::Vector3d(1,0,0);

    PointCloudConversion::TransformationVector transforms;
    transforms.push_back(t0);
    transforms.push_back(t1);
    transforms.push_back(t2);
    std::vector<Eigen::Vector3d> pc;

    // test horizontal compensation
    BOOST_CHECK(PointCloudConversion::convertToPointCloud(depth_map, transforms, Horizontal, pc));
    BOOST_CHECK(pc.size() == 3);
    BOOST_CHECK(pc[0].isApprox(Eigen::Vector3d(0,1,0)));
    BOOST_CHECK(pc[1].isApprox(Eigen::Vector3d(0,0,0)));
    BOOST_CHECK(pc[2].isApprox(Eigen::Vector3d(-2,-1,0)));

    transforms.clear();
    pc.clear();
    transforms.push_back(t0);
    transforms.push_back(t2);

    // test horizontal interpolated compensation
    BOOST_CHECK(PointCloudConversion::convertToPointCloud(depth_map, transforms, HorizontalInterpolation, pc));
    BOOST_CHECK(pc.size() == 3);
    BOOST_CHECK(pc[0].isApprox(Eigen::Vector3d(0,1,0)));
    BOOST_CHECK(pc[1].isApprox(Eigen::Vector3d(0,0,0)));
    BOOST_CHECK(pc[2].isApprox(Eigen::Vector3d(-2,-1,0)));
}
