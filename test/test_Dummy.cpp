#include <boost/test/unit_test.hpp>
#include <depth_map_preprocessing/Dummy.hpp>

using namespace depth_map_preprocessing;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    depth_map_preprocessing::DummyClass dummy;
    dummy.welcome();
}
