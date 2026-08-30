#include "infrastructure/Track.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace tcas;
using namespace tcas::infrastructure;

TEST(TrackTest, StoresTrackInformation)
{
    Track track(
        100,
        1,
        2,
        1500.0,
        30.0,
        0.01
    );

    EXPECT_EQ(track.id(), 100);
    EXPECT_EQ(track.source(), 1);
    EXPECT_EQ(track.destination(), 2);

    EXPECT_DOUBLE_EQ(track.length(), 1500.0);
    EXPECT_DOUBLE_EQ(track.speedLimit(), 30.0);
    EXPECT_DOUBLE_EQ(track.gradient(), 0.01);
}
TEST(TrackTest, RejectsZeroLength)
{
    EXPECT_THROW(
        Track(
            1,
            1,
            2,
            0.0,
            30.0,
            0.0
        ),
        std::invalid_argument
    );
}

TEST(TrackTest, RejectsNegativeLength)
{
    EXPECT_THROW(
        Track(
            1,
            1,
            2,
            -100.0,
            30.0,
            0.0
        ),
        std::invalid_argument
    );
}

TEST(TrackTest, RejectsNegativeSpeedLimit)
{
    EXPECT_THROW(
        Track(
            1,
            1,
            2,
            1000.0,
            -10.0,
            0.0
        ),
        std::invalid_argument
    );
}

TEST(TrackTest, RejectsNonFiniteValues)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    EXPECT_THROW(Track(1, 1, 2, nan, 30.0, 0.0), std::invalid_argument);
    EXPECT_THROW(Track(1, 1, 2, 100.0, infinity, 0.0), std::invalid_argument);
    EXPECT_THROW(Track(1, 1, 2, 100.0, 30.0, nan), std::invalid_argument);
}

TEST(TrackTest, AllowsZeroSpeedLimit)
{
    EXPECT_NO_THROW(
        Track(
            1,
            1,
            2,
            1000.0,
            0.0,
            0.0
        )
    );
}
