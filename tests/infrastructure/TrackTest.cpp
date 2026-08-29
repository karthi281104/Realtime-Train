#include "infrastructure/Track.hpp"

#include <gtest/gtest.h>

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