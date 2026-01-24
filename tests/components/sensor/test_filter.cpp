#include "esphome/core/hal.h"
#include "esphome/core/application.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensor/filter.h"
#include "../test_helpers.h"
#include <gtest/gtest.h>
#include <cmath>

namespace esphome::sensor {
namespace {

class FilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Default accuracy is 0, which means rounding to integers for comparisons in some filters.
    // Setting it to 2 to allow finer grained comparisons if needed.
    sensor_.set_accuracy_decimals(2);
    set_millis(0);
  }

  Sensor sensor_;
};

TEST_F(FilterTest, OffsetFilter) {
  OffsetFilter filter(10.0f);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 15.0f);
}

TEST_F(FilterTest, MultiplyFilter) {
  MultiplyFilter filter(2.0f);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);
}

TEST_F(FilterTest, FilterOutValueFilter) {
  FilterOutValueFilter filter({10.0f, 20.0f});
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 5.0f);
  EXPECT_EQ(callback_count, 1);

  sensor_.publish_state(10.0f);
  EXPECT_EQ(callback_count, 1);  // Should not update

  sensor_.publish_state(20.0f);
  EXPECT_EQ(callback_count, 1);  // Should not update

  sensor_.publish_state(21.0f);
  EXPECT_FLOAT_EQ(received_value, 21.0f);
  EXPECT_EQ(callback_count, 2);
}

TEST_F(FilterTest, MinFilter) {
  MinFilter filter(/*window_size=*/3, /*send_every=*/1, /*send_first_at=*/1);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(10.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 5.0f);

  sensor_.publish_state(8.0f);
  EXPECT_FLOAT_EQ(received_value, 5.0f);

  sensor_.publish_state(12.0f);
  // Window logic:
  // 10 -> [10], min 10
  // 5 -> [10, 5], min 5
  // 8 -> [10, 5, 8], min 5
  // 12 -> [5, 8, 12] (overwrite 10), min 5
  EXPECT_FLOAT_EQ(received_value, 5.0f);

  sensor_.publish_state(20.0f);
  // Window: [8, 12, 20] (overwrite 5), min 8
  EXPECT_FLOAT_EQ(received_value, 8.0f);
}

TEST_F(FilterTest, MaxFilter) {
  MaxFilter filter(/*window_size=*/3, /*send_every=*/1, /*send_first_at=*/1);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(10.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);

  sensor_.publish_state(15.0f);
  EXPECT_FLOAT_EQ(received_value, 15.0f);

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 15.0f);

  sensor_.publish_state(20.0f);  // [10, 15, 5] -> [15, 5, 20] (overwrite 10)
  EXPECT_FLOAT_EQ(received_value, 20.0f);
}

TEST_F(FilterTest, SlidingWindowMovingAverageFilter) {
  SlidingWindowMovingAverageFilter filter(/*window_size=*/3, /*send_every=*/1, /*send_first_at=*/1);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(10.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);  // [10] / 1

  sensor_.publish_state(20.0f);
  EXPECT_FLOAT_EQ(received_value, 15.0f);  // [10, 20] / 2

  sensor_.publish_state(30.0f);
  EXPECT_FLOAT_EQ(received_value, 20.0f);  // [10, 20, 30] / 3

  sensor_.publish_state(40.0f);  // [20, 30, 40] / 3 = 90 / 3 = 30
  EXPECT_FLOAT_EQ(received_value, 30.0f);
}

TEST_F(FilterTest, LambdaFilter) {
  auto lambda = [](float value) -> optional<float> {
    if (value < 0)
      return {};
    return value * 2;
  };
  LambdaFilter filter(lambda);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);
  EXPECT_EQ(callback_count, 1);

  sensor_.publish_state(-5.0f);
  EXPECT_EQ(callback_count, 1);  // Should be filtered out
}

TEST_F(FilterTest, ClampFilter) {
  ClampFilter filter(/*min=*/0.0f, /*max=*/100.0f, /*ignore_out_of_range=*/false);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(50.0f);
  EXPECT_FLOAT_EQ(received_value, 50.0f);

  sensor_.publish_state(-10.0f);
  EXPECT_FLOAT_EQ(received_value, 0.0f);

  sensor_.publish_state(150.0f);
  EXPECT_FLOAT_EQ(received_value, 100.0f);
}

TEST_F(FilterTest, ClampFilterIgnore) {
  ClampFilter filter(/*min=*/0.0f, /*max=*/100.0f, /*ignore_out_of_range=*/true);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(50.0f);
  EXPECT_FLOAT_EQ(received_value, 50.0f);
  EXPECT_EQ(callback_count, 1);

  sensor_.publish_state(-10.0f);
  EXPECT_EQ(callback_count, 1);  // Should be ignored

  sensor_.publish_state(150.0f);
  EXPECT_EQ(callback_count, 1);  // Should be ignored
}

TEST_F(FilterTest, MedianFilter) {
  MedianFilter filter(/*window_size=*/3, /*send_every=*/1, /*send_first_at=*/1);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(10.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);

  sensor_.publish_state(50.0f);
  EXPECT_FLOAT_EQ(received_value, 30.0f);  // [10, 50] -> 30

  sensor_.publish_state(20.0f);
  EXPECT_FLOAT_EQ(received_value, 20.0f);  // [10, 50, 20] -> sorted [10, 20, 50] -> median 20

  sensor_.publish_state(100.0f);
  EXPECT_FLOAT_EQ(received_value, 50.0f);  // [50, 20, 100] -> sorted [20, 50, 100] -> median 50
}

TEST_F(FilterTest, QuantileFilter) {
  QuantileFilter filter(/*window_size=*/5, /*send_every=*/1, /*send_first_at=*/1, /*quantile=*/0.9f);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(1.0f);
  EXPECT_FLOAT_EQ(received_value, 1.0f);

  sensor_.publish_state(2.0f);
  EXPECT_FLOAT_EQ(received_value, 2.0f);  // [1, 2] -> ceil(2*0.9)-1 = 1 -> index 1 -> 2

  sensor_.publish_state(3.0f);
  EXPECT_FLOAT_EQ(received_value, 3.0f);  // [1, 2, 3] -> ceil(3*0.9)-1 = 2 -> index 2 -> 3

  sensor_.publish_state(4.0f);
  EXPECT_FLOAT_EQ(received_value, 4.0f);  // [1, 2, 3, 4] -> ceil(4*0.9)-1 = 3 -> index 3 -> 4

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 5.0f);  // [1, 2, 3, 4, 5] -> ceil(5*0.9)-1 = 4 -> index 4 -> 5

  sensor_.publish_state(0.0f);
  // Window: [0, 2, 3, 4, 5]. Sorted: [0, 2, 3, 4, 5].
  // Quantile 0.9. Size 5. Position = ceil(5 * 0.9) - 1 = ceil(4.5) - 1 = 5 - 1 = 4.
  // Value at index 4 is 5.
  EXPECT_FLOAT_EQ(received_value, 5.0f);
}

TEST_F(FilterTest, ExponentialMovingAverageFilter) {
  ExponentialMovingAverageFilter filter(/*alpha=*/0.1f, /*send_every=*/1, /*send_first_at=*/1);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(10.0f);
  EXPECT_FLOAT_EQ(received_value, 10.0f);  // First value is initialized

  sensor_.publish_state(20.0f);
  // (0.1 * 20) + (0.9 * 10) = 2 + 9 = 11
  EXPECT_FLOAT_EQ(received_value, 11.0f);

  sensor_.publish_state(20.0f);
  // (0.1 * 20) + (0.9 * 11) = 2 + 9.9 = 11.9
  EXPECT_FLOAT_EQ(received_value, 11.9f);
}

TEST_F(FilterTest, CalibrateLinearFilter) {
  // x < 10: y = x (slope 1, offset 0)
  // x >= 10: y = 2x + 1 (slope 2, offset 1)
  CalibrateLinearFilter filter({
      {1.0f, 0.0f, 10.0f},
      {2.0f, 1.0f, INFINITY},
  });
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(5.0f);
  EXPECT_FLOAT_EQ(received_value, 5.0f);

  sensor_.publish_state(10.0f);
  // Matches second range (>= 10 checks < 10 fails, then < inf passes)
  // 10 * 2 + 1 = 21
  EXPECT_FLOAT_EQ(received_value, 21.0f);

  sensor_.publish_state(100.0f);
  // 100 * 2 + 1 = 201
  EXPECT_FLOAT_EQ(received_value, 201.0f);
}

TEST_F(FilterTest, SkipInitialFilter) {
  SkipInitialFilter filter(/*num_to_ignore=*/2);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(1.0f);
  EXPECT_EQ(callback_count, 0);

  sensor_.publish_state(2.0f);
  EXPECT_EQ(callback_count, 0);

  sensor_.publish_state(3.0f);
  EXPECT_EQ(callback_count, 1);
  EXPECT_FLOAT_EQ(received_value, 3.0f);
}

TEST_F(FilterTest, DeltaFilter) {
  DeltaFilter filter(/*min_a0=*/5.0f, /*min_a1=*/0.0f, /*max_a0=*/INFINITY, /*max_a1=*/0.0f);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(10.0f);
  EXPECT_EQ(callback_count, 1);
  EXPECT_FLOAT_EQ(received_value, 10.0f);

  sensor_.publish_state(12.0f);  // Delta 2 < 5
  EXPECT_EQ(callback_count, 1);

  sensor_.publish_state(16.0f);  // Delta 6 > 5
  EXPECT_EQ(callback_count, 2);
  EXPECT_FLOAT_EQ(received_value, 16.0f);
}

TEST_F(FilterTest, RoundFilter) {
  RoundFilter filter(/*precision=*/1);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(1.234f);
  EXPECT_FLOAT_EQ(received_value, 1.2f);

  sensor_.publish_state(1.256f);
  EXPECT_FLOAT_EQ(received_value, 1.3f);
}

TEST_F(FilterTest, RoundMultipleFilter) {
  RoundMultipleFilter filter(/*multiple=*/0.25f);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(1.1f);
  EXPECT_FLOAT_EQ(received_value, 1.0f);

  sensor_.publish_state(1.15f);
  EXPECT_FLOAT_EQ(received_value, 1.25f);
}

TEST_F(FilterTest, CalibratePolynomialFilter) {
  // y = 1 + 2x + 3x^2
  CalibratePolynomialFilter filter({1.0f, 2.0f, 3.0f});
  sensor_.add_filter(&filter);

  float received_value = NAN;
  sensor_.add_on_state_callback([&](float value) { received_value = value; });

  sensor_.publish_state(2.0f);
  // 1 + 2*2 + 3*4 = 1 + 4 + 12 = 17
  EXPECT_FLOAT_EQ(received_value, 17.0f);
}

TEST_F(FilterTest, StreamingMinFilter) {
  StreamingMinFilter filter(/*window_size=*/3, /*send_first_at=*/3);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(10.0f);
  EXPECT_EQ(callback_count, 0);

  sensor_.publish_state(5.0f);
  EXPECT_EQ(callback_count, 0);

  sensor_.publish_state(15.0f);
  EXPECT_EQ(callback_count, 1);
  EXPECT_FLOAT_EQ(received_value, 5.0f);

  sensor_.publish_state(20.0f);
  sensor_.publish_state(1.0f);
  sensor_.publish_state(30.0f);
  EXPECT_EQ(callback_count, 2);
  EXPECT_FLOAT_EQ(received_value, 1.0f);
}

TEST_F(FilterTest, StreamingMaxFilter) {
  StreamingMaxFilter filter(/*window_size=*/3, /*send_first_at=*/3);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(10.0f);
  sensor_.publish_state(25.0f);
  sensor_.publish_state(5.0f);
  EXPECT_EQ(callback_count, 1);
  EXPECT_FLOAT_EQ(received_value, 25.0f);
}

TEST_F(FilterTest, StreamingMovingAverageFilter) {
  StreamingMovingAverageFilter filter(/*window_size=*/3, /*send_first_at=*/3);
  sensor_.add_filter(&filter);

  float received_value = NAN;
  int callback_count = 0;
  sensor_.add_on_state_callback([&](float value) {
    received_value = value;
    callback_count++;
  });

  sensor_.publish_state(10.0f);
  sensor_.publish_state(20.0f);
  sensor_.publish_state(30.0f);
  EXPECT_EQ(callback_count, 1);
  EXPECT_FLOAT_EQ(received_value, 20.0f);
}

}  // namespace
}  // namespace esphome::sensor
