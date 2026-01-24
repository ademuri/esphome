#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensor/filter.h"
#include <gtest/gtest.h>
#include <cmath>

namespace esphome {
namespace sensor {

class SensorTest : public ::testing::Test {
 protected:
  Sensor sensor_;
};

TEST_F(SensorTest, InitialState) {
  EXPECT_TRUE(std::isnan(sensor_.state));
  EXPECT_TRUE(std::isnan(sensor_.raw_state));
  EXPECT_FALSE(sensor_.has_state());
}

TEST_F(SensorTest, PublishState_NoFilters) {
  float received_state = NAN;
  float received_raw = NAN;

  sensor_.add_on_state_callback([&](float val) { received_state = val; });
  sensor_.add_on_raw_state_callback([&](float val) { received_raw = val; });

  sensor_.publish_state(42.5f);

  EXPECT_FLOAT_EQ(sensor_.state, 42.5f);
  EXPECT_FLOAT_EQ(sensor_.raw_state, 42.5f);
  EXPECT_FLOAT_EQ(received_state, 42.5f);
  EXPECT_FLOAT_EQ(received_raw, 42.5f);
  EXPECT_TRUE(sensor_.has_state());
}

TEST_F(SensorTest, PublishState_WithFilter) {
  // Add a MultiplyFilter(2.0)
  MultiplyFilter filter(2.0f);
  sensor_.add_filter(&filter);

  float received_state = NAN;
  float received_raw = NAN;

  sensor_.add_on_state_callback([&](float val) { received_state = val; });
  sensor_.add_on_raw_state_callback([&](float val) { received_raw = val; });

  sensor_.publish_state(10.0f);

  // Raw state should be original value
  EXPECT_FLOAT_EQ(sensor_.raw_state, 10.0f);
  EXPECT_FLOAT_EQ(received_raw, 10.0f);

  // Filtered state should be multiplied
  EXPECT_FLOAT_EQ(sensor_.state, 20.0f);
  EXPECT_FLOAT_EQ(received_state, 20.0f);
}

TEST_F(SensorTest, AccuracyDecimals) {
  // Default behavior
  EXPECT_EQ(sensor_.get_accuracy_decimals(), 0);

  // Set Override
  sensor_.set_accuracy_decimals(3);
  EXPECT_EQ(sensor_.get_accuracy_decimals(), 3);

  // Verify it doesn't affect state value itself (it's mostly for display/logging)
  sensor_.publish_state(3.14159f);
  EXPECT_FLOAT_EQ(sensor_.state, 3.14159f);
}

TEST_F(SensorTest, StateClass) {
  // Default behavior
  EXPECT_EQ(sensor_.get_state_class(), STATE_CLASS_NONE);

  // Set Override
  sensor_.set_state_class(STATE_CLASS_MEASUREMENT);
  EXPECT_EQ(sensor_.get_state_class(), STATE_CLASS_MEASUREMENT);

  sensor_.set_state_class(STATE_CLASS_TOTAL_INCREASING);
  EXPECT_EQ(sensor_.get_state_class(), STATE_CLASS_TOTAL_INCREASING);
}

TEST_F(SensorTest, ForceUpdate) {
  // Default is false
  EXPECT_FALSE(sensor_.get_force_update());

  sensor_.set_force_update(true);
  EXPECT_TRUE(sensor_.get_force_update());
}

}  // namespace sensor
}  // namespace esphome
