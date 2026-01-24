#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensor/automation.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/preferences.h"
#include "tests/mocks/mock_preferences.h"
#include <gtest/gtest.h>
#include <cmath>

namespace esphome {
namespace sensor {

class SensorAutomationTest : public ::testing::Test {
 protected:
  void SetUp() override { global_preferences = &mock_prefs_; }

  void TearDown() override { global_preferences = nullptr; }

  Sensor sensor_;
  MockPreferences mock_prefs_;
};

TEST_F(SensorAutomationTest, SensorStateTrigger) {
  SensorStateTrigger trigger(&sensor_);
  float triggered_value = NAN;
  bool triggered = false;

  auto *automation = new esphome::Automation<float>(&trigger);
  auto *action = new esphome::LambdaAction<float>([&](float x) {
    triggered_value = x;
    triggered = true;
  });
  automation->add_action(action);

  sensor_.publish_state(42.0f);

  EXPECT_TRUE(triggered);
  EXPECT_FLOAT_EQ(triggered_value, 42.0f);

  delete automation;
  delete action;
  // Trigger is on stack, no delete needed.
}

TEST_F(SensorAutomationTest, SensorRawStateTrigger) {
  SensorRawStateTrigger trigger(&sensor_);
  float triggered_value = NAN;
  bool triggered = false;

  auto *automation = new esphome::Automation<float>(&trigger);
  auto *action = new esphome::LambdaAction<float>([&](float x) {
    triggered_value = x;
    triggered = true;
  });
  automation->add_action(action);

  // Add a filter that changes the value to ensure we get RAW state
  auto *filter = new MultiplyFilter(2.0f);
  sensor_.add_filter(filter);

  sensor_.publish_state(10.0f);

  EXPECT_TRUE(triggered);
  EXPECT_FLOAT_EQ(triggered_value, 10.0f);  // Should be 10 (raw), not 20 (filtered)

  delete automation;
  delete action;
  delete filter;
  sensor_.clear_filters();  // Detach filter from sensor to avoid double-free or dangling usage if sensor_ persists
                            // (though it's destroyed after test)
}

TEST_F(SensorAutomationTest, ValueRangeTrigger_Basic) {
  ValueRangeTrigger *trigger = new ValueRangeTrigger(&sensor_);
  trigger->set_min(10.0f);
  trigger->set_max(20.0f);
  trigger->setup();

  bool triggered = false;
  float triggered_value = NAN;

  auto *automation = new esphome::Automation<float>(trigger);
  auto *action = new esphome::LambdaAction<float>([&](float x) {
    triggered_value = x;
    triggered = true;
  });
  automation->add_action(action);

  // Start below range
  sensor_.publish_state(5.0f);
  EXPECT_FALSE(triggered);

  // Enter range
  sensor_.publish_state(15.0f);
  EXPECT_TRUE(triggered);
  EXPECT_FLOAT_EQ(triggered_value, 15.0f);

  // Reset trigger flag
  triggered = false;
  triggered_value = NAN;

  // Stay in range - should NOT trigger again (hysteresis)
  sensor_.publish_state(16.0f);
  EXPECT_FALSE(triggered);

  // Exit range
  sensor_.publish_state(25.0f);
  EXPECT_FALSE(triggered);  // Exiting doesn't trigger, only entering/being in range?
                            // Wait, code says: if (in_range != this->previous_in_range_ && in_range)
                            // So it only triggers on raising edge of "in_range" condition.

  // Re-enter range
  sensor_.publish_state(19.0f);
  EXPECT_TRUE(triggered);
  EXPECT_FLOAT_EQ(triggered_value, 19.0f);

  delete automation;
  delete action;
  delete trigger;
}

TEST_F(SensorAutomationTest, ValueRangeTrigger_Persistence) {
  // Test that state is saved to preferences
  // We use local dynamic sensor to control lifetime and clear callbacks

  // 1. Create sensor and trigger, enter range
  Sensor *local_sensor = new Sensor();
  ValueRangeTrigger *trigger1 = new ValueRangeTrigger(local_sensor);
  trigger1->set_min(10.0f);
  trigger1->set_max(20.0f);

  trigger1->setup();

  local_sensor->publish_state(15.0f);  // Enter range -> previous_in_range_ becomes true

  delete trigger1;
  delete local_sensor;  // This clears the callback

  // 2. Simulate Reboot: Create NEW sensor and NEW trigger
  local_sensor = new Sensor();
  ValueRangeTrigger *trigger2 = new ValueRangeTrigger(local_sensor);
  trigger2->set_min(10.0f);
  trigger2->set_max(20.0f);

  bool triggered = false;
  auto *automation = new esphome::Automation<float>(trigger2);
  auto *action = new esphome::LambdaAction<float>([&](float x) { triggered = true; });
  automation->add_action(action);

  trigger2->setup();  // Should load previous state (true)

  // 3. Publish in-range value again.
  // If state was NOT restored, previous_in_range_ would be false (default), and this WOULD trigger.
  // If state WAS restored, previous_in_range_ is true, and this would NOT trigger (no change).
  local_sensor->publish_state(16.0f);

  EXPECT_FALSE(triggered);

  delete automation;  // cleans up trigger2
  delete action;
  delete trigger2;
  delete local_sensor;
}

TEST_F(SensorAutomationTest, SensorInRangeCondition) {
  SensorInRangeCondition<float> condition(&sensor_);
  condition.set_min(10.0f);
  condition.set_max(20.0f);

  sensor_.state = 5.0f;
  EXPECT_FALSE(condition.check(0.0f));

  sensor_.state = 15.0f;
  EXPECT_TRUE(condition.check(0.0f));

  sensor_.state = 25.0f;
  EXPECT_FALSE(condition.check(0.0f));

  // Open ended
  condition.set_min(NAN);
  condition.set_max(20.0f);
  sensor_.state = -100.0f;
  EXPECT_TRUE(condition.check(0.0f));
  sensor_.state = 21.0f;
  EXPECT_FALSE(condition.check(0.0f));
}

}  // namespace sensor
}  // namespace esphome
