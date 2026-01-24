#pragma once

#include "esphome/core/preferences.h"
#include <map>
#include <vector>
#include <cstring>

namespace esphome {

class MockPreferenceBackend : public ESPPreferenceBackend {
 public:
  std::vector<uint8_t> data;

  bool save(const uint8_t *data, size_t len) override {
    this->data.assign(data, data + len);
    return true;
  }

  bool load(uint8_t *data, size_t len) override {
    if (this->data.size() != len)
      return false;
    std::memcpy(data, this->data.data(), len);
    return true;
  }
};

class MockPreferences : public ESPPreferences {
 public:
  std::map<uint32_t, MockPreferenceBackend *> backends;

  ESPPreferenceObject make_preference(size_t length, uint32_t type, bool in_flash) override {
    if (backends.find(type) == backends.end()) {
      backends[type] = new MockPreferenceBackend();
    }
    return ESPPreferenceObject(backends[type]);
  }

  ESPPreferenceObject make_preference(size_t length, uint32_t type) override {
    return make_preference(length, type, false);
  }

  bool sync() override { return true; }
  bool reset() override {
    for (auto &pair : backends) {
      delete pair.second;
    }
    backends.clear();
    return true;
  }

  virtual ~MockPreferences() { reset(); }
};

}  // namespace esphome
