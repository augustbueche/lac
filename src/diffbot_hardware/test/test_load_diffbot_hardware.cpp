#include <gtest/gtest.h>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/system_interface.hpp"

class DiffbotHardwarePluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    loader_ = std::make_shared<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
      "hardware_interface", "hardware_interface::SystemInterface");
  }

  std::shared_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> loader_;
};

TEST_F(DiffbotHardwarePluginTest, LoadPluginSuccessfully)
{
  // Replace with your full plugin name: "<your_package_name>/<YourPluginClass>"
  const std::string plugin_name = "diffbot_hardware/DiffBotSystem";

  EXPECT_NO_THROW({
    auto system = loader_->createSharedInstance(plugin_name);
    EXPECT_NE(system, nullptr);
  });
}
