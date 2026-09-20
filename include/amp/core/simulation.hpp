#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <expected>
#include <filesystem>
#include <memory>
#include <span>
#include <string>

namespace amp {

class Simulation {
 public:
  static std::expected<Simulation, std::string> load(const std::filesystem::path& scene_path);

  Simulation(Simulation&&) noexcept = default;
  Simulation& operator=(Simulation&&) noexcept = default;
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  void reset();
  void step(std::size_t count = 1);

  [[nodiscard]] std::span<mjtNum> controls() noexcept;
  [[nodiscard]] const mjModel& model() const noexcept { return *model_; }
  [[nodiscard]] mjData& data() noexcept { return *data_; }
  [[nodiscard]] const mjData& data() const noexcept { return *data_; }
  [[nodiscard]] const std::filesystem::path& scene_path() const noexcept { return scene_path_; }

 private:
  struct ModelDeleter {
    void operator()(mjModel* model) const noexcept;
  };
  struct DataDeleter {
    void operator()(mjData* data) const noexcept;
  };

  Simulation(std::unique_ptr<mjModel, ModelDeleter> model,
             std::unique_ptr<mjData, DataDeleter> data, std::filesystem::path scene_path);

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  std::filesystem::path scene_path_;
};

}  // namespace amp
