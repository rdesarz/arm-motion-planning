#include "amp/core/simulation.hpp"

#include <array>
#include <system_error>
#include <utility>

namespace amp {

void Simulation::ModelDeleter::operator()(mjModel* model) const noexcept { mj_deleteModel(model); }

void Simulation::DataDeleter::operator()(mjData* data) const noexcept { mj_deleteData(data); }

Simulation::Simulation(std::unique_ptr<mjModel, ModelDeleter> model,
                       std::unique_ptr<mjData, DataDeleter> data, std::filesystem::path scene_path)
    : model_(std::move(model)), data_(std::move(data)), scene_path_(std::move(scene_path)) {}

std::expected<Simulation, std::string> Simulation::load(const std::filesystem::path& scene_path) {
  std::error_code filesystem_error;
  const auto absolute_path = std::filesystem::absolute(scene_path, filesystem_error);
  if (filesystem_error) {
    return std::unexpected("Cannot resolve scene path '" + scene_path.string() +
                           "': " + filesystem_error.message());
  }
  if (!std::filesystem::is_regular_file(absolute_path, filesystem_error)) {
    return std::unexpected("MuJoCo scene does not exist: " + absolute_path.string());
  }

  std::array<char, 2048> error{};
  auto model = std::unique_ptr<mjModel, ModelDeleter>(
      mj_loadXML(absolute_path.string().c_str(), nullptr, error.data(), error.size()));
  if (!model) {
    return std::unexpected("MuJoCo could not load '" + absolute_path.string() +
                           "': " + error.data());
  }

  auto data = std::unique_ptr<mjData, DataDeleter>(mj_makeData(model.get()));
  if (!data) {
    return std::unexpected("MuJoCo could not allocate simulation data for: " +
                           absolute_path.string());
  }

  mj_forward(model.get(), data.get());
  return Simulation(std::move(model), std::move(data), absolute_path);
}

void Simulation::reset() {
  mj_resetData(model_.get(), data_.get());
  mj_forward(model_.get(), data_.get());
}

void Simulation::step(const std::size_t count) {
  for (std::size_t step = 0; step < count; ++step) {
    mj_step(model_.get(), data_.get());
  }
}

std::span<mjtNum> Simulation::controls() noexcept {
  return {data_->ctrl, static_cast<std::size_t>(model_->nu)};
}

}  // namespace amp
