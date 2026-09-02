#pragma once

#include <cstddef>
#include <memory>
#include <unordered_map>

#include "train/Train.hpp"

namespace tcas::train {

class TrainManager {
 public:
  TrainManager() = default;

  bool addTrain(std::unique_ptr<Train> train);

  bool removeTrain(TrainId id);

  [[nodiscard]]
  Train* getTrain(TrainId id) noexcept;

  [[nodiscard]]
  const Train* getTrain(TrainId id) const noexcept;

  [[nodiscard]]
  bool contains(TrainId id) const noexcept;

  [[nodiscard]]
  std::size_t trainCount() const noexcept;

  void clear() noexcept;

 private:
  std::unordered_map<TrainId, std::unique_ptr<Train> > trains_;
};

}  // namespace tcas::train