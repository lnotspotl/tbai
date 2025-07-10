#pragma once

#include <cstddef>
#include <vector>

#include <tbai_core/Utils.hpp>

namespace tbai {
namespace np3o {

class HistoryBuffer {
   public:
    HistoryBuffer(std::size_t numObs, std::size_t historyLength);
    void addObservation(tbai::vector_t obs);
    tbai::vector_t getFinalObservation();

   private:
    std::size_t numObs_;
    std::size_t historyLength_;
    std::size_t currentIndex_;
    std::vector<tbai::vector_t> history_;
};

}  // namespace np3o
}  // namespace tbai
