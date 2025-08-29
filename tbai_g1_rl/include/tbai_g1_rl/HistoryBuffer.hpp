#pragma once

#include <cstddef>
#include <deque>

#include <tbai_core/Utils.hpp>

namespace tbai {
namespace g1_rl {

class HistoryBuffer {
   public:
    HistoryBuffer(std::size_t numObs, std::size_t historyLength);
    void addObservation(tbai::vector_t obs);
    tbai::vector_t getFinalObservation();

   private:
    std::size_t numObs_;
    std::size_t historyLength_;
    std::deque<tbai::vector_t> history_;
};

}  // namespace g1_rl
}  // namespace tbai
