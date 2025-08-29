#include <functional>
#include <iostream>

#include <tbai_g1_rl/HistoryBuffer.hpp>

namespace tbai {
namespace g1_rl {
HistoryBuffer::HistoryBuffer(std::size_t numObs, std::size_t historyLength)
    : numObs_(numObs), historyLength_(historyLength) {
    for (std::size_t i = 0; i < historyLength; i++) {
        history_.push_back(tbai::vector_t::Zero(numObs));
    }
}

void HistoryBuffer::addObservation(tbai::vector_t obs) {
    if (history_.size() == historyLength_) {
        history_.pop_front();
    }
    history_.push_back(std::move(obs));
}

tbai::vector_t HistoryBuffer::getFinalObservation() {
    std::vector<std::reference_wrapper<const tbai::vector_t>> historyRefs;
    historyRefs.reserve(history_.size());
    for (const auto& obs : history_) {
        historyRefs.push_back(obs);
    }
    return tbai::vvstack(historyRefs);
}

}  // namespace g1_rl
}  // namespace tbai