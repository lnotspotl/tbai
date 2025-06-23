#include <functional>
#include <iostream>

#include <tbai_wtw/HistoryWrapper.hpp>

namespace tbai {
namespace wtw {

HistoryBuffer::HistoryBuffer(std::size_t numObs, std::size_t historyLength)
    : numObs_(numObs), historyLength_(historyLength), currentIndex_(0) {
    for (std::size_t i = 0; i < historyLength; i++) {
        history_.push_back(tbai::vector_t::Zero(numObs));
    }
}

void HistoryBuffer::addObservation(tbai::vector_t obs) {
    history_[currentIndex_] = std::move(obs);
    currentIndex_ = (currentIndex_ + 1) % historyLength_;
}

tbai::vector_t HistoryBuffer::getFinalObservation() {
    std::vector<std::reference_wrapper<const tbai::vector_t>> historyRefs;
    for (std::size_t i = 0; i < historyLength_; i++) {
        const std::size_t index = (currentIndex_ - 1 - i + historyLength_) % historyLength_;
        historyRefs.push_back(history_[index]);
    }
    return tbai::vvstack(historyRefs);
}

}  // namespace wtw
}  // namespace tbai