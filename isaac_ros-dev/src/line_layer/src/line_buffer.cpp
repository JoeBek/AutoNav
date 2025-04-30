#include "line_layer/line_buffer.hpp"

std::optional<T> LineBuffer::read() {
    std::lock_guard<std::mutex> lock(the_great_line_guardian_);
    if (q_.empty()) {return std::nullopt;}
    return q_.back();
    
}

void LineBuffer::buffer(T data) {
    std::lock_guard<std::mutex> lock(the_great_line_guardian_);
    q_.push_back(data);
    
}

