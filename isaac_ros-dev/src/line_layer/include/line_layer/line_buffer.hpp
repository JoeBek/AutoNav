#ifndef LINE_BUFFER_HPP
#define LINE_BUFFER_HPP
// quick buffer class for topic reading
#include <mutex>
#include <vector>
#include <optional>
template <typename T>
class LineBuffer {

  public:

   LineBuffer() = default;
  
  std::mutex the_great_line_guardian_;
  

    std::optional<T> read() {
    std::lock_guard<std::mutex> lock(the_great_line_guardian_);
    if (q_.empty()) {return std::nullopt;}
    return q_.back();
    
    }

    void buffer(T data) {
    std::lock_guard<std::mutex> lock(the_great_line_guardian_);
    q_.push_back(data);
    // no more mr. infinite vector
    if (q_.size() > buf_size) {
      q_.erase(q_.begin());
    }
    
    }



  private:

  std::vector<T> q_;
  long unsigned int buf_size = 10;



};

#endif
