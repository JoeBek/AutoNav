#ifndef LINE_BUFFER_HPP
#define LINE_BUFFER_HPP
// quick buffer class for topic reading
template <typename T>
class LineBuffer {

  public:

  LineBuffer();
  
  std::mutex the_great_line_guardian_;
  
  std::optional<T> read() {
    std::lock_guard<std::mutex> lock(the_great_line_guardian_);
    if (q_.empty()) {return std::nullopt;}
    return q_.back();
    
  }
  void buffer(T data) {
    std::lock_guard<std::mutex> lock(the_great_line_guardian_);
    q_.push_back(data);
    
  }

  private:

  std::vector<T> q_;



};
#endif
