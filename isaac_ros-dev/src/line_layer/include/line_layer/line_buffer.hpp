#ifndef LINE_BUFFER_HPP
#define LINE_BUFFER_HPP
// quick buffer class for topic reading
template <typename T>
class LineBuffer {

  public:

  LineBuffer();
  
  std::mutex the_great_line_guardian_;
  
  std::optional<T> read();
  void buffer(T data);

  private:

  std::vector<T> q_;



};

#endif
