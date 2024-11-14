#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <QApplication>

namespace viz {

class GUI : public rviz_common::Panel {
public:
  GUI(QWidget* parent = nullptr) : rviz_common::Panel(parent) {
    // Initialize RViz GUI components here
    auto layout = new QLayout;
    auto button = new QPushButton("Click me!");
    layout->addWidget(button);
    setLayout(layout);
  }
};

}  // namespace rviz_gui_node

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("rviz_gui_node");
  
  viz::GUI gui;
  gui.show();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

