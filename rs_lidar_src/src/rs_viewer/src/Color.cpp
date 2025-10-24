#include "rs_viewer/Color.h"
#include <unordered_map>

namespace robosense::viewer {

Color Color::fromName(const std::string& color_name) {
    static const std::unordered_map<std::string, Color> color_map = {
        {"red", {255, 0, 0}},
        {"green", {0, 255, 0}},
        {"blue", {0, 0, 255}},
        {"yellow", {255, 255, 0}},
        {"white", {255, 255, 255}},
        {"black", {0, 0, 0}}
    };
    return color_map.at(color_name);  // 不检查错误，默认用户输入正确
}

}  // namespace robosense::viewer