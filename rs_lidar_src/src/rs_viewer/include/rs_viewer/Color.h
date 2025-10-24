#ifndef RS_VIEWER_COLOR_H_
#define RS_VIEWER_COLOR_H_

#include <string>
#include <cstdint>

namespace robosense::viewer {

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    Color(uint8_t r = 255, uint8_t g = 255, uint8_t b = 255)
        : r(r), g(g), b(b) {}

    // 从颜色名称获取Color（仅支持指定名称，不检查错误）
    static Color fromName(const std::string& color_name);
};

}  // namespace robosense::viewer

#endif  // RS_VIEWER_COLOR_H_