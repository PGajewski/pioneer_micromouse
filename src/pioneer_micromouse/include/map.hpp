#ifndef MICROMOUSE_MAP_HPP
#define MICROMOUSE_MAP_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include "algorithm/dijkstra/Point.h"

class map {
public:

    static constexpr std::size_t max_size = 1'000;

    using tile = std::uint8_t;

    enum tile_connectivity : std::uint8_t {
        TC_TOP = 0x01u,
        TC_BOTTOM = 0x02u,
        TC_RIGHT = 0x04u,
        TC_LEFT = 0x08u
    };

    explicit map(std::size_t width, std::size_t height, std::unique_ptr<tile[]> tiles);

    explicit map(std::size_t width, std::size_t height, Point * pnt);

    std::size_t get_width() const { return width; }

    std::size_t get_height() const { return height; }

    std::uint8_t get_connectivity(std::size_t row, std::size_t col) const;

    static char32_t tile_connectivity_to_char(std::uint8_t c);

    static std::optional<std::uint8_t> char_to_tile_connectivity(char32_t c);

    static Point char_to_Point(char32_t c, std::size_t it);

    Point * get_Point(){
        return this->pnt;
    }

private:
    std::size_t width, height;
    std::unique_ptr<tile[]> tiles;
    Point * pnt;

    std::uint8_t& get_tile(std::size_t row, std::size_t col);

    std::uint8_t get_tile(std::size_t row, std::size_t col) const;

    void verify_connectivity_consistency() const;
};

#endif
