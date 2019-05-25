#include "map.hpp"
#include "algorithm/dijkstra/Point.h"
#include <iostream>


#include <cassert>

map::map(std::size_t width, std::size_t height, std::unique_ptr<tile[]> tiles) :
        width{width}, height{height}, tiles{std::move(tiles)} {
    assert(this->width < max_size);
    assert(this->height < max_size);
    assert(this->tiles);

    verify_connectivity_consistency();
}

map::map(std::size_t width, std::size_t height, Point * pointer) :
        width{width}, height{height}, pnt{pointer} {
    assert(this->width < max_size);
    assert(this->height < max_size);
    assert(this->pnt);

    //verify_connectivity_consistency();
}



std::uint8_t map::get_connectivity(std::size_t row, std::size_t col) const {
    return get_tile(row, col) & (TC_LEFT | TC_RIGHT | TC_TOP | TC_BOTTOM);
}

char32_t map::tile_connectivity_to_char(std::uint8_t c) {
    switch (c & (TC_LEFT | TC_RIGHT | TC_BOTTOM | TC_TOP)) {
        case 0:
            return U' ';

        case TC_LEFT:
            return U'╡';
        case TC_RIGHT:
            return U'╞';
        case TC_BOTTOM:
            return U'╥';
        case TC_TOP:
            return U'╨';

        case TC_LEFT | TC_BOTTOM:
            return U'╗';
        case TC_RIGHT | TC_BOTTOM:
            return U'╔';
        case TC_LEFT | TC_TOP:
            return U'╝';
        case TC_RIGHT | TC_TOP:
            return U'╚';

        case TC_BOTTOM | TC_TOP:
            return U'║';
        case TC_LEFT | TC_RIGHT:
            return U'═';

        case TC_LEFT | TC_TOP | TC_BOTTOM:
            return U'╣';
        case TC_RIGHT | TC_TOP | TC_BOTTOM:
            return U'╠';
        case TC_LEFT | TC_RIGHT | TC_BOTTOM:
            return U'╦';
        case TC_LEFT | TC_RIGHT | TC_TOP:
            return U'╩';

        case TC_LEFT | TC_RIGHT | TC_BOTTOM | TC_TOP:
            return U'╬';

        default:
            assert(false);
    }
}



std::optional<std::uint8_t> map::char_to_tile_connectivity(char32_t c) {
    switch (c) {
        case U' ':
            return 0;

        case U'╡':
            return TC_LEFT;
        case U'╞':
            return TC_RIGHT;
        case U'╥':
            return TC_BOTTOM;
        case U'╨':
            return TC_TOP;

        case U'╗':
            return TC_LEFT | TC_BOTTOM;
        case U'╔':
            return TC_RIGHT | TC_BOTTOM;
        case U'╝':
            return TC_LEFT | TC_TOP;
        case U'╚':
            return TC_RIGHT | TC_TOP;

        case U'║':
            return TC_BOTTOM | TC_TOP;
        case U'═':
            return TC_LEFT | TC_RIGHT;

        case U'╣':
            return TC_LEFT | TC_TOP | TC_BOTTOM;
        case U'╠':
            return TC_RIGHT | TC_TOP | TC_BOTTOM;
        case U'╦':
            return TC_LEFT | TC_RIGHT | TC_BOTTOM;
        case U'╩':
            return TC_LEFT | TC_RIGHT | TC_TOP;

        case U'╬':
            return TC_LEFT | TC_RIGHT | TC_BOTTOM | TC_TOP;

        default:
            return std::nullopt;
    }
}

Point map::char_to_Point(char32_t c, std::size_t it) {
    Point p;
    p.setIsRight(false);
    p.setIsDown(false);
    p.setIsUp(false);
    p.setIsLeft(false);
    switch (c) {
        case U'╡':
            p.setIsLeft(true);
            break;
        case U'╞':
            p.setIsRight(true);
            break;
        case U'╥':
            p.setIsDown(true);
            break;
        case U'╨':
            p.setIsUp(true);
            break;
        case U'╗':
            p.setIsDown(true);
            p.setIsLeft(true);
            break;
        case U'╔':
            p.setIsRight(true);
            p.setIsDown(true);
            break;
        case U'╝':
            p.setIsLeft(true);
            p.setIsUp(true);
            break;
        case U'╚':
            p.setIsRight(true);
            p.setIsUp(true);
            break;
        case U'║':
            p.setIsUp(true);
            p.setIsDown(true);
            break;
        case U'═':
            p.setIsLeft(true);
            p.setIsRight(true);
            break;
        case U'╣':
            p.setIsLeft(true);
            p.setIsDown(true);
            p.setIsUp(true);
            break;
        case U'╠':
            p.setIsDown(true);
            p.setIsUp(true);
            p.setIsRight(true);
            break;
        case U'╦':
            p.setIsDown(true);
            p.setIsRight(true);
            p.setIsLeft(true);
            break;
        case U'╩':
            p.setIsRight(true);
            p.setIsLeft(true);
            p.setIsUp(true);
            break;
        case U'╬':
            p.setIsLeft(true);
            p.setIsUp(true);
            p.setIsDown(true);
            p.setIsRight(true);
            break;
    }
    //p.setX(column);
    //p.setY(row);
    p.setIndex(it);

    return p;
}

std::uint8_t& map::get_tile(std::size_t row, std::size_t col) {
    assert(row < height);
    assert(col < width);
    return tiles[row * width + col];
}

std::uint8_t map::get_tile(std::size_t row, std::size_t col) const {
    assert(row < height);
    assert(col < width);
    return tiles[row * width + col];
}

void map::verify_connectivity_consistency() const {
    for (std::size_t r = 0; r < height; r++) {
        for (std::size_t c = 0; c < width; c++) {
            auto tile = get_tile(r, c);

            if (c == 0 && (tile & TC_LEFT) > 0) {
                throw std::runtime_error{"Most-left tiles cannot connect to the left"};
            }

            if (c == width - 1 && (tile & TC_RIGHT) > 0) {
                throw std::runtime_error{"Most-right tiles cannot connect to the right"};
            }

            if (r == 0 && (tile & TC_TOP) > 0) {
                throw std::runtime_error{"Most-top tiles cannot connect to the top"};
            }

            if (r == height - 1 && (tile & TC_BOTTOM) > 0) {
                throw std::runtime_error{"Most-bottom tiles cannot connect to the bottom"};
            }

            if (c > 0) {
                auto left_tile = get_tile(r, c - 1);
                if ((left_tile & TC_RIGHT) > 0 ^ (tile & TC_LEFT) > 0) {
                    throw std::runtime_error{"Inconsistent left-right connectivity"};
                }
            }

            if (r > 0) {
                auto top_tile = get_tile(r - 1, c);
                if ((top_tile & TC_BOTTOM) > 0 ^ (tile & TC_TOP) > 0) {
                    throw std::runtime_error{"Inconsistent top-bottom connectivity"};
                }
            }
        }
    }
}

