/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2022  gatecat <gatecat@ds0.me>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef FELINE_INTERNAL_H
#define FELINE_INTERNAL_H
#include "feline.h"
#include "hashlib.h"
#include "nextpnr.h"

#include <limits>

NEXTPNR_NAMESPACE_BEGIN
namespace Feline {
// TODO: do we also need a wire type field
struct GCell
{
    int16_t x = std::numeric_limits<int16_t>::lowest(), y = std::numeric_limits<int16_t>::lowest();
    GCell(){};
    GCell(int16_t x, int16_t y) : x(x), y(y){};
    explicit GCell(Loc loc) : x(loc.x), y(loc.y){};
    bool operator==(const GCell &other) const { return x == other.x && y == other.y; }
    bool operator<(const GCell &other) const { return y < other.y || ((y == other.y) && (x < other.x)); }
    bool operator<=(const GCell &other) const { return (*this == other) || (*this < other); }
    bool operator>=(const GCell &other) const { return !(*this < other); }
    bool operator!=(const GCell &other) const { return x != other.x || y != other.y; }
    unsigned hash() const { return mkhash(x, y); }
    int mdist(GCell other) const { return std::abs(x - other.x) + std::abs(y - other.y); }
};

struct GBox
{
    int16_t x0 = std::numeric_limits<int16_t>::max(), y0 = std::numeric_limits<int16_t>::max();
    int16_t x1 = std::numeric_limits<int16_t>::lowest(), y1 = std::numeric_limits<int16_t>::lowest();
    GBox(){};
    GBox(int16_t x, int16_t y) : x0(x), y0(y), x1(x), y1(y){};
    GBox(int16_t x0, int16_t y0, int16_t x1, int16_t y1) : x0(x0), y0(y0), x1(x1), y1(y1){};
    bool operator==(const GBox &other) const
    {
        return x0 == other.x0 && y0 == other.y0 && x1 == other.x1 && y1 == other.y1;
    }
    bool operator!=(const GBox &other) const { return !(*this == other); }
    inline void extend(GCell p)
    {
        x0 = std::min(x0, p.x);
        y0 = std::min(y0, p.y);
        x1 = std::max(x1, p.x);
        y1 = std::max(y1, p.y);
    }
};

// A sorted set of GCells
struct GCellSet
{
    std::vector<GCell> cells;
    bool dirty = false;
    void clear();
    void push(GCell cell);
    void do_sort();
    // get previous and next cell, if any
    GCell prev_cell(GCell c) const;
    GCell next_cell(GCell c) const;
    // next non-empty row in either direction
    int16_t prev_y(int16_t y) const;
    int16_t next_y(int16_t y) const;
};

// A {Steiner, Spanning} tree structure
struct STreeNode
{
    GCell uphill;          // singly linked, at least for now...
    int port_count = 0;    // port_count==0 means this is a Steiner node
    float criticality = 0; // timing criticality in [0, 1]
};
struct STree
{
    GCell source;
    dict<GCell, STreeNode> nodes;
    GCellSet ports;
    GBox box;
    static STree init_nodes(const Context *ctx, const NetInfo *net);
    void dump_svg(const std::string &filename) const;
    void iterate_neighbours(GCell cell, std::function<void(GCell)> func);
    void run_prim_djistrka(float alpha);
    void get_leaves(dict<GCell, pool<GCell>> &leaves);
    void do_edge_flips(float alpha);
    void steinerise_hvw();
};

} // namespace Feline
NEXTPNR_NAMESPACE_END
#endif
