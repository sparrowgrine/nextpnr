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

#include <fstream>
#include <queue>

#include "feline_internal.h"
#include "log.h"
/*
 * cite:  Prim-Dijkstra Revisited: Achieving Superior Timing-driven Routing Trees
 *        https://vlsicad.ucsd.edu/Publications/Conferences/355/c355.pdf
 * cite:  New Algorithms for the Rectilinear Steiner Tree Problem
 *        https://limsk.ece.gatech.edu/course/ece6133/papers/l-shape.pdf
 */

NEXTPNR_NAMESPACE_BEGIN

namespace Feline {

// A sorted set of GCells
void GCellSet::clear()
{
    dirty = false;
    cells.clear();
}
void GCellSet::push(GCell cell)
{
    dirty = true;
    cells.push_back(cell);
}
void GCellSet::do_sort()
{
    dirty = false;
    std::sort(cells.begin(), cells.end());
}
// get previous and next cell, if any
GCell GCellSet::prev_cell(GCell c) const
{
    NPNR_ASSERT(!dirty);
    int b = 0, e = int(cells.size());
    while (b < e) {
        int idx = (b + e) / 2;
        if (cells.at(idx) < c) {
            b = idx + 1;
        } else {
            e = idx;
        }
    }
    return (b < int(cells.size())) && (b > 0) ? cells.at(b - 1) : GCell();
}
GCell GCellSet::next_cell(GCell c) const
{
    NPNR_ASSERT(!dirty);
    int b = 0, e = int(cells.size());
    while (b < e) {
        int idx = (b + e) / 2;
        if (cells.at(idx) <= c) {
            b = idx + 1;
        } else {
            e = idx;
        }
    }
    return b < int(cells.size()) ? cells.at(b) : GCell();
}
// get next non-empty row
int16_t GCellSet::prev_y(int16_t y) const
{
    auto c = prev_cell(GCell(0, y));
    return (c == GCell()) ? -1 : c.y;
}
int16_t GCellSet::next_y(int16_t y) const
{
    auto c = next_cell(GCell(std::numeric_limits<int16_t>::max(), y));
    return (c == GCell()) ? -1 : c.y;
}

STree STree::init_nodes(const Context *ctx, const NetInfo *net)
{
    STree result;
    // TODO: getting actual GCell of relevant general routing wire for the port, rather than just the bel location
    if (net->driver.cell) {
        GCell drv_gcell = GCell(ctx->getBelLocation(net->driver.cell->bel));
        result.source = drv_gcell;
        result.nodes[result.source].port_count += 1;
        result.box.extend(drv_gcell);
        result.ports.push(drv_gcell);
    }
    for (auto &usr : net->users) {
        GCell usr_gcell = GCell(ctx->getBelLocation(usr.cell->bel));
        result.nodes[usr_gcell].port_count += 1;
        result.box.extend(usr_gcell);
        result.ports.push(usr_gcell);
    }
    result.ports.do_sort();
    return result;
}
void STree::dump_svg(const std::string &filename) const
{
    const float scale = 50;
    std::ofstream out(filename);
    NPNR_ASSERT(out);
    int x0 = box.x0 - 1, y0 = box.y0 - 1;
    int width = (box.x1 - x0 + 1);
    int height = (box.y1 - y0 + 1);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << std::endl;
    out << stringf("<svg viewBox=\"0 0 %f %f\" width=\"%f\" height=\"%f\" xmlns=\"http://www.w3.org/2000/svg\">",
                   width * scale, height * scale, width * scale, height * scale)
        << std::endl;
    out << "<defs>" << std::endl;
    out << "<marker id=\"arrowhead\" markerWidth=\"10\" markerHeight=\"7\" refX=\"0\" refY=\"3.5\" orient=\"auto\">"
        << std::endl;
    out << "    <polygon points=\"0 0, 10 3.5, 0 7\" /> " << std::endl;
    out << "  </marker>" << std::endl;
    out << "</defs>" << std::endl;
    out << "<rect x=\"0\" y=\"0\" width=\"100%\" height=\"100%\" stroke=\"#fff\" fill=\"#fff\"/>" << std::endl;
    const float obj_size = 10;
    // edges
    for (auto &node : nodes) {
        if (node.second.uphill == GCell())
            continue;
        float lx0 = (node.second.uphill.x - x0) * scale, ly0 = (node.second.uphill.y - y0) * scale;
        float lx1 = (node.first.x - x0) * scale, ly1 = (node.first.y - y0) * scale;
        out << stringf("<polyline points=\"%f,%f %f,%f %f,%f\" stroke=\"black\" "
                       "marker-mid=\"url(#arrowhead)\"/>",
                       lx0, ly0, (lx0 + lx1) / 2.0, (ly0 + ly1) / 2.0, lx1, ly1)
            << std::endl;
    }
    // nodes
    for (auto &node : nodes) {
        if (node.first == source)
            out << stringf("<rect x=\"%f\" y=\"%f\" width=\"%f\" height=\"%f\" "
                           "style=\"fill:red;stroke:black;stroke-width:1\" />",
                           (node.first.x - x0) * scale - obj_size / 2, (node.first.y - y0) * scale - obj_size / 2,
                           obj_size, obj_size)
                << std::endl;
        else if (node.second.port_count > 0)
            out << stringf("<rect x=\"%f\" y=\"%f\" width=\"%f\" height=\"%f\" "
                           "style=\"fill:blue;stroke:black;stroke-width:1\" />",
                           (node.first.x - x0) * scale - obj_size / 2, (node.first.y - y0) * scale - obj_size / 2,
                           obj_size, obj_size)
                << std::endl;
        else
            out << stringf("<circle cx=\"%f\" cy=\"%f\" r=\"%f\" style=\"fill:black;stroke:black;stroke-width:1\" />",
                           (node.first.x - x0) * scale, (node.first.y - y0) * scale, obj_size / 2)
                << std::endl;
    }
    out << "</svg>" << std::endl;
}

void STree::iterate_neighbours(GCell cell, std::function<void(GCell)> func)
{
    // As per p3 of https://vlsicad.ucsd.edu/Publications/Conferences/355/c355.pdf
    // neighbours are defined as any node where the minimim bounding box containing the neighbour and this node contains
    // no other node
    GCell prev = ports.prev_cell(cell), next = ports.next_cell(cell);
    // same-Y neighbours
    if (prev.y == cell.y)
        func(prev);
    if (next.y == cell.y)
        func(next);
    // decreasing Y direction
    {
        int x0 = (prev.y == cell.y) ? prev.x : box.x0;
        int x1 = (next.y == cell.y) ? next.x : box.x1;
        int y = ports.prev_y(cell.y);
        while (y != -1 && (x0 <= cell.x || x1 > cell.x)) {
            if (x0 <= cell.x) {
                GCell l = ports.prev_cell(GCell(cell.x + 1, y));
                if (l.y == y && l.x >= x0) {
                    func(l);
                    x0 = l.x + 1;
                }
            }
            if (x1 > cell.x) {
                GCell r = ports.next_cell(GCell(cell.x, y));
                if (r.y == y && r.x <= x1) {
                    func(r);
                    x1 = r.x - 1;
                }
            }
            y = ports.prev_y(y);
        }
    }
    // increasing Y direction
    {
        int x0 = (prev.y == cell.y) ? prev.x : box.x0;
        int x1 = (next.y == cell.y) ? next.x : box.x1;
        int y = ports.next_y(cell.y);
        while (y != -1 && (x0 <= cell.x || x1 > cell.x)) {
            if (x0 <= cell.x) {
                GCell l = ports.prev_cell(GCell(cell.x + 1, y));
                if (l.y == y && l.x >= x0) {
                    func(l);
                    x0 = l.x + 1;
                }
            }
            if (x1 > cell.x) {
                GCell r = ports.next_cell(GCell(cell.x, y));
                if (r.y == y && r.x <= x1) {
                    func(r);
                    x1 = r.x - 1;
                }
            }
            y = ports.next_y(y);
        }
    }
}

namespace {
struct QueueEntry
{
    QueueEntry(GCell node, GCell uphill, int path_dist, float cost)
            : node(node), uphill(uphill), path_dist(path_dist), cost(cost){};
    GCell node;
    GCell uphill;
    int path_dist;
    float cost;
    bool operator<(const QueueEntry &other) const
    {
        // want the lowest cost highest to be the top of the queue
        return (cost > other.cost) || (cost == other.cost && node < other.node);
    }
};
} // namespace

void STree::run_prim_djistrka(float alpha)
{
    std::priority_queue<QueueEntry> to_visit;
    dict<GCell, float> best_cost;
    best_cost[source] = 0;
    auto do_expand = [&](int path_dist, GCell cell) {
        iterate_neighbours(cell, [&](GCell neighbour) {
            int edge_cost = std::abs(neighbour.x - cell.x) + std::abs(neighbour.y - cell.y);
            int next_path_dist = path_dist + edge_cost;
            float node_cost = alpha * next_path_dist + edge_cost;
            if (best_cost.count(neighbour) && best_cost.at(neighbour) <= node_cost)
                return;
            // TODO: revisiting?
            if (nodes.at(neighbour).uphill != GCell())
                return;
            to_visit.emplace(neighbour, cell, next_path_dist, node_cost);
            best_cost[neighbour] = node_cost;
        });
    };
    do_expand(0, source);
    while (!to_visit.empty()) {
        auto next = to_visit.top();
        to_visit.pop();
        auto &node = nodes.at(next.node);
        if (node.uphill != GCell())
            continue;
        node.uphill = next.uphill;
        do_expand(next.path_dist, next.node);
    }
}

void STree::get_leaves(dict<GCell, pool<GCell>> &leaves)
{
    for (auto &node : nodes) {
        if (node.second.uphill != GCell())
            leaves[node.second.uphill].insert(node.first);
    }
}

namespace {
int get_total_leaf_count(GCell cursor, const dict<GCell, pool<GCell>> &leaves, dict<GCell, int> &leaf_count)
{
    int count = 0;
    NPNR_ASSERT(!leaf_count.count(cursor)); // topological ordering - should never visit nodes more than once
    if (leaves.count(cursor))
        for (auto leaf : leaves.at(cursor)) {
            count += get_total_leaf_count(leaf, leaves, leaf_count) + 1;
        }
    leaf_count[cursor] = count;
    return count;
}
struct SEdge
{
    SEdge() : src(), dst(){};
    SEdge(GCell src, GCell dst) : src(src), dst(dst){};
    GCell src, dst;
    SEdge flip() const { return SEdge(dst, src); }
    int dist() const { return dst.mdist(src); }
};
} // namespace

void STree::do_edge_flips(float alpha)
{
    // Algorithm based on Algorithm 1 (PD-II) of https://vlsicad.ucsd.edu/Publications/Conferences/355/c355.pdf
    // We need a doubly linked graph, and also the total leaf count of a node to compute detour cost deltas
    dict<GCell, pool<GCell>> leaves;
    dict<GCell, int> total_leaf_count;
    auto rem_edge = [&](SEdge e) {
        auto &ndst = nodes.at(e.dst);
        NPNR_ASSERT(ndst.uphill == e.src);
        ndst.uphill = GCell();
        NPNR_ASSERT(leaves.at(e.src).count(e.dst));
        leaves.at(e.src).erase(e.dst);
    };
    auto add_edge = [&](SEdge e) {
        auto &ndst = nodes.at(e.dst);
        NPNR_ASSERT(ndst.uphill == GCell());
        ndst.uphill = e.src;
        leaves[e.src].insert(e.dst);
    };
    get_leaves(leaves);
    get_total_leaf_count(source, leaves, total_leaf_count);
    // For the best move
    // only consider D=1 case with 1 flip for simplicity
    SEdge best_rem, best_add, best_flp;
    float best_delta = 0;
    int moves_made = 0;
    do {
        best_delta = 0;
        // TODO: actual move search
        for (auto &node : nodes) {
            if (node.second.uphill == GCell())
                continue;
            // TODO: are there other patterns we need to check? have we over-specialised this to the one example case in
            // the paper? can't see many other things that could be checked without hitting excessive complexity what
            // about using the neighbour stuff like in the initial PD itself?? maybe this is the preserved topological
            // order stuff. to think about lying in bed awake tonight...
            if (!leaves.count(node.second.uphill) || !leaves.count(node.first))
                continue;
            for (auto &new_src : leaves.at(node.second.uphill)) {
                if (new_src == node.first)
                    continue;
                for (auto &new_dst : leaves.at(node.first)) {
                    SEdge remd(node.second.uphill, node.first);
                    SEdge added(new_src, new_dst);
                    SEdge flipped(node.first, new_dst);
                    // Compute delta in total path costs to compute detour cost
                    // Note the manhattan distance part of detour cost doesn't change and isn't computed
                    // the part we care about - that's changing - can be simplified to (K+1)*dist where K+1 is the total
                    // leaf count, as path distance to a node counts towards all its leaves detour cost too
                    int orig_path_cost = remd.dist() * (1 + total_leaf_count.at(node.first)) +
                                         flipped.dist() * (1 + total_leaf_count.at(new_dst));
                    int new_path_cost = (added.dist() + flipped.dist()) * ((1 + total_leaf_count.at(node.first)) -
                                                                           (1 + total_leaf_count.at(new_dst))) +
                                        added.dist() * (1 + total_leaf_count.at(new_dst));
                    float delta = alpha * (new_path_cost - orig_path_cost) + (1 - alpha) * (added.dist() - remd.dist());
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_rem = remd;
                        best_add = added;
                        best_flp = flipped;
                    }
                }
            }
        }
        if (best_delta < 0) {
            // commit move
            rem_edge(best_rem);
            if (best_flp.src != GCell())
                rem_edge(best_flp);
            add_edge(best_add);
            if (best_flp.src != GCell())
                add_edge(best_flp.flip());
            ++moves_made;
            // TODO: update cost structures
        }
    } while (best_delta < 0);
    log_info("edge flipping made %d moves\n", moves_made);
}

void STree::steinerise_hvw()
{
    // Stub steinerisation: just pick the lower-L-shape
    dict<GCell, pool<GCell>> leaves;
    get_leaves(leaves);
    for (auto &node_entry : leaves) {
        GCell node = node_entry.first;
        auto &node_data = nodes[node];
        std::vector<GCell> fixed_edges; // the degenerate case of a single rectilinear line
        std::vector<GCell> edges;       // edges where we have a choice
        if (node_data.uphill.x == node.x || node_data.uphill.y == node.y)
            fixed_edges.push_back(node_data.uphill);
        else if (node_data.uphill != GCell())
            edges.push_back(node_data.uphill);
        for (auto leaf : node_entry.second) {
            if (leaf.x == node.x || leaf.y == node.y)
                fixed_edges.push_back(leaf);
            else
                edges.push_back(leaf);
        }
        if (edges.empty())
            continue; // nothing to do
        int best_overlap = -1;
        uint32_t best_choice = 0;
        NPNR_ASSERT(edges.size() < 10); // what if this fails?
        // try both type of U-shape for each edge...
        for (uint32_t choice = 0; choice < (1U << edges.size()); choice++) {
            // TODO: faster ways of finding overlap
            std::vector<std::pair<GCell, GCell>> line_segs;
            auto process_seg = [&](GCell a, GCell b) {
                // The types of overlap that can actually happen are quite limited...
                // TODO: less bruteforce overall....
                for (auto &seg : line_segs) {
                    if (seg.first != a)
                        continue;
                    if (seg.second.x == b.x && seg.first.x == seg.second.x) {
                        if (seg.second.y < seg.first.y) { // -y direction
                            if (b.y < seg.second.y) {
                                int ovl = (seg.first.y - seg.second.y);
                                seg.second.y = b.y;
                                return ovl;
                            } else {
                                return (seg.first.y - b.y);
                            }
                        } else {
                            if (b.y > seg.second.y) { // +y direction
                                int ovl = (seg.second.y - seg.first.y);
                                seg.second.y = b.y;
                                return ovl;
                            } else {
                                return (b.y - seg.first.y);
                            }
                        }
                    } else if (seg.second.y == b.y && seg.first.y == seg.second.y) {
                        if (seg.second.x < seg.first.x) { // -x direction
                            if (b.x < seg.second.x) {
                                int ovl = (seg.first.x - seg.second.x);
                                seg.second.x = b.x;
                                return ovl;
                            } else {
                                return (seg.first.x - b.x);
                            }
                        } else {
                            if (b.x > seg.second.x) { // +x direction
                                int ovl = (seg.second.x - seg.first.x);
                                seg.second.x = b.x;
                                return ovl;
                            } else {
                                return (b.x - seg.first.x);
                            }
                        }
                    }
                }
                line_segs.emplace_back(a, b);
                return 0;
            };
            for (auto &e : fixed_edges) {
                line_segs.emplace_back(node, e);
            }
            int overlap = 0;
            for (uint32_t e = 0; e < edges.size(); e++) {
                bool shape = (choice & (1U << e)) != 0U;
                GCell mid(shape ? node.x : edges.at(e).x, shape ? edges.at(e).y : node.y);
                overlap += process_seg(node, mid);
                overlap += process_seg(mid, edges.at(e));
            }
            if (overlap > best_overlap) {
                best_overlap = overlap;
                best_choice = choice;
            }
        }
        for (uint32_t e = 0; e < edges.size(); e++) {
            bool shape = (best_choice & (1U << e)) != 0U;
            GCell mid(shape ? node.x : edges.at(e).x, shape ? edges.at(e).y : node.y);
            if (edges.at(e) == node_data.uphill) {
                // special case: splitting the driving edge
                if (!nodes.count(mid))
                    nodes[mid].uphill = edges.at(e);
                node_data.uphill = mid;
            } else {
                if (!nodes.count(mid))
                    nodes[mid].uphill = node;
                nodes[edges.at(e)].uphill = mid;
            }
        }
    }
}

} // namespace Feline

NEXTPNR_NAMESPACE_END
