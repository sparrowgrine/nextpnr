import argparse
from os import path
import sys
import re
sys.path.append(path.join(path.dirname(__file__), "../.."))
from himbaechel_dbgen.chip import *
import tritium_tiles


# def create_switch_matrix(tt: TileType, inputs: list[str], outputs: list[str]):
#     # FIXME: terrible routing matrix, just for a toy example...
#     # constant wires
#     tt.create_wire("GND", "GND", const_value="GND")
#     tt.create_wire("VCC", "VCC", const_value="VCC")
#     # switch wires
#     for i in range(Wl):
#         tt.create_wire(f"SWITCH{i}", "SWITCH")
#     # neighbor wires
#     for i in range(Wl):
#         for d, dx, dy in dirs:
#             tt.create_wire(f"{d}{i}", f"NEIGH_{d}")
#     # input pips
#     for i, w in enumerate(inputs):
#         for j in range((i % Si), Wl, Si):
#             tt.create_pip(f"SWITCH{j}", w, timing_class="SWINPUT")
#     # output pips
#     for i, w in enumerate(outputs):
#         for j in range((i % Sq), Wl, Sq):
#             tt.create_pip(w, f"SWITCH{j}", timing_class="SWINPUT")
#     # constant pips
#     for i in range(Wl):
#         tt.create_pip("GND", f"SWITCH{i}")
#         tt.create_pip("VCC", f"SWITCH{i}")
#     # neighbour local pips
#     for i in range(Wl):
#         for j, (d, dx, dy) in enumerate(dirs):
#             tt.create_pip(f"{d}{(i + j) % Wl}", f"SWITCH{i}", timing_class="SWNEIGH")
#     # clock "ladder"
#     if not tt.has_wire("CLK"):
#         tt.create_wire(f"CLK", "TILE_CLK")
#     tt.create_wire(f"CLK_PREV", "CLK_ROUTE")
#     tt.create_pip(f"CLK_PREV", f"CLK")

# def create_nodes(ch):
#     for y in range(Y):
#         print(f"generating nodes for row {y}")
#         for x in range(X):
#             if not is_corner(x, y):
#                 # connect up actual neighbours
#                 local_nodes = [[NodeWire(x, y, f"SWITCH{i}")] for i in range(Wl)]
#                 for d, dx, dy in dirs:
#                     x1 = x - dx
#                     y1 = y - dy
#                     if x1 < 0 or x1 >= X or y1 < 0 or y1 >= Y or is_corner(x1, y1):
#                         continue
#                     for i in range(Wl):
#                         local_nodes[i].append(NodeWire(x1, y1, f"{d}{i}"))
#                 for n in local_nodes:
#                     ch.add_node(n)
#             # connect up clock ladder (not intended to be a sensible clock structure)
#             if y != 1: # special case where the node has 3 wires
#                 if y == 0:
#                     if x == 0:
#                         # clock source: IO
#                         clk_node = [NodeWire(1, 0, "GCLK_OUT")]
#                     else:
#                         # clock source: left
#                         clk_node = [NodeWire(x-1, y, "CLK")]
#                 else:
#                     # clock source: above
#                     clk_node = [NodeWire(x, y-1, "CLK")]
#                 clk_node.append(NodeWire(x, y, "CLK_PREV"))
#                 if y == 0:
#                     clk_node.append(NodeWire(x, y+1, "CLK_PREV"))
#                 ch.add_node(clk_node)

# def set_timings(ch):
#     speed = "DEFAULT"
#     tmg = ch.set_speed_grades([speed])
#     # --- Routing Delays ---
#     # Notes: A simpler timing model could just use intrinsic delay and ignore R and Cs.
#     # R and C values don't have to be physically realistic, just in agreement with themselves to provide
#     # a meaningful scaling of delay with fanout. Units are subject to change.
#     tmg.set_pip_class(grade=speed, name="SWINPUT",
#         delay=TimingValue(80), # 80ps intrinstic delay
#         in_cap=TimingValue(5000), # 5pF
#         out_res=TimingValue(1000), # 1ohm
#     )
#     tmg.set_pip_class(grade=speed, name="SWOUTPUT",
#         delay=TimingValue(100), # 100ps intrinstic delay
#         in_cap=TimingValue(5000), # 5pF
#         out_res=TimingValue(800), # 0.8ohm
#     )
#     tmg.set_pip_class(grade=speed, name="SWNEIGH",
#         delay=TimingValue(120), # 120ps intrinstic delay
#         in_cap=TimingValue(7000), # 7pF
#         out_res=TimingValue(1200), # 1.2ohm
#     )
#     # TODO: also support node/wire delays and add an example of them

#     # --- Cell delays ---
#     lut = ch.timing.add_cell_variant(speed, "LUT4")
#     for j in range(K):
#         lut.add_comb_arc(f"I[{j}]", "F", TimingValue(150 + j * 15))
#     dff = ch.timing.add_cell_variant(speed, "DFF")
#     dff.add_setup_hold("CLK", "D", ClockEdge.RISING, TimingValue(150), TimingValue(25))
#     dff.add_clock_out("CLK", "Q", ClockEdge.RISING, TimingValue(200))

def main():
    trbase = path.join(path.dirname(path.realpath(__file__)))
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", help="name of device to export", type=str, required=True)
    parser.add_argument("--constids", help="name of nextpnr constids file to read", type=str, default=path.join(trbase, "constids.inc"))
    parser.add_argument("--beldb", help="path to beldb files", type=str, default=path.join(trbase, "beldb"))
    parser.add_argument("--bba", help="bba file to write", type=str, required=True)
    args = parser.parse_args()

    dev_name_regex = re.compile("oph_(\d+)x(\d+)_b\d+_d\d+")
    beldb_entry_regex = re.compile("type: (\w+) x: (\d+) y: (\d+)")

    dev_match = dev_name_regex.match(args.device)
    if dev_match is None:
        print("ERR: Invalid devicce name.")
        sys.exit(-1)
   

    beldb_dir_path = path.join(path.dirname(__file__), args.beldb)
    if not path.exists(beldb_dir_path):
        print("ERR: Invalid beldb path.")
        sys.exit(-1)
    if not path.exists(path.join(beldb_dir_path, f"{args.device}.beldb")):
        print(f"ERR: Device {args.device} either does not exist, or lacks a beldb.")
        sys.exit(-1)
    
    ch = Chip("tritium", args.device, int(dev_match[1])+2, int(dev_match[2])+2)
    # Init constant ids
    ch.strs.read_constids(path.join(path.dirname(__file__), args.constids))

    efl = tritium_tiles.create_efl_tiletype(ch)
    eft = tritium_tiles.create_eft_tiletype(ch)
    io = tritium_tiles.create_io_tiletype(ch)
    eftio = tritium_tiles.create_eftio_tiletype(ch)
    mem = tritium_tiles.create_mem_tiletype(ch)
    mult = tritium_tiles.create_mult_tiletype(ch)
    gbuf = tritium_tiles.create_gbuf_tiletype(ch)
    gbufce = tritium_tiles.create_gbufce_tiletype(ch)
    
    with open(path.join(beldb_dir_path, f"{args.device}.beldb")) as beldb:
        dbentries = beldb.readlines()
        for entry in dbentries:
            entry_data = beldb_entry_regex.match(entry)
            if entry_data is None:
                continue
            type = entry_data[1]
            x = int(entry_data[2])
            y = int(entry_data[3])
            ch.set_tile_type(x,y, type.upper())        
    # Create nodes between tiles
    # create_nodes(ch)
    # set_timings(ch)
    ch.write_bba(path.join(path.dirname(__file__),args.bba))

if __name__ == '__main__':
    main()
