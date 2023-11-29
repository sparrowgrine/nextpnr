from os import path
import sys
sys.path.append(path.join(path.dirname(__file__), "../.."))
from himbaechel_dbgen.chip import *

def create_efl_tiletype(chip: Chip):
    tt = chip.create_tile_type("EFL")
    # setup wires
    inputs = []
    outputs = []
    tt.create_wire("L4_I0","LUT_INPUT")
    tt.create_wire("L4_I1","LUT_INPUT")
    tt.create_wire("L4_I2","LUT_INPUT")
    tt.create_wire("L4_I3","LUT_INPUT")
    tt.create_wire("L4_O","LUT_OUTPUT")

    tt.create_wire("FA_I0","FA_INPUT")
    tt.create_wire("FA_I1","FA_INPUT")
    tt.create_wire("FA_CI","FA_INPUT")
    tt.create_wire("FA_CO","FA_OUTPUT")
    tt.create_wire("FA_O","FA_OUTPUT")

    lut = tt.create_bel("L4", "LUT4", z=0)
    fa = tt.create_bel("FA", "FA", z=1)

    tt.add_bel_pin(lut, "I[0]", "L4_I0",PinType.INPUT)
    tt.add_bel_pin(lut, "I[1]", "L4_I1",PinType.INPUT)
    tt.add_bel_pin(lut, "I[2]", "L4_I2",PinType.INPUT)
    tt.add_bel_pin(lut, "I[3]", "L4_I3",PinType.INPUT)
    tt.add_bel_pin(lut, "O", "L4_O",PinType.OUTPUT)

    tt.add_bel_pin(fa, "I[0]", "FA_I0",PinType.INPUT)
    tt.add_bel_pin(fa, "I[1]", "FA_I1",PinType.INPUT)
    tt.add_bel_pin(fa, "CI", "FA_CI",PinType.INPUT)
    tt.add_bel_pin(fa, "CO", "FA_CO",PinType.OUTPUT)
    tt.add_bel_pin(fa, "O", "FA_O",PinType.OUTPUT)

    #TODO: muxes
    return tt

def create_eft_tiletype(chip: Chip):
    tt = chip.create_tile_type("EFT")
    # setup wires
    inputs = []
    outputs = []
    tt.create_wire("L4_I0","LUT_INPUT")
    tt.create_wire("L4_I1","LUT_INPUT")
    tt.create_wire("L4_I2","LUT_INPUT")
    tt.create_wire("L4_I3","LUT_INPUT")
    tt.create_wire("L4_O","LUT_OUTPUT")

    tt.create_wire("FA_I0","FA_INPUT")
    tt.create_wire("FA_I1","FA_INPUT")
    tt.create_wire("FA_CI","FA_INPUT")
    tt.create_wire("FA_CO","FA_OUTPUT")
    tt.create_wire("FA_O","FA_OUTPUT")

    tt.create_wire("FF_CLK", "FF_CLK")
    tt.create_wire("FF_CE", "FF_CLKEN")
    tt.create_wire("FF_SR", "FF_SYNCRST")
    tt.create_wire("FF_D", "FF_DATA")
    tt.create_wire("FF_Q", "FF_OUT")

    lut = tt.create_bel("L4", "LUT4", z=0)
    fa = tt.create_bel("FA", "FA", z=0)
    ff = tt.create_bel("FF", "DFF", z=1)

    tt.add_bel_pin(lut, "I[0]", "L4_I0",PinType.INPUT)
    tt.add_bel_pin(lut, "I[1]", "L4_I1",PinType.INPUT)
    tt.add_bel_pin(lut, "I[2]", "L4_I2",PinType.INPUT)
    tt.add_bel_pin(lut, "I[3]", "L4_I3",PinType.INPUT)
    tt.add_bel_pin(lut, "O", "L4_O",PinType.OUTPUT)

    tt.add_bel_pin(fa, "I[0]", "FA_I0",PinType.INPUT)
    tt.add_bel_pin(fa, "I[1]", "FA_I1",PinType.INPUT)
    tt.add_bel_pin(fa, "CI", "FA_CI",PinType.INPUT)
    tt.add_bel_pin(fa, "CO", "FA_CO",PinType.OUTPUT)
    tt.add_bel_pin(fa, "O", "FA_O",PinType.OUTPUT)

    tt.add_bel_pin(ff, "CLK", "FF_CLK", PinType.INPUT)
    tt.add_bel_pin(ff, "CE", "FF_CE", PinType.INPUT)
    tt.add_bel_pin(ff, "SR", "FF_SR", PinType.INPUT)
    tt.add_bel_pin(ff, "D", "FF_D", PinType.INPUT)
    tt.add_bel_pin(ff, "Q", "FF_Q", PinType.OUTPUT)

    #TODO: muxes
    return tt


def create_io_tiletype(chip: Chip):
    tt = chip.create_tile_type("IO")
    # setup wires
    tt.create_wire("IPAD","IO_INPUT")
    tt.create_wire("OPAD","IO_OUTPUT")

    io = tt.create_bel("IO", "IO", z=0)
    tt.add_bel_pin(io,"IPAD","IPAD",PinType.INPUT)
    tt.add_bel_pin(io,"OPAD","OPAD",PinType.OUTPUT)

    #TODO: muxes
    return tt

def create_eftio_tiletype(chip: Chip):
    tt = chip.create_tile_type("EFTIO")

    #TODO: muxes
    return tt


def create_gbufce_tiletype(chip: Chip):
    tt = chip.create_tile_type("GBUF_CTRL_BLOCK")

    #TODO: muxes
    return tt

def create_gbuf_tiletype(chip: Chip):
    tt = chip.create_tile_type("GBUF_BLOCK")

    #TODO: muxes
    return tt

def create_mult_tiletype(chip: Chip):
    tt = chip.create_tile_type("MULT")

    #TODO: muxes
    return tt


def create_mem_tiletype(chip: Chip):
    tt = chip.create_tile_type("MEMORY")

    #TODO: muxes
    return tt