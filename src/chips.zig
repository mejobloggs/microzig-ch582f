const std = @import("std");
const microzig = @import("../deps/microzig/build.zig");

fn root_dir() []const u8 {
    return std.fs.path.dirname(@src().file) orelse ".";
}

const chip_path = std.fmt.comptimePrint("{s}/chips/ch58x.zig", .{root_dir()});
//const hal_path = std.fmt.comptimePrint("{s}/hal.zig", .{root_dir()});
const json_register_schema_path = std.fmt.comptimePrint("{s}/chips/ch58x.json", .{root_dir()});

//flash and ram values got from here: https://github.com/WeActStudio/WeActStudio.WCH-BLE-Core/blob/master/Examples/CH582/template/Ld/Link.ld

pub const CH58x = microzig.Chip{
    .name = "CH583F",
    .source = .{ .path = chip_path },
    //.hal = .{ .path = hal_path },
    .cpu = microzig.cpus.riscv32_imac,
    .memory_regions = &.{
        .{ .kind = .flash, .offset = 0x00000000, .length = 448 * 1024 },
        .{ .kind = .ram, .offset = 0x20000000, .length = 32 * 1024 },
    },
    .json_register_schema = .{
        .path = json_register_schema_path,
    },
};
