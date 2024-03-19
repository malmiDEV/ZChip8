const std = @import("std");
const sdl = @cImport(@cInclude("SDL.h"));
const cstd = @cImport(@cInclude("stdlib.h"));
const time = @cImport(@cInclude("time.h"));

const START = 0x200;

const FONT = [_]u8{
    0xF0, 0x90, 0x90, 0x90, 0xF0,
    0x20, 0x60, 0x20, 0x20, 0x70,
    0xF0, 0x10, 0xF0, 0x80, 0xF0,
    0xF0, 0x10, 0xF0, 0x10, 0xF0,
    0x90, 0x90, 0xF0, 0x10, 0x10,
    0xF0, 0x80, 0xF0, 0x10, 0xF0,
    0xF0, 0x80, 0xF0, 0x90, 0xF0,
    0xF0, 0x10, 0x20, 0x40, 0x40,
    0xF0, 0x90, 0xF0, 0x90, 0xF0,
    0xF0, 0x90, 0xF0, 0x10, 0xF0,
    0xF0, 0x90, 0xF0, 0x90, 0x90,
    0xE0, 0x90, 0xE0, 0x90, 0xE0,
    0xF0, 0x80, 0x80, 0x80, 0xF0,
    0xE0, 0x90, 0x90, 0x90, 0xE0,
    0xF0, 0x80, 0xF0, 0x80, 0xF0,
    0xF0, 0x80, 0xF0, 0x80, 0x80,
};

pub const Memory = struct {
    ram: [0xfff]u8,

    pub fn new() Memory {
        var mem = std.mem.zeroes(Memory);
        // clear ram
        for (&mem.ram) |*i| {
            i.* = 0;
        }
        // load font
        for (FONT, 0..) |c, i| {
            mem.ram[i] = c;
        }
        return mem;
    }

    pub fn write(self: *Memory, addr: u16, value: u8) void {
        self.ram[addr] = value;
    }

    pub fn read(self: *Memory, addr: u16) u8 {
        return self.ram[addr];
    }

    pub fn loadRom(self: *Memory) !void {
        var inputFile = try std.fs.cwd().openFile("roms/PONG2", .{});
        defer inputFile.close();

        var size = try inputFile.getEndPos();
        std.debug.print("ROM File Size {}\n", .{size});
        var reader = inputFile.reader();

        for (0..size) |i| {
            self.write(@intCast(START + i), try reader.readByte());
        }
    }
};

pub const Display = struct {
    fbuf: [64*32]u8,

    pub fn new() Display {
        var display = std.mem.zeroes(Display);
        // clear feamebuffer
        display.clear();
        return display;
    }

    pub fn clear(self: *Display) void {
        for (&self.fbuf) |*i| {
            i.* = 0;
        }
    }

    pub fn write(self: *Display, x: usize, y: usize) void {
        self.fbuf[y * 64 + x] ^= 1;
    }

    pub fn read(self: *Display, x: usize, y: usize) u8 {
        return self.fbuf[y * 64 + x];
    }
};

pub const Bus = struct {
    memory: Memory,
    display: Display,

    pub fn new() Bus {
        var bus = std.mem.zeroes(Bus);
        bus.memory = Memory.new();
        bus.display = Display.new();
        return bus;
    }
};

pub const Chip8 = struct {
    key: [16]u1,
    vn: [16]u8,
    i: u16,
    ret_stack: [16]u16,
    pc: u16,
    sp: u8,
    draw: bool,
    delay_timer: u8,
    sound_timer: u8,
    bus: Bus,

    pub fn new() Chip8 {
        cstd.srand(@as(u32, @intCast(time.time(0))));
        var sys = std.mem.zeroes(Chip8);
        // clear keys
        for (&sys.key) |*i| {
            i.* = 0;
        }
        // clear registers
        for (&sys.vn) |*i| {
            i.* = 0;
        }
        // clear stack
        for (&sys.ret_stack) |*i| {
            i.* = 0;
        }
        sys.i = 0;
        sys.pc = START;
        sys.sp = 0;
        sys.bus = Bus.new();
        return sys;
    }

    pub fn code_decode(self: *Chip8) !void {
        var hi = @as(u16, self.bus.memory.read(self.pc));
        var lo = @as(u16, self.bus.memory.read(self.pc + 1));
        var instruction: u16 = hi << 8 | lo;

        var opcode: u16 = @truncate((instruction & 0xf000) >> 12);
        var nnn: u16 = @truncate(instruction & 0x0fff);
        var nn: u8 = @truncate(instruction & 0x00ff);
        var n: u8 = @truncate(instruction & 0x000f);
        var x: u8 = @truncate((instruction & 0x0f00) >> 8);
        var y: u8 = @truncate((instruction & 0x00f0) >> 4);

        var oldpc = self.pc;
        switch (opcode) {
            0x0 => switch (nn) {
                0xe0 => {
                    self.bus.display.clear();
                    self.draw = true;
                    self.pc += 2;
                },
                0xee => {
                    self.sp -= 1;
                    var ret = self.ret_stack[self.sp];
                    self.pc = ret;
                    self.pc += 2;
                },
                else => {
                    std.debug.panic("INVALID INSTRUCTION: {x}", .{instruction});
                },
            },
            0x1 => {
                self.pc = nnn;
            },
            0x2 => {
                self.ret_stack[self.sp] = self.pc;
                self.sp += 1;
                self.pc = nnn;
            },
            0x3 => {
                if (self.vn[x] == nn) {
                    self.pc += 4;
                } else {
                    self.pc += 2;
                }
            },
            0x4 => {
                if (self.vn[x] != nn) {
                    self.pc += 4;
                } else {
                    self.pc += 2;
                }
            },
            0x5 => {
                if (self.vn[x] == self.vn[y]) {
                    self.pc += 4;
                } else {
                    self.pc += 2;
                }
            },
            0x6 => {
                self.vn[x] = nn;
                self.pc += 2;
            },
            0x7 => {
                self.vn[x] +%= nn;
                self.pc += 2;
            },
            0x8 => switch (n) {
                0x0 => {
                    self.vn[x] = self.vn[y];
                    self.pc += 2;
                },
                0x1 => {
                    self.vn[x] |= self.vn[y];
                    self.pc += 2;
                },
                0x2 => {
                    self.vn[x] &= self.vn[y];
                    self.pc += 2;
                },
                0x3 => {
                    self.vn[x] ^= self.vn[y];
                    self.pc += 2;
                },
                0x4 => {
                    var sum: u16 = self.vn[x];
                    sum += self.vn[y];
                    self.vn[0xf] = if (sum > 0xff) 1 else 0;
                    self.vn[x] = @truncate(sum & 0x00ff);
                    self.pc += 2;
                },
                0x5 => {
                    self.vn[0xf] = if (self.vn[x] > self.vn[y]) 1 else 0;
                    self.vn[x] -%= self.vn[y];
                    self.pc += 2;
                },
                0x6 => {
                    self.vn[0xf] = self.vn[x] & 1;
                    self.vn[x] >>= 1;
                    self.pc += 2;
                },
                0x7 => {
                    self.vn[0xf] = if (self.vn[y] > self.vn[x]) 1 else 0;
                    self.vn[x] = self.vn[y] - self.vn[x];
                    self.pc += 2;
                },
                0xe => {
                    self.vn[0xf] = if (self.vn[x] & 128 != 0) 1 else 0;
                    self.vn[x] <<= 1;
                    self.pc += 2;
                },
                else => {
                    std.debug.panic("INVALID INSTRUCTION: {x}", .{instruction});
                },
            },
            0x9 => {
                if (self.vn[x] != self.vn[y]) {
                    self.pc += 4;
                } else {
                    self.pc += 2;
                }
            },
            0xa => {
                self.i = nnn;
                self.pc += 2;
            },
            0xb => {
                self.pc = self.vn[0] + nnn;
            },
            0xc => {
                self.vn[x] = @truncate(@as(u32, @bitCast(cstd.rand())) & nn);
                self.pc += 2;
            },
            0xd => {
                self.vn[0xf] = 0;
                for (0..n) |row| {
                    var ypos = self.vn[y] % 32 + row;
                    if (ypos >= 32) break;
                    var sprite: u8 = self.bus.memory.read(@intCast(self.i + row));
                    for (0..8) |col| {
                        if ((sprite & (@as(u8, 0x80) >> @intCast(col))) != 0) {
                            var xpos = self.vn[x] % 64 + col;
                            self.bus.display.write(xpos, ypos);
                            if (self.bus.display.read(xpos, ypos) == 0) {
                                self.vn[0xf] = 1;
                            }
                            if (xpos >= 64) break;
                        }
                    }
                }
                self.draw = true;
                self.pc += 2;
            },
            0xe => switch (nn) {
                0x9e => {
                    var key = self.key[self.vn[x]];
                    if (key == 1) {
                        self.pc += 4;
                    } else {
                        self.pc += 2;
                    }
                },
                0xa1 => {
                    var key = self.key[self.vn[x]];
                    if (key != 1) {
                        self.pc += 4;
                    } else {
                        self.pc += 2;
                    }
                },
                else => {
                    std.debug.panic("INVALID INSTRUCTION: {x}", .{instruction});
                },
            },
            0xf => switch (nn) {
                0x7 => {
                    self.vn[x] = self.delay_timer;
                    self.pc += 2;
                },
                0xa => {
                    var press = false;
                    var key: u1 = 0;
                    for (0..16) |i| {
                        if (self.key[i] != 0) {
                            key = @as(u1, @intCast(i));
                            press = true;
                        }
                    }
                    if (!press) {
                        self.pc -= 2;
                    } else {
                        if (self.key[key] != 0) {
                            self.pc -= 2;
                        } else {
                            self.vn[x] = key;
                            key = 0;
                            press = false;
                        }
                    }
                    self.pc += 2;
                },
                0x15 => {
                    self.delay_timer = self.vn[x];
                    self.pc += 2;
                },
                0x18 => {
                    self.sound_timer = self.vn[n];
                    self.pc += 2;
                },
                0x1e => {
                    self.vn[0xf] = if ((self.i + self.vn[x]) > 0xfff) 1 else 0;
                    self.i += self.vn[x];
                    self.pc += 2;
                },
                0x29 => {
                    self.i = self.vn[x] * 5;
                    self.pc += 2;
                },
                0x33 => {
                    var value = self.vn[x];
                    self.bus.memory.write(self.i + 2, value % 10);
                    value /= 10;
                    self.bus.memory.write(self.i + 1, value % 10);
                    value /= 10;
                    self.bus.memory.write(self.i + 0, value);
                    self.pc += 2;
                },
                0x55 => {
                    for (0..x + 1) |i| {
                        self.bus.memory.write(self.i, self.vn[i]);
                        self.i += 1;
                    }
                    self.pc += 2;
                },
                0x65 => {
                    for (0..x + 1) |i| {
                        self.vn[i] = self.bus.memory.read(self.i);
                        self.i += 1;
                    }
                    self.pc += 2;
                },
                else => {
                    std.debug.panic("INVALID INSTRUCTION: {x}", .{instruction});
                },
            },
            else => {
                std.debug.panic("INVALID INSTRUCTION: {x}", .{instruction});
            },
        }
        std.debug.print("inst:{x}\tpc:{x}\tvx|{x}:{x}\tvy|{x}:{x}\ti:{x}\tsp:{x}\tdelay:{x}\n", .{ instruction, oldpc, x, self.vn[x], y, self.vn[y], self.i, self.sp, self.delay_timer });
    }
};

pub fn main() !void {
    var sys = Chip8.new();
    try sys.bus.memory.loadRom();

    _ = sdl.SDL_Init(sdl.SDL_INIT_VIDEO);
    defer sdl.SDL_Quit();

    var window = sdl.SDL_CreateWindow("ZChip8", sdl.SDL_WINDOWPOS_CENTERED, sdl.SDL_WINDOWPOS_CENTERED, 640, 400, 0);
    defer sdl.SDL_DestroyWindow(window);

    var renderer = sdl.SDL_CreateRenderer(window, 0, sdl.SDL_RENDERER_PRESENTVSYNC);
    defer sdl.SDL_DestroyRenderer(renderer);

    var texture = sdl.SDL_CreateTexture(renderer, sdl.SDL_PIXELFORMAT_RGBA8888, sdl.SDL_TEXTUREACCESS_STREAMING, 64, 32);
    defer sdl.SDL_DestroyTexture(texture);

    var start: f64 = 0;
    var end: f64 = 0;
    var delta: f64 = 0;
    var accu: f64 = 0;
    var threshold: f64 = 1 / 60;

    mainloop: while (true) {
        delta = end - start;
        start = @as(f64, @floatFromInt(sdl.SDL_GetTicks()));
        accu += delta;
        if (accu >= threshold) {
            try sys.code_decode();
            var sdl_event: sdl.SDL_Event = undefined;
            while (sdl.SDL_PollEvent(&sdl_event) > 0) {
                switch (sdl_event.type) {
                    sdl.SDL_QUIT => break :mainloop,
                    sdl.SDL_KEYDOWN => {
                        switch (sdl_event.key.keysym.sym) {
                            sdl.SDLK_1 => sys.key[0x1] = 1,
                            sdl.SDLK_2 => sys.key[0x2] = 1,
                            sdl.SDLK_3 => sys.key[0x3] = 1,
                            sdl.SDLK_4 => sys.key[0xC] = 1,
                            sdl.SDLK_q => sys.key[0x4] = 1,
                            sdl.SDLK_w => sys.key[0x5] = 1,
                            sdl.SDLK_e => sys.key[0x6] = 1,
                            sdl.SDLK_r => sys.key[0xD] = 1,
                            sdl.SDLK_a => sys.key[0x7] = 1,
                            sdl.SDLK_s => sys.key[0x8] = 1,
                            sdl.SDLK_d => sys.key[0x9] = 1,
                            sdl.SDLK_f => sys.key[0xE] = 1,
                            sdl.SDLK_z => sys.key[0xA] = 1,
                            sdl.SDLK_x => sys.key[0x0] = 1,
                            sdl.SDLK_c => sys.key[0xB] = 1,
                            sdl.SDLK_v => sys.key[0xF] = 1,
                            sdl.SDLK_ESCAPE => sys.bus.display.clear(),
                            else => {},
                        }
                    },
                    sdl.SDL_KEYUP => {
                        for (0..16) |i| {
                            sys.key[i] = 0;
                        }
                    },
                    else => {},
                }
            }

            // decode vram
            if (sys.draw) {
                _ = sdl.SDL_RenderClear(renderer);
                var ram: ?[*]u32 = null;
                var pitch: c_int = 0;
                _ = sdl.SDL_LockTexture(texture, null, @as([*]?*anyopaque, @ptrCast(&ram)), &pitch);
                for (0..32) |y| {
                    for (0..64) |x| {
                        ram.?[y * 64 + x] = if (sys.bus.display.read(x, y) == 1) 0x00ff00ff else 0x00000000;
                    }
                }
                sdl.SDL_UnlockTexture(texture);
                var dest = sdl.SDL_Rect{ .x = 0, .y = 0, .w = 640, .h = 400 };
                _ = sdl.SDL_RenderCopy(renderer, texture, null, &dest);
                sdl.SDL_RenderPresent(renderer);
                sys.draw = false;
            }

            if (sys.delay_timer > 0) {
                sys.delay_timer -= 1;
            }
            if (sys.sound_timer > 0) {
                sys.sound_timer -= 1;
            }
            accu -= threshold;
        }
        end = @as(f64, @floatFromInt(sdl.SDL_GetTicks()));
    }
}
