//! wav and wave64 packet reading and riff parsing
pub const Reader = @import("Wave.zig");
pub const riff = @import("riff.zig");
pub const parse = Reader.parse;
pub const wav = riff.wav;
pub const wave64 = riff.wave64;
