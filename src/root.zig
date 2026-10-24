//! audio decoding and stream properties
pub const features = @import("build_options");
pub const Decoder = @import("Decoder.zig");
pub const Source = @import("Source.zig");
pub const Info = @import("types.zig").Info;
pub const Format = @import("types.zig").Format;
pub const DecoderError = Decoder.Error;
