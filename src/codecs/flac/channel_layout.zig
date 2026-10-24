//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
/// flac channel order and stereo decorrelation modes
pub const ChannelLayout = enum(u4) {
    none = 0,

    mono = 1,

    /// left, right
    stereo = 2,

    /// left, right, center
    center = 3,

    /// front left, front right, back left, back right
    quad = 4,

    /// front left, front right, center, back left, back right
    surround_5_0 = 5,

    /// front left, front right, center, lfe, back left, back right
    surround_5_1 = 6,

    /// front left, front right, center, lfe, back center, side left, side right
    surround_6_1 = 7,

    /// front left, front right, center, lfe, back left, back right, side left, side right
    surround_7_1 = 8,

    /// left, side; right = left - side
    left_side = 9,

    /// side, right; left = side + right
    right_side = 10,

    /// mid, side; the side low bit restores the precision lost when averaging
    mid_side = 11,

    pub inline fn isSideLayout(self: ChannelLayout) bool {
        const first_side: u4 = comptime @intFromEnum(ChannelLayout.left_side);
        return @as(u4, @intFromEnum(self)) >= first_side;
    }

    pub inline fn count(self: ChannelLayout) u8 {
        return switch (self) {
            .left_side, .right_side, .mid_side => 2,
            else => |chan| @intFromEnum(chan),
        };
    }
};
