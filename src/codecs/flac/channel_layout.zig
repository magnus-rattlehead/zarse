//! Adapted from audiophile/flac, Copyright (c) 2026 Eric Freed (MIT).
//! Source: https://codeberg.org/audiophile/flac/commit/8681b5be53bc950101f4f33e291b7f1e1f117059
/// Standard audio channel layouts and speaker configurations.
///
/// This enum defines both physical speaker placement maps and joint-stereo
/// transform spaces (such as Mid/Side). Channel order strictly dictates
/// the layout of samples when interleaved in a multi-channel PCM buffer.
pub const ChannelLayout = enum(u4) {
    /// An unassigned, invalid, or custom channel layout configuration.
    none = 0,

    /// Single-channel monophonic audio.
    /// - Ch0: Mono (C)
    mono = 1,

    /// Standard stereophonic audio.
    /// - Ch0: Left (L)
    /// - Ch1: Right (R)
    stereo = 2,

    /// Three-channel cinematic front array layout.
    /// - Ch0: Left (L)
    /// - Ch1: Right (R)
    /// - Ch2: Center (C)
    center = 3,

    /// Four-channel quadraphonic surround array layout.
    /// - Ch0: Front Left (L)
    /// - Ch1: Front Right (R)
    /// - Ch2: Rear/Back Left (BL)
    /// - Ch3: Rear/Back Right (BR)
    quad = 4,

    /// Five-channel discrete surround sound configuration (No LFE sub-bass).
    /// - Ch0: Front Left (L)
    /// - Ch1: Front Right (R)
    /// - Ch2: Front Center (C)
    /// - Ch3: Rear/Back Left (BL)
    /// - Ch4: Rear/Back Right (BR)
    surround_5_0 = 5,

    /// Standard 5.1 surround sound layout including low-frequency effects.
    /// - Ch0: Front Left (L)
    /// - Ch1: Front Right (R)
    /// - Ch2: Front Center (C)
    /// - Ch3: Low-Frequency Effects / Subwoofer (LFE)
    /// - Ch4: Rear/Back Left (BL)
    /// - Ch5: Rear/Back Right (BR)
    surround_5_1 = 6,

    /// Extended 6.1 surround sound layout with a discrete center-back channel.
    /// - Ch0: Front Left (L)
    /// - Ch1: Front Right (R)
    /// - Ch2: Front Center (C)
    /// - Ch3: Low-Frequency Effects / Subwoofer (LFE)
    /// - Ch4: Rear/Back Center (BC)
    /// - Ch5: Side Left (SL)
    /// - Ch6: Side Right (SR)
    surround_6_1 = 7,

    /// Standard 8-channel theater surround sound layout.
    /// - Ch0: Front Left (L)
    /// - Ch1: Front Right (R)
    /// - Ch2: Front Center (C)
    /// - Ch3: Low-Frequency Effects / Subwoofer (LFE)
    /// - Ch4: Rear/Back Left (BL)
    /// - Ch5: Rear/Back Right (BR)
    /// - Ch6: Side Left (SL)
    /// - Ch7: Side Right (SR)
    surround_7_1 = 8,

    /// Left-Side joint stereo matrixing space.
    /// - Ch0: Left signal
    /// - Ch1: Side (Difference) signal
    ///
    /// Reconstruction equations:
    /// - `Left  = Ch0`
    /// - `Right = Ch0 - Ch1`
    left_side = 9,

    /// Right-Side joint stereo matrixing space.
    /// - Ch0: Side (Difference) signal
    /// - Ch1: Right signal
    ///
    /// Reconstruction equations:
    /// - `Left  = Ch1 + Ch0`
    /// - `Right = Ch1`
    right_side = 10,

    /// Mid-Side (MS) matrixed stereo space, typical for FLAC and joint-stereo MP3.
    /// - Ch0: Mid (Average/Sum) signal
    /// - Ch1: Side (Difference) signal
    ///
    /// Reconstruction equations:
    /// - `Left  = Mid + (Side >> 1)` or `Mid + Side / 2`
    /// - `Right = Mid - (Side - (Side >> 1))` or `Mid - Side / 2`
    mid_side = 11,

    /// Returns flag indicating if this corresponds to a side-channel layout.
    pub inline fn isSideLayout(self: ChannelLayout) bool {
        const first_side: u4 = comptime @intFromEnum(ChannelLayout.left_side);
        return @as(u4, @intFromEnum(self)) >= first_side;
    }

    /// Returns the number of distinct channels used for this layout.
    pub inline fn count(self: ChannelLayout) u8 {
        return switch (self) {
            .left_side, .right_side, .mid_side => 2,
            else => |chan| @intFromEnum(chan),
        };
    }
};
