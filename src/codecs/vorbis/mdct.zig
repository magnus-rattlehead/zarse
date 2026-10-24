//! inverse modified discrete cosine transform and window slopes
//! ref: stb_vorbis.c inverse_mdct() and compute_window(),
//! libvorbis mdct.c and window.c, spec section 4.6
const std = @import("std");
const root = @import("root.zig");

const Error = root.Error;

/// transform tables of one block size
pub const Transform = struct {
    /// block size in samples
    size: usize = 0,
    storage: []f32 = &.{},
    /// even-stage twiddle factors
    a: []f32 = &.{},
    /// odd-stage twiddle factors
    b: []f32 = &.{},
    /// final-stage twiddle factors
    c: []f32 = &.{},
    /// final-stage bit reversed indices
    bit_reverse: []u16 = &.{},
    /// rising half-window slope
    window: []f32 = &.{},

    /// build transform and window tables for a vorbis block of 64..8192 samples
    pub fn init(allocator: std.mem.Allocator, size: usize) Error!Transform {
        if (
            size < 1 << root.min_block_size_log2
                or size > 1 << root.max_block_size_log2
                or size & (size - 1) != 0
        )
            return error.InvalidData;
        var self: Transform = .{ .size = size };
        errdefer self.deinit(allocator);
        const half = size / 2;
        const quarter = size / 4;
        const eighth = size / 8;
        self.storage = try allocator.alloc(f32, half * 3 + quarter);
        self.a = self.storage[0..half];
        self.b = self.storage[half..][0..half];
        self.c = self.storage[half * 2..][0..quarter];
        self.window = self.storage[half * 2 + quarter..][0..half];
        self.bit_reverse = try allocator.alloc(u16, eighth);
        computeTwiddles(size, self.a, self.b, self.c);
        computeBitReverse(size, self.bit_reverse);
        computeWindow(size, self.window);
        return self;
    }

    pub fn deinit(self: *Transform, allocator: std.mem.Allocator) void {
        allocator.free(self.storage);
        allocator.free(self.bit_reverse);
        self.* = .{};
    }

    /// replace one channel block with unwindowed time samples
    /// scratch needs half a block, `Decoder.lap` applies window and previous overlap
    pub fn inverseMdct(
        self: *const Transform,
        buffer: []f32,
        scratch: []f32,
    ) void {
        const n = self.size;
        const n2 = n / 2;
        const n4 = n / 4;
        const n8 = n / 8;
        const u = buffer;
        const v = scratch;

        // step zero copies the spectral data into the scratch buffer and
        // reflects it, the twiddle factors of the first stage rotate the
        // pairs of values
        {
            var d: isize = @intCast(n2 - 2);
            var aa: usize = 0;
            var e: isize = 0;
            while (e != @as(isize, @intCast(n2))) : (e += 4) {
                set(
                    v,
                    d + 1,
                    get(u, e) * self.a[aa] - get(u, e + 2) * self.a[aa + 1],
                );
                set(
                    v,
                    d,
                    get(u, e) * self.a[aa + 1] + get(u, e + 2) * self.a[aa],
                );
                d -= 2;
                aa += 2;
            }
            e = @intCast(n2 - 3);
            while (d >= 0) : ({
                    d -= 2;
                    aa += 2;
                    e -= 4;
                }) {
                set(
                    v,
                    d + 1,
                    -get(u, e + 2) * self.a[aa] + get(u, e) * self.a[aa + 1],
                );
                set(
                    v,
                    d,
                    -get(u, e + 2) * self.a[aa + 1] - get(u, e) * self.a[aa],
                );
            }
        }

        // step two mixes the halves of the scratch buffer into the output
        // buffer, the twiddle factors of the second stage rotate the pairs
        {
            var aa: isize = @intCast(n2 - 8);
            var e0: usize = n4;
            var e1: usize = 0;
            var d0: usize = n4;
            var d1: usize = 0;
            while (aa >= 0) : ({
                    aa -= 8;
                    d0 += 4;
                    d1 += 4;
                    e0 += 4;
                    e1 += 4;
                }) {
                const v41_21 = v[e0 + 1] - v[e1 + 1];
                const v40_20 = v[e0] - v[e1];
                u[d0 + 1] = v[e0 + 1] + v[e1 + 1];
                u[d0] = v[e0] + v[e1];
                u[d1 + 1] = v41_21 * self.a[@intCast(aa + 4)]
                    - v40_20 * self.a[@intCast(aa + 5)];
                u[d1] = v40_20 * self.a[@intCast(aa + 4)]
                    + v41_21 * self.a[@intCast(aa + 5)];

                const w41_21 = v[e0 + 3] - v[e1 + 3];
                const w40_20 = v[e0 + 2] - v[e1 + 2];
                u[d0 + 3] = v[e0 + 3] + v[e1 + 3];
                u[d0 + 2] = v[e0 + 2] + v[e1 + 2];
                u[d1 + 3] = w41_21 * self.a[@intCast(aa)]
                    - w40_20 * self.a[@intCast(aa + 1)];
                u[d1 + 2] = w40_20 * self.a[@intCast(aa)]
                    + w41_21 * self.a[@intCast(aa + 1)];
            }
        }

        // step three runs the butterfly stages, the first iteration uses its
        // own loop, the second iteration uses the row loop and the
        // remaining iterations use the column loop
        const ld: usize = std.math.log2_int(usize, n);
        self.step3Iter0(
            u,
            n / 16,
            @as(isize, @intCast(n2 - 1)),
            -@as(isize, @intCast(n8)),
        );
        self.step3Iter0(
            u,
            n / 16,
            @as(isize, @intCast(n2 - 1 - n4)),
            -@as(isize, @intCast(n8)),
        );
        self.step3Row(
            u,
            n / 32,
            @as(isize, @intCast(n2 - 1)),
            -@as(isize, @intCast(n / 16)),
            16,
        );
        self.step3Row(
            u,
            n / 32,
            @as(isize, @intCast(n2 - 1 - n8)),
            -@as(isize, @intCast(n / 16)),
            16,
        );
        self.step3Row(
            u,
            n / 32,
            @as(isize, @intCast(n2 - 1 - 2 * n8)),
            -@as(isize, @intCast(n / 16)),
            16,
        );
        self.step3Row(
            u,
            n / 32,
            @as(isize, @intCast(n2 - 1 - 3 * n8)),
            -@as(isize, @intCast(n / 16)),
            16,
        );

        var l: usize = 2;
        while (l < ((ld - 3) >> 1)) : (l += 1) {
            const k0 = n >> @as(u6, @intCast(l + 2));
            const k0_2 = k0 >> 1;
            const lim = @as(usize, 1) << @as(u6, @intCast(l + 1));
            var i: usize = 0;
            while (i < lim) : (i += 1) {
                self.step3Row(
                    u,
                    n >> @as(u6, @intCast(l + 4)),
                    @as(isize, @intCast(n2 - 1 - k0 * i)),
                    -@as(isize, @intCast(k0_2)),
                    @as(usize, 1) << @as(u6, @intCast(l + 3)),
                );
            }
        }
        while (l + 7 <= ld) : (l += 1) {
            const k0 = n >> @as(u6, @intCast(l + 2));
            const k1 = @as(usize, 1) << @as(u6, @intCast(l + 3));
            const k0_2 = k0 >> 1;
            const rlim = n >> @as(u6, @intCast(l + 6));
            const lim = @as(usize, 1) << @as(u6, @intCast(l + 1));
            var i_off: isize = @intCast(n2 - 1);
            var aa: usize = 0;
            var r: usize = rlim;
            while (r > 0) : (r -= 1) {
                self.step3Column(
                    u,
                    lim,
                    i_off,
                    -@as(isize, @intCast(k0_2)),
                    aa,
                    k1,
                    k0,
                );
                aa += k1 * 4;
                i_off -= 8;
            }
        }
        self.step3ColumnLast(u, n / 32, @as(isize, @intCast(n2 - 1)), n);

        // step four copies the output of the butterfly into the scratch
        // buffer with the bit reversed order
        {
            var d0: isize = @intCast(n4 - 4);
            var d1: isize = @intCast(n2 - 4);
            var index: usize = 0;
            while (d0 >= 0) : ({
                    d0 -= 4;
                    d1 -= 4;
                    index += 2;
                }) {
                const k4 = self.bit_reverse[index];
                set(v, d1 + 3, u[k4]);
                set(v, d1 + 2, u[k4 + 1]);
                set(v, d0 + 3, u[k4 + 2]);
                set(v, d0 + 2, u[k4 + 3]);

                const k5 = self.bit_reverse[index + 1];
                set(v, d1 + 1, u[k5]);
                set(v, d1, u[k5 + 1]);
                set(v, d0 + 1, u[k5 + 2]);
                set(v, d0, u[k5 + 3]);
            }
        }

        // step seven rotates the pairs of the scratch buffer in place
        {
            var d: isize = 0;
            var e: isize = @intCast(n2 - 4);
            var aa: usize = 0;
            while (d < e) : ({
                    aa += 4;
                    d += 4;
                    e -= 4;
                }) {
                var a02 = get(v, d) - get(v, e + 2);
                var a11 = get(v, d + 1) + get(v, e + 3);

                var b0 = self.c[aa + 1] * a02 + self.c[aa] * a11;
                var b1 = self.c[aa + 1] * a11 - self.c[aa] * a02;

                const b2 = get(v, d) + get(v, e + 2);
                const b3 = get(v, d + 1) - get(v, e + 3);

                set(v, d, b2 + b0);
                set(v, d + 1, b3 + b1);
                set(v, e + 2, b2 - b0);
                set(v, e + 3, b1 - b3);

                a02 = get(v, d + 2) - get(v, e);
                a11 = get(v, d + 3) + get(v, e + 1);

                b0 = self.c[aa + 3] * a02 + self.c[aa + 2] * a11;
                b1 = self.c[aa + 3] * a11 - self.c[aa + 2] * a02;

                const b2b = get(v, d + 2) + get(v, e);
                const b3b = get(v, d + 3) - get(v, e + 1);

                set(v, d + 2, b2b + b0);
                set(v, d + 3, b3b + b1);
                set(v, e, b2b - b0);
                set(v, e + 1, b1 - b3b);
            }
        }

        // step eight writes the time domain samples into the output buffer
        {
            var aa: isize = @intCast(n2 - 8);
            var e: isize = @intCast(n2 - 8);
            var d0: usize = 0;
            var d1: isize = @intCast(n2 - 4);
            var d2: usize = n2;
            var d3: isize = @intCast(n - 4);
            while (e >= 0) : ({
                    aa -= 8;
                    e -= 8;
                    d0 += 4;
                    d2 += 4;
                    d1 -= 4;
                    d3 -= 4;
                }) {
                const p3 = get(v, e + 6) * self.b[@intCast(aa + 7)]
                    - get(v, e + 7) * self.b[@intCast(aa + 6)];
                const p2 = -get(v, e + 6) * self.b[@intCast(aa + 6)]
                    - get(v, e + 7) * self.b[@intCast(aa + 7)];

                u[d0] = p3;
                set(u, d1 + 3, -p3);
                u[d2] = p2;
                set(u, d3 + 3, p2);

                const p1 = get(v, e + 4) * self.b[@intCast(aa + 5)]
                    - get(v, e + 5) * self.b[@intCast(aa + 4)];
                const p0 = -get(v, e + 4) * self.b[@intCast(aa + 4)]
                    - get(v, e + 5) * self.b[@intCast(aa + 5)];

                u[d0 + 1] = p1;
                set(u, d1 + 2, -p1);
                u[d2 + 1] = p0;
                set(u, d3 + 2, p0);

                const q3 = get(v, e + 2) * self.b[@intCast(aa + 3)]
                    - get(v, e + 3) * self.b[@intCast(aa + 2)];
                const q2 = -get(v, e + 2) * self.b[@intCast(aa + 2)]
                    - get(v, e + 3) * self.b[@intCast(aa + 3)];

                u[d0 + 2] = q3;
                set(u, d1 + 1, -q3);
                u[d2 + 2] = q2;
                set(u, d3 + 1, q2);

                const q1 = get(v, e) * self.b[@intCast(aa + 1)]
                    - get(v, e + 1) * self.b[@intCast(aa)];
                const q0 = -get(v, e) * self.b[@intCast(aa)]
                    - get(v, e + 1) * self.b[@intCast(aa + 1)];

                u[d0 + 3] = q1;
                set(u, d1, -q1);
                u[d2 + 3] = q0;
                set(u, d3, q0);
            }
        }
    }

    /// first iteration of the butterfly stage
    fn step3Iter0(
        self: *const Transform,
        u: []f32,
        count: usize,
        i_off: isize,
        k_off: isize,
    ) void {
        var ee0 = i_off;
        var ee2 = i_off + k_off;
        var aa: usize = 0;
        var i = count / 4;
        while (i > 0) : (i -= 1) {
            var k00_20 = get(u, ee0) - get(u, ee2);
            var k01_21 = get(u, ee0 - 1) - get(u, ee2 - 1);
            set(u, ee0, get(u, ee0) + get(u, ee2));
            set(u, ee0 - 1, get(u, ee0 - 1) + get(u, ee2 - 1));
            set(u, ee2, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, ee2 - 1, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += 8;

            k00_20 = get(u, ee0 - 2) - get(u, ee2 - 2);
            k01_21 = get(u, ee0 - 3) - get(u, ee2 - 3);
            set(u, ee0 - 2, get(u, ee0 - 2) + get(u, ee2 - 2));
            set(u, ee0 - 3, get(u, ee0 - 3) + get(u, ee2 - 3));
            set(u, ee2 - 2, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, ee2 - 3, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += 8;

            k00_20 = get(u, ee0 - 4) - get(u, ee2 - 4);
            k01_21 = get(u, ee0 - 5) - get(u, ee2 - 5);
            set(u, ee0 - 4, get(u, ee0 - 4) + get(u, ee2 - 4));
            set(u, ee0 - 5, get(u, ee0 - 5) + get(u, ee2 - 5));
            set(u, ee2 - 4, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, ee2 - 5, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += 8;

            k00_20 = get(u, ee0 - 6) - get(u, ee2 - 6);
            k01_21 = get(u, ee0 - 7) - get(u, ee2 - 7);
            set(u, ee0 - 6, get(u, ee0 - 6) + get(u, ee2 - 6));
            set(u, ee0 - 7, get(u, ee0 - 7) + get(u, ee2 - 7));
            set(u, ee2 - 6, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, ee2 - 7, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += 8;

            ee0 -= 8;
            ee2 -= 8;
        }
    }

    /// row iteration of the butterfly stage
    fn step3Row(
        self: *const Transform,
        u: []f32,
        limit: usize,
        d0: isize,
        k_off: isize,
        k1: usize,
    ) void {
        var e0 = d0;
        var e2 = d0 + k_off;
        var aa: usize = 0;
        var i = limit / 4;
        while (i > 0) : (i -= 1) {
            var k00_20 = get(u, e0) - get(u, e2);
            var k01_21 = get(u, e0 - 1) - get(u, e2 - 1);
            set(u, e0, get(u, e0) + get(u, e2));
            set(u, e0 - 1, get(u, e0 - 1) + get(u, e2 - 1));
            set(u, e2, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, e2 - 1, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += k1;

            k00_20 = get(u, e0 - 2) - get(u, e2 - 2);
            k01_21 = get(u, e0 - 3) - get(u, e2 - 3);
            set(u, e0 - 2, get(u, e0 - 2) + get(u, e2 - 2));
            set(u, e0 - 3, get(u, e0 - 3) + get(u, e2 - 3));
            set(u, e2 - 2, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, e2 - 3, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += k1;

            k00_20 = get(u, e0 - 4) - get(u, e2 - 4);
            k01_21 = get(u, e0 - 5) - get(u, e2 - 5);
            set(u, e0 - 4, get(u, e0 - 4) + get(u, e2 - 4));
            set(u, e0 - 5, get(u, e0 - 5) + get(u, e2 - 5));
            set(u, e2 - 4, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, e2 - 5, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += k1;

            k00_20 = get(u, e0 - 6) - get(u, e2 - 6);
            k01_21 = get(u, e0 - 7) - get(u, e2 - 7);
            set(u, e0 - 6, get(u, e0 - 6) + get(u, e2 - 6));
            set(u, e0 - 7, get(u, e0 - 7) + get(u, e2 - 7));
            set(u, e2 - 6, k00_20 * self.a[aa] - k01_21 * self.a[aa + 1]);
            set(u, e2 - 7, k01_21 * self.a[aa] + k00_20 * self.a[aa + 1]);
            aa += k1;

            e0 -= 8;
            e2 -= 8;
        }
    }

    /// column iteration of the butterfly stage
    fn step3Column(
        self: *const Transform,
        u: []f32,
        count: usize,
        i_off: isize,
        k_off: isize,
        aa: usize,
        a_off: usize,
        k0: usize,
    ) void {
        const a0 = self.a[aa];
        const a1 = self.a[aa + 1];
        const a2 = self.a[aa + a_off];
        const a3 = self.a[aa + a_off + 1];
        const a4 = self.a[aa + a_off * 2];
        const a5 = self.a[aa + a_off * 2 + 1];
        const a6 = self.a[aa + a_off * 3];
        const a7 = self.a[aa + a_off * 3 + 1];
        var ee0 = i_off;
        var ee2 = i_off + k_off;
        var i = count;
        while (i > 0) : (i -= 1) {
            var k00 = get(u, ee0) - get(u, ee2);
            var k11 = get(u, ee0 - 1) - get(u, ee2 - 1);
            set(u, ee0, get(u, ee0) + get(u, ee2));
            set(u, ee0 - 1, get(u, ee0 - 1) + get(u, ee2 - 1));
            set(u, ee2, k00 * a0 - k11 * a1);
            set(u, ee2 - 1, k11 * a0 + k00 * a1);

            k00 = get(u, ee0 - 2) - get(u, ee2 - 2);
            k11 = get(u, ee0 - 3) - get(u, ee2 - 3);
            set(u, ee0 - 2, get(u, ee0 - 2) + get(u, ee2 - 2));
            set(u, ee0 - 3, get(u, ee0 - 3) + get(u, ee2 - 3));
            set(u, ee2 - 2, k00 * a2 - k11 * a3);
            set(u, ee2 - 3, k11 * a2 + k00 * a3);

            k00 = get(u, ee0 - 4) - get(u, ee2 - 4);
            k11 = get(u, ee0 - 5) - get(u, ee2 - 5);
            set(u, ee0 - 4, get(u, ee0 - 4) + get(u, ee2 - 4));
            set(u, ee0 - 5, get(u, ee0 - 5) + get(u, ee2 - 5));
            set(u, ee2 - 4, k00 * a4 - k11 * a5);
            set(u, ee2 - 5, k11 * a4 + k00 * a5);

            k00 = get(u, ee0 - 6) - get(u, ee2 - 6);
            k11 = get(u, ee0 - 7) - get(u, ee2 - 7);
            set(u, ee0 - 6, get(u, ee0 - 6) + get(u, ee2 - 6));
            set(u, ee0 - 7, get(u, ee0 - 7) + get(u, ee2 - 7));
            set(u, ee2 - 6, k00 * a6 - k11 * a7);
            set(u, ee2 - 7, k11 * a6 + k00 * a7);

            ee0 -= @intCast(k0);
            ee2 -= @intCast(k0);
        }
    }

    /// last iterations of the butterfly stage
    fn step3ColumnLast(
        self: *const Transform,
        u: []f32,
        count: usize,
        i_off: isize,
        base_n: usize,
    ) void {
        const a2 = self.a[base_n >> 3];
        var z = i_off;
        const base = z - @as(isize, @intCast(16 * count));
        while (z > base) : (z -= 16) {
            var k00 = get(u, z) - get(u, z - 8);
            var k11 = get(u, z - 1) - get(u, z - 9);
            var l00 = get(u, z - 2) - get(u, z - 10);
            var l11 = get(u, z - 3) - get(u, z - 11);
            set(u, z, get(u, z) + get(u, z - 8));
            set(u, z - 1, get(u, z - 1) + get(u, z - 9));
            set(u, z - 2, get(u, z - 2) + get(u, z - 10));
            set(u, z - 3, get(u, z - 3) + get(u, z - 11));
            set(u, z - 8, k00);
            set(u, z - 9, k11);
            set(u, z - 10, (l00 + l11) * a2);
            set(u, z - 11, (l11 - l00) * a2);

            k00 = get(u, z - 4) - get(u, z - 12);
            k11 = get(u, z - 5) - get(u, z - 13);
            l00 = get(u, z - 6) - get(u, z - 14);
            l11 = get(u, z - 7) - get(u, z - 15);
            set(u, z - 4, get(u, z - 4) + get(u, z - 12));
            set(u, z - 5, get(u, z - 5) + get(u, z - 13));
            set(u, z - 6, get(u, z - 6) + get(u, z - 14));
            set(u, z - 7, get(u, z - 7) + get(u, z - 15));
            set(u, z - 12, k11);
            set(u, z - 13, -k00);
            set(u, z - 14, (l11 - l00) * a2);
            set(u, z - 15, (l00 + l11) * -a2);

            iter54(u, z);
            iter54(u, z - 8);
        }
    }
};

/// mix the values of a butterfly group
fn iter54(u: []f32, z: isize) void {
    const k00 = get(u, z) - get(u, z - 4);
    const y0 = get(u, z) + get(u, z - 4);
    const y2 = get(u, z - 2) + get(u, z - 6);
    const k22 = get(u, z - 2) - get(u, z - 6);

    set(u, z, y0 + y2);
    set(u, z - 2, y0 - y2);

    const k33 = get(u, z - 3) - get(u, z - 7);

    set(u, z - 4, k00 + k33);
    set(u, z - 6, k00 - k33);

    const k11 = get(u, z - 1) - get(u, z - 5);
    const y1 = get(u, z - 1) + get(u, z - 5);
    const y3 = get(u, z - 3) + get(u, z - 7);

    set(u, z - 1, y1 + y3);
    set(u, z - 3, y1 - y3);
    set(u, z - 5, k11 - k22);
    set(u, z - 7, k11 + k22);
}

/// read a value with a signed index
inline fn get(values: []f32, index: isize) f32 {
    return values[@intCast(index)];
}

/// write a value with a signed index
inline fn set(values: []f32, index: isize, value: f32) void {
    values[@intCast(index)] = value;
}

/// compute twiddles using stb_vorbis.c compute_twiddle_factors() layout
/// even stage uses cos and sin pairs, odd uses halves, final uses quarters
pub fn computeTwiddles(size: usize, a: []f32, b: []f32, c: []f32) void {
    const pi = std.math.pi;
    const quarter = size / 4;
    const eighth = size / 8;
    var index: usize = 0;
    while (index < quarter) : (index += 1) {
        const angle = 4.0
            * @as(f64, @floatFromInt(index))
            * pi
            / @as(f64, @floatFromInt(size));
        a[index * 2] = @floatCast(@cos(angle));
        a[index * 2 + 1] = @floatCast(-@sin(angle));
        const odd_angle = (@as(f64, @floatFromInt(index)) * 2 + 1)
            * pi
            / @as(f64, @floatFromInt(size))
            / 2;
        b[index * 2] = @floatCast(@cos(odd_angle) * 0.5);
        b[index * 2 + 1] = @floatCast(@sin(odd_angle) * 0.5);
    }
    index = 0;
    while (index < eighth) : (index += 1) {
        const angle = 2.0
            * (@as(f64, @floatFromInt(index)) * 2 + 1)
            * pi
            / @as(f64, @floatFromInt(size));
        c[index * 2] = @floatCast(@cos(angle));
        c[index * 2 + 1] = @floatCast(-@sin(angle));
    }
}

/// compute final stage bit reversed indices
/// ref: stb_vorbis.c compute_bitreverse()
pub fn computeBitReverse(size: usize, reverse: []u16) void {
    const bits = std.math.log2_int(usize, size);
    for (reverse, 0..) |*value, index| {
        value.* = @intCast(
            (@bitReverse(@as(u32, @intCast(index)))
                >> @as(u5, @intCast(32 - bits + 3)))
                << 2,
        );
    }
}

/// compute rising half window, falling slope is its reverse
/// sin(pi/2 * sin^2((x + 0.5) / n * pi))
pub fn computeWindow(size: usize, window: []f32) void {
    const pi = std.math.pi;
    const half = size / 2;
    for (window[0..half], 0..) |*value, index| {
        const inner = @sin(
            (@as(f64, @floatFromInt(index)) + 0.5)
                / @as(f64, @floatFromInt(half))
                * 0.5
                * pi,
        );
        value.* = @floatCast(@sin(0.5 * pi * inner * inner));
    }
}
