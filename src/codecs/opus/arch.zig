// SPDX-FileCopyrightText: © 2026 Jeffrey C. Ollie <jeff@ocjtech.us>
// SPDX-FileCopyrightText: © 2014 Xiph.Org Foundation
// SPDX-FileCopyrightText: © 2014 Jean-Marc Valin
// SPDX-License-Identifier: MIT AND BSD-2-Clause

//! Which of libopus's kernels to compute with: its portable C, or the
//! SSE2, SSE4.1 or AVX2 versions libopus uses on an x86-64 CPU that has
//! them. libopus's `arch`, and `x86cpu.c`'s `opus_select_arch`.
//!
//! libopus built for x86-64 presumes SSE and SSE2, which every x86-64
//! CPU has, and uses its SSE and SSE2 kernels whatever the level. It
//! chooses SSE4.1 and AVX2 at run time. So on x86-64 it has three levels,
//! `.sse2`, `.sse4_1` and `.avx2`, and `.generic` is libopus as it is
//! built for any other CPU.
//!
//! The kernels are CELT inner products, cross-correlations and the pitch
//! post-filter. Their sum order matches libopus's kernels at the selected
//! level, using Zig vectors and fused multiply-adds where appropriate.

const builtin = @import("builtin");

pub const Arch = enum(u3) {
    /// libopus's portable C, as it is built for anything but x86-64: what
    /// `detect` gives elsewhere, and the only level there.
    generic = 0,
    /// libopus built for x86-64, with no more than it presumes: its SSE
    /// and SSE2 kernels. libopus's levels 0 to 2 there.
    sse2 = 2,
    sse4_1 = 3,
    /// AVX2 with FMA, as libopus requires both.
    avx2 = 4,

    /// What libopus would choose on this CPU: the most it has, of SSE4.1
    /// and AVX2 with FMA, over the SSE2 every x86-64 has, `.generic` on
    /// anything else.
    /// `opus_select_arch`.
    pub fn detect() Arch {
        return switch (builtin.cpu.arch) {
            .x86_64 => detectX86(),
            else => .generic,
        };
    }

    fn detectX86() Arch {
        const leaf0 = cpuid(0, 0);
        // SSE and SSE2 are part of x86-64, and presumed.
        if (leaf0.eax < 1) return .sse2;
        const leaf1 = cpuid(1, 0);
        const sse41 = leaf1.ecx & (1 << 19) != 0;
        // AVX and FMA, and then AVX2 from leaf 7. libopus does not ask
        // whether the operating system saves the AVX registers, and nor
        // does this.
        var avx2 = leaf1.ecx & (1 << 28) != 0 and leaf1.ecx & (1 << 12) != 0;
        if (avx2 and leaf0.eax >= 7) {
            avx2 = cpuid(7, 0).ebx & (1 << 5) != 0;
        } else {
            avx2 = false;
        }
        if (!sse41) return .sse2;
        if (!avx2) return .sse4_1;
        return .avx2;
    }
};

const Cpuid = struct { eax: u32, ebx: u32, ecx: u32, edx: u32 };

fn cpuid(leaf: u32, subleaf: u32) Cpuid {
    var eax: u32 = undefined;
    var ebx: u32 = undefined;
    var ecx: u32 = undefined;
    var edx: u32 = undefined;
    asm volatile(
        "cpuid"
        : [eax] "={eax}" (eax),
          [ebx] "={ebx}" (ebx),
          [ecx] "={ecx}" (ecx),
          [edx] "={edx}" (edx),
        : [leaf] "{eax}" (leaf),
          [subleaf] "{ecx}" (subleaf)
    );
    return .{ .eax = eax, .ebx = ebx, .ecx = ecx, .edx = edx };
}
