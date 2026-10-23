//! bound seek index memory; callers define locations and frame positions
const std = @import("std");
const Error = @import("../types.zig").Error;

/// owned index with caller defined locations and bounded storage
pub fn Index(comptime Location: type) type {
    return struct {
        const Self = @This();
        /// untrimmed frame position paired with container seek location
        pub const Entry = struct { frame: u64, location: Location };

        entries: std.ArrayList(Entry) = .empty,

        /// append nondecreasing frames during initialization
        /// keep entries and spare capacity within the byte limit
        pub fn append(
            self: *Self,
            allocator: std.mem.Allocator,
            limit: usize,
            entry: Entry,
        ) Error!void {
            if (
                self.entries.items.len != 0
                    and entry.frame
                        < self.entries.items[self.entries.items.len - 1].frame
            ) {
                return error.InvalidData;
            }
            // budget includes spare capacity as well as populated entries
            const max_entries = limit / @sizeOf(Entry);
            if (self.entries.items.len >= max_entries) {
                return error.LimitExceeded;
            }
            if (self.entries.items.len == self.entries.capacity) {
                // start with up to 64 entries; double within the budget
                const capacity = @min(
                    max_entries,
                    @max(@as(usize, 64), self.entries.capacity *| 2),
                );
                try self.entries.ensureTotalCapacityPrecise(
                    allocator,
                    capacity,
                );
            }
            self.entries.appendAssumeCapacity(entry);
        }

        /// find the last entry at or before target; clamp earlier targets to the first
        /// return null for an empty index
        pub fn atOrBefore(self: *const Self, target: u64) ?Entry {
            const index = self.indexAtOrBefore(target) orelse return null;
            return self.entries.items[index];
        }

        /// position of last entry at or before target, clamp earlier targets to first
        /// return null for empty index
        pub fn indexAtOrBefore(self: *const Self, target: u64) ?usize {
            if (self.entries.items.len == 0) return null;
            var low: usize = 0;
            var high = self.entries.items.len;
            // upper bound search, first entry strictly after target
            while (low < high) {
                const midpoint = low + (high - low) / 2;
                if (self.entries.items[midpoint].frame <= target) {
                    low = midpoint + 1;
                } else {
                    high = midpoint;
                }
            }
            // use preceding entry, clamp earlier targets to first entry
            return low -| 1;
        }

        /// transfer entries; leave self empty and safe to deinit
        pub fn take(self: *Self) Self {
            const owned = self.*;
            self.* = .{};
            return owned;
        }

        /// free all index entries and reset to empty
        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            self.entries.deinit(allocator);
            self.* = .{};
        }
    };
}
