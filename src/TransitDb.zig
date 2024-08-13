const c = @cImport({
    @cInclude("sqlite3.h");
});

const std = @import("std");
const Allocator = std.mem.Allocator;

const Point = struct {
    lon: f32,
    lat: f32,
};

db: *c.sqlite3,

const Db = @This();

const gen_db_script = @embedFile("TransitDb/create_sqlite_db.sql");

pub fn makeDb(alloc: Allocator, db_path: []const u8) !*c.sqlite3 {
    var seed: u64 = undefined;
    try std.posix.getrandom(std.mem.asBytes(&seed));
    var rng = std.rand.DefaultPrng.init(seed);

    var tmp_path_og: [16]u8 = undefined;
    rng.fill(&tmp_path_og);

    var tmp_path_buf: [32]u8 = undefined;
    const tmp_path = std.base64.url_safe.Encoder.encode(&tmp_path_buf, &tmp_path_og);

    const cwd = std.fs.cwd();
    var dir = try cwd.makeOpenPath(tmp_path, .{});
    defer {
        dir.close();
        cwd.deleteTree(tmp_path) catch |e| {
            std.log.err("Failed to delete tmp path: {s} ({s})", .{ tmp_path, @errorName(e) });
        };
    }

    const db_file = try cwd.openFile(db_path, .{});
    defer db_file.close();

    try std.zip.extract(dir, db_file.seekableStream(), .{});

    const sqlite_script = try dir.createFile("create_sqlite_db.sql", .{});
    try sqlite_script.writeAll(gen_db_script);

    const gen_result = try std.process.Child.run(.{
        .allocator = alloc,
        .argv = &.{ "sqlite3", "-init", "create_sqlite_db.sql", "sqlite.db", ".quit" },
        .cwd_dir = dir,
    });

    if (gen_result.term != .Exited) {
        return error.GenDb;
    }

    if (gen_result.term.Exited != 0) {
        return error.GenDb;
    }

    var db_opt: ?*c.sqlite3 = undefined;
    const sqlite_path = try std.fs.path.joinZ(alloc, &.{ tmp_path, "sqlite.db" });
    defer alloc.free(sqlite_path);

    if (c.sqlite3_open(sqlite_path, &db_opt) != c.SQLITE_OK) {
        closeDb(db_opt);
        return error.Sql;
    }

    return db_opt orelse return error.DbInit;
}

pub fn init(alloc: Allocator, db_path: []const u8) !Db {
    const db = try makeDb(alloc, db_path);

    return .{
        .db = db,
    };
}

pub fn deinit(self: *Db) void {
    closeDb(self.db);
}

pub const Stops = struct {
    ids: []const i64,
    points: []const Point,

    pub fn deinit(self: *Stops, alloc: Allocator) void {
        alloc.free(self.ids);
        alloc.free(self.points);
    }
};

pub const TripInfo = struct {
    trip_id: i64,
    stop_ids: []const i64,
    stop_times: []u32,

    pub fn deinit(self: *TripInfo, alloc: Allocator) void {
        alloc.free(self.stop_ids);
        alloc.free(self.stop_times);
    }
};

pub const RouteInfo = struct {
    trip_to_route_idx: std.AutoHashMap(i64, usize),
    route_short_names: []const []const u8,
    route_long_names: []const []const u8,

    pub fn deinit(self: *RouteInfo, alloc: Allocator) void {
        for (self.route_short_names) |name| {
            alloc.free(name);
        }

        for (self.route_long_names) |name| {
            alloc.free(name);
        }

        alloc.free(self.route_short_names);
        alloc.free(self.route_long_names);
        self.trip_to_route_idx.deinit();
    }
};

const StopSequence = struct {
    ids: []i64,
    points: []Point,

    fn deinit(self: *StopSequence, alloc: Allocator) void {
        alloc.free(self.ids);
        alloc.free(self.points);
    }
};

pub fn getStops(self: *Db, alloc: Allocator) !Stops {
    const statement = try makeStatement(self.db,
        \\ SELECT stops.stop_lon, stops.stop_lat, stops.stop_id
        \\ FROM stops
    , "get stop points");
    defer finalizeStatement(statement);

    var ret_points = std.ArrayList(Point).init(alloc);
    defer ret_points.deinit();

    var ret_ids = std.ArrayList(i64).init(alloc);
    defer ret_ids.deinit();

    while (true) {
        const sqlite_ret = c.sqlite3_step(statement);
        if (sqlite_ret == c.SQLITE_DONE) {
            break;
        }

        if (sqlite_ret != c.SQLITE_ROW) {
            return error.Invalid;
        }
        const lon_s = extractColumnTextTemporary(statement, 0) orelse return error.NoLon;
        const lon = try std.fmt.parseFloat(f32, lon_s);

        const lat_s = extractColumnTextTemporary(statement, 1) orelse return error.NoLat;
        const lat = try std.fmt.parseFloat(f32, lat_s);

        const stop_id = c.sqlite3_column_int64(statement, 2);

        try ret_points.append(.{
            .lon = lon,
            .lat = lat,
        });

        try ret_ids.append(stop_id);
    }

    return .{
        .points = try ret_points.toOwnedSlice(),
        .ids = try ret_ids.toOwnedSlice(),
    };
}

pub fn getTrips(self: *Db, alloc: Allocator) ![]TripInfo {
    const statement = try makeStatement(self.db,
        \\ SELECT stop_times.stop_id, stop_times.departure_time, stop_times.trip_id
        \\ FROM stop_times
        \\ ORDER BY cast(trip_id as integer), cast(stop_times.stop_sequence as integer);
    , "get stop times");
    defer finalizeStatement(statement);

    var ret = std.ArrayList(TripInfo).init(alloc);
    defer ret.deinit();

    var this_trip_stops = std.ArrayList(i64).init(alloc);
    defer this_trip_stops.deinit();

    var this_trip_stop_times = std.ArrayList(u32).init(alloc);
    defer this_trip_stop_times.deinit();

    var trip_id: i64 = -1;
    while (true) {
        const sqlite_ret = c.sqlite3_step(statement);
        if (sqlite_ret == c.SQLITE_DONE) {
            break;
        }

        if (sqlite_ret != c.SQLITE_ROW) {
            return error.Invalid;
        }
        const stop_id = c.sqlite3_column_int64(statement, 0);

        const departure_time_s = extractColumnTextTemporary(statement, 1) orelse return error.NoTime;
        var time_it = std.mem.splitScalar(u8, departure_time_s, ':');

        var hours_s = time_it.next() orelse return error.NoHours;
        var minutes_s = time_it.next() orelse return error.NoMinutes;
        var seconds_s = time_it.next() orelse return error.NoSeconds;

        hours_s = std.mem.trim(u8, hours_s, &std.ascii.whitespace);
        minutes_s = std.mem.trim(u8, minutes_s, &std.ascii.whitespace);
        seconds_s = std.mem.trim(u8, seconds_s, &std.ascii.whitespace);

        const hours = std.fmt.parseInt(u32, hours_s, 10) catch return error.InvalidHours;
        const minutes = std.fmt.parseInt(u32, minutes_s, 10) catch return error.InvalidMinutes;
        const seconds = std.fmt.parseInt(u32, seconds_s, 10) catch return error.InvalidSeconds;

        const departure_time = hours * 60 * 60 + minutes * 60 + seconds;

        const this_trip_id = c.sqlite3_column_int64(statement, 2);

        if (trip_id != this_trip_id) {
            if (trip_id != -1) {
                const stops = try this_trip_stops.toOwnedSlice();
                const times = try this_trip_stop_times.toOwnedSlice();

                try ret.append(.{
                    .trip_id = trip_id,
                    .stop_ids = stops,
                    .stop_times = times,
                });
            }
            trip_id = this_trip_id;
        }

        try this_trip_stops.append(stop_id);
        try this_trip_stop_times.append(departure_time);
    }

    const stops = try this_trip_stops.toOwnedSlice();
    const times = try this_trip_stop_times.toOwnedSlice();

    try ret.append(.{
        .trip_id = trip_id,
        .stop_ids = stops,
        .stop_times = times,
    });

    return try ret.toOwnedSlice();
}

pub fn getRoutes(self: *Db, alloc: Allocator) !RouteInfo {
    var route_id_to_idx = std.AutoHashMap(i64, usize).init(alloc);
    defer route_id_to_idx.deinit();

    var short_names = std.ArrayList([]const u8).init(alloc);
    defer {
        for (short_names.items) |name| {
            alloc.free(name);
        }
        short_names.deinit();
    }

    var long_names = std.ArrayList([]const u8).init(alloc);
    defer {
        for (long_names.items) |name| {
            alloc.free(name);
        }
        long_names.deinit();
    }

    const statement = try makeStatement(self.db,
        \\ SELECT route_id, route_short_name, route_long_name FROM routes;
    , "get routes");
    defer finalizeStatement(statement);

    while (true) {
        const sqlite_ret = c.sqlite3_step(statement);
        if (sqlite_ret == c.SQLITE_DONE) {
            break;
        }

        if (sqlite_ret != c.SQLITE_ROW) {
            return error.Invalid;
        }

        const route_id = c.sqlite3_column_int64(statement, 0);
        const short_name = extractColumnTextTemporary(statement, 1) orelse return error.NoShortName;
        const long_name = extractColumnTextTemporary(statement, 2) orelse return error.NoLongName;

        try route_id_to_idx.put(route_id, short_names.items.len);
        try short_names.append(try alloc.dupe(u8, short_name));
        try long_names.append(try alloc.dupe(u8, long_name));
    }

    var trip_to_route_idx = std.AutoHashMap(i64, usize).init(alloc);
    errdefer trip_to_route_idx.deinit();

    const trips_statement = try makeStatement(self.db,
        \\ SELECT route_id, trip_id FROM trips;
    , "get trips");
    defer finalizeStatement(trips_statement);

    while (true) {
        const sqlite_ret = c.sqlite3_step(trips_statement);
        if (sqlite_ret == c.SQLITE_DONE) {
            break;
        }

        if (sqlite_ret != c.SQLITE_ROW) {
            return error.Invalid;
        }

        const route_id = c.sqlite3_column_int64(trips_statement, 0);
        const trip_id = c.sqlite3_column_int64(trips_statement, 1);

        try trip_to_route_idx.put(trip_id, route_id_to_idx.get(route_id).?);
    }

    return .{
        .trip_to_route_idx = trip_to_route_idx,
        .route_short_names = try short_names.toOwnedSlice(),
        .route_long_names = try long_names.toOwnedSlice(),
    };
}

fn toSqlLen(len: usize) !c_int {
    return std.math.cast(c_int, len) orelse {
        return error.Sql;
    };
}

fn fromSqlLen(len: c_int) !usize {
    return std.math.cast(usize, len) orelse {
        return error.Sql;
    };
}

fn closeDb(db: ?*c.sqlite3) void {
    if (c.sqlite3_close(db) != c.SQLITE_OK) {
        std.log.err("Failed to close db\n", .{});
    }
}

fn makeStatement(db: *c.sqlite3, sql: [:0]const u8, purpose: []const u8) !*c.sqlite3_stmt {
    var statement: ?*c.sqlite3_stmt = null;

    const ret = c.sqlite3_prepare_v2(db, sql, try toSqlLen(sql.len + 1), &statement, null);

    if (ret != c.SQLITE_OK) {
        std.log.err("Failed to prepare {s} statement", .{purpose});
        return error.Sql;
    }
    return statement orelse unreachable;
}

fn finalizeStatement(statement: *c.sqlite3_stmt) void {
    _ = c.sqlite3_finalize(statement);
}

fn checkSqliteRet(purpose: []const u8, ret: i32) !void {
    if (ret != c.SQLITE_OK) {
        std.log.err("Failed to {s}", .{purpose});
        return error.Sql;
    }
}

fn extractColumnTextTemporary(statement: *c.sqlite3_stmt, column_id: c_int) ?[]const u8 {
    const item_opt: ?[*]const u8 = @ptrCast(c.sqlite3_column_text(statement, column_id));
    const item_len = c.sqlite3_column_bytes(statement, column_id);
    if (item_opt == null) {
        return null;
    }
    return item_opt.?[0..@intCast(item_len)];
}
