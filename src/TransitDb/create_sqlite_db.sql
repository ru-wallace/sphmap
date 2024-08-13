.import trips.txt --csv trips
.import routes.txt --csv routes
.import stops.txt --csv stops
.import stop_times.txt --csv stop_times

CREATE INDEX stop_times_index ON stop_times(stop_id);
CREATE INDEX stop_times_trip_index ON stop_times(trip_id);
CREATE INDEX stops_index ON stops(stop_id);
CREATE INDEX trips_index ON trips(trip_id);
CREATE INDEX trips_route_index ON trips(route_id);
CREATE INDEX route_index ON routes(route_id);
