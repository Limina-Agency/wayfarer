# OSM compiler regression fixtures

`nowotarska-junctions.json` contains eight ways around Nowotarska in Zakopane,
exported on 2026-09-18 from the locally installed Małopolskie OSM PBF. It retains
way/node IDs, WGS84 coordinates in longitude/latitude order, and source tags.

© OpenStreetMap contributors, ODbL 1.0. See https://www.openstreetmap.org/copyright.

The fixture covers the four service-road connectors that an earlier road
connectivity investigation identified. The compiler test reconstructs their
shared network, publishes native z14 with retained z17 detail, encodes and decodes
MVT, and checks that each service road shares an exact encoded main-road vertex.
