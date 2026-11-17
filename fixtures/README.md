# Compiler fixtures

`monaco.osm.pbf` is a small Monaco extract, copied from Planetiler's test
resources. It contains
OpenStreetMap data, © OpenStreetMap contributors, under the Open Database License
1.0: https://www.openstreetmap.org/copyright.

Both compilers use this fixture for local verification without network access.
`monaco-info.json` is the description of the routing package
`scripts/check.sh` builds from it, with its byte length and dataset hash, which
are the same on every platform; record an intended change with
`UPDATE_MONACO_INFO=1 scripts/check.sh`.

`krakow-centre.osm.pbf` is the routable part of Kraków's centre (the Old Town
with the Rynek, Kazimierz and part of Podgórze), cut from the Geofabrik extract
`europe/poland/malopolskie` of 2026-09-29 with
`cargo run --release --example extract_bbox --
malopolskie-260929.osm.pbf krakow-centre.osm.pbf 19.920 50.040 19.965 50.072` in a checkout of
[`osm-pbf-ingest`](https://github.com/Limina-Agency/osm-pbf-ingest).
It also contains OpenStreetMap data, © OpenStreetMap contributors, under the
Open Database License 1.0: https://www.openstreetmap.org/copyright.
`scripts/check.sh` runs the Kraków centre regression gate on it (see
`benchmarks/README.md`).
