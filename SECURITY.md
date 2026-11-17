# Security

Please report vulnerabilities privately through GitHub: the repository's
**Security** tab, **Report a vulnerability**. Don't open a public issue for
them.

Areas of particular interest:

- Opening or verifying a malformed or hostile `routing.mrt` package
  (`RoutingPackage::open`, `PackageFile::verify`, the C ABI's
  `wayfarer_open` and `wayfarer_verify`).
- Request JSON passed through the C ABI.
- Overlay sets received from other devices (`RouteOverlaySet::from_json`).

Each of these has a fuzz target in `fuzz/` (`package`, `request`,
`overlay_set`; see `fuzz/README.md`). A crashing input from them is a welcome
report.

Fixes go into the latest release only while the crates are at 0.x.
