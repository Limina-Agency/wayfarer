//! Regenerate the small archive used by native bridge tests.
#[path = "../../fixtures/routing_fixture.rs"]
mod routing_fixture;

fn main() {
    let path = std::env::args_os()
        .nth(1)
        .expect("pass an output .mrt path");
    routing_fixture::write(std::path::Path::new(&path));
}
