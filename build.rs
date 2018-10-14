extern crate cc;

fn main() {
    cc::Build::new()
        .file("c_src/SharedWithRust.c")
        .warnings(false)
        .flag("-std=c11")
        .compile("SharedWithRust");
}
