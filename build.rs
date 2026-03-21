fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    embuild::espidf::sysenv::output();
}
