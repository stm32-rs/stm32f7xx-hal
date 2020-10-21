use std::{
    env,
    fs::File,
    io::{self, prelude::*},
    path::PathBuf,
};

fn main() -> Result<(), Error> {
    let target = Target::read();

    copy_memory_config(target)?;

    println!("cargo:rerun-if-changed=build.rs");

    Ok(())
}

/// Make `memory.x` available to dependent crates
fn copy_memory_config(target: Target) -> Result<(), Error> {
    let memory_x = match target.sub_family {
        SubFamily::Stm32f722
        | SubFamily::Stm32f723
        | SubFamily::Stm32f732
        | SubFamily::Stm32f733 => include_bytes!("memory_512_176.x").as_ref(),
        SubFamily::Stm32f745 | SubFamily::Stm32f746 | SubFamily::Stm32f756 => {
            include_bytes!("memory_1024_320.x").as_ref()
        }
        SubFamily::Stm32f765
        | SubFamily::Stm32f767
        | SubFamily::Stm32f769
        | SubFamily::Stm32f777
        | SubFamily::Stm32f778
        | SubFamily::Stm32f779 => include_bytes!("memory_2048_368.x").as_ref(),
        SubFamily::Stm32f730 => include_bytes!("memory_64_176.x").as_ref(),
    };

    let out_dir = env::var("OUT_DIR")?;
    let out_dir = PathBuf::from(out_dir);

    File::create(out_dir.join("memory.x"))?.write_all(memory_x)?;

    // Tell Cargo where to find the file.
    println!("cargo:rustc-link-search={}", out_dir.display());

    println!("cargo:rerun-if-changed=memory_1024_320.x");
    println!("cargo:rerun-if-changed=memory_2048_368.x");
    println!("cargo:rerun-if-changed=memory_512_176.x");
    println!("cargo:rerun-if-changed=memory_64_176.x");
    println!("cargo:rerun-if-changed=memory_64_240.x");

    Ok(())
}

#[derive(Clone, Copy)]
struct Target {
    sub_family: SubFamily,
}

impl Target {
    fn read() -> Self {
        let sub_family = SubFamily::read();

        Self { sub_family }
    }
}

#[derive(Clone, Copy)]
enum SubFamily {
    Stm32f722,
    Stm32f723,
    Stm32f730,
    Stm32f732,
    Stm32f733,
    Stm32f745,
    Stm32f746,
    Stm32f756,
    Stm32f765,
    Stm32f767,
    Stm32f769,
    Stm32f777,
    Stm32f778,
    Stm32f779,
}

impl SubFamily {
    fn read() -> Self {
        if cfg!(feature = "stm32f722") {
            SubFamily::Stm32f722
        } else if cfg!(feature = "stm32f723") {
            SubFamily::Stm32f723
        } else if cfg!(feature = "stm32f730") {
            SubFamily::Stm32f730
        } else if cfg!(feature = "stm32f732") {
            SubFamily::Stm32f732
        } else if cfg!(feature = "stm32f733") {
            SubFamily::Stm32f733
        } else if cfg!(feature = "stm32f745") {
            SubFamily::Stm32f745
        } else if cfg!(feature = "stm32f746") {
            SubFamily::Stm32f746
        } else if cfg!(feature = "stm32f756") {
            SubFamily::Stm32f756
        } else if cfg!(feature = "stm32f765") {
            SubFamily::Stm32f765
        } else if cfg!(feature = "stm32f767") {
            SubFamily::Stm32f767
        } else if cfg!(feature = "stm32f769") {
            SubFamily::Stm32f769
        } else if cfg!(feature = "stm32f777") {
            SubFamily::Stm32f777
        } else if cfg!(feature = "stm32f778") {
            SubFamily::Stm32f778
        } else if cfg!(feature = "stm32f779") {
            SubFamily::Stm32f779
        } else {
            error("You must select a target.
If you added Stm32f7xx HAL as a dependency to your crate, you can select a target by enabling the respective feature in `Cargo.toml`.
If you're running an example from the repository, select a target by passing the desired target as a command-line argument, for example `--features=stm32f746`.
Please refer to the documentation for more details."
                )
        }
    }
}

#[derive(Debug)]
enum Error {
    Env(env::VarError),
    Io(io::Error),
}

impl From<env::VarError> for Error {
    fn from(error: env::VarError) -> Self {
        Self::Env(error)
    }
}

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

fn error(message: &str) -> ! {
    panic!("\n\n\n{}\n\n\n", message);
}
