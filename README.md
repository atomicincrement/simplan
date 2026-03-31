# Simplan

A 3-D flight simulator built with [Bevy](https://bevyengine.org/) (v0.15) and a custom flight-dynamics model (FDM) based on the JSBSim equations of motion.

Features:
- F-35A aerodynamics and F135 engine thrust model
- Avian physics integration (rigid-body dynamics, half-space ground collider)
- Follow camera, checkerboard terrain, mountain landmark
- HUD: compass, altimeter, airspeed (knots), throttle bar, stick indicator
- Mouse stick control (hold LMB inside stick box) and scroll-wheel throttle

---

## Installing Rust

### Linux

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Follow the on-screen prompts and then reload your shell:

```bash
source "$HOME/.cargo/env"
```

**Bevy also requires a few system libraries.** On Debian / Ubuntu:

```bash
sudo apt-get install -y \
    pkg-config libx11-dev libxkbcommon-dev libwayland-dev \
    libasound2-dev libudev-dev libvulkan-dev
```

On Fedora / RHEL:

```bash
sudo dnf install -y \
    pkgconf-pkg-config libX11-devel libxkbcommon-devel wayland-devel \
    alsa-lib-devel systemd-devel vulkan-loader-devel
```

On Arch:

```bash
sudo pacman -S --needed \
    pkgconf libx11 libxkbcommon wayland \
    alsa-lib systemd vulkan-icd-loader
```

### Windows

Download and run the Rust installer from:

<https://win.rustup.rs/x86_64>

The installer will also prompt you to install the **Visual Studio C++ build tools** if they are not already present — accept that prompt.

After installation, open a new **Command Prompt** or **PowerShell** window; the `cargo` and `rustc` commands will be available immediately.

> **GPU drivers:** make sure your graphics drivers are up to date. Bevy uses Vulkan (preferred) or DirectX 12.

### macOS

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Follow the prompts and reload your shell:

```bash
source "$HOME/.cargo/env"
```

You may also need the Xcode command-line tools (required for the linker):

```bash
xcode-select --install
```

Bevy uses Metal on macOS — no additional GPU setup is required.

---

## Building and Running

Clone the repository and enter its directory:

```bash
git clone https://github.com/atomicincrement/simplan.git
cd simplan
```

Run the simulator:

```bash
cargo run
```

The first build will take a few minutes while dependencies compile. Subsequent builds are much faster because the workspace is configured with `opt-level = 1` for your own code and `opt-level = 3` for dependencies.

To run in release mode (best performance):

```bash
cargo run --release
```

> **Note:** the debug build uses Bevy's `dynamic_linking` feature to speed up incremental recompiles during development. This is disabled automatically in `--release`.

### FDM reference simulation

To run the standalone flight-dynamics simulation (prints telemetry to stdout, no window):

```bash
cargo run -p fdm --example simulate
```

---

## Controls

| Input | Action |
|---|---|
| Hold **LMB** inside stick box (bottom-right) | Deflect elevator / aileron |
| Release LMB or move cursor outside box | Centre stick |
| **Scroll wheel** | Throttle up / down (5 % per notch) |
| **Escape** | Quit |

The stick box is 80 × 80 px in the bottom-right corner of the window. Cursor position within the box maps linearly to ±5° (0.087 rad) deflection on each axis.

---

## Project Structure

```
simplan/
├── Cargo.toml          # workspace
├── crates/
│   ├── sim/            # Bevy application (main.rs)
│   └── fdm/            # flight dynamics library
│       ├── src/
│       │   ├── atmo.rs         # ISA atmosphere model
│       │   ├── eom.rs          # equations of motion (RK4)
│       │   ├── f35/
│       │   │   ├── aero.rs     # aerodynamic coefficients
│       │   │   ├── prop.rs     # F135 engine thrust
│       │   │   └── mod.rs      # FlightModel wrapper
│       │   └── c172/           # Cessna 172 model (reference)
│       └── examples/
│           └── simulate.rs     # headless FDM run
└── doc/
    └── conversation.md # development log
```
