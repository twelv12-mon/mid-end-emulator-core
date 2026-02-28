# MindWorks 3DS Emulator

A complete 3DS emulation framework built on the MindWorks VM architecture.

---

## Files

| File | Purpose |
|------|---------|
| `mindworks_vm.cs` | Core VM: assembler, compiler, memory, cache, GPU, scheduler |
| `mindworks_3ds.cs` | 3DS hardware emulation layer |
| `mindworks_launcher.cs` | Entry point / menu |
| `mindworks.csproj` | .NET project file |

---

## Build Requirements

- **.NET 8.0 SDK** — `dotnet build`
- **SDL2** — `libSDL2-2.0.so.0` (Linux) or `SDL2.dll` (Windows)
- **OpenAL** — `libopenal.so.1` (Linux) or `OpenAL32.dll` (Windows)

### Linux
```bash
sudo apt install libsdl2-dev libopenal-dev dotnet-sdk-8.0
dotnet build -c Release
dotnet run -- game.3ds
```

### Windows
```powershell
# Install SDL2 and place SDL2.dll next to the .exe
dotnet build -c Release
.\bin\Release\net8.0\mindworks.exe game.3ds
```

### macOS
```bash
brew install sdl2 openal-soft
dotnet build -c Release
dotnet run -- game.3ds
```

---

## Usage

```bash
# Load a ROM
dotnet run -- game.3ds

# Load homebrew ELF
dotnet run -- homebrew.elf

# Display test (no ROM needed)
dotnet run -- --test

# VM demo
dotnet run -- --vm

# Interactive menu
dotnet run
```

---

## Controls

| Key | 3DS Button |
|-----|-----------|
| Arrow Keys | D-Pad |
| Z | A |
| X | B |
| A | Y |
| S | X |
| Q | L |
| W | R |
| Enter | Start |
| Right Shift | Select |
| Mouse click (bottom screen) | Touchscreen |
| Escape | Quit |

---

## What's Emulated

### CPU
- **ARM11 MPCore interpreter** — Full Thumb + ARM32 instruction sets
- Condition codes (all 16 conditions)
- Barrel shifter with all shift types (LSL/LSR/ASR/ROR/RRX)
- CPSR / SPSR flags (N/Z/C/V)
- Mode switching (User/SVC/IRQ/FIQ/Abort/Undefined)
- Data processing (AND/EOR/SUB/RSB/ADD/ADC/SBC/RSC/TST/TEQ/CMP/CMN/ORR/MOV/BIC/MVN)
- Load/Store (LDR/STR/LDM/STM with all addressing modes)
- Branch (B/BL/BX/BLX)
- Two-core simulation (main core + sub-core for threads)

### Memory
- **128MB FCRAM** — full main RAM
- **6MB VRAM** — framebuffer memory
- **DSP RAM, AXI WRAM** — audio/shared memory
- **DTCM/ITCM** — ARM9 tightly coupled memory
- Complete MMIO address mapping

### GPU (PICA200)
- Software rasterizer with edge-function triangle filling
- Barycentric interpolation for vertex colors
- 3DS rotated BGR framebuffer format (columns stored vertically)
- Command list processing
- Real-time VRAM → SDL texture upload

### LCD / Display
- Dual-screen rendering: 400×240 top + 320×240 bottom
- 2× scale factor (800×480 top, 640×480 bottom)
- SDL2-accelerated (falls back to software renderer)
- Correct BGR→RGB conversion and 90° rotation de-scramble
- VBlank interrupt generation at 60Hz
- Framebuffer A/B double buffering

### Audio (DSP)
- 24 audio channels
- PCM8, PCM16, and ADPCM decoding
- Per-channel volume and stereo panning
- Software mixing to 32728 Hz stereo
- SDL2 audio queue (non-blocking)

### Input (HID)
- All 12 digital buttons (A/B/X/Y/L/R/Start/Select/D-pad)
- Touchscreen (mapped to bottom screen mouse area)
- Circle pad (analog stick, keyboard-driven or gamepad)
- IRQ injection on button events

### Timers
- 4 hardware timers with prescaler
- IRQ support on overflow

### Interrupt Controller
- Priority-based IRQ dispatch
- VBlank, HID, timer, DSP interrupts

### OS HLE (Horizon OS syscall stubs)
- `ControlMemory` — heap allocation
- `QueryMemory`
- `SleepThread`
- `CreateEvent / SignalEvent / CloseHandle`
- `ConnectToPort / SendSyncRequest` — IPC dispatch
- `CreateThread / ExitThread`
- `OutputDebugString`
- `GetSystemTick`
- **Service HLE stubs:**
  - `srv:` — Service discovery
  - `APT:U` — Applet manager (init, notification, IsRegistered)
  - `gsp::Gpu` — GPU service (FlushCache, SetBufferSwap, SetCommandList)
  - `hid:USER` — Input handles
  - `fs:USER` — File system (Open stubs)
  - `cfg:u` — System config (region, language)
  - `ptm:u` — Power (battery level)
  - `dsp::DSP` — DSP pipe
  - `csnd:SND`, `mic:u`, `ir:USER`, `ndm:u`, `ac:u`, `am:u`, `nwm::UDS`

### ROM Loader
- **NCSD (.3ds)** — Parses NCSD header, locates NCCH partition 0
- **NCCH** — Parses ExeFS, extracts and loads `.code`, `.data` segments
- **ELF** — Full PT_LOAD segment loader (homebrew)
- **Raw** — Falls back to raw binary load

---

## Limitations (honest)

- **Not full-speed** — Interpreter-based, no JIT. Expect 5-30% real 3DS speed on modern hardware.
- **No encryption** — Only works with **decrypted** ROMs (use `Decrypt9` or similar tools on your own dumps).
- **No firmware** — No official Nintendo OS. HLE stubs cover the most common calls but some games will crash.
- **No 3D** — The PICA200 rasterizer is very simplified. Complex shader effects won't render correctly.
- **No network** — NWM/AC services are stubbed out.
- **No camera / gyro** — Hardware not emulated.
- **Some games won't boot** — Anything with heavy DRM, anti-tamper, or unusual syscall usage will fail.

### Games most likely to work
- Simple 2D homebrew (best chance)
- Early 3DS launch titles with minimal OS dependency
- Games that don't use heavy 3D shaders or encryption

### Games unlikely to work
- Pokémon X/Y and later (heavy encryption + OS reliance)
- Games with online requirements
- Anything using camera/gyro/NFC

---

## Roadmap (future work)
- [ ] ARM JIT compiler (x86-64 or via LLVM) — would give 3-10× speedup
- [ ] Full PICA200 shader emulation (vertex/geometry shaders)
- [ ] Save state support
- [ ] Gamepad support (SDL2 GameController API)
- [ ] Better ExeFS decryption support
- [ ] ARM9 co-processor emulation
- [ ] mii:cfg, boss:U, and more service stubs

---

## Legal Notice

This emulator does **not** include any copyrighted Nintendo code, firmware, or keys.
You must use your own legally obtained ROM dumps.
This project is for educational and research purposes.