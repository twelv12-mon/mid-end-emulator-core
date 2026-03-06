# mindworks

> [!NOTE]
>
> **The current progress of the project is running slowly due to the rendering portion. I have made vast progress on lots of other things, though.**
>
> - **__AVERAGE FPS__**
>   - **15.6-21.9**
>   - **May spike to as low as 5.2 but only for short periods of time, as this is the best I can do for the emulator for now.**
>  
> - **__Rom Loading speed__**
>   - **Almost instant, depending on file size. The longest it took me to load any 3ds rom was 12 seconds.**

> [!CAUTION]
>
> ### CURRENT ERRORS
> **There are a few remaining bugs, especially in rendering and requests.**
> 
>  **OFFSET ERROR**
> ```
> error: XDG_RUNTIME_DIR is invalid or not set in the environment.
> ```
>
> Gotta fix this for the rendering : )


> Uhh thats really it once i get those out the way and fix the rendering it should be ready for release.

# mid-end-emulator-core
An advanced emulator core that can be used for newer gen consoles. Made in csharp.

> [!IMPORTANT]
>
> **This is not finished yet, it will be done soon, probably by March 22nd, 2026.**

> [!WARNING]
>
> **As of current, this VM will only be able to emulate very few 3ds games, but I will be constantly working on it so it can get up to par with most 3ds games and possibly some Nintendo Switch games.**

### contents
> **This core/cpu is extremely versatile and includes the following (and more):**

* **Memory + MMU + TLB + L1 instruction/data cache**
* **Branch predictor (2-bit saturating)**
* **Preemptive scheduler (quantum-based)**
* **User/Kernel mode separation**
* **Virtual filesystem**
* **Binary loader v2 (header + segments)**
* **Assembler core**
* **CLI debugger**
* **Shader micro-language interpreter**
* **GPU command queue**
* **Timing model**
* **Interrupt controller**
* **Compiler pipeline**
* **Bytecode optimizer**
* **Virtual devices**
* **Profiling system + performance counters + memory/IO region tracking**
* **Instruction scheduling**
* **Multi-core, multi-threaded execution**
* **Full cycle accounting**

> [!TIP]
>
> **Run this in GitHub Codespaces if on chromebook!**
> 
> **Also uhh credits to me : )**
