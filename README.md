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
> This is not a crucial error and doesn't really affect the emulator, but does mess up the performance a bit.
>
> Heres currently the biggest error that I'm working on fixing:
>
 ```
 info: Microsoft.AspNetCore.Hosting.Diagnostics[1]
      Request starting HTTP/1.1 GET http://localhost:8080/ - - -
info: Microsoft.AspNetCore.StaticFiles.StaticFileMiddleware[2]
      Sending file. Request path: '/index.html'. Physical path: '/workspaces/mid-end-emulator-core/site/webui/index.html'
info: Microsoft.AspNetCore.Hosting.Diagnostics[2]
      Request finished HTTP/1.1 GET http://localhost:8080/index.html - 200 878 text/html 141.2080ms
[EMU] FPS: 14.2 | Cycles: 37,439,550,676,282 | PC: 0x0FFB0EA0
info: Microsoft.AspNetCore.Hosting.Diagnostics[1]
      Request starting HTTP/1.1 GET http://localhost:8080/style.css - - -
info: Microsoft.AspNetCore.StaticFiles.StaticFileMiddleware[2]
      Sending file. Request path: '/style.css'. Physical path: '/workspaces/mid-end-emulator-core/site/webui/style.css'
info: Microsoft.AspNetCore.Hosting.Diagnostics[2]
      Request finished HTTP/1.1 GET http://localhost:8080/style.css - 200 2872 text/css 6.1779ms
info: Microsoft.AspNetCore.Hosting.Diagnostics[1]
      Request starting HTTP/1.1 GET http://localhost:8080/app.js - - -
info: Microsoft.AspNetCore.StaticFiles.StaticFileMiddleware[2]
      Sending file. Request path: '/app.js'. Physical path: '/workspaces/mid-end-emulator-core/site/webui/app.js'
info: Microsoft.AspNetCore.Hosting.Diagnostics[2]
      Request finished HTTP/1.1 GET http://localhost:8080/app.js - 200 3300 text/javascript 106.7069ms
[EMU] FPS: 9.4 | Cycles: 62,399,251,127,402 | PC: 0x1AA26E20
  [HLE] Unhandled SVC 0x551A at PC=0x20104430
info: Microsoft.AspNetCore.Hosting.Diagnostics[1]
      Request starting HTTP/1.1 GET http://localhost:8080/api/v1/features/environment - - -
info: Microsoft.AspNetCore.Hosting.Diagnostics[2]
      Request finished HTTP/1.1 GET http://localhost:8080/api/v1/features/environment - 404 0 - 0.5909ms
info: Microsoft.AspNetCore.Hosting.Diagnostics[16]
      Request reached the end of the middleware pipeline without being handled by application code. Request path: GET http://localhost:8080/api/v1/features/environment, Response status code: 404
[EMU] FPS: 20.7 | Cycles: 114,814,615,289,204 | PC: 0x10F1E940
info: Microsoft.AspNetCore.Hosting.Diagnostics[1]
      Request starting HTTP/1.1 GET http://localhost:8080/favicon.ico - - -
info: Microsoft.AspNetCore.Hosting.Diagnostics[2]
      Request finished HTTP/1.1 GET http://localhost:8080/favicon.ico - 404 0 - 0.3317ms
info: Microsoft.AspNetCore.Hosting.Diagnostics[16]
      Request reached the end of the middleware pipeline without being handled by application code. Request path: GET http://localhost:8080/favicon.ico, Response status code: 404
info: Microsoft.AspNetCore.Hosting.Diagnostics[1]
      Request starting HTTP/1.1 GET http://localhost:8080/ws - - -
info: Microsoft.AspNetCore.Routing.EndpointMiddleware[0]
      Executing endpoint '/ws'
```

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
