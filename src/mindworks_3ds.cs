// ============================================================
// MINDWORKS 3DS EMULATION LAYER
// Extends mindworks_vm.cs with full 3DS hardware emulation
//
// REQUIRES: mindworks_vm.cs compiled together
//           SDL2:    libSDL2-2.0.so.0
//           OpenAL:  libopenal.so.1
//           OpenGL:  libGL.so.1
//
// BUILD:
//   dotnet add package SDL2-CS
//   dotnet add package OpenTK
//   OR compile both files together:
//   csc mindworks_vm.cs mindworks_3ds.cs -out:mindworks.exe
//
// USAGE:
//   var emu = new DS3Emulator();
//   emu.LoadROM("game.3ds");   // .3ds, .cci, or homebrew .elf
//   emu.Run();
// ============================================================

using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace mindworks
{
    // ============================================================
    // SDL2 P/INVOKE BINDINGS
    // ============================================================
    public static class SDL2
    {
        const string SDL_LIB = "libSDL2-2.0.so.0";

        public const uint SDL_INIT_VIDEO = 0x00000020;
        public const uint SDL_INIT_AUDIO = 0x00000010;
        public const uint SDL_INIT_EVENTS = 0x00004000;
        public const uint SDL_INIT_JOYSTICK = 0x00000200;
        public const uint SDL_INIT_GAMECONTROLLER = 0x00002000;

        public const uint SDL_WINDOWPOS_CENTERED = 0x2FFF0000;
        public const uint SDL_WINDOW_SHOWN = 0x00000004;
        public const uint SDL_WINDOW_RESIZABLE = 0x00000020;
        public const uint SDL_WINDOW_OPENGL = 0x00000002;

        public const int SDL_PIXELFORMAT_RGBA8888 = 0x16462004;
        public const int SDL_PIXELFORMAT_RGB888 = 0x16161804;
        public const int SDL_PIXELFORMAT_BGR24 = 0x17401803;
        public const int SDL_TEXTUREACCESS_STREAMING = 1;
        public const int SDL_BLENDMODE_NONE = 0;
        public const int SDL_BLENDMODE_BLEND = 1;

        // Event types
        public const uint SDL_QUIT = 0x100;
        public const uint SDL_KEYDOWN = 0x300;
        public const uint SDL_KEYUP = 0x301;
        public const uint SDL_MOUSEMOTION = 0x400;
        public const uint SDL_MOUSEBUTTONDOWN = 0x401;
        public const uint SDL_MOUSEBUTTONUP = 0x402;
        public const uint SDL_JOYAXISMOTION = 0x600;
        public const uint SDL_JOYBUTTONDOWN = 0x603;
        public const uint SDL_JOYBUTTONUP = 0x604;
        public const uint SDL_CONTROLLERAXISMOTION = 0x650;
        public const uint SDL_CONTROLLERBUTTONDOWN = 0x653;
        public const uint SDL_CONTROLLERBUTTONUP = 0x654;

        // Scancodes (subset)
        public const int SDL_SCANCODE_UP = 82;
        public const int SDL_SCANCODE_DOWN = 81;
        public const int SDL_SCANCODE_LEFT = 80;
        public const int SDL_SCANCODE_RIGHT = 79;
        public const int SDL_SCANCODE_Z = 29;   // A
        public const int SDL_SCANCODE_X = 27;   // B
        public const int SDL_SCANCODE_A = 4;    // Y
        public const int SDL_SCANCODE_S = 22;   // X
        public const int SDL_SCANCODE_Q = 20;   // L
        public const int SDL_SCANCODE_W = 26;   // R
        public const int SDL_SCANCODE_RETURN = 40; // Start
        public const int SDL_SCANCODE_RSHIFT = 229; // Select
        public const int SDL_SCANCODE_ESCAPE = 41;

        // Audio format
        public const ushort AUDIO_S16SYS = 0x8010;
        public const ushort AUDIO_F32SYS = 0x8120;

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_Rect { public int x, y, w, h; }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_Point { public int x, y; }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_Color { public byte r, g, b, a; }

        [StructLayout(LayoutKind.Explicit, Size = 56)]
        public struct SDL_Event
        {
            [FieldOffset(0)] public uint type;
            [FieldOffset(0)] public SDL_KeyboardEvent key;
            [FieldOffset(0)] public SDL_MouseMotionEvent motion;
            [FieldOffset(0)] public SDL_MouseButtonEvent button;
            [FieldOffset(0)] public SDL_ControllerAxisEvent caxis;
            [FieldOffset(0)] public SDL_ControllerButtonEvent cbutton;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_Keysym { public int scancode; public int sym; public ushort mod; public uint unused; }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_KeyboardEvent
        {
            public uint type; public uint timestamp; public uint windowID;
            public byte state; public byte repeat; public byte pad1; public byte pad2;
            public SDL_Keysym keysym;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_MouseMotionEvent
        {
            public uint type; public uint timestamp; public uint windowID;
            public uint which; public uint state; public int x, y, xrel, yrel;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_MouseButtonEvent
        {
            public uint type; public uint timestamp; public uint windowID;
            public uint which; public byte button; public byte state;
            public byte clicks; public byte padding; public int x, y;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_ControllerAxisEvent
        {
            public uint type; public uint timestamp; public int which;
            public byte axis; public byte pad1, pad2, pad3; public short value; public ushort pad4;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_ControllerButtonEvent
        {
            public uint type; public uint timestamp; public int which;
            public byte button; public byte state; public byte pad1, pad2;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct SDL_AudioSpec
        {
            public int freq; public ushort format; public byte channels; public byte silence;
            public ushort samples; public ushort padding; public uint size;
            public IntPtr callback; public IntPtr userdata;
        }

        // Core SDL functions
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_Init(uint flags);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_Quit();
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr SDL_GetError();
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint SDL_GetTicks();
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_Delay(uint ms);

        // Window
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr SDL_CreateWindow(string title, int x, int y, int w, int h, uint flags);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_DestroyWindow(IntPtr window);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_SetWindowTitle(IntPtr window, string title);

        // Renderer
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr SDL_CreateRenderer(IntPtr window, int index, uint flags);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_DestroyRenderer(IntPtr renderer);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_RenderClear(IntPtr renderer);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_RenderPresent(IntPtr renderer);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_RenderCopy(IntPtr renderer, IntPtr texture, IntPtr srcrect, ref SDL_Rect dstrect);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_RenderCopy(IntPtr renderer, IntPtr texture, ref SDL_Rect srcrect, ref SDL_Rect dstrect);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_SetRenderDrawColor(IntPtr renderer, byte r, byte g, byte b, byte a);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_RenderFillRect(IntPtr renderer, ref SDL_Rect rect);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_RenderDrawLine(IntPtr renderer, int x1, int y1, int x2, int y2);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_SetRenderDrawBlendMode(IntPtr renderer, int blendMode);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_RenderSetScale(IntPtr renderer, float scaleX, float scaleY);

        // Texture
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr SDL_CreateTexture(IntPtr renderer, uint format, int access, int w, int h);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_DestroyTexture(IntPtr texture);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_UpdateTexture(IntPtr texture, IntPtr rect, IntPtr pixels, int pitch);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_LockTexture(IntPtr texture, IntPtr rect, out IntPtr pixels, out int pitch);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_UnlockTexture(IntPtr texture);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_SetTextureBlendMode(IntPtr texture, int blendMode);

        // Events
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_PollEvent(out SDL_Event sdlEvent);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_PumpEvents();

        // Audio
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint SDL_OpenAudioDevice(string device, int iscapture, ref SDL_AudioSpec desired, out SDL_AudioSpec obtained, int allowed_changes);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_CloseAudioDevice(uint dev);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_PauseAudioDevice(uint dev, int pause_on);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_QueueAudio(uint dev, IntPtr data, uint len);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint SDL_GetQueuedAudioSize(uint dev);

        // Joystick/Controller
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SDL_NumJoysticks();
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr SDL_GameControllerOpen(int joystick_index);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDL_GameControllerClose(IntPtr gamecontroller);
        [DllImport(SDL_LIB, CallingConvention = CallingConvention.Cdecl)]
        public static extern short SDL_GameControllerGetAxis(IntPtr gamecontroller, int axis);

        public static string GetError() => Marshal.PtrToStringAnsi(SDL_GetError()) ?? "";
    }

    // ============================================================
    // 3DS HARDWARE CONSTANTS
    // ============================================================
    public static class DS3Constants
    {
        // Screen dimensions
        public const int TOP_WIDTH = 400;
        public const int TOP_HEIGHT = 240;
        public const int BOT_WIDTH = 320;
        public const int BOT_HEIGHT = 240;

        // CPU speeds
        public const int ARM11_CLOCK_HZ = 268_111_856;   // ~268 MHz
        public const int ARM9_CLOCK_HZ  = 134_055_928;   // ~134 MHz
        public const int TARGET_FPS     = 60;
        public const int SAMPLE_RATE    = 32728; // native audio sample rate
        public const int CYCLES_PER_FRAME = ARM11_CLOCK_HZ / TARGET_FPS;

        // RAM layout
        public const uint FCRAM_BASE    = 0x20000000; // 128MB main RAM
        public const uint FCRAM_SIZE    = 0x08000000;
        public const uint VRAM_BASE     = 0x18000000; // 6MB VRAM
        public const uint VRAM_SIZE     = 0x00600000;
        public const uint DSP_RAM_BASE  = 0x1FF00000;
        public const uint DSP_RAM_SIZE  = 0x00080000;
        public const uint AXI_WRAM_BASE = 0x1FF80000;
        public const uint AXI_WRAM_SIZE = 0x00080000;
        public const uint IO_BASE       = 0x10000000;
        public const uint IO_SIZE       = 0x08000000;
        public const uint BOOT11_BASE   = 0x00000000;
        public const uint DTCM_BASE     = 0xFFF00000; // ARM9 DTCM

        // Framebuffer addresses (in VRAM)
        public const uint FB_TOP_LEFT_A  = 0x18000000;
        public const uint FB_TOP_LEFT_B  = 0x18000000 + TOP_WIDTH * TOP_HEIGHT * 3;
        public const uint FB_BOT_A       = 0x18000000 + 0x00100000;
        public const uint FB_BOT_B       = 0x18000000 + 0x00100000 + BOT_WIDTH * BOT_HEIGHT * 3;

        // LCD MMIO
        public const uint LCD_TOP_BASE    = 0x10400000;
        public const uint LCD_BOT_BASE    = 0x10400100;
        public const uint LCD_FB_A_ADDR   = 0x10400068; // Top screen FB A address
        public const uint LCD_FB_B_ADDR   = 0x1040006C; // Top screen FB B address
        public const uint LCD_STRIDE      = 0x10400090;
        public const uint LCD_SELECT      = 0x10400078; // which FB active
        public const uint LCD_BOT_FB_A    = 0x10400568;
        public const uint LCD_BOT_FB_B    = 0x1040056C;

        // GPU MMIO (PICA200)
        public const uint GPU_BASE        = 0x10400000;
        public const uint GPU_CMDLIST_ADR = 0x10400018;
        public const uint GPU_CMDLIST_SZ  = 0x10400020;
        public const uint GPU_CMDLIST_RUN = 0x10400028;

        // DSP MMIO
        public const uint DSP_BASE        = 0x10140000;
        public const uint DSP_PDATA_ADDR  = 0x10140000;

        // Service port MMIO
        public const uint SRV_BASE        = 0x10180000;

        // Timer MMIO
        public const uint TIMER_BASE      = 0x10003000;

        // HID (input) MMIO
        public const uint HID_BASE        = 0x10146000;
        public const uint HID_PAD         = 0x10146000;
        public const uint HID_PAD_CX      = 0x10146002; // Circle pad X
        public const uint HID_PAD_CY      = 0x10146004; // Circle pad Y

        // Input bit masks (active low in real HW, we handle both)
        public const uint BTN_A      = (1 << 0);
        public const uint BTN_B      = (1 << 1);
        public const uint BTN_SELECT = (1 << 2);
        public const uint BTN_START  = (1 << 3);
        public const uint BTN_RIGHT  = (1 << 4);
        public const uint BTN_LEFT   = (1 << 5);
        public const uint BTN_UP     = (1 << 6);
        public const uint BTN_DOWN   = (1 << 7);
        public const uint BTN_R      = (1 << 8);
        public const uint BTN_L      = (1 << 9);
        public const uint BTN_X      = (1 << 10);
        public const uint BTN_Y      = (1 << 11);

        // Touchscreen
        public const uint TOUCH_BASE = 0x10146100;

        // Interrupt IDs
        public const int IRQ_TIMER0   = 0x28;
        public const int IRQ_VBLANK   = 0x2A;
        public const int IRQ_HID      = 0x4B;
        public const int IRQ_DSP      = 0x4F;
        public const int IRQ_DMA      = 0x1C;

        // OS/Kernel (HLE syscall numbers for Horizon OS)
        public const uint SVC_SLEEP_THREAD     = 0x0A;
        public const uint SVC_QUERY_MEMORY     = 0x02;
        public const uint SVC_CONTROL_MEMORY   = 0x01;
        public const uint SVC_CREATE_EVENT     = 0x17;
        public const uint SVC_SIGNAL_EVENT     = 0x18;
        public const uint SVC_CLOSE_HANDLE     = 0x23;
        public const uint SVC_GET_THREAD_ID    = 0x26;
        public const uint SVC_GET_PROCESS_ID   = 0x35;
        public const uint SVC_OUTPUT_DBG_STR   = 0x3D;
        public const uint SVC_CONNECT_TO_PORT  = 0x2D;
        public const uint SVC_SEND_SYNC_REQ    = 0x32;
        public const uint SVC_BREAK            = 0x3C;
        public const uint SVC_CREATE_THREAD    = 0x08;
        public const uint SVC_EXIT_THREAD      = 0x09;
        public const uint SVC_CREATE_SEMAPHORE = 0x13;
        public const uint SVC_RELEASE_SEMA     = 0x15;
        public const uint SVC_MAP_MEMORY_BLOCK = 0x1F;
        public const uint SVC_CREATE_MUTEX     = 0x13;
        public const uint SVC_RELEASE_MUTEX    = 0x14;
        public const uint SVC_GET_SYSTEM_TICK  = 0x28;

        // ROM header offsets
        public const int NCCH_MAGIC_OFFSET   = 0x100;
        public const int NCCH_SIZE_OFFSET    = 0x104;
        public const int NCCH_FLAGS_OFFSET   = 0x188;
        public const int EXHEADER_OFFSET     = 0x200;
        public const int EXHEADER_SIZE       = 0x400;
        public const int CODE_OFFSET_IN_EXEFS = 0;
    }

    // ============================================================
    // ARM11 CPU (Thumb/ARM interpreter)
    // ============================================================
    public enum ARMMode { ARM, Thumb }

    public class ARM11CPU
    {
        // 16 general-purpose registers
        // R0-R12: general, R13=SP, R14=LR, R15=PC
        public uint[] R = new uint[16];
        public uint CPSR; // Current Program Status Register
        public uint SPSR; // Saved PSR
        public bool Running = true;

        // Banked registers for different modes
        Dictionary<uint, uint[]> bankedR = new Dictionary<uint, uint[]>
        {
            { 0x10, new uint[7] }, // User
            { 0x13, new uint[7] }, // Supervisor (SVC)
            { 0x17, new uint[7] }, // Abort
            { 0x1B, new uint[7] }, // Undefined
            { 0x12, new uint[7] }, // IRQ
            { 0x11, new uint[7] }, // FIQ
        };

        public ARMMode Mode => (CPSR & (1 << 5)) != 0 ? ARMMode.Thumb : ARMMode.ARM;
        public bool FlagN => (CPSR & (1u << 31)) != 0;
        public bool FlagZ => (CPSR & (1u << 30)) != 0;
        public bool FlagC => (CPSR & (1u << 29)) != 0;
        public bool FlagV => (CPSR & (1u << 28)) != 0;
        public bool IRQDisabled => (CPSR & (1 << 7)) != 0;

        public uint PC { get => R[15]; set => R[15] = value; }
        public uint SP { get => R[13]; set => R[13] = value; }
        public uint LR { get => R[14]; set => R[14] = value; }

        public void SetFlags(bool n, bool z, bool c, bool v)
        {
            CPSR = (CPSR & 0x0FFFFFFF)
                | (n ? (1u << 31) : 0)
                | (z ? (1u << 30) : 0)
                | (c ? (1u << 29) : 0)
                | (v ? (1u << 28) : 0);
        }

        public void SetFlagsNZ(uint result) => SetFlags((result >> 31) != 0, result == 0, FlagC, FlagV);

        public void SetMode(uint mode)
        {
            CPSR = (CPSR & 0xFFFFFFE0) | (mode & 0x1F);
        }

        public void EnterSVC()
        {
            SPSR = CPSR;
            SetMode(0x13);
            CPSR |= (1 << 7); // disable IRQ
            LR = PC - (Mode == ARMMode.Thumb ? 2u : 4u);
        }

        public void EnterIRQ()
        {
            SPSR = CPSR;
            SetMode(0x12);
            CPSR |= (1 << 7);
            CPSR &= ~(1u << 5); // switch to ARM mode
            LR = PC + 4;
        }

        public bool ConditionPasses(uint cond)
        {
            return cond switch
            {
                0x0 => FlagZ,                         // EQ
                0x1 => !FlagZ,                        // NE
                0x2 => FlagC,                         // CS/HS
                0x3 => !FlagC,                        // CC/LO
                0x4 => FlagN,                         // MI
                0x5 => !FlagN,                        // PL
                0x6 => FlagV,                         // VS
                0x7 => !FlagV,                        // VC
                0x8 => FlagC && !FlagZ,               // HI
                0x9 => !FlagC || FlagZ,               // LS
                0xA => FlagN == FlagV,                // GE
                0xB => FlagN != FlagV,                // LT
                0xC => !FlagZ && (FlagN == FlagV),    // GT
                0xD => FlagZ || (FlagN != FlagV),     // LE
                0xE => true,                          // AL
                0xF => false,                         // NV (reserved)
                _ => false
            };
        }
    }

    // ============================================================
    // 3DS MEMORY BUS
    // ============================================================
    public class DS3MemoryBus
    {
        // Physical memory regions
        public byte[] FCRAM  = new byte[DS3Constants.FCRAM_SIZE];   // 128MB main RAM
        public byte[] VRAM   = new byte[DS3Constants.VRAM_SIZE];    // 6MB VRAM
        public byte[] DSPRam = new byte[DS3Constants.DSP_RAM_SIZE]; // DSP RAM
        public byte[] AxiWram = new byte[DS3Constants.AXI_WRAM_SIZE];
        public byte[] DTCM   = new byte[0x4000];  // ARM9 DTCM 16KB
        public byte[] ITCM   = new byte[0x8000];  // ARM9 ITCM 32KB

        public DS3HID HID;
        public DS3LCD LCD;
        public DS3DSP DSP;
        public DS3Timer TimerController;
        public DS3GPU PICA200;
        public DS3HLE HLE;

        // Page table for fast translation
        const int PAGE_BITS = 12;
        const int PAGE_SIZE = 1 << PAGE_BITS;
        const int PAGE_MASK = PAGE_SIZE - 1;

        public uint Read32(uint addr)
        {
            var region = MapAddress(addr, out uint offset);
            if (region == null) return 0xFFFFFFFF;
            if (offset + 3 >= region.Length) return 0;
            return (uint)(region[offset] | (region[offset+1] << 8) | (region[offset+2] << 16) | (region[offset+3] << 24));
        }

        public ushort Read16(uint addr)
        {
            var region = MapAddress(addr, out uint offset);
            if (region == null) return 0xFFFF;
            if (offset + 1 >= region.Length) return 0;
            return (ushort)(region[offset] | (region[offset+1] << 8));
        }

        public byte Read8(uint addr)
        {
            var region = MapAddress(addr, out uint offset);
            if (region == null) return 0xFF;
            return region[offset];
        }

        public void Write32(uint addr, uint value)
        {
            if (HandleMMIOWrite32(addr, value)) return;
            var region = MapAddress(addr, out uint offset);
            if (region == null || offset + 3 >= region.Length) return;
            region[offset]   = (byte)(value);
            region[offset+1] = (byte)(value >> 8);
            region[offset+2] = (byte)(value >> 16);
            region[offset+3] = (byte)(value >> 24);
        }

        public void Write16(uint addr, ushort value)
        {
            if (HandleMMIOWrite16(addr, value)) return;
            var region = MapAddress(addr, out uint offset);
            if (region == null || offset + 1 >= region.Length) return;
            region[offset]   = (byte)(value);
            region[offset+1] = (byte)(value >> 8);
        }

        public void Write8(uint addr, byte value)
        {
            if (HandleMMIOWrite8(addr, value)) return;
            var region = MapAddress(addr, out uint offset);
            if (region == null) return;
            region[offset] = value;
        }

        byte[] MapAddress(uint addr, out uint offset)
        {
            // FCRAM: 0x20000000 - 0x28000000
            if (addr >= DS3Constants.FCRAM_BASE && addr < DS3Constants.FCRAM_BASE + DS3Constants.FCRAM_SIZE)
            { offset = addr - DS3Constants.FCRAM_BASE; return FCRAM; }

            // VRAM: 0x18000000 - 0x18600000
            if (addr >= DS3Constants.VRAM_BASE && addr < DS3Constants.VRAM_BASE + DS3Constants.VRAM_SIZE)
            { offset = addr - DS3Constants.VRAM_BASE; return VRAM; }

            // DSP RAM
            if (addr >= DS3Constants.DSP_RAM_BASE && addr < DS3Constants.DSP_RAM_BASE + DS3Constants.DSP_RAM_SIZE)
            { offset = addr - DS3Constants.DSP_RAM_BASE; return DSPRam; }

            // AXI WRAM
            if (addr >= DS3Constants.AXI_WRAM_BASE && addr < DS3Constants.AXI_WRAM_BASE + DS3Constants.AXI_WRAM_SIZE)
            { offset = addr - DS3Constants.AXI_WRAM_BASE; return AxiWram; }

            // DTCM (ARM9)
            if (addr >= DS3Constants.DTCM_BASE && addr < DS3Constants.DTCM_BASE + 0x4000)
            { offset = addr - DS3Constants.DTCM_BASE; return DTCM; }

            // IO region — read handled by individual devices
            if (addr >= DS3Constants.IO_BASE && addr < DS3Constants.IO_BASE + DS3Constants.IO_SIZE)
            {
                offset = addr - DS3Constants.IO_BASE;
                return HandleMMIORead(addr);
            }

            // Linear heap mirror (0x14000000-0x1C000000 maps to FCRAM)
            if (addr >= 0x14000000 && addr < 0x1C000000)
            { offset = addr - 0x14000000; return FCRAM; }

            offset = 0;
            return null;
        }

        // MMIO read returns a temp buffer
        byte[] ioReadBuf = new byte[4];
        byte[] HandleMMIORead(uint addr)
        {
            uint val = 0;
            // HID
            if (addr == DS3Constants.HID_PAD)
                val = HID?.ReadPad() ?? 0xFFF;
            else if (addr == DS3Constants.HID_PAD_CX)
                val = HID?.CircleX ?? 0;
            else if (addr == DS3Constants.HID_PAD_CY)
                val = HID?.CircleY ?? 0;
            // LCD
            else if (addr == DS3Constants.LCD_FB_A_ADDR)
                val = DS3Constants.FB_TOP_LEFT_A;
            else if (addr == DS3Constants.LCD_BOT_FB_A)
                val = DS3Constants.FB_BOT_A;
            else if (addr == DS3Constants.LCD_SELECT)
                val = LCD?.DisplaySelect ?? 0;
            // Timer
            else if (addr >= DS3Constants.TIMER_BASE && addr < DS3Constants.TIMER_BASE + 0x40)
                val = TimerController?.Read(addr) ?? 0;
            // DSP
            else if (addr >= DS3Constants.DSP_BASE && addr < DS3Constants.DSP_BASE + 0x1000)
                val = DSP?.Read(addr) ?? 0;
            else
                val = 0xFFFFFFFF;

            ioReadBuf[0] = (byte)(val);
            ioReadBuf[1] = (byte)(val >> 8);
            ioReadBuf[2] = (byte)(val >> 16);
            ioReadBuf[3] = (byte)(val >> 24);
            return ioReadBuf;
        }

        bool HandleMMIOWrite32(uint addr, uint value)
        {
            if (addr < DS3Constants.IO_BASE || addr >= DS3Constants.IO_BASE + DS3Constants.IO_SIZE)
                return false;
            if (addr == DS3Constants.LCD_FB_A_ADDR) { LCD?.SetTopFB(value, true); return true; }
            if (addr == DS3Constants.LCD_FB_B_ADDR) { LCD?.SetTopFB(value, false); return true; }
            if (addr == DS3Constants.LCD_BOT_FB_A)  { LCD?.SetBotFB(value, true);  return true; }
            if (addr == DS3Constants.LCD_BOT_FB_B)  { LCD?.SetBotFB(value, false); return true; }
            if (addr == DS3Constants.LCD_SELECT)     { if(LCD!=null) LCD.DisplaySelect = value; return true; }
            if (addr == DS3Constants.LCD_STRIDE)     { if(LCD!=null) LCD.Stride = value; return true; }
            if (addr >= DS3Constants.TIMER_BASE && addr < DS3Constants.TIMER_BASE + 0x40)
            { TimerController?.Write(addr, value); return true; }
            if (addr == DS3Constants.GPU_CMDLIST_RUN && value == 1)
            { PICA200?.ExecuteCommandList(this); return true; }
            if (addr >= DS3Constants.DSP_BASE && addr < DS3Constants.DSP_BASE + 0x1000)
            { DSP?.Write(addr, value); return true; }
            return false;
        }

        bool HandleMMIOWrite16(uint addr, ushort value) => HandleMMIOWrite32(addr, value);
        bool HandleMMIOWrite8(uint addr, byte value) => HandleMMIOWrite32(addr, value);

        public void LoadAt(byte[] data, uint addr)
        {
            var region = MapAddress(addr, out uint offset);
            if (region == null) return;
            ulong copyLen = Math.Min((ulong)data.Length, (ulong)(region.Length - offset));
            Buffer.BlockCopy(data, 0, region, (int)offset, (int)copyLen);
        }

        // Read a null-terminated string from memory
        public string ReadString(uint addr, int maxLen = 256)
        {
            var sb = new StringBuilder();
            for (int i = 0; i < maxLen; i++)
            {
                byte b = Read8(addr + (uint)i);
                if (b == 0) break;
                sb.Append((char)b);
            }
            return sb.ToString();
        }
    }

    // ============================================================
    // HID — INPUT CONTROLLER
    // ============================================================
    public class DS3HID
    {
        uint pressedButtons = 0xFFF; // active low — all released = all 1s
        public uint CircleX = 0;
        public uint CircleY = 0;
        public bool TouchActive = false;
        public int TouchX = 0, TouchY = 0;
        public InterruptController PIC;

        public uint ReadPad() => ~pressedButtons & 0xFFF; // return pressed as 1

        public void Press(uint btn)
        {
            pressedButtons |= btn;
            PIC?.Raise(DS3Constants.IRQ_HID);
        }

        public void Release(uint btn)
        {
            pressedButtons &= ~btn;
        }

        public void SetCirclePad(int x, int y)
        {
            CircleX = (uint)(x & 0xFFFF);
            CircleY = (uint)(y & 0xFFFF);
        }

        public void SetTouch(bool active, int x = 0, int y = 0)
        {
            TouchActive = active;
            TouchX = Math.Clamp(x, 0, DS3Constants.BOT_WIDTH - 1);
            TouchY = Math.Clamp(y, 0, DS3Constants.BOT_HEIGHT - 1);
        }
    }

    // ============================================================
    // LCD CONTROLLER
    // ============================================================
    public class DS3LCD
    {
        public uint TopFB_A = DS3Constants.FB_TOP_LEFT_A;
        public uint TopFB_B = DS3Constants.FB_TOP_LEFT_B;
        public uint BotFB_A = DS3Constants.FB_BOT_A;
        public uint BotFB_B = DS3Constants.FB_BOT_B;
        public uint DisplaySelect = 0; // 0 = A active, 1 = B active
        public uint Stride = DS3Constants.TOP_WIDTH * 3;
        public bool TopActive = true;
        public bool BotActive = true;
        public uint VBlankCount = 0;
        public InterruptController PIC;

        public void SetTopFB(uint addr, bool isA) { if (isA) TopFB_A = addr; else TopFB_B = addr; }
        public void SetBotFB(uint addr, bool isA) { if (isA) BotFB_A = addr; else BotFB_B = addr; }

        public uint ActiveTopFB => (DisplaySelect & 1) == 0 ? TopFB_A : TopFB_B;
        public uint ActiveBotFB => (DisplaySelect & 1) == 0 ? BotFB_A : BotFB_B;

        public void VBlank()
        {
            VBlankCount++;
            PIC?.Raise(DS3Constants.IRQ_VBLANK);
        }
    }

    // ============================================================
    // TIMER CONTROLLER (4 timers)
    // ============================================================
    public class DS3Timer
    {
        struct TimerState { public uint Counter; public uint Reload; public bool Running; public bool IRQEnabled; public uint Prescaler; }
        TimerState[] timers = new TimerState[4];
        public InterruptController PIC;

        public uint Read(uint addr)
        {
            int idx = (int)((addr - DS3Constants.TIMER_BASE) / 4);
            if (idx < 0 || idx >= 4) return 0;
            return timers[idx].Counter;
        }

        public void Write(uint addr, uint value)
        {
            int idx = (int)((addr - DS3Constants.TIMER_BASE) / 4);
            if (idx < 0 || idx >= 4) return;
            if ((value & 0x80000000) != 0) timers[idx].Running = true;
            if ((value & 0x40000000) != 0) timers[idx].IRQEnabled = true;
            timers[idx].Reload = value & 0xFFFF;
            timers[idx].Prescaler = (uint)(1 << (int)((value >> 2) & 3));
        }

        public void Tick(int cycles)
        {
            for (int i = 0; i < 4; i++)
            {
                if (!timers[i].Running) continue;
                timers[i].Counter += (uint)(cycles / (int)Math.Max(1, timers[i].Prescaler));
                if (timers[i].Counter >= 0xFFFF)
                {
                    timers[i].Counter = timers[i].Reload;
                    if (timers[i].IRQEnabled) PIC?.Raise(DS3Constants.IRQ_TIMER0 + i);
                }
            }
        }
    }

    // ============================================================
    // DSP — AUDIO PROCESSING (software mixer)
    // ============================================================
    public class DS3DSP
    {
        const int SAMPLE_RATE = 32728; // 3DS native
        const int CHANNELS = 24;
        const int BUFFER_SAMPLES = 1024;

        struct AudioChannel
        {
            public bool Active;
            public bool Looping;
            public uint SampleAddr;
            public uint LoopStart, SampleEnd;
            public uint SamplePos;
            public float Volume;
            public float Pan;     // -1 = left, 1 = right
            public float Pitch;   // 1.0 = normal
            public int  Format;   // 0=PCM8, 1=PCM16, 2=ADPCM
            // ADPCM state
            public int ADPCMPredictor;
            public int ADPCMStepIndex;
        }

        AudioChannel[] channels = new AudioChannel[CHANNELS];
        DS3MemoryBus Bus;
        uint audioDevId;
        public bool Ready;

        // SDL Audio
        uint sdlAudioDev;

        static readonly int[] ADPCM_INDEX_TABLE = { -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8 };
        static readonly int[] ADPCM_STEP_TABLE = {
            7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,
            73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,
            449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,
            2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,
            10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
        };

        public DS3DSP(DS3MemoryBus bus) { Bus = bus; }

        public void Init()
        {
            var spec = new SDL2.SDL_AudioSpec
            {
                freq = SAMPLE_RATE,
                format = SDL2.AUDIO_S16SYS,
                channels = 2,
                samples = BUFFER_SAMPLES,
                callback = IntPtr.Zero,
                userdata = IntPtr.Zero
            };
            if (OperatingSystem.IsBrowser())
            {
                BrowserPlatform.InitAudio(SAMPLE_RATE);
                Ready = true;
            }
            else
            {
                sdlAudioDev = SDL2.SDL_OpenAudioDevice(null, 0, ref spec, out _, 0);
                if (sdlAudioDev > 0)
                {
                    SDL2.SDL_PauseAudioDevice(sdlAudioDev, 0);
                    Ready = true;
                }
            }
        }

        public uint Read(uint addr) => 0;
        public void Write(uint addr, uint value)
        {
            // DSP channel register writes
            int ch = (int)((addr - DS3Constants.DSP_BASE) / 0x100) % CHANNELS;
            int reg = (int)((addr - DS3Constants.DSP_BASE) & 0xFF);
            switch (reg)
            {
                case 0x00: channels[ch].Active = (value & 1) != 0; break;
                case 0x04: channels[ch].SampleAddr = value; break;
                case 0x08: channels[ch].SampleEnd = value; break;
                case 0x0C: channels[ch].LoopStart = value; break;
                case 0x10: channels[ch].Volume = (value & 0xFFFF) / 32767.0f; break;
                case 0x14: channels[ch].Pan = ((value & 0xFF) - 128) / 128.0f; break;
                case 0x18: channels[ch].Format = (int)(value & 3); break;
                case 0x1C: channels[ch].Looping = (value & 1) != 0; break;
            }
        }

        public void MixAndQueue(int numSamples)
        {
            if (!Ready) return;
            // Native: don't overflow the queue
            if (!OperatingSystem.IsBrowser())
            {
                if (sdlAudioDev == 0) return;
                if (SDL2.SDL_GetQueuedAudioSize(sdlAudioDev) > (uint)(SAMPLE_RATE / 10 * 4)) return;
            }

            short[] mixed = new short[numSamples * 2]; // stereo

            for (int ch = 0; ch < CHANNELS; ch++)
            {
                ref var c = ref channels[ch];
                if (!c.Active || c.SampleAddr == 0) continue;

                for (int s = 0; s < numSamples; s++)
                {
                    float sample = 0;
                    uint pos = c.SamplePos;

                    if (c.Format == 0) // PCM8
                    {
                        sample = (Bus.Read8(c.SampleAddr + pos) - 128) / 128.0f;
                    }
                    else if (c.Format == 1) // PCM16
                    {
                        uint bytePos = pos * 2;
                        sample = (short)Bus.Read16(c.SampleAddr + bytePos) / 32768.0f;
                    }
                    else // ADPCM (simplified)
                    {
                        byte nibbleByte = Bus.Read8(c.SampleAddr + pos / 2);
                        int nibble = (pos & 1) == 0 ? (nibbleByte & 0xF) : (nibbleByte >> 4);
                        int step = ADPCM_STEP_TABLE[c.ADPCMStepIndex];
                        int delta = step >> 3;
                        if ((nibble & 4) != 0) delta += step;
                        if ((nibble & 2) != 0) delta += step >> 1;
                        if ((nibble & 1) != 0) delta += step >> 2;
                        if ((nibble & 8) != 0) delta = -delta;
                        c.ADPCMPredictor = Math.Clamp(c.ADPCMPredictor + delta, -32768, 32767);
                        c.ADPCMStepIndex = Math.Clamp(c.ADPCMStepIndex + ADPCM_INDEX_TABLE[nibble & 7], 0, 88);
                        sample = c.ADPCMPredictor / 32768.0f;
                    }

                    sample *= c.Volume;
                    float sampleL = sample * (1.0f - Math.Max(0, c.Pan));
                    float sampleR = sample * (1.0f + Math.Min(0, c.Pan));
                    mixed[s * 2]     = (short)Math.Clamp(mixed[s * 2]     + (short)(sampleL * 32767), -32768, 32767);
                    mixed[s * 2 + 1] = (short)Math.Clamp(mixed[s * 2 + 1] + (short)(sampleR * 32767), -32768, 32767);

                    c.SamplePos++;
                    if (c.SamplePos >= c.SampleEnd)
                    {
                        if (c.Looping) c.SamplePos = c.LoopStart;
                        else { c.Active = false; break; }
                    }
                }
            }

            if (OperatingSystem.IsBrowser())
            {
                // Browser bridge expects short[] PCM interleaved
                BrowserPlatform.QueueAudio(mixed);
            }
            else
            {
                // Queue to SDL
                var handle = GCHandle.Alloc(mixed, GCHandleType.Pinned);
                try
                {
                    SDL2.SDL_QueueAudio(sdlAudioDev, handle.AddrOfPinnedObject(), (uint)(mixed.Length * 2));
                }
                finally { handle.Free(); }
            }
        }

        public void Shutdown()
        {
            if (OperatingSystem.IsBrowser()) return;
            if (sdlAudioDev > 0) { SDL2.SDL_CloseAudioDevice(sdlAudioDev); sdlAudioDev = 0; }
        }
    }

    // ============================================================
    // PICA200 GPU (3DS Graphics)
    // ============================================================
    public class DS3GPU
    {
        // PICA200 command list processor
        // We HLE the most common draw commands
        uint cmdListAddr;
        uint cmdListSize;

        public struct Triangle { public uint V0, V1, V2; } // vertex buffer indices
        public struct Vertex { public float X, Y, Z, W, U, V, R, G, B, A; }

        List<Vertex> vertexBuffer = new List<Vertex>();
        uint[] regs = new uint[0x300];

        DS3MemoryBus bus;

        public DS3GPU(DS3MemoryBus b) { bus = b; }

        public void WriteReg(uint reg, uint value)
        {
            if (reg < regs.Length) regs[reg] = value;
        }

        public void ExecuteCommandList(DS3MemoryBus mem)
        {
            uint addr = regs[0x18 / 4];
            uint size  = regs[0x20 / 4];
            if (addr == 0 || size == 0) return;

            uint end = addr + size;
            while (addr < end)
            {
                uint header  = mem.Read32(addr); addr += 4;
                uint param   = mem.Read32(addr); addr += 4;
                uint regId   = header & 0xFFFF;
                uint extra   = (header >> 16) & 0xFF;
                WriteReg(regId, param);
                for (uint i = 0; i < extra; i++) { WriteReg(regId + 1 + i, mem.Read32(addr)); addr += 4; }
            }
            // After processing command list, rasterize if draw command was issued
            Rasterize(mem);
        }

        void Rasterize(DS3MemoryBus mem)
        {
            // Simplified software rasterizer — draws to VRAM
            uint fbAddr = regs[0x111]; // color buffer address register (approx)
            if (fbAddr == 0) fbAddr = DS3Constants.FB_TOP_LEFT_A;

            int fbW = DS3Constants.TOP_WIDTH;
            int fbH = DS3Constants.TOP_HEIGHT;

            // Get vertex buffer from GPU registers
            uint vtxBase = regs[0x200]; // approximate VBO address register
            uint vtxCount = regs[0x228]; // approximate draw count
            if (vtxBase < DS3Constants.FCRAM_BASE || vtxCount == 0 || vtxCount > 10000) return;

            // Very simplified: draw triangles from vertex buffer
            for (uint i = 0; i + 2 < vtxCount; i += 3)
            {
                // Each vertex: 3 floats position + 2 floats UV + 4 floats color = 36 bytes
                uint v0 = vtxBase + i * 36;
                Vertex va = ReadVertex(mem, v0);
                Vertex vb = ReadVertex(mem, v0 + 36);
                Vertex vc = ReadVertex(mem, v0 + 72);
                DrawTriangle(mem, fbAddr, fbW, fbH, va, vb, vc);
            }
        }

        Vertex ReadVertex(DS3MemoryBus mem, uint addr)
        {
            float ReadF(uint a) => BitConverter.ToSingle(BitConverter.GetBytes(mem.Read32(a)), 0);
            return new Vertex
            {
                X = ReadF(addr),      Y = ReadF(addr+4),  Z = ReadF(addr+8), W = ReadF(addr+12),
                U = ReadF(addr+16),   V = ReadF(addr+20),
                R = ReadF(addr+24),   G = ReadF(addr+28), B = ReadF(addr+32), A = ReadF(addr+36)
            };
        }

        void DrawTriangle(DS3MemoryBus mem, uint fbAddr, int w, int h, Vertex a, Vertex b, Vertex c)
        {
            // Clip to NDC then to screen
            static int ToScreen(float ndc, int dim) => (int)((ndc * 0.5f + 0.5f) * dim);
            int ax = ToScreen(a.X, w), ay = ToScreen(a.Y, h);
            int bx = ToScreen(b.X, w), by = ToScreen(b.Y, h);
            int cx = ToScreen(c.X, w), cy = ToScreen(c.Y, h);

            // Bounding box
            int minX = Math.Max(0, Math.Min(ax, Math.Min(bx, cx)));
            int maxX = Math.Min(w-1, Math.Max(ax, Math.Max(bx, cx)));
            int minY = Math.Max(0, Math.Min(ay, Math.Min(by, cy)));
            int maxY = Math.Min(h-1, Math.Max(ay, Math.Max(by, cy)));

            // Edge function rasterizer
            for (int py = minY; py <= maxY; py++)
            for (int px = minX; px <= maxX; px++)
            {
                float e0 = EdgeFunction(ax,ay,bx,by,px,py);
                float e1 = EdgeFunction(bx,by,cx,cy,px,py);
                float e2 = EdgeFunction(cx,cy,ax,ay,px,py);
                if (e0 >= 0 && e1 >= 0 && e2 >= 0)
                {
                    float area = EdgeFunction(ax,ay,bx,by,cx,cy);
                    if (area <= 0) continue;
                    float w0 = e0/area, w1 = e1/area, w2 = e2/area;
                    byte r = (byte)Math.Clamp((a.R*w0 + b.R*w1 + c.R*w2)*255, 0, 255);
                    byte g = (byte)Math.Clamp((a.G*w0 + b.G*w1 + c.G*w2)*255, 0, 255);
                    byte bv = (byte)Math.Clamp((a.B*w0 + b.B*w1 + c.B*w2)*255, 0, 255);

                    // 3DS framebuffer is BGR, rotated 90 degrees
                    // Bottom-left origin, columns stored vertically
                    uint pixelAddr = fbAddr - DS3Constants.VRAM_BASE;
                    uint off = (uint)(px * h + (h - 1 - py)) * 3;
                    if (off + 2 < mem.VRAM.Length)
                    {
                        mem.VRAM[off]   = bv;
                        mem.VRAM[off+1] = g;
                        mem.VRAM[off+2] = r;
                    }
                }
            }
        }

        float EdgeFunction(int ax, int ay, int bx, int by, int px, int py)
            => (px - ax) * (float)(by - ay) - (py - ay) * (float)(bx - ax);
    }

    // ============================================================
    // HORIZON OS HLE (High-Level Emulation of syscalls)
    // ============================================================
    public class DS3HLE
    {
        DS3MemoryBus Bus;
        DS3Emulator Emu;
        Dictionary<uint, string> handles = new Dictionary<uint, string>();
        uint nextHandle = 0x100;
        uint heapBase = DS3Constants.FCRAM_BASE + 0x04000000; // 64MB into FCRAM
        uint heapPtr;
        List<(uint addr, uint size)> freeBlocks = new List<(uint, uint)>();

        // Named service sessions
        Dictionary<string, Action<ARM11CPU, DS3MemoryBus>> serviceHandlers;

        public DS3HLE(DS3MemoryBus bus, DS3Emulator emu)
        {
            Bus = bus;
            Emu = emu;
            heapPtr = heapBase;
            serviceHandlers = new Dictionary<string, Action<ARM11CPU, DS3MemoryBus>>
            {
                { "srv:",     HandleSRV     },
                { "APT:U",    HandleAPT     },
                { "apt:a",    HandleAPT     },
                { "gsp::Gpu", HandleGSP     },
                { "hid:USER", HandleHID     },
                { "hid:SPVR", HandleHID     },
                { "fs:USER",  HandleFS      },
                { "ndm:u",    HandleNDM     },
                { "cfg:u",    HandleCFG     },
                { "cfg:s",    HandleCFG     },
                { "ptm:u",    HandlePTM     },
                { "am:u",     HandleAM      },
                { "ac:u",     HandleAC      },
                { "nwm::UDS", HandleNWM     },
                { "dsp::DSP", HandleDSP     },
                { "csnd:SND", HandleCSND    },
                { "mic:u",    HandleMIC     },
                { "ir:USER",  HandleIR      },
                { "cdc:CHK",  HandleCDC     },
            };
        }

        uint AllocHandle(string name) { uint h = nextHandle++; handles[h] = name; return h; }
        string HandleName(uint h) => handles.TryGetValue(h, out var n) ? n : "unknown";

        public uint Alloc(uint size)
        {
            size = (size + 0xFFF) & ~0xFFFu; // page align
            // Check free blocks first
            for (int i = 0; i < freeBlocks.Count; i++)
            {
                if (freeBlocks[i].size >= size)
                {
                    uint addr = freeBlocks[i].addr;
                    freeBlocks.RemoveAt(i);
                    return addr;
                }
            }
            uint result = heapPtr;
            heapPtr += size;
            return result;
        }

        public void Free(uint addr, uint size) => freeBlocks.Add((addr, (size + 0xFFFu) & ~0xFFFu));

        // Entry point for SVC handler
        public void HandleSVC(ARM11CPU cpu, uint svcId)
        {
            switch (svcId)
            {
                case DS3Constants.SVC_CONTROL_MEMORY:
                    // ControlMemory(addr0, addr1, size, operation, permissions)
                    uint size = cpu.R[3];
                    uint addr = Alloc(size);
                    cpu.R[0] = 0; // RESULT_SUCCESS
                    cpu.R[1] = addr;
                    break;

                case DS3Constants.SVC_QUERY_MEMORY:
                    cpu.R[0] = 0;
                    cpu.R[1] = cpu.R[2]; // base addr
                    cpu.R[2] = 0x1000;   // size
                    cpu.R[3] = 3;        // permissions (RW)
                    cpu.R[4] = 1;        // type = ALLOCATED
                    break;

                case DS3Constants.SVC_SLEEP_THREAD:
                    // Sleep for nanoseconds in R0/R1, we just yield
                    Thread.Yield();
                    cpu.R[0] = 0;
                    break;

                case DS3Constants.SVC_CREATE_EVENT:
                    uint evtHandle = AllocHandle("Event");
                    cpu.R[0] = 0;
                    cpu.R[1] = evtHandle;
                    break;

                case DS3Constants.SVC_SIGNAL_EVENT:
                    cpu.R[0] = 0; // Just succeed
                    break;

                case DS3Constants.SVC_CLOSE_HANDLE:
                    handles.Remove(cpu.R[0]);
                    cpu.R[0] = 0;
                    break;

                case DS3Constants.SVC_GET_THREAD_ID:
                    cpu.R[0] = 0;
                    cpu.R[1] = 1; // Thread ID 1
                    break;

                case DS3Constants.SVC_GET_PROCESS_ID:
                    cpu.R[0] = 0;
                    cpu.R[1] = 0x20; // Process ID
                    break;

                case DS3Constants.SVC_CONNECT_TO_PORT:
                {
                    string portName = Bus.ReadString(cpu.R[1]);
                    uint sessionHandle = AllocHandle(portName);
                    cpu.R[0] = 0;
                    cpu.R[1] = sessionHandle;
                    Console.WriteLine($"  [HLE] ConnectToPort: '{portName}' -> handle 0x{sessionHandle:X}");
                    break;
                }

                case DS3Constants.SVC_SEND_SYNC_REQ:
                {
                    uint handle = cpu.R[0];
                    string svc = HandleName(handle);
                    if (serviceHandlers.TryGetValue(svc, out var handler))
                        handler(cpu, Bus);
                    else
                    {
                        // Unknown service — return success to avoid crash
                        uint cmdId = Bus.Read32(cpu.SP + 0x80);
                        // Write success result to IPC response buffer
                        Bus.Write32(cpu.SP + 0x80, 0); // success header
                        Bus.Write32(cpu.SP + 0x84, 0); // result code
                    }
                    cpu.R[0] = 0;
                    break;
                }

                case DS3Constants.SVC_CREATE_THREAD:
                {
                    // Create a thread — for now just register it
                    uint entryPoint = cpu.R[1];
                    uint arg = cpu.R[2];
                    uint stackAddr = cpu.R[3];
                    uint priority = cpu.R[4];
                    uint thHandle = AllocHandle("Thread");
                    Console.WriteLine($"  [HLE] CreateThread: entry=0x{entryPoint:X}, stack=0x{stackAddr:X}");
                    // Start thread on second CPU core if available
                    Emu.SpawnThread(entryPoint, arg, stackAddr);
                    cpu.R[0] = 0;
                    cpu.R[1] = thHandle;
                    break;
                }

                case DS3Constants.SVC_EXIT_THREAD:
                    cpu.Running = false;
                    cpu.R[0] = 0;
                    break;

                case DS3Constants.SVC_OUTPUT_DBG_STR:
                {
                    string msg = Bus.ReadString(cpu.R[0], (int)cpu.R[1]);
                    Console.WriteLine($"  [3DS DEBUG] {msg}");
                    cpu.R[0] = 0;
                    break;
                }

                case DS3Constants.SVC_GET_SYSTEM_TICK:
                    ulong ticks = (ulong)(Environment.TickCount64 * (DS3Constants.ARM11_CLOCK_HZ / 1000L));
                    cpu.R[0] = (uint)(ticks & 0xFFFFFFFF);
                    cpu.R[1] = (uint)(ticks >> 32);
                    break;

                case DS3Constants.SVC_MAP_MEMORY_BLOCK:
                    cpu.R[0] = 0; // Just succeed
                    break;

                case DS3Constants.SVC_BREAK:
                    Console.WriteLine($"[3DS] SVC BREAK called! reason={cpu.R[0]}");
                    cpu.R[0] = 0;
                    break;

                case DS3Constants.SVC_CREATE_MUTEX:
                    cpu.R[0] = 0;
                    cpu.R[1] = AllocHandle("Mutex");
                    break;

                case DS3Constants.SVC_RELEASE_MUTEX:
                    cpu.R[0] = 0;
                    break;

                default:
                    Console.WriteLine($"  [HLE] Unhandled SVC 0x{svcId:X2} at PC=0x{cpu.PC:X}");
                    cpu.R[0] = 0;
                    break;
            }
        }

        // IPC helpers
        uint ReadIPCCmd(uint sp) => Bus.Read32(sp + 0x80);
        uint ReadIPCParam(uint sp, int i) => Bus.Read32(sp + 0x84 + (uint)(i * 4));
        void WriteIPCReply(uint sp, params uint[] vals)
        {
            Bus.Write32(sp + 0x80, 0); // success
            for (int i = 0; i < vals.Length; i++) Bus.Write32(sp + 0x84 + (uint)(i * 4), vals[i]);
        }

        void HandleSRV(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 1: // Initialize
                    WriteIPCReply(cpu.SP, 0);
                    break;
                case 2: // GetServiceHandle
                {
                    // Service name is packed in IPC buffer
                    byte[] nameBuf = new byte[8];
                    for (int i = 0; i < 8; i++) nameBuf[i] = bus.Read8(cpu.SP + 0x84 + (uint)i);
                    string svcName = Encoding.ASCII.GetString(nameBuf).TrimEnd('\0');
                    uint h = AllocHandle(svcName);
                    Console.WriteLine($"  [HLE] SRV::GetServiceHandle: '{svcName}' -> 0x{h:X}");
                    WriteIPCReply(cpu.SP, 0, h);
                    break;
                }
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }

        void HandleAPT(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 1: // GetLockHandle
                    WriteIPCReply(cpu.SP, 0, AllocHandle("APTLock"), 0, AllocHandle("AppJump"));
                    break;
                case 2: // Initialize — signals APT is ready
                    WriteIPCReply(cpu.SP, 0, AllocHandle("APTSignal"), AllocHandle("APTResume"));
                    break;
                case 4: // NotifyToWait
                case 0x1E: // AppletUtility
                    WriteIPCReply(cpu.SP, 0, 0);
                    break;
                case 0x10: // IsRegistered
                    WriteIPCReply(cpu.SP, 0, 1); // yes it's registered
                    break;
                case 0x11: // InquireNotification
                    WriteIPCReply(cpu.SP, 0, 0); // no notification
                    break;
                case 0x46: // AppletUtility (extended)
                    WriteIPCReply(cpu.SP, 0, 0);
                    break;
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }

        void HandleGSP(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 2: // WriteHWRegs — write GPU registers
                    WriteIPCReply(cpu.SP, 0);
                    break;
                case 4: // FlushDataCache
                    WriteIPCReply(cpu.SP, 0);
                    break;
                case 8: // RegisterInterruptRelayQueue
                    WriteIPCReply(cpu.SP, 0, AllocHandle("GSPEvent"), 0);
                    break;
                case 0x12: // SetBufferSwap — sets which framebuffer is active
                    uint fbIdx = ReadIPCParam(cpu.SP, 0);
                    WriteIPCReply(cpu.SP, 0);
                    break;
                case 0x13: // SetCommandList — submit GPU command list
                    uint addr = ReadIPCParam(cpu.SP, 0);
                    uint size = ReadIPCParam(cpu.SP, 1);
                    Emu.GPU.ExecuteCommandList(bus);
                    WriteIPCReply(cpu.SP, 0);
                    break;
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }

        void HandleHID(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 0xA: // GetIPCHandles
                    WriteIPCReply(cpu.SP, 0, AllocHandle("HIDEvent"), AllocHandle("HIDAccel"), AllocHandle("HIDGyro"), AllocHandle("HIDDebug"));
                    break;
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }

        void HandleFS(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 0x801: // Initialize
                    WriteIPCReply(cpu.SP, 0);
                    break;
                case 0x802: // OpenFile (simplified — always succeed)
                    WriteIPCReply(cpu.SP, 0, AllocHandle("FileHandle"), 0);
                    break;
                case 0x803: // OpenFileDirectly
                    WriteIPCReply(cpu.SP, 0, AllocHandle("FileHandle"), 0);
                    break;
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }

        // Stub handlers for remaining services
        void HandleNDM(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleCFG(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 0x1: // GetConfigInfoBlk2 — return system config
                    uint size = ReadIPCParam(cpu.SP, 0);
                    uint blkId = ReadIPCParam(cpu.SP, 1);
                    uint outAddr = ReadIPCParam(cpu.SP, 2);
                    // Fill with defaults
                    for (uint i = 0; i < Math.Min(size, 64u); i++) bus.Write8(outAddr + i, 0);
                    // Region: USA=1
                    if (blkId == 0xA0002) bus.Write8(outAddr, 1);
                    WriteIPCReply(cpu.SP, 0);
                    break;
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }
        void HandlePTM(ARM11CPU cpu, DS3MemoryBus bus)
        {
            WriteIPCReply(cpu.SP, 0, 100); // 100% battery
        }
        void HandleAM(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleAC(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleNWM(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleDSP(ARM11CPU cpu, DS3MemoryBus bus)
        {
            uint cmd = ReadIPCCmd(cpu.SP);
            switch (cmd >> 16)
            {
                case 1: // RecvData
                    WriteIPCReply(cpu.SP, 0, 1); // DSP ready
                    break;
                case 3: // WriteProcessPipe
                    WriteIPCReply(cpu.SP, 0);
                    break;
                default:
                    WriteIPCReply(cpu.SP, 0);
                    break;
            }
        }
        void HandleCSND(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleMIC(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleIR(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
        void HandleCDC(ARM11CPU cpu, DS3MemoryBus bus) => WriteIPCReply(cpu.SP, 0);
    }

    // ============================================================
    // ARM INTERPRETER (Thumb + ARM)
    // ============================================================
    public class ARMInterpreter
    {
        DS3MemoryBus Bus;
        DS3HLE HLE;
        public long InstructionsExecuted = 0;
        public long Cycles = 0;

        public ARMInterpreter(DS3MemoryBus bus, DS3HLE hle)
        {
            Bus = bus;
            HLE = hle;
        }

        public int RunCycles(ARM11CPU cpu, int cyclesToRun)
        {
            int ran = 0;
            while (ran < cyclesToRun && cpu.Running)
            {
                if (cpu.Mode == ARMMode.Thumb)
                    ran += StepThumb(cpu);
                else
                    ran += StepARM(cpu);
                InstructionsExecuted++;
                Cycles += ran;
            }
            return ran;
        }

        int StepThumb(ARM11CPU cpu)
        {
            ushort instr = Bus.Read16(cpu.PC);
            cpu.PC += 2;
            return ExecuteThumb(cpu, instr);
        }

        int StepARM(ARM11CPU cpu)
        {
            uint instr = Bus.Read32(cpu.PC);
            cpu.PC += 4;
            return ExecuteARM(cpu, instr);
        }

        // ---- THUMB DECODER ----
        int ExecuteThumb(ARM11CPU cpu, ushort i)
        {
            uint op = (uint)(i >> 13);

            switch (i >> 11)
            {
                // LSL immediate
                case 0b00000:
                { int imm5=(i>>6)&0x1F; int rm=(i>>3)&7; int rd=i&7; uint v=cpu.R[rm]; if(imm5>0){cpu.CPSR=(cpu.CPSR&~(1u<<29))|((v>>(32-imm5))<<29&(1u<<29));v<<=imm5;} cpu.R[rd]=v; cpu.SetFlagsNZ(v); return 1; }
                // LSR immediate
                case 0b00001:
                { int imm5=(i>>6)&0x1F; if(imm5==0)imm5=32; int rm=(i>>3)&7; int rd=i&7; uint v=imm5<32?cpu.R[rm]>>imm5:0; cpu.R[rd]=v; cpu.SetFlagsNZ(v); return 1; }
                // ASR immediate
                case 0b00010:
                { int imm5=(i>>6)&0x1F; if(imm5==0)imm5=32; int rm=(i>>3)&7; int rd=i&7; int v=imm5<32?(int)cpu.R[rm]>>imm5:(int)cpu.R[rm]>>31; cpu.R[rd]=(uint)v; cpu.SetFlagsNZ((uint)v); return 1; }

                // ADD/SUB register/imm3
                case 0b00011:
                {
                    bool sub=(i&(1<<9))!=0, imm=(i&(1<<10))!=0;
                    int rn=(i>>3)&7, rd=i&7;
                    uint val=imm?(uint)((i>>6)&7):cpu.R[(i>>6)&7];
                    uint res=sub?cpu.R[rn]-val:cpu.R[rn]+val;
                    cpu.R[rd]=res;
                    bool n=(res>>31)!=0,z=res==0,c=sub?cpu.R[rn]>=val:res<cpu.R[rn],
                         v=sub?((cpu.R[rn]^val)&(cpu.R[rn]^res)&0x80000000)!=0:((~(cpu.R[rn]^val))&(cpu.R[rn]^res)&0x80000000)!=0;
                    cpu.SetFlags(n,z,c,v); return 1;
                }

                // MOV/CMP/ADD/SUB imm8
                case 0b00100: { int rd=(i>>8)&7; cpu.R[rd]=(uint)(i&0xFF); cpu.SetFlagsNZ(cpu.R[rd]); return 1; }
                case 0b00101:
                { int rn=(i>>8)&7; uint res=cpu.R[rn]-(uint)(i&0xFF); cpu.SetFlags((res>>31)!=0,res==0,cpu.R[rn]>=(uint)(i&0xFF),false); return 1; }
                case 0b00110: { int rd=(i>>8)&7; uint res=cpu.R[rd]+(uint)(i&0xFF); bool c2=res<cpu.R[rd]; cpu.R[rd]=res; cpu.SetFlags((res>>31)!=0,res==0,c2,false); return 1; }
                case 0b00111: { int rd=(i>>8)&7; uint res=cpu.R[rd]-(uint)(i&0xFF); cpu.R[rd]=res; cpu.SetFlags((res>>31)!=0,res==0,cpu.R[rd]+(uint)(i&0xFF)>=cpu.R[rd],false); return 1; }

                // Data processing (ALU)
                case 0b01000 when (i>>10&1)==0:
                    return ThumbALU(cpu, i);

                // Hi register ops + BX
                case 0b01000 when (i>>10&1)!=0:
                    return ThumbHiRegBX(cpu, i);

                // LDR PC-relative
                case 0b01001:
                { int rd=(i>>8)&7; uint addr=(cpu.PC&~3u)+(uint)((i&0xFF)<<2); cpu.R[rd]=Bus.Read32(addr); return 2; }

                // STR/LDR register
                case 0b01010 or 0b01011:
                    return ThumbLoadStoreReg(cpu, i);

                // STR/LDR immediate
                case 0b01100: { int rb=(i>>3)&7,rd=i&7; uint addr=cpu.R[rb]+(uint)((i>>6&0x1F)<<2); Bus.Write32(addr,cpu.R[rd]); return 2; }
                case 0b01101: { int rb=(i>>3)&7,rd=i&7; uint addr=cpu.R[rb]+(uint)((i>>6&0x1F)<<2); cpu.R[rd]=Bus.Read32(addr); return 2; }
                case 0b01110: { int rb=(i>>3)&7,rd=i&7; uint addr=cpu.R[rb]+(uint)(i>>6&0x1F); Bus.Write8(addr,(byte)cpu.R[rd]); return 2; }
                case 0b01111: { int rb=(i>>3)&7,rd=i&7; uint addr=cpu.R[rb]+(uint)(i>>6&0x1F); cpu.R[rd]=Bus.Read8(addr); return 2; }

                // STR/LDR SP-relative
                case 0b10010: { int rd=(i>>8)&7; Bus.Write32(cpu.SP+(uint)((i&0xFF)<<2),cpu.R[rd]); return 2; }
                case 0b10011: { int rd=(i>>8)&7; cpu.R[rd]=Bus.Read32(cpu.SP+(uint)((i&0xFF)<<2)); return 2; }

                // ADD PC/SP + imm8
                case 0b10100: { int rd=(i>>8)&7; cpu.R[rd]=(cpu.PC&~3u)+(uint)((i&0xFF)<<2); return 1; }
                case 0b10101: { int rd=(i>>8)&7; cpu.R[rd]=cpu.SP+(uint)((i&0xFF)<<2); return 1; }

                // Misc
                case 0b10110 or 0b10111:
                    return ThumbMisc(cpu, i);

                // LDMIA/STMIA
                case 0b11000: { int rb=(i>>8)&7; uint addr=cpu.R[rb]; for(int b=0;b<8;b++)if((i&(1<<b))!=0){Bus.Write32(addr,cpu.R[b]);addr+=4;} cpu.R[rb]=addr; return 3; }
                case 0b11001: { int rb=(i>>8)&7; uint addr=cpu.R[rb]; for(int b=0;b<8;b++)if((i&(1<<b))!=0){cpu.R[b]=Bus.Read32(addr);addr+=4;} cpu.R[rb]=addr; return 3; }

                // Conditional branch
                case 0b11010:
                { int cond=(i>>8)&0xF; if(cond==0xF){HandleSVC(cpu,(uint)(i&0xFF));return 5;} if(cpu.ConditionPasses((uint)cond)){int off=(sbyte)(i&0xFF);cpu.PC=(uint)(cpu.PC+off*2);} return 1; }

                // Unconditional branch
                case 0b11100:
                { int off=(i&0x7FF)<<1; if((off&0x800)!=0)off|=unchecked((int)0xFFFFF000); cpu.PC=(uint)(cpu.PC+off); return 2; }

                // BL/BLX
                case 0b11110:
                { int h=(i>>11)&3;
                  if(h==2){int off=(i&0x7FF)<<12;if((off&0x400000)!=0)off|=unchecked((int)0xFF800000);cpu.LR=(uint)(cpu.PC+off);return 1;}
                  else if(h==3){uint target=cpu.LR+(uint)((i&0x7FF)<<1);cpu.LR=(cpu.PC-2)|1;cpu.PC=target;return 2;}
                  return 1; }

                default:
                    // NOP for anything we don't handle yet
                    return 1;
            }
        }

        int ThumbALU(ARM11CPU cpu, ushort i)
        {
            int op=(i>>6)&0xF, rs=(i>>3)&7, rd=i&7;
            uint s=cpu.R[rs],d=cpu.R[rd],res=0;
            switch(op)
            {
                case 0: res=d&s; break;         // AND
                case 1: res=d^s; break;         // EOR/XOR
                case 2: res=d<<(int)(s&0xFF); break; // LSL
                case 3: res=d>>(int)(s&0xFF); break; // LSR
                case 4: res=(uint)((int)d>>(int)(s&0xFF)); break; // ASR
                case 5: { ulong r=(ulong)d+s+(cpu.FlagC?1u:0u);res=(uint)r;cpu.SetFlags((res>>31)!=0,res==0,r>0xFFFFFFFF,false);return 1; } // ADC
                case 6: { res=d-s-(cpu.FlagC?0u:1u);cpu.SetFlags((res>>31)!=0,res==0,d>=s,false);cpu.R[rd]=res;return 1; } // SBC
                case 7: { int sc=(int)(s&0xFF);res=sc==0?d:((d>>(32-sc))|(d<<sc)); // ROR
                    cpu.SetFlags((res>>31)!=0,res==0,(res&1)!=0,cpu.FlagV);cpu.R[rd]=res;return 1; }
                case 8: res=d&s; cpu.SetFlagsNZ(res); return 1; // TST (no writeback)
                case 9: { res=0-s; bool c=s==0; cpu.SetFlags((res>>31)!=0,res==0,c,s==0x80000000); cpu.R[rd]=res; return 1; } // NEG
                case 10:{ res=d-s; cpu.SetFlags((res>>31)!=0,res==0,d>=s,((d^s)&(d^res)&0x80000000)!=0); return 1; } // CMP
                case 11:{ res=d+s; bool c=res<d; cpu.SetFlags((res>>31)!=0,res==0,c,((~(d^s))&(d^res)&0x80000000)!=0); return 1; } // CMN
                case 12: res=d|s; break;        // ORR
                case 13: res=d*s; break;        // MUL
                case 14: res=d&~s; break;       // BIC
                case 15: res=~s; break;         // MVN
            }
            cpu.R[rd]=res; cpu.SetFlagsNZ(res); return 1;
        }

        int ThumbHiRegBX(ARM11CPU cpu, ushort i)
        {
            int op=(i>>8)&3;
            int rm=((i>>3)&0xF), rd=(i&7)|((i>>4)&8);
            switch(op)
            {
                case 0: { uint res=cpu.R[rd]+cpu.R[rm]; if(rd==15)cpu.PC=res; else cpu.R[rd]=res; break; } // ADD
                case 1: { uint res=cpu.R[rd]-cpu.R[rm]; cpu.SetFlags((res>>31)!=0,res==0,cpu.R[rd]>=cpu.R[rm],false); break; } // CMP
                case 2: if(rd==15)cpu.PC=cpu.R[rm]; else cpu.R[rd]=cpu.R[rm]; break; // MOV
                case 3: // BX/BLX
                {
                    uint target=cpu.R[rm];
                    if(op==3 && (i&(1<<7))!=0) { cpu.LR=(cpu.PC-2)|1; } // BLX
                    if((target&1)!=0){cpu.CPSR|=(1<<5);cpu.PC=target&~1u;}
                    else{cpu.CPSR&=~(uint)(1<<5);cpu.PC=target&~3u;}
                    return 2;
                }
            }
            return 1;
        }

        int ThumbLoadStoreReg(ARM11CPU cpu, ushort i)
        {
            int op=(i>>9)&7, ro=(i>>6)&7, rb=(i>>3)&7, rd=i&7;
            uint addr=cpu.R[rb]+cpu.R[ro];
            switch(op)
            {
                case 0: Bus.Write32(addr,cpu.R[rd]); return 2;   // STR
                case 1: Bus.Write16(addr,(ushort)cpu.R[rd]); return 2; // STRH
                case 2: Bus.Write8(addr,(byte)cpu.R[rd]); return 2;   // STRB
                case 3: cpu.R[rd]=(uint)(short)Bus.Read16(addr); return 2; // LDRSB
                case 4: cpu.R[rd]=Bus.Read32(addr); return 2;   // LDR
                case 5: cpu.R[rd]=Bus.Read16(addr); return 2;  // LDRH
                case 6: cpu.R[rd]=Bus.Read8(addr); return 2;   // LDRB
                case 7: cpu.R[rd]=(uint)(short)Bus.Read16(addr); return 2; // LDRSH
            }
            return 1;
        }

        int ThumbMisc(ARM11CPU cpu, ushort i)
        {
            switch((i>>8)&0xFF)
            {
                case 0b01101100: // PUSH without LR
                case 0b10101100: // PUSH with LR
                {
                    bool lr=(i&(1<<8))!=0;
                    if(lr){cpu.SP-=4;Bus.Write32(cpu.SP,cpu.LR);}
                    for(int b=7;b>=0;b--){if((i&(1<<b))!=0){cpu.SP-=4;Bus.Write32(cpu.SP,cpu.R[b]);}}
                    return 3;
                }
                case 0b10110000: // ADD SP, #imm
                { int imm=(i&0x7F)<<2; if((i&0x80)!=0)cpu.SP-=(uint)imm; else cpu.SP+=(uint)imm; return 1; }
                case 0b11010000: // POP without PC
                case 0b11110000: // POP with PC
                {
                    bool pc=(i&(1<<8))!=0;
                    for(int b=0;b<8;b++){if((i&(1<<b))!=0){cpu.R[b]=Bus.Read32(cpu.SP);cpu.SP+=4;}}
                    if(pc){uint target=Bus.Read32(cpu.SP);cpu.SP+=4;if((target&1)!=0){cpu.CPSR|=(1<<5);cpu.PC=target&~1u;}else{cpu.CPSR&=~(uint)(1<<5);cpu.PC=target&~3u;}}
                    return 3;
                }
                case 0b11000000: // BKPT
                    Console.WriteLine($"[3DS] BKPT at PC=0x{cpu.PC:X}");
                    return 1;
                default:
                    return 1;
            }
        }

        void HandleSVC(ARM11CPU cpu, uint id) => HLE?.HandleSVC(cpu, id);

        // ---- ARM32 DECODER ----
        int ExecuteARM(ARM11CPU cpu, uint i)
        {
            uint cond = i >> 28;
            if (!cpu.ConditionPasses(cond)) return 1;

            uint type = (i >> 25) & 7;

            switch (type)
            {
                case 0b000: case 0b001: // Data processing
                    return ARMDataProc(cpu, i);

                case 0b010: case 0b011: // Load/Store
                    return ARMLoadStore(cpu, i);

                case 0b100: // LDM/STM
                    return ARMBlockTransfer(cpu, i);

                case 0b101: // Branch
                {
                    int off = (int)(i << 8) >> 6; // sign extend 24-bit * 4
                    if ((i & (1 << 24)) != 0) cpu.LR = cpu.PC - 4; // BL
                    cpu.PC = (uint)(cpu.PC + off);
                    return 3;
                }

                case 0b111: // SVC / Coprocessor
                    if ((i & 0x0F000000) == 0x0F000000)
                    {
                        uint svcId = i & 0xFFFFFF;
                        HandleSVC(cpu, svcId);
                        return 5;
                    }
                    return 1;

                default:
                    return 1;
            }
        }

        int ARMDataProc(ARM11CPU cpu, uint i)
        {
            uint op   = (i >> 21) & 0xF;
            uint s    = (i >> 20) & 1;
            uint rn   = (i >> 16) & 0xF;
            uint rd   = (i >> 12) & 0xF;
            uint operand2 = GetARMOperand2(cpu, i, s != 0);

            uint a = cpu.R[rn];
            ulong res64;
            uint result = 0;
            bool writeResult = true;

            switch (op)
            {
                case 0x0: result = a & operand2; break;                   // AND
                case 0x1: result = a ^ operand2; break;                   // EOR
                case 0x2: result = a - operand2; break;                   // SUB
                case 0x3: result = operand2 - a; break;                   // RSB
                case 0x4: result = a + operand2; break;                   // ADD
                case 0x5: result = a + operand2 + (cpu.FlagC?1u:0u); break; // ADC
                case 0x6: result = a - operand2 - (cpu.FlagC?0u:1u); break; // SBC
                case 0x7: result = operand2 - a - (cpu.FlagC?0u:1u); break; // RSC
                case 0x8: result = a & operand2; writeResult = false; break; // TST
                case 0x9: result = a ^ operand2; writeResult = false; break; // TEQ
                case 0xA: result = a - operand2; writeResult = false; break; // CMP
                case 0xB: result = a + operand2; writeResult = false; break; // CMN
                case 0xC: result = a | operand2; break;                   // ORR
                case 0xD: result = operand2; break;                       // MOV
                case 0xE: result = a & ~operand2; break;                  // BIC
                case 0xF: result = ~operand2; break;                      // MVN
            }

            if (s != 0)
            {
                bool n = (result >> 31) != 0, z = result == 0;
                bool c = op switch { 0x2 or 0x6 or 0xA => a >= operand2, 0x4 or 0x5 or 0xB => result < a, _ => cpu.FlagC };
                bool v = op switch { 0x2 or 0xA => ((a ^ operand2) & (a ^ result) & 0x80000000) != 0,
                                     0x4 or 0xB => ((~(a ^ operand2)) & (a ^ result) & 0x80000000) != 0, _ => cpu.FlagV };
                if (rd == 15) { cpu.CPSR = cpu.SPSR; }
                else cpu.SetFlags(n, z, c, v);
            }

            if (writeResult)
            {
                if (rd == 15) { cpu.PC = result & (cpu.Mode == ARMMode.Thumb ? ~1u : ~3u); }
                else cpu.R[rd] = result;
            }
            return 1;
        }

        uint GetARMOperand2(ARM11CPU cpu, uint i, bool setFlags)
        {
            if ((i & (1 << 25)) != 0)
            {
                // Immediate
                uint imm8 = i & 0xFF;
                int rotate = (int)((i >> 8) & 0xF) * 2;
                return rotate == 0 ? imm8 : (imm8 >> rotate) | (imm8 << (32 - rotate));
            }
            else
            {
                // Register shift
                uint rm = i & 0xF;
                uint val = cpu.R[rm];
                uint shiftType = (i >> 5) & 3;
                int shiftAmt;
                if ((i & (1 << 4)) != 0) shiftAmt = (int)(cpu.R[(i >> 8) & 0xF] & 0xFF);
                else shiftAmt = (int)((i >> 7) & 0x1F);

                return shiftType switch
                {
                    0 => shiftAmt == 0 ? val : val << shiftAmt,                           // LSL
                    1 => shiftAmt == 0 ? 0 : val >> shiftAmt,                             // LSR
                    2 => shiftAmt == 0 ? (uint)((int)val >> 31) : (uint)((int)val >> shiftAmt), // ASR
                    3 => shiftAmt == 0 ? ((cpu.FlagC ? (1u << 31) : 0) | (val >> 1)) :   // RRX / ROR
                         (val >> shiftAmt) | (val << (32 - shiftAmt)),
                    _ => val
                };
            }
        }

        int ARMLoadStore(ARM11CPU cpu, uint i)
        {
            bool load  = (i & (1 << 20)) != 0;
            bool byte_ = (i & (1 << 22)) != 0;
            bool pre   = (i & (1 << 24)) != 0;
            bool up    = (i & (1 << 23)) != 0;
            bool wb    = (i & (1 << 21)) != 0;
            uint rn    = (i >> 16) & 0xF;
            uint rd    = (i >> 12) & 0xF;

            uint base_ = cpu.R[rn];
            uint offset;
            if ((i & (1 << 25)) != 0) offset = GetARMOperand2(cpu, i, false);
            else offset = i & 0xFFF;

            uint addr = pre ? (up ? base_ + offset : base_ - offset) : base_;

            if (load)
            {
                uint val = byte_ ? Bus.Read8(addr) : Bus.Read32(addr);
                if (rd == 15) { cpu.PC = val & ~3u; } else cpu.R[rd] = val;
            }
            else
            {
                uint val = rd == 15 ? cpu.PC + 4 : cpu.R[rd];
                if (byte_) Bus.Write8(addr, (byte)val);
                else Bus.Write32(addr, val);
            }

            if (!pre) addr = up ? base_ + offset : base_ - offset;
            if (wb || !pre) cpu.R[rn] = addr;

            return load ? 3 : 2;
        }

        int ARMBlockTransfer(ARM11CPU cpu, uint i)
        {
            bool load = (i & (1 << 20)) != 0;
            bool up   = (i & (1 << 23)) != 0;
            bool pre  = (i & (1 << 24)) != 0;
            bool wb   = (i & (1 << 21)) != 0;
            uint rn   = (i >> 16) & 0xF;
            uint regList = i & 0xFFFF;
            uint addr = cpu.R[rn];

            int count = 0;
            for (int b = 0; b < 16; b++) if ((regList & (1u << b)) != 0) count++;

            if (!up) addr -= (uint)(count * 4);
            uint startAddr = addr;

            for (int b = 0; b < 16; b++)
            {
                if ((regList & (1u << b)) == 0) continue;
                uint ea = pre ? (up ? addr + 4 : addr) : addr;
                if (load)
                {
                    uint v = Bus.Read32(ea);
                    if (b == 15) { cpu.PC = v & ~3u; } else cpu.R[b] = v;
                }
                else Bus.Write32(ea, b == 15 ? cpu.PC + 4 : cpu.R[b]);
                if (up) addr += 4; else addr -= 4;
            }

            if (wb) cpu.R[rn] = up ? startAddr + (uint)(count * 4) : startAddr;
            return count + 2;
        }
    }

    // ============================================================
    // 3DS ROM LOADER (.3ds / .cci / ELF)
    // ============================================================
    public class DS3ROMLoader
    {
        public struct LoadedROM
        {
            public string Title;
            public uint EntryPoint;
            public uint StackPointer;
            public bool IsValid;
        }

        public static LoadedROM Load3DS(string path, DS3MemoryBus bus)
        {
            if (!File.Exists(path))
                return new LoadedROM { IsValid = false };

            byte[] data = File.ReadAllBytes(path);
            Console.WriteLine($"[LOADER] Reading '{path}' ({data.Length:N0} bytes)");

            // Check for ELF (homebrew)
            if (data.Length > 4 && data[0] == 0x7F && data[1] == 'E' && data[2] == 'L' && data[3] == 'F')
                return LoadELF(data, bus);

            // Check for 3DS NCSD header
            if (data.Length > 0x200)
            {
                // NCSD magic at 0x100
                if (data[0x100] == 'N' && data[0x101] == 'C' && data[0x102] == 'S' && data[0x103] == 'D')
                    return LoadNCSD(data, bus);

                // NCCH magic at 0x100 (single partition)
                if (data[0x100] == 'N' && data[0x101] == 'C' && data[0x102] == 'C' && data[0x103] == 'H')
                    return LoadNCCH(data, 0, bus);
            }

            Console.WriteLine("[LOADER] Unknown ROM format, attempting raw load at 0x100000");
            bus.LoadAt(data, 0x00100000);
            return new LoadedROM { Title = Path.GetFileNameWithoutExtension(path), EntryPoint = 0x00100000, StackPointer = DS3Constants.FCRAM_BASE + 0x03F00000, IsValid = true };
        }

        static LoadedROM LoadELF(byte[] data, DS3MemoryBus bus)
        {
            Console.WriteLine("[LOADER] ELF format detected");
            uint entry = ReadU32(data, 0x18);
            ushort phCount = ReadU16(data, 0x2C);
            ushort phSize  = ReadU16(data, 0x2A);
            uint phOff     = ReadU32(data, 0x1C);
            string title = "Homebrew ELF";

            for (int i = 0; i < phCount; i++)
            {
                uint phBase = phOff + (uint)(i * phSize);
                uint ptype  = ReadU32(data, phBase);
                if (ptype != 1) continue; // PT_LOAD only

                uint foff   = ReadU32(data, phBase + 4);
                uint vaddr  = ReadU32(data, phBase + 8);
                uint filesz = ReadU32(data, phBase + 16);
                uint memsz  = ReadU32(data, phBase + 20);

                Console.WriteLine($"  [ELF] PT_LOAD: 0x{foff:X} -> 0x{vaddr:X}, {filesz:N0} bytes (mem {memsz:N0})");
                if (foff + filesz <= data.Length)
                {
                    byte[] seg = new byte[filesz];
                    Buffer.BlockCopy(data, (int)foff, seg, 0, (int)filesz);
                    bus.LoadAt(seg, vaddr);
                }
            }
            Console.WriteLine($"[LOADER] ELF entry: 0x{entry:X}");
            return new LoadedROM { Title = title, EntryPoint = entry, StackPointer = DS3Constants.FCRAM_BASE + 0x03F00000, IsValid = true };
        }

        static LoadedROM LoadNCSD(byte[] data, DS3MemoryBus bus)
        {
            Console.WriteLine("[LOADER] NCSD format detected (.3ds)");
            // First partition (game NCCH) starts at offset 0x4000
            // Partition 0 offset and size are at 0x120 (in media units = 512 bytes)
            uint p0Offset = ReadU32(data, 0x120) * 512;
            uint p0Size   = ReadU32(data, 0x124) * 512;
            Console.WriteLine($"  [NCSD] Partition 0: offset=0x{p0Offset:X}, size=0x{p0Size:X}");

            if (p0Offset < data.Length)
                return LoadNCCH(data, p0Offset, bus);

            return new LoadedROM { IsValid = false };
        }

        static LoadedROM LoadNCCH(byte[] data, uint ncchBase, DS3MemoryBus bus)
        {
            Console.WriteLine($"[LOADER] NCCH at offset 0x{ncchBase:X}");
            // Read exheader to get code segment info
            uint exHeaderOff = ncchBase + 0x200;
            if (exHeaderOff + 0x400 > data.Length) { Console.WriteLine("[LOADER] No exheader"); return new LoadedROM { IsValid = false }; }

            uint codeAddr = ReadU32(data, exHeaderOff + 0x10);  // .text vaddr
            uint codeSize = ReadU32(data, exHeaderOff + 0x08);  // .text size
            uint stackSz  = ReadU32(data, exHeaderOff + 0x04);
            uint bssSize  = ReadU32(data, exHeaderOff + 0x18);
            uint rodataAddr= ReadU32(data, exHeaderOff + 0x20);
            uint rodataSz = ReadU32(data, exHeaderOff + 0x18);
            uint dataAddr = ReadU32(data, exHeaderOff + 0x30);
            uint dataSz   = ReadU32(data, exHeaderOff + 0x28);

            // Game title (UTF-16 in exheader at 0x0)
            var titleBytes = new byte[16];
            Buffer.BlockCopy(data, (int)exHeaderOff, titleBytes, 0, 16);
            string title = Encoding.ASCII.GetString(titleBytes).TrimEnd('\0', '\x00');

            Console.WriteLine($"  [NCCH] Title: '{title}'");
            Console.WriteLine($"  [NCCH] .text: 0x{codeAddr:X} size=0x{codeSize:X}");

            // ExeFS is at ncchBase + exefsOffset * mediaUnit
            uint exefsOffset = ReadU32(data, ncchBase + 0x1A0) * 512;
            uint exefsSize   = ReadU32(data, ncchBase + 0x1A4) * 512;

            if (ncchBase + exefsOffset + exefsSize > data.Length)
            {
                Console.WriteLine("[LOADER] ExeFS out of bounds, using raw code segment");
                // Fallback: load raw from right after exheader
                uint rawCode = ncchBase + 0x600;
                if (rawCode < data.Length)
                {
                    byte[] seg = new byte[Math.Min((uint)(data.Length - rawCode), 0x200000)];
                    Buffer.BlockCopy(data, (int)rawCode, seg, 0, seg.Length);
                    bus.LoadAt(seg, DS3Constants.FCRAM_BASE);
                }
                return new LoadedROM { Title = title, EntryPoint = DS3Constants.FCRAM_BASE + codeAddr, StackPointer = DS3Constants.FCRAM_BASE + 0x03F00000, IsValid = true };
            }

            // Parse ExeFS header
            Console.WriteLine($"  [NCCH] ExeFS at 0x{ncchBase + exefsOffset:X}");
            uint exefsBase = ncchBase + exefsOffset;

            // ExeFS has 8 file entries at the header, each 16 bytes
            for (int f = 0; f < 8; f++)
            {
                uint entryOff = exefsBase + (uint)(f * 16);
                if (entryOff + 16 > data.Length) break;
                var nameBuf = new byte[8];
                Buffer.BlockCopy(data, (int)entryOff, nameBuf, 0, 8);
                string fname = Encoding.ASCII.GetString(nameBuf).TrimEnd('\0');
                uint foff   = ReadU32(data, entryOff + 8);
                uint fsize  = ReadU32(data, entryOff + 12);
                if (fsize == 0) continue;

                Console.WriteLine($"    [ExeFS] '{fname}': offset=0x{foff:X} size=0x{fsize:X}");
                uint fileDataOff = exefsBase + 0x200 + foff; // ExeFS data starts 512 bytes after header
                if (fileDataOff + fsize > data.Length) continue;

                uint loadAddr = fname == ".code" ? (DS3Constants.FCRAM_BASE + codeAddr) :
                                fname == ".data" ? (DS3Constants.FCRAM_BASE + dataAddr) :
                                fname == "icon"  ? 0 : // skip
                                DS3Constants.FCRAM_BASE + 0x01000000;

                if (loadAddr == 0) continue;
                byte[] seg = new byte[fsize];
                Buffer.BlockCopy(data, (int)fileDataOff, seg, 0, (int)fsize);
                bus.LoadAt(seg, loadAddr);
            }

            uint ep = DS3Constants.FCRAM_BASE + codeAddr;
            uint sp = DS3Constants.FCRAM_BASE + 0x03F00000 - stackSz;
            Console.WriteLine($"[LOADER] ROM loaded: entry=0x{ep:X}, SP=0x{sp:X}");
            return new LoadedROM { Title = title, EntryPoint = ep, StackPointer = sp, IsValid = true };
        }

        static uint ReadU32(byte[] d, uint off) => off + 3 < d.Length ? (uint)(d[off] | (d[off+1]<<8) | (d[off+2]<<16) | (d[off+3]<<24)) : 0;
        static ushort ReadU16(byte[] d, uint off) => off + 1 < d.Length ? (ushort)(d[off] | (d[off+1]<<8)) : (ushort)0;
    }

    // ============================================================
    // SDL2 RENDERER — Dual screen with scaling
    // ============================================================
    public class DS3Renderer
    {
        IntPtr window, renderer;
        IntPtr topTexture, botTexture;
        const int SCALE = 2;
        const int BORDER = 8;
        const int WIN_W = DS3Constants.TOP_WIDTH * SCALE;
        const int WIN_H = (DS3Constants.TOP_HEIGHT + DS3Constants.BOT_HEIGHT + BORDER) * SCALE;

        // Pixel buffers
        byte[] topBuf = new byte[DS3Constants.TOP_WIDTH * DS3Constants.TOP_HEIGHT * 4];
        byte[] botBuf = new byte[DS3Constants.BOT_WIDTH * DS3Constants.BOT_HEIGHT * 4];

        bool initialized = false;

        public bool Init(string title)
        {
            // Browser: use BrowserPlatform (WebGL/Canvas) instead of SDL native
            if (OperatingSystem.IsBrowser())
            {
                bool ok = BrowserPlatform.InitDisplay(DS3Constants.TOP_WIDTH, DS3Constants.TOP_HEIGHT,
                    DS3Constants.BOT_WIDTH, DS3Constants.BOT_HEIGHT, title);
                initialized = ok;
                if (ok) Console.WriteLine("[WASM] Renderer initialized (browser)");
                return ok;
            }

            if (SDL2.SDL_Init(SDL2.SDL_INIT_VIDEO | SDL2.SDL_INIT_EVENTS) < 0)
            {
                Console.WriteLine($"[SDL] Init failed: {SDL2.GetError()}");
                return false;
            }

            window = SDL2.SDL_CreateWindow(
                $"MindWorks 3DS — {title}",
                (int)SDL2.SDL_WINDOWPOS_CENTERED,
                (int)SDL2.SDL_WINDOWPOS_CENTERED,
                WIN_W, WIN_H,
                SDL2.SDL_WINDOW_SHOWN | SDL2.SDL_WINDOW_RESIZABLE);

            if (window == IntPtr.Zero)
            {
                Console.WriteLine($"[SDL] CreateWindow failed: {SDL2.GetError()}");
                return false;
            }

            renderer = SDL2.SDL_CreateRenderer(window, -1, 4); // SDL_RENDERER_ACCELERATED
            if (renderer == IntPtr.Zero)
            {
                renderer = SDL2.SDL_CreateRenderer(window, -1, 0); // fallback: software
                Console.WriteLine("[SDL] Using software renderer");
            }

            topTexture = SDL2.SDL_CreateTexture(renderer,
                unchecked((uint)SDL2.SDL_PIXELFORMAT_RGBA8888),
                SDL2.SDL_TEXTUREACCESS_STREAMING,
                DS3Constants.TOP_WIDTH, DS3Constants.TOP_HEIGHT);

            botTexture = SDL2.SDL_CreateTexture(renderer,
                unchecked((uint)SDL2.SDL_PIXELFORMAT_RGBA8888),
                SDL2.SDL_TEXTUREACCESS_STREAMING,
                DS3Constants.BOT_WIDTH, DS3Constants.BOT_HEIGHT);

            initialized = true;
            Console.WriteLine($"[SDL] Renderer ready: {WIN_W}x{WIN_H} (2x scale)");
            return true;
        }

        // Convert 3DS VRAM framebuffer (BGR24, rotated) to RGBA for SDL
        public void UpdateFromVRAM(byte[] vram, uint topFBOffset, uint botFBOffset)
        {
            // Top screen: 400x240, stored as BGR columns rotated 90°
            // Each column is 240 bytes, 400 columns
            for (int x = 0; x < DS3Constants.TOP_WIDTH; x++)
            for (int y = 0; y < DS3Constants.TOP_HEIGHT; y++)
            {
                uint src = (topFBOffset & (uint)(vram.Length - 1)) + (uint)(x * DS3Constants.TOP_HEIGHT + y) * 3;
                if (src + 2 >= vram.Length) continue;
                int dst = (y * DS3Constants.TOP_WIDTH + x) * 4;
                topBuf[dst+0] = vram[src+2]; // R
                topBuf[dst+1] = vram[src+1]; // G
                topBuf[dst+2] = vram[src+0]; // B
                topBuf[dst+3] = 0xFF;
            }

            // Bottom screen
            for (int x = 0; x < DS3Constants.BOT_WIDTH; x++)
            for (int y = 0; y < DS3Constants.BOT_HEIGHT; y++)
            {
                uint src = (botFBOffset & (uint)(vram.Length - 1)) + (uint)(x * DS3Constants.BOT_HEIGHT + y) * 3;
                if (src + 2 >= vram.Length) continue;
                int dst = (y * DS3Constants.BOT_WIDTH + x) * 4;
                botBuf[dst+0] = vram[src+2];
                botBuf[dst+1] = vram[src+1];
                botBuf[dst+2] = vram[src+0];
                botBuf[dst+3] = 0xFF;
            }
        }

        public void Render(byte[] vram, uint topFBOff, uint botFBOff)
        {
            if (!initialized) return;

            UpdateFromVRAM(vram, topFBOff, botFBOff);

            if (OperatingSystem.IsBrowser())
            {
                // For browser, hand the RGBA buffers to the JS bridge
                BrowserPlatform.PresentFrame(topBuf, botBuf, DS3Constants.TOP_WIDTH, DS3Constants.TOP_HEIGHT,
                    DS3Constants.BOT_WIDTH, DS3Constants.BOT_HEIGHT);
                return;
            }

            // Native SDL path
            var topHandle = GCHandle.Alloc(topBuf, GCHandleType.Pinned);
            var botHandle = GCHandle.Alloc(botBuf, GCHandleType.Pinned);
            try
            {
                SDL2.SDL_UpdateTexture(topTexture, IntPtr.Zero, topHandle.AddrOfPinnedObject(), DS3Constants.TOP_WIDTH * 4);
                SDL2.SDL_UpdateTexture(botTexture, IntPtr.Zero, botHandle.AddrOfPinnedObject(), DS3Constants.BOT_WIDTH * 4);
            }
            finally { topHandle.Free(); botHandle.Free(); }

            // Draw
            SDL2.SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
            SDL2.SDL_RenderClear(renderer);

            // Top screen centered
            int topX = (WIN_W - DS3Constants.TOP_WIDTH * SCALE) / 2;
            var topRect = new SDL2.SDL_Rect { x = topX, y = 0, w = DS3Constants.TOP_WIDTH * SCALE, h = DS3Constants.TOP_HEIGHT * SCALE };
            SDL2.SDL_RenderCopy(renderer, topTexture, IntPtr.Zero, ref topRect);

            // Draw black divider
            SDL2.SDL_SetRenderDrawColor(renderer, 10, 10, 10, 255);
            var divRect = new SDL2.SDL_Rect { x = 0, y = DS3Constants.TOP_HEIGHT * SCALE, w = WIN_W, h = BORDER * SCALE };
            SDL2.SDL_RenderFillRect(renderer, ref divRect);

            // Bottom screen centered
            int botX = (WIN_W - DS3Constants.BOT_WIDTH * SCALE) / 2;
            int botY = (DS3Constants.TOP_HEIGHT + BORDER) * SCALE;
            var botRect = new SDL2.SDL_Rect { x = botX, y = botY, w = DS3Constants.BOT_WIDTH * SCALE, h = DS3Constants.BOT_HEIGHT * SCALE };
            SDL2.SDL_RenderCopy(renderer, botTexture, IntPtr.Zero, ref botRect);

            SDL2.SDL_RenderPresent(renderer);
        }

        // Returns bottom-screen touch coords relative to bottom FB, or null if not touching
        public (int x, int y)? GetBotTouchCoords(int mouseX, int mouseY)
        {
            int botX = (WIN_W - DS3Constants.BOT_WIDTH * SCALE) / 2;
            int botY  = (DS3Constants.TOP_HEIGHT + BORDER) * SCALE;
            int lx = mouseX - botX, ly = mouseY - botY;
            if (lx < 0 || ly < 0 || lx >= DS3Constants.BOT_WIDTH * SCALE || ly >= DS3Constants.BOT_HEIGHT * SCALE)
                return null;
            return (lx / SCALE, ly / SCALE);
        }

        public void SetTitle(string title) => SDL2.SDL_SetWindowTitle(window, $"MindWorks 3DS — {title}");

        public void Shutdown()
        {
            if (!initialized) return;
            if (topTexture != IntPtr.Zero) SDL2.SDL_DestroyTexture(topTexture);
            if (botTexture != IntPtr.Zero) SDL2.SDL_DestroyTexture(botTexture);
            if (renderer   != IntPtr.Zero) SDL2.SDL_DestroyRenderer(renderer);
            if (window     != IntPtr.Zero) SDL2.SDL_DestroyWindow(window);
            SDL2.SDL_Quit();
            initialized = false;
        }
    }

    // ============================================================
    // MAIN 3DS EMULATOR
    // ============================================================
    public class DS3Emulator : IDisposable
    {
        public DS3MemoryBus Bus = new DS3MemoryBus();
        public ARM11CPU MainCPU = new ARM11CPU();
        public ARM11CPU SubCPU  = new ARM11CPU(); // APP core (second ARM11)
        public ARMInterpreter Interpreter;
        public ARMInterpreter SubInterpreter;
        public DS3HID HID;
        public DS3LCD LCD;
        public DS3DSP DSP;
        public DS3Timer TimerCtrl;
        public DS3GPU GPU;
        public DS3HLE HLE;
        public DS3Renderer Renderer = new DS3Renderer();
        public InterruptController PIC = new InterruptController();

        DS3ROMLoader.LoadedROM loadedROM;
        bool running = false;
        Thread subCPUThread;
        CancellationTokenSource cts = new CancellationTokenSource();

        long frameCount = 0;
        Stopwatch fpsTimer = new Stopwatch();
        double fps = 0;

        public DS3Emulator()
        {
            // Wire everything up
            HID = new DS3HID { PIC = PIC };
            LCD = new DS3LCD { PIC = PIC };
            TimerCtrl = new DS3Timer { PIC = PIC };
            GPU = new DS3GPU(Bus);
            DSP = new DS3DSP(Bus);
            HLE = new DS3HLE(Bus, this);

            Bus.HID = HID;
            Bus.LCD = LCD;
            Bus.DSP = DSP;
            Bus.TimerController = TimerCtrl;
            Bus.PICA200 = GPU;
            Bus.HLE = HLE;

            Interpreter    = new ARMInterpreter(Bus, HLE);
            SubInterpreter = new ARMInterpreter(Bus, HLE);

            PIC.Counters = new PerformanceCounters();

            // Setup IRQ handlers
            PIC.RegisterHandler(DS3Constants.IRQ_VBLANK, () => { /* VBlank handled in main loop */ });
            PIC.RegisterHandler(DS3Constants.IRQ_TIMER0, () => { /* Timer0 IRQ */ });
            PIC.RegisterHandler(DS3Constants.IRQ_HID,    () => { /* HID IRQ */ });
        }

        public void LoadROM(string path)
        {
            Console.WriteLine($"\n[EMU] Loading ROM: {path}");
            loadedROM = DS3ROMLoader.Load3DS(path, Bus);
            if (!loadedROM.IsValid)
            {
                Console.WriteLine("[EMU] Failed to load ROM");
                return;
            }
            Console.WriteLine($"[EMU] Loaded: '{loadedROM.Title}' entry=0x{loadedROM.EntryPoint:X}");

            // Initialize main CPU
            MainCPU.PC = loadedROM.EntryPoint;
            MainCPU.SP = loadedROM.StackPointer;
            MainCPU.CPSR = 0x00000053; // ARM mode, SVC, IRQ disabled
            // In ARM mode if entry is ARM-aligned, Thumb if LSB=1
            if ((loadedROM.EntryPoint & 1) != 0)
            {
                MainCPU.PC = loadedROM.EntryPoint & ~1u;
                MainCPU.CPSR |= (1 << 5); // Thumb mode
            }

            // Boot ROM stubs — write a few NOP + BKPT at known firmware entry points
            // that games call expecting the firmware to be there
            WriteBoot11Stubs();
        }

        void WriteBoot11Stubs()
        {
            // Write minimal "success" stubs at common firmware addresses
            // These are called before the game proper starts
            // BX LR (return from function) in ARM: 0xE12FFF1E
            uint BX_LR = 0xE12FFF1E;
            uint THUMB_BX_LR = 0x47704770; // two thumb BX LR
            uint[] stubAddrs = { 0x10000, 0x20000, 0x30000, 0x40000, 0x50000 };
            foreach (var a in stubAddrs) Bus.Write32(a, BX_LR);
        }

        public void SpawnThread(uint entryPoint, uint arg, uint stackAddr)
        {
            var cpu = new ARM11CPU();
            cpu.PC = entryPoint & ~1u;
            cpu.SP = stackAddr;
            cpu.R[0] = arg;
            cpu.CPSR = MainCPU.CPSR;
            if ((entryPoint & 1) != 0) cpu.CPSR |= (1 << 5);

            var interp = new ARMInterpreter(Bus, HLE);
            var t = new Thread(() =>
            {
                while (cpu.Running && !cts.Token.IsCancellationRequested)
                    interp.RunCycles(cpu, 1000);
            }) { IsBackground = true, Name = $"3DS-Thread-0x{entryPoint:X}" };
            t.Start();
        }

        public void Run()
        {
            if (!loadedROM.IsValid) { Console.WriteLine("[EMU] No valid ROM loaded"); return; }

            Console.WriteLine("[EMU] Initializing SDL2...");
            bool hasDisplay = Renderer.Init(loadedROM.Title);
            if (!hasDisplay) Console.WriteLine("[EMU] No display — running headless");

            Console.WriteLine("[EMU] Initializing audio...");
            DSP.Init();

            running = true;
            fpsTimer.Start();

            Console.WriteLine("[EMU] Starting emulation loop");
            Console.WriteLine("[CONTROLS] Arrow keys=DPad, Z=A, X=B, A=Y, S=X, Q=L, W=R, Enter=Start, RShift=Select, Esc=Quit");

            const int CYCLES_PER_SLICE = DS3Constants.CYCLES_PER_FRAME / 4; // 4 slices per frame
            const int SLICES_PER_FRAME = 4;
            const double FRAME_TIME_MS = 1000.0 / DS3Constants.TARGET_FPS;

            Stopwatch frameTimer = new Stopwatch();

            while (running && !cts.Token.IsCancellationRequested)
            {
                frameTimer.Restart();

                // Process events
                if (hasDisplay) ProcessEvents();

                // Run CPU for one frame (split into slices to service timers/interrupts)
                for (int slice = 0; slice < SLICES_PER_FRAME && MainCPU.Running; slice++)
                {
                    int ran = Interpreter.RunCycles(MainCPU, CYCLES_PER_SLICE);
                    TimerCtrl.Tick(ran);
                    PIC.DispatchPending();
                }

                if (!MainCPU.Running)
                {
                    Console.WriteLine("[EMU] Main CPU halted");
                    break;
                }

                // VBlank
                LCD.VBlank();

                // Update display from VRAM
                if (hasDisplay)
                {
                    uint topOff = LCD.ActiveTopFB >= DS3Constants.VRAM_BASE ?
                        LCD.ActiveTopFB - DS3Constants.VRAM_BASE : 0;
                    uint botOff = LCD.ActiveBotFB >= DS3Constants.VRAM_BASE ?
                        LCD.ActiveBotFB - DS3Constants.VRAM_BASE : 0;
                    Renderer.Render(Bus.VRAM, topOff, botOff);
                }

                // Audio
                DSP.MixAndQueue(DS3Constants.SAMPLE_RATE / DS3Constants.TARGET_FPS);

                // FPS tracking
                frameCount++;
                if (fpsTimer.ElapsedMilliseconds >= 1000)
                {
                    fps = frameCount * 1000.0 / fpsTimer.ElapsedMilliseconds;
                    frameCount = 0;
                    fpsTimer.Restart();
                    if (hasDisplay)
                        Renderer.SetTitle($"{loadedROM.Title} | {fps:F1} FPS | {Interpreter.InstructionsExecuted:N0} instrs");
                    Console.WriteLine($"[EMU] FPS: {fps:F1} | Cycles: {Interpreter.Cycles:N0} | PC: 0x{MainCPU.PC:X8}");
                }

                // Frame rate limiter — soft cap at 60fps
                double elapsed = frameTimer.Elapsed.TotalMilliseconds;
                if (elapsed < FRAME_TIME_MS)
                    Thread.Sleep((int)(FRAME_TIME_MS - elapsed));
            }

            Cleanup();
        }

        void ProcessEvents()
        {
            while (SDL2.SDL_PollEvent(out var ev) != 0)
            {
                switch (ev.type)
                {
                    case SDL2.SDL_QUIT:
                        running = false;
                        break;

                    case SDL2.SDL_KEYDOWN:
                    {
                        int sc = ev.key.keysym.scancode;
                        if (sc == SDL2.SDL_SCANCODE_ESCAPE) { running = false; break; }
                        var btn = ScancodeTo3DSButton(sc);
                        if (btn.HasValue) HID.Press(btn.Value);
                        break;
                    }
                    case SDL2.SDL_KEYUP:
                    {
                        int sc = ev.key.keysym.scancode;
                        var btn = ScancodeTo3DSButton(sc);
                        if (btn.HasValue) HID.Release(btn.Value);
                        break;
                    }
                    case SDL2.SDL_MOUSEBUTTONDOWN:
                    {
                        var touch = Renderer.GetBotTouchCoords(ev.button.x, ev.button.y);
                        if (touch.HasValue) HID.SetTouch(true, touch.Value.x, touch.Value.y);
                        break;
                    }
                    case SDL2.SDL_MOUSEBUTTONUP:
                        HID.SetTouch(false);
                        break;
                    case SDL2.SDL_MOUSEMOTION:
                        if ((ev.motion.state & 1) != 0)
                        {
                            var touch = Renderer.GetBotTouchCoords(ev.motion.x, ev.motion.y);
                            if (touch.HasValue) HID.SetTouch(true, touch.Value.x, touch.Value.y);
                            else HID.SetTouch(false);
                        }
                        break;
                }
            }
        }

        uint? ScancodeTo3DSButton(int scancode) => scancode switch
        {
            SDL2.SDL_SCANCODE_UP     => DS3Constants.BTN_UP,
            SDL2.SDL_SCANCODE_DOWN   => DS3Constants.BTN_DOWN,
            SDL2.SDL_SCANCODE_LEFT   => DS3Constants.BTN_LEFT,
            SDL2.SDL_SCANCODE_RIGHT  => DS3Constants.BTN_RIGHT,
            SDL2.SDL_SCANCODE_Z      => DS3Constants.BTN_A,
            SDL2.SDL_SCANCODE_X      => DS3Constants.BTN_B,
            SDL2.SDL_SCANCODE_A      => DS3Constants.BTN_Y,
            SDL2.SDL_SCANCODE_S      => DS3Constants.BTN_X,
            SDL2.SDL_SCANCODE_Q      => DS3Constants.BTN_L,
            SDL2.SDL_SCANCODE_W      => DS3Constants.BTN_R,
            SDL2.SDL_SCANCODE_RETURN => DS3Constants.BTN_START,
            SDL2.SDL_SCANCODE_RSHIFT => DS3Constants.BTN_SELECT,
            _ => null
        };

        void Cleanup()
        {
            Console.WriteLine("[EMU] Shutting down...");
            Console.WriteLine($"[EMU] Total instructions executed: {Interpreter.InstructionsExecuted:N0}");
            Console.WriteLine($"[EMU] Total cycles: {Interpreter.Cycles:N0}");
            DSP.Shutdown();
            Renderer.Shutdown();
            cts.Cancel();
        }

        public void Dispose() { cts.Cancel(); Cleanup(); }
    }

    // ============================================================
    // ENTRY POINT
    // ============================================================
    public class DS3Main
    {
        public static void Main3DS(string[] args)
        {
            Console.WriteLine("==========================================================");
            Console.WriteLine("       MINDWORKS 3DS EMULATOR v1.0                       ");
            Console.WriteLine("==========================================================");
            Console.WriteLine();

            string romPath = args.Length > 0 ? args[0] : null;

            if (romPath == null)
            {
                Console.WriteLine("Usage: mindworks <rom.3ds>");
                Console.WriteLine("       mindworks <homebrew.elf>");
                Console.WriteLine();
                Console.WriteLine("Supported formats:");
                Console.WriteLine("  .3ds  — Decrypted 3DS NCSD ROM dump");
                Console.WriteLine("  .cci  — Decrypted 3DS NCCH ROM");
                Console.WriteLine("  .elf  — ARM11 ELF (homebrew)");
                Console.WriteLine();
                Console.WriteLine("NOTE: Only use ROM dumps of games you own legally.");
                Console.WriteLine("      This emulator does NOT include any Nintendo firmware.");
                Console.WriteLine("      It uses HLE (High Level Emulation) of the Horizon OS.");
                Console.WriteLine();

                // Demo mode: draw something to VRAM and show the window
                Console.WriteLine("[DEMO] Running display test (no ROM)...");
                RunDisplayTest();
                return;
            }

            using var emu = new DS3Emulator();
            emu.LoadROM(romPath);
            emu.Run();
        }

        static void RunDisplayTest()
        {
            // Fill top screen VRAM with a gradient, bottom with a test pattern
            var bus = new DS3MemoryBus();
            var lcd = new DS3LCD();
            var hid = new DS3HID();
            bus.LCD = lcd;
            bus.HID = hid;

            // Draw gradient to top framebuffer (stored as BGR columns)
            for (int x = 0; x < DS3Constants.TOP_WIDTH; x++)
            for (int y = 0; y < DS3Constants.TOP_HEIGHT; y++)
            {
                int off = (x * DS3Constants.TOP_HEIGHT + y) * 3;
                if (off + 2 < bus.VRAM.Length)
                {
                    bus.VRAM[off+0] = (byte)(y * 255 / DS3Constants.TOP_HEIGHT);       // B
                    bus.VRAM[off+1] = (byte)(x * 255 / DS3Constants.TOP_WIDTH);        // G
                    bus.VRAM[off+2] = (byte)((x + y) * 128 / (DS3Constants.TOP_WIDTH + DS3Constants.TOP_HEIGHT)); // R
                }
            }

            // Draw a simple checkerboard to bottom framebuffer
            uint botBase = DS3Constants.FB_BOT_A - DS3Constants.VRAM_BASE;
            for (int x = 0; x < DS3Constants.BOT_WIDTH; x++)
            for (int y = 0; y < DS3Constants.BOT_HEIGHT; y++)
            {
                uint off = botBase + (uint)(x * DS3Constants.BOT_HEIGHT + y) * 3;
                if (off + 2 < bus.VRAM.Length)
                {
                    bool check = ((x / 20) + (y / 20)) % 2 == 0;
                    bus.VRAM[off+0] = check ? (byte)200 : (byte)40;
                    bus.VRAM[off+1] = check ? (byte)200 : (byte)40;
                    bus.VRAM[off+2] = check ? (byte)200 : (byte)40;
                }
            }

            var renderer = new DS3Renderer();
            if (!renderer.Init("Display Test")) { Console.WriteLine("No display available"); return; }

            Console.WriteLine("[DEMO] Showing display test. Press Escape or close window to quit.");
            bool quit = false;
            int frame = 0;
            while (!quit)
            {
                while (SDL2.SDL_PollEvent(out var ev) != 0)
                {
                    if (ev.type == SDL2.SDL_QUIT) quit = true;
                    if (ev.type == SDL2.SDL_KEYDOWN && ev.key.keysym.scancode == SDL2.SDL_SCANCODE_ESCAPE) quit = true;
                }

                // Animate the gradient slightly
                frame++;
                for (int x = 0; x < DS3Constants.TOP_WIDTH; x += 4)
                for (int y = 0; y < DS3Constants.TOP_HEIGHT; y += 4)
                {
                    int off = (x * DS3Constants.TOP_HEIGHT + y) * 3;
                    if (off + 2 < bus.VRAM.Length)
                    {
                        bus.VRAM[off+2] = (byte)((Math.Sin(frame * 0.05 + x * 0.1) * 127 + 128));
                        bus.VRAM[off+1] = (byte)((Math.Cos(frame * 0.03 + y * 0.1) * 127 + 128));
                    }
                }

                renderer.Render(bus.VRAM, 0, DS3Constants.FB_BOT_A - DS3Constants.VRAM_BASE);
                Thread.Sleep(16);
            }
            renderer.Shutdown();
        }
    }
}