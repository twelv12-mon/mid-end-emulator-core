using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;

namespace mindworks
{
    // ================================================================
    // ENUMERATIONS + CONSTANTS
    // ================================================================
    public enum PrivilegeMode { User = 0, Kernel = 1, Hypervisor = 2 }
    public enum ExceptionType { PageFault = 0, IllegalInstruction = 1, DivideByZero = 2, Overflow = 3, BreakPoint = 4, Syscall = 5, GeneralProtection = 6 }
    public enum InstructionType { ALU, Load, Store, Branch, Jump, Syscall, Float, SIMD, Vector, Privileged }
    public enum Opcode : byte
    {
        NOP = 0x00,
        HALT = 0xFF,
        MOV = 0x01, MOVI = 0x02,
        ADD = 0x10, ADDI = 0x11, SUB = 0x12, SUBI = 0x13,
        MUL = 0x14, DIV = 0x15, MOD = 0x16,
        AND = 0x20, OR = 0x21, XOR = 0x22, NOT = 0x23,
        SHL = 0x24, SHR = 0x25, SAR = 0x26,
        CMP = 0x30, TEST = 0x31,
        JMP = 0x40, JE = 0x41, JNE = 0x42, JL = 0x43, JG = 0x44, JLE = 0x45, JGE = 0x46,
        CALL = 0x50, RET = 0x51, PUSH = 0x52, POP = 0x53,
        LD = 0x60, ST = 0x61, LDB = 0x62, STB = 0x63, LDH = 0x64, STH = 0x65,
        FADD = 0x70, FSUB = 0x71, FMUL = 0x72, FDIV = 0x73, FCMP = 0x74, FCVT = 0x75,
        VADD = 0x80, VSUB = 0x81, VMUL = 0x82, VDOT = 0x83,
        SYSCALL = 0x90, SYSRET = 0x91,
        INT = 0xA0, IRET = 0xA1,
        IN = 0xB0, OUT = 0xB1,
        FENCE = 0xC0, MFENCE = 0xC1,
        CACHE_INV = 0xD0, CACHE_FLUSH = 0xD1,
        PREFETCH = 0xE0,
        XCHG = 0xF0, CAS = 0xF1,   // atomics
    }

    // ================================================================
    // PERFORMANCE COUNTERS (expanded)
    // ================================================================
    public class PerformanceCounters
    {
        public long InstructionsExecuted;
        public long MemoryReads;
        public long MemoryWrites;
        public long IOReads;
        public long IOWrites;
        public long Allocations;
        public long Cycles;
        public long GPUCommands;
        public long CacheHitsL1I;
        public long CacheMissesL1I;
        public long CacheHitsL1D;
        public long CacheMissesL1D;
        public long CacheHitsL2;
        public long CacheMissesL2;
        public long TLBHits;
        public long TLBMisses;
        public long BranchPredictions;
        public long BranchMispredictions;
        public long PageFaults;
        public long ContextSwitches;
        public long Interrupts;
        public long Syscalls;
        public long StallCycles;
        public long FloatOps;
        public long SIMDOps;
        public long AtomicOps;
        public long BytecodeOptimizations;
        public long JITCompilations;
        public long MemoryAllocBytes;

        public void Print()
        {
            Console.WriteLine("============================================================");
            Console.WriteLine("                   VM PERFORMANCE STATS                    ");
            Console.WriteLine("============================================================");
            Console.WriteLine($"  Instructions Executed : {InstructionsExecuted,16:N0}");
            Console.WriteLine($"  Total Cycles          : {Cycles,16:N0}");
            Console.WriteLine($"  Stall Cycles          : {StallCycles,16:N0}");
            Console.WriteLine($"  IPC (est.)            : {(Cycles > 0 ? (double)InstructionsExecuted / Cycles : 0),16:F3}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine($"  Memory Reads          : {MemoryReads,16:N0}");
            Console.WriteLine($"  Memory Writes         : {MemoryWrites,16:N0}");
            Console.WriteLine($"  IO Reads              : {IOReads,16:N0}");
            Console.WriteLine($"  IO Writes             : {IOWrites,16:N0}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine($"  L1I Cache Hits        : {CacheHitsL1I,16:N0}");
            Console.WriteLine($"  L1I Cache Misses      : {CacheMissesL1I,16:N0}");
            double l1iRate = (CacheHitsL1I + CacheMissesL1I) > 0 ? 100.0 * CacheHitsL1I / (CacheHitsL1I + CacheMissesL1I) : 0;
            Console.WriteLine($"  L1I Hit Rate          : {l1iRate,15:F2}%");
            Console.WriteLine($"  L1D Cache Hits        : {CacheHitsL1D,16:N0}");
            Console.WriteLine($"  L1D Cache Misses      : {CacheMissesL1D,16:N0}");
            double l1dRate = (CacheHitsL1D + CacheMissesL1D) > 0 ? 100.0 * CacheHitsL1D / (CacheHitsL1D + CacheMissesL1D) : 0;
            Console.WriteLine($"  L1D Hit Rate          : {l1dRate,15:F2}%");
            Console.WriteLine($"  L2 Cache Hits         : {CacheHitsL2,16:N0}");
            Console.WriteLine($"  L2 Cache Misses       : {CacheMissesL2,16:N0}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine($"  TLB Hits              : {TLBHits,16:N0}");
            Console.WriteLine($"  TLB Misses            : {TLBMisses,16:N0}");
            Console.WriteLine($"  Page Faults           : {PageFaults,16:N0}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine($"  Branch Predictions    : {BranchPredictions,16:N0}");
            Console.WriteLine($"  Branch Mispredictions : {BranchMispredictions,16:N0}");
            double bpRate = BranchPredictions > 0 ? 100.0 * (BranchPredictions - BranchMispredictions) / BranchPredictions : 0;
            Console.WriteLine($"  Branch Accuracy       : {bpRate,15:F2}%");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine($"  Context Switches      : {ContextSwitches,16:N0}");
            Console.WriteLine($"  Interrupts Handled    : {Interrupts,16:N0}");
            Console.WriteLine($"  Syscalls              : {Syscalls,16:N0}");
            Console.WriteLine($"  Float Ops             : {FloatOps,16:N0}");
            Console.WriteLine($"  SIMD Ops              : {SIMDOps,16:N0}");
            Console.WriteLine($"  Atomic Ops            : {AtomicOps,16:N0}");
            Console.WriteLine("------------------------------------------------------------");
            Console.WriteLine($"  GPU Commands          : {GPUCommands,16:N0}");
            Console.WriteLine($"  Allocations           : {Allocations,16:N0}");
            Console.WriteLine($"  Alloc Bytes           : {MemoryAllocBytes,16:N0}");
            Console.WriteLine($"  Bytecode Optimizations: {BytecodeOptimizations,16:N0}");
            Console.WriteLine($"  JIT Compilations      : {JITCompilations,16:N0}");
            Console.WriteLine("============================================================");
        }
    }

    // ================================================================
    // FLAGS REGISTER
    // ================================================================
    [Flags]
    public enum StatusFlags : ulong
    {
        None = 0,
        Zero = 1 << 0,
        Sign = 1 << 1,
        Carry = 1 << 2,
        Overflow = 1 << 3,
        Parity = 1 << 4,
        Interrupt = 1 << 5,
        Direction = 1 << 6,
        Trap = 1 << 7,
    }

    // ================================================================
    // EXCEPTION / FAULT HANDLING
    // ================================================================
    public class VMException : Exception
    {
        public ExceptionType Type;
        public ulong FaultAddress;
        public VMException(ExceptionType t, string msg, ulong faultAddr = 0) : base(msg) { Type = t; FaultAddress = faultAddr; }
    }

    // ================================================================
    // BRANCH PREDICTOR — 2-bit saturating + BTB + RAS
    // ================================================================
    public class BranchPredictor
    {
        const int TABLE_SIZE = 4096;
        const int BTB_SIZE = 512;
        const int RAS_DEPTH = 32;

        byte[] bht = new byte[TABLE_SIZE];     // Branch History Table (2-bit)
        ulong[] gbhr = new ulong[1];           // Global Branch History Register
        ulong[] btb = new ulong[BTB_SIZE];     // Branch Target Buffer
        ulong[] ras = new ulong[RAS_DEPTH];    // Return Address Stack
        int rasTop = 0;

        int BHTIndex(ulong pc) => (int)((pc ^ gbhr[0]) & (TABLE_SIZE - 1));
        int BTBIndex(ulong pc) => (int)(pc & (BTB_SIZE - 1));

        public bool Predict(ulong pc)
        {
            int idx = BHTIndex(pc);
            return bht[idx] >= 2;
        }

        public ulong PredictTarget(ulong pc)
        {
            return btb[BTBIndex(pc)];
        }

        public void Update(ulong pc, bool taken, ulong target, PerformanceCounters counters)
        {
            int idx = BHTIndex(pc);
            bool predicted = bht[idx] >= 2;
            if (predicted != taken) Interlocked.Increment(ref counters.BranchMispredictions);
            Interlocked.Increment(ref counters.BranchPredictions);

            if (taken && bht[idx] < 3) bht[idx]++;
            else if (!taken && bht[idx] > 0) bht[idx]--;

            // Update global history
            gbhr[0] = (gbhr[0] << 1) | (taken ? 1UL : 0UL);

            if (taken) btb[BTBIndex(pc)] = target;
        }

        public void PushRAS(ulong returnAddr) { ras[rasTop % RAS_DEPTH] = returnAddr; rasTop++; }
        public ulong PopRAS() { if (rasTop == 0) return 0; rasTop--; return ras[rasTop % RAS_DEPTH]; }
    }

    // ================================================================
    // L2 CACHE (shared across cores)
    // ================================================================
    public class L2Cache
    {
        const int L2_LINES = 4096;
        const int LINE_SIZE = 64;
        struct CacheLine { public ulong Tag; public byte[] Data; public bool Valid; public bool Dirty; }
        CacheLine[] lines = new CacheLine[L2_LINES];
        readonly object l2Lock = new object();

        public L2Cache()
        {
            for (int i = 0; i < L2_LINES; i++)
                lines[i] = new CacheLine { Data = new byte[LINE_SIZE] };
        }

        public bool TryRead(ulong addr, out byte[] lineData, PerformanceCounters counters)
        {
            lock (l2Lock)
            {
                int idx = (int)((addr / LINE_SIZE) % L2_LINES);
                var line = lines[idx];
                if (line.Valid && line.Tag == addr / LINE_SIZE)
                {
                    Interlocked.Increment(ref counters.CacheHitsL2);
                    lineData = (byte[])line.Data.Clone();
                    return true;
                }
                Interlocked.Increment(ref counters.CacheMissesL2);
                lineData = null;
                return false;
            }
        }

        public void Write(ulong addr, byte[] data, PerformanceCounters counters)
        {
            lock (l2Lock)
            {
                int idx = (int)((addr / LINE_SIZE) % L2_LINES);
                lines[idx] = new CacheLine { Tag = addr / LINE_SIZE, Data = (byte[])data.Clone(), Valid = true, Dirty = true };
            }
        }
    }

    // ================================================================
    // MMU + TLB + MEMORY
    // ================================================================
    public class Page
    {
        public byte[] Data = new byte[4096];
        public bool Readable = true, Writable = true, Executable = true;
        public bool UserAccessible = true;
        public bool DirtyBit = false;
        public bool AccessedBit = false;
    }

    public class CacheLine
    {
        public ulong Tag;
        public byte[] Data = new byte[64];
        public bool Valid;
        public bool Dirty;
    }

    public class Memory
    {
        const ulong PageSize = 4096;
        const int L1_LINES = 512;
        const int TLB_SIZE = 256;

        Dictionary<ulong, Page> pages = new Dictionary<ulong, Page>();
        Dictionary<ulong, ulong> tlb = new Dictionary<ulong, ulong>();
        public CacheLine[] L1I = new CacheLine[L1_LINES];
        public CacheLine[] L1D = new CacheLine[L1_LINES];
        readonly object memLock = new object();
        public L2Cache L2;
        public PerformanceCounters Counters;

        public Memory(L2Cache l2, PerformanceCounters counters)
        {
            L2 = l2;
            Counters = counters;
            for (int i = 0; i < L1_LINES; i++) { L1I[i] = new CacheLine { Data = new byte[64] }; L1D[i] = new CacheLine { Data = new byte[64] }; }
        }

        public ulong Translate(ulong vaddr, bool write = false, bool exec = false, PrivilegeMode mode = PrivilegeMode.User)
        {
            ulong vpn = vaddr >> 12;
            if (tlb.TryGetValue(vpn, out ulong ppn))
            {
                Interlocked.Increment(ref Counters.TLBHits);
                // Check page permissions
                if (pages.TryGetValue(ppn, out var p))
                {
                    if (write && !p.Writable) throw new VMException(ExceptionType.PageFault, $"Write to read-only page at {vaddr:X}");
                    if (exec && !p.Executable) throw new VMException(ExceptionType.PageFault, $"Execute from non-exec page at {vaddr:X}");
                    if (mode == PrivilegeMode.User && !p.UserAccessible) throw new VMException(ExceptionType.PageFault, $"User access to kernel page at {vaddr:X}");
                    p.AccessedBit = true;
                    if (write) p.DirtyBit = true;
                }
                return (ppn << 12) | (vaddr & 0xFFF);
            }
            Interlocked.Increment(ref Counters.TLBMisses);
            // Page table walk
            if (!pages.ContainsKey(vpn))
            {
                Interlocked.Increment(ref Counters.PageFaults);
                // Demand paging: allocate page
                var newPage = new Page();
                pages[vpn] = newPage;
            }
            if (tlb.Count >= TLB_SIZE)
            {
                // LRU eviction (simplified: clear oldest quarter)
                var keys = tlb.Keys.Take(TLB_SIZE / 4).ToList();
                foreach (var k in keys) tlb.Remove(k);
            }
            tlb[vpn] = vpn;
            return vaddr;
        }

        Page ResolvePage(ulong paddr)
        {
            ulong vpn = paddr >> 12;
            if (!pages.TryGetValue(vpn, out var page)) { page = new Page(); pages[vpn] = page; }
            return page;
        }

        byte L1Read(CacheLine[] cache, ulong addr, bool isInstr)
        {
            ulong lineAddr = addr >> 6;
            int idx = (int)(lineAddr % L1_LINES);
            var line = cache[idx];
            if (line.Valid && line.Tag == lineAddr)
            {
                if (isInstr) Interlocked.Increment(ref Counters.CacheHitsL1I);
                else Interlocked.Increment(ref Counters.CacheHitsL1D);
                return line.Data[addr & 63];
            }
            // L1 miss — check L2
            if (isInstr) Interlocked.Increment(ref Counters.CacheMissesL1I);
            else Interlocked.Increment(ref Counters.CacheMissesL1D);

            ulong baseAddr = addr & ~63UL;
            if (L2 != null && L2.TryRead(baseAddr, out var l2Data, Counters))
            {
                line.Data = l2Data;
            }
            else
            {
                // Main memory fill
                var page = ResolvePage(baseAddr);
                for (int i = 0; i < 64; i++)
                    line.Data[i] = page.Data[(baseAddr + (ulong)i) & 0xFFF];
                L2?.Write(baseAddr, line.Data, Counters);
            }
            line.Tag = lineAddr;
            line.Valid = true;
            return line.Data[addr & 63];
        }

        void L1Write(ulong addr, byte value)
        {
            ulong lineAddr = addr >> 6;
            int idx = (int)(lineAddr % L1_LINES);
            var line = L1D[idx];
            if (line.Valid && line.Tag == lineAddr)
            {
                line.Data[addr & 63] = value;
                line.Dirty = true;
            }
        }

        public byte Read(ulong addr)
        {
            lock (memLock)
            {
                Interlocked.Increment(ref Counters.MemoryReads);
                ulong paddr = Translate(addr);
                return L1Read(L1D, paddr, false);
            }
        }

        public void Write(ulong addr, byte value)
        {
            lock (memLock)
            {
                Interlocked.Increment(ref Counters.MemoryWrites);
                ulong paddr = Translate(addr, write: true);
                var page = ResolvePage(paddr);
                page.Data[paddr & 0xFFF] = value;
                L1Write(paddr, value);
            }
        }

        public ulong ReadU64(ulong addr)
        {
            ulong v = 0;
            for (int i = 0; i < 8; i++) v |= (ulong)Read(addr + (ulong)i) << (i * 8);
            return v;
        }

        public void WriteU64(ulong addr, ulong value)
        {
            for (int i = 0; i < 8; i++) Write(addr + (ulong)i, (byte)(value >> (i * 8)));
        }

        public uint ReadU32(ulong addr)
        {
            uint v = 0;
            for (int i = 0; i < 4; i++) v |= (uint)Read(addr + (ulong)i) << (i * 8);
            return v;
        }

        public void WriteU32(ulong addr, uint value)
        {
            for (int i = 0; i < 4; i++) Write(addr + (ulong)i, (byte)(value >> (i * 8)));
        }

        public byte FetchInstruction(ulong addr)
        {
            lock (memLock)
            {
                ulong paddr = Translate(addr, exec: true);
                return L1Read(L1I, paddr, true);
            }
        }

        public void LoadSegment(byte[] data, ulong vaddr)
        {
            for (ulong i = 0; i < (ulong)data.Length; i++) Write(vaddr + i, data[i]);
        }

        public void InvalidateL1()
        {
            for (int i = 0; i < L1_LINES; i++) { L1I[i].Valid = false; L1D[i].Valid = false; }
        }

        public void FlushDirtyL1()
        {
            for (int i = 0; i < L1_LINES; i++)
            {
                var line = L1D[i];
                if (line.Valid && line.Dirty)
                {
                    ulong baseAddr = line.Tag << 6;
                    var page = ResolvePage(baseAddr);
                    for (int j = 0; j < 64; j++) page.Data[(baseAddr + (ulong)j) & 0xFFF] = line.Data[j];
                    line.Dirty = false;
                }
            }
        }

        public byte[] DumpRange(ulong addr, ulong length)
        {
            var result = new byte[length];
            for (ulong i = 0; i < length; i++) result[i] = Read(addr + i);
            return result;
        }
    }

    // ================================================================
    // INTERRUPT CONTROLLER (PIC — Programmable Interrupt Controller)
    // ================================================================
    public class InterruptController
    {
        const int IRQ_COUNT = 256;
        Action[] handlers = new Action[IRQ_COUNT];
        byte[] priority = new byte[IRQ_COUNT];
        bool[] masked = new bool[IRQ_COUNT];
        Queue<int> pending = new Queue<int>();
        readonly object picLock = new object();
        public PerformanceCounters Counters;

        public void RegisterHandler(int irq, Action handler, byte prio = 128)
        {
            if (irq < 0 || irq >= IRQ_COUNT) throw new ArgumentException("Invalid IRQ");
            handlers[irq] = handler;
            priority[irq] = prio;
        }

        public void MaskIRQ(int irq) { lock (picLock) masked[irq] = true; }
        public void UnmaskIRQ(int irq) { lock (picLock) masked[irq] = false; }

        public void Raise(int irq)
        {
            lock (picLock)
            {
                if (!masked[irq]) pending.Enqueue(irq);
            }
        }

        // Returns true if an interrupt was dispatched
        public bool DispatchPending()
        {
            lock (picLock)
            {
                if (pending.Count == 0) return false;
                // Sort by priority
                var sorted = pending.OrderByDescending(i => priority[i]).ToList();
                pending.Clear();
                // Re-enqueue lower priority ones after handling highest
                int irq = sorted[0];
                for (int i = 1; i < sorted.Count; i++) pending.Enqueue(sorted[i]);
                if (handlers[irq] != null)
                {
                    Interlocked.Increment(ref Counters.Interrupts);
                    handlers[irq]();
                }
                return true;
            }
        }
    }

    // ================================================================
    // DEVICES
    // ================================================================
    public interface IMemoryRegion
    {
        ulong Start { get; }
        ulong End { get; }
        byte Read(ulong addr);
        void Write(ulong addr, byte value);
    }

    public class TimerDevice : IMemoryRegion
    {
        public ulong Counter;
        public ulong Start => 0xFFFF0000;
        public ulong End => 0xFFFF0010;
        public InterruptController PIC;
        public int IRQ = 0;
        ulong interval = 1000;

        public byte Read(ulong a)
        {
            int offset = (int)(a - Start);
            return (byte)((Counter >> (offset * 8)) & 0xFF);
        }

        public void Write(ulong a, byte v)
        {
            int offset = (int)(a - Start);
            if (offset == 0) { Counter = 0; interval = v == 0 ? 1000 : (ulong)v * 100; }
        }

        public void Tick()
        {
            Counter++;
            if (Counter % interval == 0) PIC?.Raise(IRQ);
        }
    }

    public class UartDevice : IMemoryRegion
    {
        public ulong Start => 0xFFFF1000;
        public ulong End => 0xFFFF1010;
        public InterruptController PIC;
        public int IRQ = 4;
        Queue<byte> rxBuffer = new Queue<byte>();
        readonly object uartLock = new object();

        public byte Read(ulong a)
        {
            int offset = (int)(a - Start);
            if (offset == 0) { lock (uartLock) { return rxBuffer.Count > 0 ? rxBuffer.Dequeue() : (byte)0; } }
            if (offset == 1) { lock (uartLock) return (byte)(rxBuffer.Count > 0 ? 1 : 0); } // status
            return 0;
        }

        public void Write(ulong a, byte v)
        {
            int offset = (int)(a - Start);
            if (offset == 0) { Console.Write((char)v); PIC?.Raise(IRQ); }
        }

        public void InjectRx(byte data) { lock (uartLock) { rxBuffer.Enqueue(data); PIC?.Raise(IRQ); } }
    }

    public class DMADevice : IMemoryRegion
    {
        public ulong Start => 0xFFFF2000;
        public ulong End => 0xFFFF2020;
        Memory Mem;
        ulong srcAddr, dstAddr, length;

        public DMADevice(Memory mem) { Mem = mem; }
        public byte Read(ulong a) => 0;
        public void Write(ulong a, byte v)
        {
            int offset = (int)(a - Start);
            switch (offset)
            {
                case 0: srcAddr = (srcAddr & ~0xFFUL) | v; break;
                case 1: srcAddr = (srcAddr & ~0xFF00UL) | ((ulong)v << 8); break;
                case 8: dstAddr = (dstAddr & ~0xFFUL) | v; break;
                case 9: dstAddr = (dstAddr & ~0xFF00UL) | ((ulong)v << 8); break;
                case 16: length = (length & ~0xFFUL) | v; break;
                case 24: // trigger
                    if (v == 1) Transfer();
                    break;
            }
        }

        void Transfer()
        {
            for (ulong i = 0; i < length; i++)
                Mem.Write(dstAddr + i, Mem.Read(srcAddr + i));
            Console.WriteLine($"[DMA] Transferred {length} bytes from {srcAddr:X} to {dstAddr:X}");
        }
    }

    // ================================================================
    // GPU — FRAMEBUFFER + SHADER PIPELINE + COMMAND QUEUE
    // ================================================================
    public class ShaderProgram
    {
        public string Name;
        public string Source;
        public Dictionary<string, ulong> Uniforms = new Dictionary<string, ulong>();
    }

    public class GPUDevice
    {
        public byte[,] Framebuffer = new byte[1024, 1024];
        public int Layers = 8;
        public int Width = 1024, Height = 1024;
        ConcurrentQueue<Func<ulong>> cmdQueue = new ConcurrentQueue<Func<ulong>>();
        Dictionary<string, ShaderProgram> shaderCache = new Dictionary<string, ShaderProgram>();
        public PerformanceCounters Counters;

        // VRAM
        byte[] vram = new byte[64 * 1024 * 1024]; // 64MB VRAM

        public void Enqueue(Func<ulong> shader) => cmdQueue.Enqueue(shader);

        public void ExecuteQueue()
        {
            while (cmdQueue.TryDequeue(out var shader))
            {
                ulong result = shader();
                Interlocked.Increment(ref Counters.GPUCommands);
            }
        }

        public void ClearFramebuffer(byte r = 0, byte g = 0, byte b = 0)
        {
            for (int y = 0; y < Height; y++)
                for (int x = 0; x < Width; x++)
                { Framebuffer[y, x * 3 + 0] = r; }
        }

        public void DrawPixel(int x, int y, byte r, byte g, byte b)
        {
            if (x < 0 || x >= Width || y < 0 || y >= Height) return;
            Framebuffer[y, x] = r; // simplified
        }

        public void DrawRect(int x, int y, int w, int h, byte r, byte g, byte b)
        {
            for (int dy = 0; dy < h; dy++)
                for (int dx = 0; dx < w; dx++)
                    DrawPixel(x + dx, y + dy, r, g, b);
        }

        public void RegisterShader(string name, string source)
        {
            shaderCache[name] = new ShaderProgram { Name = name, Source = source };
        }

        public ulong RunNamedShader(string name, ulong input)
        {
            if (!shaderCache.TryGetValue(name, out var prog)) return 0;
            return RunShaderSource(prog.Source, input, prog.Uniforms);
        }

        public ulong RunShaderSource(string code, ulong input, Dictionary<string, ulong> uniforms = null)
        {
            ulong val = input;
            float fval = BitConverter.ToSingle(BitConverter.GetBytes((uint)input), 0);

            foreach (var rawLine in code.Split('\n'))
            {
                var line = rawLine.Trim();
                if (string.IsNullOrEmpty(line) || line.StartsWith("//")) continue;
                var parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0) continue;
                switch (parts[0].ToUpper())
                {
                    case "ADD": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong addV)) val += addV; break;
                    case "SUB": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong subV)) val -= subV; break;
                    case "MUL": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong mulV)) val *= mulV; break;
                    case "DIV": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong divV) && divV != 0) val /= divV; break;
                    case "AND": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong andV)) val &= andV; break;
                    case "OR": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong orV)) val |= orV; break;
                    case "XOR": if (parts.Length > 1 && ulong.TryParse(parts[1], out ulong xorV)) val ^= xorV; break;
                    case "SHL": if (parts.Length > 1 && int.TryParse(parts[1], out int shlV)) val <<= shlV; break;
                    case "SHR": if (parts.Length > 1 && int.TryParse(parts[1], out int shrV)) val >>= shrV; break;
                    case "FADD": if (parts.Length > 1 && float.TryParse(parts[1], out float faddV)) { fval += faddV; Interlocked.Increment(ref Counters.FloatOps); } break;
                    case "FMUL": if (parts.Length > 1 && float.TryParse(parts[1], out float fmulV)) { fval *= fmulV; Interlocked.Increment(ref Counters.FloatOps); } break;
                    case "FSQRT": fval = MathF.Sqrt(fval); Interlocked.Increment(ref Counters.FloatOps); break;
                    case "FSIN": fval = MathF.Sin(fval); Interlocked.Increment(ref Counters.FloatOps); break;
                    case "FCOS": fval = MathF.Cos(fval); Interlocked.Increment(ref Counters.FloatOps); break;
                    case "FPOW": if (parts.Length > 1 && float.TryParse(parts[1], out float fpowV)) { fval = MathF.Pow(fval, fpowV); Interlocked.Increment(ref Counters.FloatOps); } break;
                    case "FTOI": val = (ulong)(uint)(int)fval; break;
                    case "ITOF": fval = (float)(long)val; break;
                    case "UNIFORM":
                        if (parts.Length > 1 && uniforms != null && uniforms.TryGetValue(parts[1], out ulong uval)) val = uval;
                        break;
                    case "OUT": Console.WriteLine($"[GPU SHADER] Output: {val:X}"); break;
                }
            }
            return val;
        }
    }

    // ================================================================
    // VIRTUAL FILESYSTEM (VFS)
    // ================================================================
    public class VFSNode
    {
        public string Name;
        public bool IsDirectory;
        public byte[] Data;
        public Dictionary<string, VFSNode> Children = new Dictionary<string, VFSNode>();
        public DateTime CreatedAt = DateTime.Now;
        public DateTime ModifiedAt = DateTime.Now;
        public ulong Permissions = 0b110_100_100; // rw-r--r--
    }

    public class VirtualFileSystem
    {
        VFSNode root = new VFSNode { Name = "/", IsDirectory = true };
        Dictionary<int, VFSNode> fileDescriptors = new Dictionary<int, VFSNode>();
        int nextFD = 3; // 0=stdin, 1=stdout, 2=stderr
        readonly object vfsLock = new object();

        VFSNode Navigate(string path, bool createDirs = false)
        {
            if (path == "/") return root;
            var parts = path.TrimStart('/').Split('/');
            var node = root;
            for (int i = 0; i < parts.Length - 1; i++)
            {
                if (!node.Children.TryGetValue(parts[i], out var child))
                {
                    if (!createDirs) return null;
                    child = new VFSNode { Name = parts[i], IsDirectory = true };
                    node.Children[parts[i]] = child;
                }
                node = child;
            }
            return node;
        }

        string FileName(string path) => path.TrimStart('/').Split('/').Last();

        public void Mkdir(string path)
        {
            lock (vfsLock)
            {
                var parent = Navigate(path, true);
                string name = FileName(path);
                if (!parent.Children.ContainsKey(name))
                    parent.Children[name] = new VFSNode { Name = name, IsDirectory = true };
            }
        }

        public void WriteFile(string path, byte[] data)
        {
            lock (vfsLock)
            {
                var parent = Navigate(path, true);
                string name = FileName(path);
                if (!parent.Children.TryGetValue(name, out var file))
                { file = new VFSNode { Name = name }; parent.Children[name] = file; }
                file.Data = (byte[])data.Clone();
                file.ModifiedAt = DateTime.Now;
            }
        }

        public void WriteText(string path, string text) => WriteFile(path, Encoding.UTF8.GetBytes(text));

        public byte[] ReadFile(string path)
        {
            lock (vfsLock)
            {
                var parent = Navigate(path);
                if (parent == null) return null;
                string name = FileName(path);
                return parent.Children.TryGetValue(name, out var file) ? (byte[])file.Data?.Clone() : null;
            }
        }

        public string ReadText(string path)
        {
            var data = ReadFile(path);
            return data != null ? Encoding.UTF8.GetString(data) : null;
        }

        public bool Delete(string path)
        {
            lock (vfsLock)
            {
                var parent = Navigate(path);
                string name = FileName(path);
                return parent?.Children.Remove(name) ?? false;
            }
        }

        public List<string> ListDir(string path)
        {
            lock (vfsLock)
            {
                var parent = Navigate(path);
                if (parent == null) return null;
                string name = FileName(path);
                var dir = name == "" ? parent : (parent.Children.TryGetValue(name, out var d) ? d : null);
                return dir?.Children.Keys.ToList();
            }
        }

        public int Open(string path)
        {
            lock (vfsLock)
            {
                var parent = Navigate(path);
                string name = FileName(path);
                if (parent != null && parent.Children.TryGetValue(name, out var file))
                {
                    fileDescriptors[nextFD] = file;
                    return nextFD++;
                }
                return -1;
            }
        }

        public void Close(int fd) { lock (vfsLock) fileDescriptors.Remove(fd); }
        public bool Exists(string path)
        {
            var parent = Navigate(path);
            return parent?.Children.ContainsKey(FileName(path)) ?? false;
        }

        public void PrintTree(string indent = "", VFSNode node = null)
        {
            node ??= root;
            Console.WriteLine($"{indent}{(node.IsDirectory ? "[DIR]" : "[FILE]")} {node.Name}" + (node.Data != null ? $" ({node.Data.Length} bytes)" : ""));
            foreach (var child in node.Children.Values) PrintTree(indent + "  ", child);
        }
    }

    // ================================================================
    // ASSEMBLER — TEXT -> BYTECODE
    // ================================================================
    public class AssemblerSymbol { public ulong Address; public bool IsLabel; }

    public class Assembler
    {
        Dictionary<string, AssemblerSymbol> symbols = new Dictionary<string, AssemblerSymbol>();
        Dictionary<string, ulong> macros = new Dictionary<string, ulong>();
        List<(int offset, string label)> patchList = new List<(int, string)>();

        public byte[] Assemble(string source)
        {
            var output = new List<byte>();
            ulong currentAddr = 0;
            var lines = source.Split('\n');

            // Pass 1: collect labels and equates
            ulong addr = 0;
            foreach (var rawLine in lines)
            {
                var line = StripComment(rawLine).Trim();
                if (string.IsNullOrEmpty(line)) continue;
                if (line.StartsWith(".equ"))
                {
                    var parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 3 && ulong.TryParse(parts[2].Replace("0x", ""), System.Globalization.NumberStyles.HexNumber, null, out ulong val))
                        macros[parts[1]] = val;
                    continue;
                }
                if (line.EndsWith(":"))
                {
                    symbols[line.TrimEnd(':')] = new AssemblerSymbol { Address = addr, IsLabel = true };
                    continue;
                }
                addr += InstructionSize(line);
            }

            // Pass 2: emit bytes
            foreach (var rawLine in lines)
            {
                var line = StripComment(rawLine).Trim();
                if (string.IsNullOrEmpty(line) || line.EndsWith(":") || line.StartsWith(".equ")) continue;
                if (line.StartsWith(".org"))
                {
                    var parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 2) ulong.TryParse(parts[1].Replace("0x", ""), System.Globalization.NumberStyles.HexNumber, null, out currentAddr);
                    continue;
                }
                if (line.StartsWith(".db"))
                {
                    var parts = line.Substring(3).Split(',');
                    foreach (var p in parts)
                    {
                        var t = p.Trim();
                        if (t.StartsWith("\""))
                        {
                            var str = t.Trim('"');
                            foreach (var c in str) output.Add((byte)c);
                            output.Add(0); // null terminator
                        }
                        else if (byte.TryParse(t, out byte dbv)) output.Add(dbv);
                    }
                    continue;
                }
                EmitInstruction(output, line, currentAddr, patchList);
                currentAddr += (ulong)output.Count;
            }

            // Pass 3: patch labels
            foreach (var (offset, label) in patchList)
            {
                if (symbols.TryGetValue(label, out var sym))
                {
                    ulong target = sym.Address;
                    if (offset + 8 <= output.Count)
                        for (int i = 0; i < 8; i++) output[offset + i] = (byte)(target >> (i * 8));
                }
            }

            return output.ToArray();
        }

        string StripComment(string line)
        {
            int idx = line.IndexOf(';');
            return idx >= 0 ? line.Substring(0, idx) : line;
        }

        ulong InstructionSize(string line)
        {
            if (string.IsNullOrEmpty(line)) return 0;
            var parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 0) return 0;
            return parts[0].ToUpper() switch
            {
                "NOP" => 1,
                "HALT" => 1,
                "RET" => 1,
                "FENCE" or "MFENCE" => 1,
                _ => 10 // opcode + operands
            };
        }

        ulong Resolve(string token, Dictionary<string, ulong> macros, Dictionary<string, AssemblerSymbol> symbols)
        {
            token = token.Trim();
            if (token.StartsWith("0x") && ulong.TryParse(token.Substring(2), System.Globalization.NumberStyles.HexNumber, null, out ulong hexV)) return hexV;
            if (ulong.TryParse(token, out ulong decV)) return decV;
            if (macros.TryGetValue(token, out ulong mv)) return mv;
            if (symbols.TryGetValue(token, out var sym)) return sym.Address;
            // register reference: r0..r31
            if (token.StartsWith("r") && int.TryParse(token.Substring(1), out int regIdx)) return (ulong)regIdx;
            return 0;
        }

        void Emit(List<byte> output, byte opcode, ulong rd = 0, ulong rs1 = 0, ulong rs2 = 0, ulong imm = 0)
        {
            output.Add(opcode);
            output.Add((byte)rd);
            output.Add((byte)rs1);
            output.Add((byte)rs2);
            for (int i = 0; i < 6; i++) output.Add((byte)(imm >> (i * 8)));
        }

        void EmitInstruction(List<byte> output, string line, ulong pc, List<(int, string)> patchList)
        {
            var parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 0) return;
            string mnemonic = parts[0].ToUpper();

            ulong ParseOp(int idx) => parts.Length > idx ? Resolve(parts[idx], macros, symbols) : 0;

            switch (mnemonic)
            {
                case "NOP": output.Add((byte)Opcode.NOP); return;
                case "HALT": output.Add((byte)Opcode.HALT); return;
                case "RET": output.Add((byte)Opcode.RET); return;
                case "FENCE": output.Add((byte)Opcode.FENCE); return;
                case "MFENCE": output.Add((byte)Opcode.MFENCE); return;
                case "MOV": Emit(output, (byte)Opcode.MOV, ParseOp(1), ParseOp(2)); return;
                case "MOVI": Emit(output, (byte)Opcode.MOVI, ParseOp(1), 0, 0, ParseOp(2)); return;
                case "ADD": Emit(output, (byte)Opcode.ADD, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "ADDI": Emit(output, (byte)Opcode.ADDI, ParseOp(1), ParseOp(2), 0, ParseOp(3)); return;
                case "SUB": Emit(output, (byte)Opcode.SUB, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "MUL": Emit(output, (byte)Opcode.MUL, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "DIV": Emit(output, (byte)Opcode.DIV, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "MOD": Emit(output, (byte)Opcode.MOD, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "AND": Emit(output, (byte)Opcode.AND, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "OR": Emit(output, (byte)Opcode.OR, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "XOR": Emit(output, (byte)Opcode.XOR, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "NOT": Emit(output, (byte)Opcode.NOT, ParseOp(1), ParseOp(2)); return;
                case "SHL": Emit(output, (byte)Opcode.SHL, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "SHR": Emit(output, (byte)Opcode.SHR, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "CMP": Emit(output, (byte)Opcode.CMP, 0, ParseOp(1), ParseOp(2)); return;
                case "LD": Emit(output, (byte)Opcode.LD, ParseOp(1), ParseOp(2)); return;
                case "ST": Emit(output, (byte)Opcode.ST, 0, ParseOp(1), 0, ParseOp(2)); return;
                case "LDB": Emit(output, (byte)Opcode.LDB, ParseOp(1), ParseOp(2)); return;
                case "STB": Emit(output, (byte)Opcode.STB, 0, ParseOp(1), 0, ParseOp(2)); return;
                case "PUSH": Emit(output, (byte)Opcode.PUSH, 0, ParseOp(1)); return;
                case "POP": Emit(output, (byte)Opcode.POP, ParseOp(1)); return;
                case "CALL":
                    int callOffset = output.Count + 4;
                    string callLabel = parts.Length > 1 ? parts[1] : "";
                    Emit(output, (byte)Opcode.CALL, 0, 0, 0, ParseOp(1));
                    if (!ulong.TryParse(callLabel, out _) && symbols.ContainsKey(callLabel))
                        patchList.Add((callOffset, callLabel));
                    return;
                case "JMP":
                    int jmpOffset = output.Count + 4;
                    string jmpLabel = parts.Length > 1 ? parts[1] : "";
                    Emit(output, (byte)Opcode.JMP, 0, 0, 0, ParseOp(1));
                    if (!ulong.TryParse(jmpLabel, out _) && !jmpLabel.StartsWith("0x"))
                        patchList.Add((jmpOffset, jmpLabel));
                    return;
                case "JE": Emit(output, (byte)Opcode.JE, 0, 0, 0, ParseOp(1)); return;
                case "JNE": Emit(output, (byte)Opcode.JNE, 0, 0, 0, ParseOp(1)); return;
                case "JL": Emit(output, (byte)Opcode.JL, 0, 0, 0, ParseOp(1)); return;
                case "JG": Emit(output, (byte)Opcode.JG, 0, 0, 0, ParseOp(1)); return;
                case "JLE": Emit(output, (byte)Opcode.JLE, 0, 0, 0, ParseOp(1)); return;
                case "JGE": Emit(output, (byte)Opcode.JGE, 0, 0, 0, ParseOp(1)); return;
                case "SYSCALL": Emit(output, (byte)Opcode.SYSCALL); return;
                case "INT": Emit(output, (byte)Opcode.INT, 0, 0, 0, ParseOp(1)); return;
                case "FADD": Emit(output, (byte)Opcode.FADD, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "FSUB": Emit(output, (byte)Opcode.FSUB, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "FMUL": Emit(output, (byte)Opcode.FMUL, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "FDIV": Emit(output, (byte)Opcode.FDIV, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "VADD": Emit(output, (byte)Opcode.VADD, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "VMUL": Emit(output, (byte)Opcode.VMUL, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "XCHG": Emit(output, (byte)Opcode.XCHG, ParseOp(1), ParseOp(2)); return;
                case "CAS": Emit(output, (byte)Opcode.CAS, ParseOp(1), ParseOp(2), ParseOp(3)); return;
                case "IN": Emit(output, (byte)Opcode.IN, ParseOp(1), 0, 0, ParseOp(2)); return;
                case "OUT": Emit(output, (byte)Opcode.OUT, 0, ParseOp(1), 0, ParseOp(2)); return;
                default:
                    Console.WriteLine($"[ASM] Unknown mnemonic: {mnemonic}");
                    output.Add(0); // emit NOP for unknown
                    return;
            }
        }

        public void PrintSymbolTable()
        {
            Console.WriteLine("==== SYMBOL TABLE ====");
            foreach (var kv in symbols.OrderBy(x => x.Value.Address))
                Console.WriteLine($"  {kv.Key,-30} 0x{kv.Value.Address:X8}");
        }
    }

    // ================================================================
    // IR NODE (Intermediate Representation for Compiler)
    // ================================================================
    public enum IROpcode { Mov, Add, Sub, Mul, Div, And, Or, Xor, Not, Shl, Shr, Cmp, Jmp, Je, Jne, Jl, Jg, Call, Ret, Load, Store, Phi, Label, Nop, Halt }
    public class IRNode
    {
        public IROpcode Op;
        public string Dest;
        public string Src1;
        public string Src2;
        public ulong Imm;
        public string Label;
        public List<IRNode> Successors = new List<IRNode>();
        public override string ToString() => $"{Op} {Dest ?? "_"}, {Src1 ?? "_"}, {Src2 ?? ""}  (imm={Imm}, lbl={Label})";
    }

    // ================================================================
    // COMPILER PIPELINE: SOURCE -> TOKENS -> AST -> IR -> BYTECODE
    // ================================================================
    public enum TokenKind { Number, Ident, Plus, Minus, Star, Slash, Percent, Eq, EqEq, NotEq, Lt, Gt, Le, Ge, LParen, RParen, LBrace, RBrace, Semicolon, Comma, Amp, Pipe, Caret, Tilde, Bang, Shl, Shr, Arrow, If, Else, While, For, Return, Let, Fn, EOF }
    public class Token { public TokenKind Kind; public string Text; public ulong NumVal; }

    public class Lexer
    {
        string src; int pos;
        public Lexer(string source) { src = source; pos = 0; }

        bool IsAlpha(char c) => char.IsLetter(c) || c == '_';
        bool IsAlNum(char c) => char.IsLetterOrDigit(c) || c == '_';

        public List<Token> Tokenize()
        {
            var tokens = new List<Token>();
            while (pos < src.Length)
            {
                while (pos < src.Length && char.IsWhiteSpace(src[pos])) pos++;
                if (pos >= src.Length) break;

                char c = src[pos];
                if (src[pos..].StartsWith("//")) { while (pos < src.Length && src[pos] != '\n') pos++; continue; }
                if (src[pos..].StartsWith("/*")) { while (pos < src.Length - 1 && !(src[pos] == '*' && src[pos + 1] == '/')) pos++; pos += 2; continue; }

                if (char.IsDigit(c))
                {
                    int start = pos;
                    bool hex = c == '0' && pos + 1 < src.Length && src[pos + 1] == 'x';
                    if (hex) pos += 2;
                    while (pos < src.Length && (char.IsDigit(src[pos]) || (hex && "abcdefABCDEF".Contains(src[pos])))) pos++;
                    string numStr = src[start..pos];
                    ulong val = hex ? Convert.ToUInt64(numStr.Substring(2), 16) : ulong.Parse(numStr);
                    tokens.Add(new Token { Kind = TokenKind.Number, Text = numStr, NumVal = val });
                    continue;
                }
                if (IsAlpha(c))
                {
                    int start = pos;
                    while (pos < src.Length && IsAlNum(src[pos])) pos++;
                    string word = src[start..pos];
                    var kind = word switch { "if" => TokenKind.If, "else" => TokenKind.Else, "while" => TokenKind.While, "for" => TokenKind.For, "return" => TokenKind.Return, "let" => TokenKind.Let, "fn" => TokenKind.Fn, _ => TokenKind.Ident };
                    tokens.Add(new Token { Kind = kind, Text = word });
                    continue;
                }
                // Multi-char operators
                string twoChar = pos + 1 < src.Length ? src[pos..(pos + 2)] : "";
                switch (twoChar)
                {
                    case "==": tokens.Add(new Token { Kind = TokenKind.EqEq, Text = "==" }); pos += 2; continue;
                    case "!=": tokens.Add(new Token { Kind = TokenKind.NotEq, Text = "!=" }); pos += 2; continue;
                    case "<=": tokens.Add(new Token { Kind = TokenKind.Le, Text = "<=" }); pos += 2; continue;
                    case ">=": tokens.Add(new Token { Kind = TokenKind.Ge, Text = ">=" }); pos += 2; continue;
                    case "<<": tokens.Add(new Token { Kind = TokenKind.Shl, Text = "<<" }); pos += 2; continue;
                    case ">>": tokens.Add(new Token { Kind = TokenKind.Shr, Text = ">>" }); pos += 2; continue;
                    case "->": tokens.Add(new Token { Kind = TokenKind.Arrow, Text = "->" }); pos += 2; continue;
                }
                var singleKind = c switch
                {
                    '+' => TokenKind.Plus, '-' => TokenKind.Minus, '*' => TokenKind.Star, '/' => TokenKind.Slash, '%' => TokenKind.Percent,
                    '=' => TokenKind.Eq, '<' => TokenKind.Lt, '>' => TokenKind.Gt, '(' => TokenKind.LParen, ')' => TokenKind.RParen,
                    '{' => TokenKind.LBrace, '}' => TokenKind.RBrace, ';' => TokenKind.Semicolon, ',' => TokenKind.Comma,
                    '&' => TokenKind.Amp, '|' => TokenKind.Pipe, '^' => TokenKind.Caret, '~' => TokenKind.Tilde, '!' => TokenKind.Bang,
                    _ => TokenKind.EOF
                };
                tokens.Add(new Token { Kind = singleKind, Text = c.ToString() });
                pos++;
            }
            tokens.Add(new Token { Kind = TokenKind.EOF, Text = "" });
            return tokens;
        }
    }

    // Simple expression/statement AST
    public abstract class ASTNode { }
    public class NumberNode : ASTNode { public ulong Value; }
    public class IdentNode : ASTNode { public string Name; }
    public class BinOpNode : ASTNode { public string Op; public ASTNode Left, Right; }
    public class UnOpNode : ASTNode { public string Op; public ASTNode Operand; }
    public class AssignNode : ASTNode { public string Name; public ASTNode Value; }
    public class LetNode : ASTNode { public string Name; public ASTNode Init; }
    public class IfNode : ASTNode { public ASTNode Cond; public List<ASTNode> Then, Else; }
    public class WhileNode : ASTNode { public ASTNode Cond; public List<ASTNode> Body; }
    public class ReturnNode : ASTNode { public ASTNode Value; }
    public class CallNode : ASTNode { public string Name; public List<ASTNode> Args; }
    public class FnNode : ASTNode { public string Name; public List<string> Params; public List<ASTNode> Body; }
    public class BlockNode : ASTNode { public List<ASTNode> Stmts; }

    public class Parser
    {
        List<Token> tokens; int pos;
        Token Cur => tokens[pos];
        Token Eat() => tokens[pos++];
        Token Expect(TokenKind k) { if (Cur.Kind != k) throw new Exception($"Expected {k} got {Cur.Kind} ('{Cur.Text}')"); return Eat(); }

        public Parser(List<Token> toks) { tokens = toks; }

        public List<ASTNode> ParseProgram()
        {
            var nodes = new List<ASTNode>();
            while (Cur.Kind != TokenKind.EOF) nodes.Add(ParseStatement());
            return nodes;
        }

        ASTNode ParseStatement()
        {
            if (Cur.Kind == TokenKind.Fn) return ParseFn();
            if (Cur.Kind == TokenKind.Let) return ParseLet();
            if (Cur.Kind == TokenKind.If) return ParseIf();
            if (Cur.Kind == TokenKind.While) return ParseWhile();
            if (Cur.Kind == TokenKind.Return) { Eat(); var val = ParseExpr(); Expect(TokenKind.Semicolon); return new ReturnNode { Value = val }; }
            var expr = ParseExpr();
            if (Cur.Kind == TokenKind.Eq) { Eat(); var rhs = ParseExpr(); Expect(TokenKind.Semicolon); if (expr is IdentNode id) return new AssignNode { Name = id.Name, Value = rhs }; }
            else Expect(TokenKind.Semicolon);
            return expr;
        }

        FnNode ParseFn()
        {
            Expect(TokenKind.Fn);
            string name = Expect(TokenKind.Ident).Text;
            Expect(TokenKind.LParen);
            var parms = new List<string>();
            while (Cur.Kind != TokenKind.RParen)
            {
                parms.Add(Expect(TokenKind.Ident).Text);
                if (Cur.Kind == TokenKind.Comma) Eat();
            }
            Expect(TokenKind.RParen);
            Expect(TokenKind.LBrace);
            var body = new List<ASTNode>();
            while (Cur.Kind != TokenKind.RBrace) body.Add(ParseStatement());
            Expect(TokenKind.RBrace);
            return new FnNode { Name = name, Params = parms, Body = body };
        }

        LetNode ParseLet()
        {
            Expect(TokenKind.Let);
            string name = Expect(TokenKind.Ident).Text;
            Expect(TokenKind.Eq);
            var init = ParseExpr();
            Expect(TokenKind.Semicolon);
            return new LetNode { Name = name, Init = init };
        }

        IfNode ParseIf()
        {
            Expect(TokenKind.If);
            Expect(TokenKind.LParen);
            var cond = ParseExpr();
            Expect(TokenKind.RParen);
            Expect(TokenKind.LBrace);
            var then = new List<ASTNode>();
            while (Cur.Kind != TokenKind.RBrace) then.Add(ParseStatement());
            Expect(TokenKind.RBrace);
            var elseB = new List<ASTNode>();
            if (Cur.Kind == TokenKind.Else) { Eat(); Expect(TokenKind.LBrace); while (Cur.Kind != TokenKind.RBrace) elseB.Add(ParseStatement()); Expect(TokenKind.RBrace); }
            return new IfNode { Cond = cond, Then = then, Else = elseB };
        }

        WhileNode ParseWhile()
        {
            Expect(TokenKind.While);
            Expect(TokenKind.LParen);
            var cond = ParseExpr();
            Expect(TokenKind.RParen);
            Expect(TokenKind.LBrace);
            var body = new List<ASTNode>();
            while (Cur.Kind != TokenKind.RBrace) body.Add(ParseStatement());
            Expect(TokenKind.RBrace);
            return new WhileNode { Cond = cond, Body = body };
        }

        ASTNode ParseExpr() => ParseCmp();

        ASTNode ParseCmp()
        {
            var left = ParseAddSub();
            while (Cur.Kind is TokenKind.EqEq or TokenKind.NotEq or TokenKind.Lt or TokenKind.Gt or TokenKind.Le or TokenKind.Ge)
            {
                string op = Eat().Text;
                left = new BinOpNode { Op = op, Left = left, Right = ParseAddSub() };
            }
            return left;
        }

        ASTNode ParseAddSub()
        {
            var left = ParseMulDiv();
            while (Cur.Kind is TokenKind.Plus or TokenKind.Minus)
            {
                string op = Eat().Text;
                left = new BinOpNode { Op = op, Left = left, Right = ParseMulDiv() };
            }
            return left;
        }

        ASTNode ParseMulDiv()
        {
            var left = ParseUnary();
            while (Cur.Kind is TokenKind.Star or TokenKind.Slash or TokenKind.Percent or TokenKind.Amp or TokenKind.Pipe or TokenKind.Caret)
            {
                string op = Eat().Text;
                left = new BinOpNode { Op = op, Left = left, Right = ParseUnary() };
            }
            return left;
        }

        ASTNode ParseUnary()
        {
            if (Cur.Kind == TokenKind.Minus) { Eat(); return new UnOpNode { Op = "-", Operand = ParsePrimary() }; }
            if (Cur.Kind == TokenKind.Bang) { Eat(); return new UnOpNode { Op = "!", Operand = ParsePrimary() }; }
            if (Cur.Kind == TokenKind.Tilde) { Eat(); return new UnOpNode { Op = "~", Operand = ParsePrimary() }; }
            return ParsePrimary();
        }

        ASTNode ParsePrimary()
        {
            if (Cur.Kind == TokenKind.Number) { var v = Eat().NumVal; return new NumberNode { Value = v }; }
            if (Cur.Kind == TokenKind.Ident)
            {
                string name = Eat().Text;
                if (Cur.Kind == TokenKind.LParen)
                {
                    Eat();
                    var args = new List<ASTNode>();
                    while (Cur.Kind != TokenKind.RParen) { args.Add(ParseExpr()); if (Cur.Kind == TokenKind.Comma) Eat(); }
                    Eat();
                    return new CallNode { Name = name, Args = args };
                }
                return new IdentNode { Name = name };
            }
            if (Cur.Kind == TokenKind.LParen) { Eat(); var e = ParseExpr(); Expect(TokenKind.RParen); return e; }
            throw new Exception($"Unexpected token {Cur.Kind} ('{Cur.Text}')");
        }
    }

    // ================================================================
    // IR CODE GENERATOR
    // ================================================================
    public class IRGenerator
    {
        int tempCount = 0;
        int labelCount = 0;
        string NewTemp() => $"t{tempCount++}";
        string NewLabel() => $"L{labelCount++}";

        public List<IRNode> Generate(List<ASTNode> program)
        {
            var ir = new List<IRNode>();
            foreach (var node in program) GenNode(node, ir);
            return ir;
        }

        string GenNode(ASTNode node, List<IRNode> ir)
        {
            switch (node)
            {
                case NumberNode n:
                    var t = NewTemp();
                    ir.Add(new IRNode { Op = IROpcode.Mov, Dest = t, Imm = n.Value });
                    return t;
                case IdentNode id:
                    return id.Name;
                case BinOpNode bin:
                    var lv = GenNode(bin.Left, ir);
                    var rv = GenNode(bin.Right, ir);
                    var dest = NewTemp();
                    var op = bin.Op switch
                    {
                        "+" => IROpcode.Add, "-" => IROpcode.Sub, "*" => IROpcode.Mul, "/" => IROpcode.Div,
                        "&" => IROpcode.And, "|" => IROpcode.Or, "^" => IROpcode.Xor,
                        "==" => IROpcode.Cmp, "!=" => IROpcode.Cmp, "<" => IROpcode.Cmp, ">" => IROpcode.Cmp,
                        _ => IROpcode.Add
                    };
                    ir.Add(new IRNode { Op = op, Dest = dest, Src1 = lv, Src2 = rv });
                    return dest;
                case UnOpNode un:
                    var uv = GenNode(un.Operand, ir);
                    var ud = NewTemp();
                    ir.Add(new IRNode { Op = IROpcode.Not, Dest = ud, Src1 = uv });
                    return ud;
                case LetNode let:
                    var initVal = GenNode(let.Init, ir);
                    ir.Add(new IRNode { Op = IROpcode.Mov, Dest = let.Name, Src1 = initVal });
                    return let.Name;
                case AssignNode asgn:
                    var aval = GenNode(asgn.Value, ir);
                    ir.Add(new IRNode { Op = IROpcode.Mov, Dest = asgn.Name, Src1 = aval });
                    return asgn.Name;
                case ReturnNode ret:
                    var rv2 = GenNode(ret.Value, ir);
                    ir.Add(new IRNode { Op = IROpcode.Ret, Src1 = rv2 });
                    return null;
                case IfNode ifn:
                    var condv = GenNode(ifn.Cond, ir);
                    string elseLabel = NewLabel(), endLabel = NewLabel();
                    ir.Add(new IRNode { Op = IROpcode.Je, Src1 = condv, Label = elseLabel });
                    foreach (var s in ifn.Then) GenNode(s, ir);
                    ir.Add(new IRNode { Op = IROpcode.Jmp, Label = endLabel });
                    ir.Add(new IRNode { Op = IROpcode.Label, Label = elseLabel });
                    foreach (var s in ifn.Else) GenNode(s, ir);
                    ir.Add(new IRNode { Op = IROpcode.Label, Label = endLabel });
                    return null;
                case WhileNode wh:
                    string loopStart = NewLabel(), loopEnd = NewLabel();
                    ir.Add(new IRNode { Op = IROpcode.Label, Label = loopStart });
                    var wcond = GenNode(wh.Cond, ir);
                    ir.Add(new IRNode { Op = IROpcode.Je, Src1 = wcond, Label = loopEnd });
                    foreach (var s in wh.Body) GenNode(s, ir);
                    ir.Add(new IRNode { Op = IROpcode.Jmp, Label = loopStart });
                    ir.Add(new IRNode { Op = IROpcode.Label, Label = loopEnd });
                    return null;
                case FnNode fn:
                    ir.Add(new IRNode { Op = IROpcode.Label, Label = fn.Name });
                    foreach (var s in fn.Body) GenNode(s, ir);
                    return null;
                case CallNode call:
                    foreach (var arg in call.Args) GenNode(arg, ir);
                    var callDest = NewTemp();
                    ir.Add(new IRNode { Op = IROpcode.Call, Dest = callDest, Label = call.Name });
                    return callDest;
                default:
                    return null;
            }
        }
    }

    // ================================================================
    // BYTECODE OPTIMIZER (IR -> Optimized IR)
    // ================================================================
    public class BytecodeOptimizer
    {
        public PerformanceCounters Counters;

        public List<IRNode> Optimize(List<IRNode> ir)
        {
            ir = ConstantFolding(ir);
            ir = DeadCodeElimination(ir);
            ir = CopyPropagation(ir);
            ir = CommonSubexpressionElimination(ir);
            ir = StrengthReduction(ir);
            return ir;
        }

        List<IRNode> ConstantFolding(List<IRNode> ir)
        {
            var result = new List<IRNode>();
            var constMap = new Dictionary<string, ulong>();

            foreach (var node in ir)
            {
                if (node.Op == IROpcode.Mov && node.Src1 == null)
                {
                    constMap[node.Dest] = node.Imm;
                    result.Add(node);
                    continue;
                }
                if (node.Src1 != null && constMap.TryGetValue(node.Src1, out ulong v1) &&
                    node.Src2 != null && constMap.TryGetValue(node.Src2, out ulong v2))
                {
                    ulong folded = node.Op switch
                    {
                        IROpcode.Add => v1 + v2, IROpcode.Sub => v1 - v2,
                        IROpcode.Mul => v1 * v2, IROpcode.Div => v2 != 0 ? v1 / v2 : 0,
                        IROpcode.And => v1 & v2, IROpcode.Or => v1 | v2, IROpcode.Xor => v1 ^ v2,
                        _ => 0
                    };
                    constMap[node.Dest] = folded;
                    result.Add(new IRNode { Op = IROpcode.Mov, Dest = node.Dest, Imm = folded });
                    Interlocked.Increment(ref Counters.BytecodeOptimizations);
                    continue;
                }
                result.Add(node);
            }
            return result;
        }

        List<IRNode> DeadCodeElimination(List<IRNode> ir)
        {
            var used = new HashSet<string>();
            foreach (var node in ir)
            {
                if (node.Src1 != null) used.Add(node.Src1);
                if (node.Src2 != null) used.Add(node.Src2);
                if (node.Label != null) used.Add(node.Label);
            }
            var result = new List<IRNode>();
            foreach (var node in ir)
            {
                if (node.Dest != null && !used.Contains(node.Dest) &&
                    node.Op != IROpcode.Ret && node.Op != IROpcode.Call && node.Op != IROpcode.Store)
                {
                    Interlocked.Increment(ref Counters.BytecodeOptimizations);
                    continue;
                }
                result.Add(node);
            }
            return result;
        }

        List<IRNode> CopyPropagation(List<IRNode> ir)
        {
            var copies = new Dictionary<string, string>();
            var result = new List<IRNode>();
            foreach (var node in ir)
            {
                string Resolve(string s) { while (s != null && copies.TryGetValue(s, out var r)) s = r; return s; }
                if (node.Op == IROpcode.Mov && node.Src1 != null && node.Src2 == null)
                {
                    copies[node.Dest] = node.Src1;
                    Interlocked.Increment(ref Counters.BytecodeOptimizations);
                    continue;
                }
                result.Add(new IRNode { Op = node.Op, Dest = node.Dest, Src1 = Resolve(node.Src1), Src2 = Resolve(node.Src2), Imm = node.Imm, Label = node.Label });
            }
            return result;
        }

        List<IRNode> CommonSubexpressionElimination(List<IRNode> ir)
        {
            var seen = new Dictionary<string, string>(); // expr -> temp
            var result = new List<IRNode>();
            foreach (var node in ir)
            {
                if (node.Src1 != null && node.Src2 != null)
                {
                    string key = $"{node.Op}:{node.Src1}:{node.Src2}";
                    if (seen.TryGetValue(key, out var existing))
                    {
                        result.Add(new IRNode { Op = IROpcode.Mov, Dest = node.Dest, Src1 = existing });
                        Interlocked.Increment(ref Counters.BytecodeOptimizations);
                        continue;
                    }
                    seen[key] = node.Dest;
                }
                result.Add(node);
            }
            return result;
        }

        List<IRNode> StrengthReduction(List<IRNode> ir)
        {
            var result = new List<IRNode>();
            foreach (var node in ir)
            {
                // Mul by power of 2 -> shift
                if (node.Op == IROpcode.Mul && node.Src2 == null && node.Imm > 0 && (node.Imm & (node.Imm - 1)) == 0)
                {
                    int shift = (int)Math.Log2(node.Imm);
                    result.Add(new IRNode { Op = IROpcode.Mov, Dest = node.Dest, Src1 = node.Src1, Imm = (ulong)shift }); // represent as SHL
                    Interlocked.Increment(ref Counters.BytecodeOptimizations);
                    continue;
                }
                result.Add(node);
            }
            return result;
        }
    }

    // ================================================================
    // JIT STUB (IR -> native delegate via dynamic method)
    // ================================================================
    public class JITCompiler
    {
        public PerformanceCounters Counters;
        Dictionary<string, Func<ulong[], ulong>> cache = new Dictionary<string, Func<ulong[], ulong>>();

        public Func<ulong[], ulong> Compile(List<IRNode> ir, string fnName)
        {
            string key = fnName;
            if (cache.TryGetValue(key, out var cached)) return cached;
            Interlocked.Increment(ref Counters.JITCompilations);
            // Interpret-based JIT stub
            Func<ulong[], ulong> fn = (args) =>
            {
                var regs = new Dictionary<string, ulong>();
                for (int i = 0; i < args.Length; i++) regs[$"arg{i}"] = args[i];
                ulong retVal = 0;
                foreach (var node in ir)
                {
                    ulong R(string s) => s == null ? 0 : (ulong.TryParse(s, out ulong v) ? v : (regs.TryGetValue(s, out ulong rv) ? rv : 0));
                    switch (node.Op)
                    {
                        case IROpcode.Mov: regs[node.Dest] = node.Src1 != null ? R(node.Src1) : node.Imm; break;
                        case IROpcode.Add: regs[node.Dest] = R(node.Src1) + R(node.Src2); break;
                        case IROpcode.Sub: regs[node.Dest] = R(node.Src1) - R(node.Src2); break;
                        case IROpcode.Mul: regs[node.Dest] = R(node.Src1) * R(node.Src2); break;
                        case IROpcode.Div: { ulong d = R(node.Src2); regs[node.Dest] = d != 0 ? R(node.Src1) / d : 0; } break;
                        case IROpcode.And: regs[node.Dest] = R(node.Src1) & R(node.Src2); break;
                        case IROpcode.Or: regs[node.Dest] = R(node.Src1) | R(node.Src2); break;
                        case IROpcode.Xor: regs[node.Dest] = R(node.Src1) ^ R(node.Src2); break;
                        case IROpcode.Ret: retVal = R(node.Src1); goto done;
                    }
                }
                done:
                return retVal;
            };
            cache[key] = fn;
            return fn;
        }
    }

    // ================================================================
    // INSTRUCTION SCHEDULER (reorder for pipeline efficiency)
    // ================================================================
    public class InstructionScheduler
    {
        public List<IRNode> Schedule(List<IRNode> ir)
        {
            // Topological sort based on data dependencies (list scheduling)
            var result = new List<IRNode>();
            var ready = new Queue<IRNode>();
            var inDegree = new Dictionary<IRNode, int>();
            var successors = new Dictionary<IRNode, List<IRNode>>();
            var defs = new Dictionary<string, IRNode>();

            foreach (var node in ir)
            {
                inDegree[node] = 0;
                successors[node] = new List<IRNode>();
            }

            // Build dependency graph
            foreach (var node in ir)
            {
                if (node.Dest != null) defs[node.Dest] = node;
            }

            foreach (var node in ir)
            {
                foreach (var src in new[] { node.Src1, node.Src2 })
                {
                    if (src != null && defs.TryGetValue(src, out var dep) && dep != node)
                    {
                        successors[dep].Add(node);
                        inDegree[node]++;
                    }
                }
            }

            // Initialize ready queue
            foreach (var node in ir) if (inDegree[node] == 0) ready.Enqueue(node);

            while (ready.Count > 0)
            {
                var node = ready.Dequeue();
                result.Add(node);
                foreach (var succ in successors[node])
                {
                    inDegree[succ]--;
                    if (inDegree[succ] == 0) ready.Enqueue(succ);
                }
            }

            // Add any remaining (cycles)
            foreach (var node in ir) if (!result.Contains(node)) result.Add(node);

            return result;
        }
    }

    // ================================================================
    // BINARY LOADER V2 (ELF-like with sections + relocations)
    // ================================================================
    public enum SectionType { Code, Data, BSS, ReadOnly, Stack }
    public class Section { public string Name; public SectionType Type; public ulong VAddr; public byte[] Data; public ulong Flags; }
    public class Relocation { public string Symbol; public ulong Offset; public int Type; }
    public class ELFBinary
    {
        public string Name = "unnamed";
        public ulong EntryPoint;
        public ulong StackStart = 0x7FFF0000;
        public ulong HeapStart = 0x10000000;
        public List<Section> Sections = new List<Section>();
        public List<Relocation> Relocations = new List<Relocation>();
        public Dictionary<string, ulong> ExportedSymbols = new Dictionary<string, ulong>();
        public byte[] Header = new byte[64]; // ELF header stub
    }

    // ================================================================
    // MEMORY ALLOCATOR (bump + free list)
    // ================================================================
    public class MemoryAllocator
    {
        ulong heapBase;
        ulong brk;
        SortedList<ulong, ulong> freeList = new SortedList<ulong, ulong>(); // addr -> size
        Memory mem;
        public PerformanceCounters Counters;

        public MemoryAllocator(ulong heapBase, Memory mem) { this.heapBase = heapBase; this.brk = heapBase; this.mem = mem; }

        public ulong Alloc(ulong size)
        {
            size = (size + 15) & ~15UL; // align to 16
            // Check free list first
            foreach (var kv in freeList)
            {
                if (kv.Value >= size)
                {
                    ulong addr = kv.Key;
                    freeList.Remove(kv.Key);
                    if (kv.Value > size) freeList[addr + size] = kv.Value - size;
                    Interlocked.Increment(ref Counters.Allocations);
                    Interlocked.Add(ref Counters.MemoryAllocBytes, (long)size);
                    return addr;
                }
            }
            // Bump allocate
            ulong result = brk;
            brk += size;
            // Zero initialize
            for (ulong i = 0; i < size; i++) mem.Write(result + i, 0);
            Interlocked.Increment(ref Counters.Allocations);
            Interlocked.Add(ref Counters.MemoryAllocBytes, (long)size);
            return result;
        }

        public void Free(ulong addr, ulong size)
        {
            size = (size + 15) & ~15UL;
            freeList[addr] = size;
            // Coalesce adjacent free blocks
            CoalesceFreeList();
        }

        void CoalesceFreeList()
        {
            var keys = freeList.Keys.ToList();
            for (int i = 0; i < keys.Count - 1; i++)
            {
                if (keys[i] + freeList[keys[i]] == keys[i + 1])
                {
                    freeList[keys[i]] += freeList[keys[i + 1]];
                    freeList.Remove(keys[i + 1]);
                    keys.RemoveAt(i + 1);
                    i--;
                }
            }
        }

        public ulong HeapUsed => brk - heapBase;
        public ulong FreeBytes => freeList.Values.Aggregate(0UL, (a, v) => a + v);
    }

    // ================================================================
    // CPU CORE
    // ================================================================
    public class Core
    {
        public int CoreID;
        public ulong[] R = new ulong[32];      // General purpose registers
        public double[] F = new double[32];    // Float registers
        public ulong[] V = new ulong[32];      // Vector registers (simplified as ulong)
        public ulong PC;
        public ulong SP => R[31];              // R31 = stack pointer
        public ulong FP => R[30];              // R30 = frame pointer
        public ulong RA => R[29];              // R29 = return address
        public StatusFlags Flags;
        public bool Running = true;
        public PrivilegeMode Mode = PrivilegeMode.User;
        public int Cycles;
        public Memory Mem;
        public MemoryAllocator Allocator;
        public Thread Thread;
        public BranchPredictor Predictor = new BranchPredictor();
        public List<ulong> Breakpoints = new List<ulong>();
        public ulong KernelStackBase;
        public ulong UserStackBase;
        public CancellationTokenSource CTS = new CancellationTokenSource();

        // Pipeline state
        public ulong DecodedInstr;
        public bool StallPipeline = false;
        public int StallCycles = 0;

        // CSR (Control/Status Registers)
        public ulong[] CSR = new ulong[256];

        // Exception vector base
        public ulong ExceptionBase = 0xFFFF8000;
    }

    // ================================================================
    // SCHEDULER (preemptive, round-robin + priority)
    // ================================================================
    public class ProcessState
    {
        public int PID;
        public string Name;
        public ulong[] SavedR = new ulong[32];
        public double[] SavedF = new double[32];
        public ulong SavedPC;
        public StatusFlags SavedFlags;
        public PrivilegeMode SavedMode;
        public int Priority = 0;
        public int TimeSliceRemaining = 100;
        public bool Blocked = false;
        public ulong WaitingOn; // for sync primitives
    }

    public class Scheduler
    {
        Queue<ProcessState> runQueue = new Queue<ProcessState>();
        List<ProcessState> allProcesses = new List<ProcessState>();
        int nextPID = 1;
        public PerformanceCounters Counters;
        readonly object schedLock = new object();

        public ProcessState CreateProcess(string name, ulong entryPC, int priority = 0)
        {
            var ps = new ProcessState { PID = nextPID++, Name = name, SavedPC = entryPC, Priority = priority, TimeSliceRemaining = 100 + priority * 10 };
            lock (schedLock) { allProcesses.Add(ps); runQueue.Enqueue(ps); }
            return ps;
        }

        public void SaveContext(Core c, ProcessState ps)
        {
            Array.Copy(c.R, ps.SavedR, 32);
            Array.Copy(c.F, ps.SavedF, 32);
            ps.SavedPC = c.PC;
            ps.SavedFlags = c.Flags;
            ps.SavedMode = c.Mode;
        }

        public void RestoreContext(Core c, ProcessState ps)
        {
            Array.Copy(ps.SavedR, c.R, 32);
            Array.Copy(ps.SavedF, c.F, 32);
            c.PC = ps.SavedPC;
            c.Flags = ps.SavedFlags;
            c.Mode = ps.SavedMode;
            Interlocked.Increment(ref Counters.ContextSwitches);
        }

        public ProcessState Next()
        {
            lock (schedLock)
            {
                while (runQueue.Count > 0)
                {
                    var ps = runQueue.Dequeue();
                    if (!ps.Blocked) { runQueue.Enqueue(ps); return ps; }
                }
                return null;
            }
        }

        public void Block(int pid)
        {
            lock (schedLock)
            {
                var ps = allProcesses.FirstOrDefault(p => p.PID == pid);
                if (ps != null) ps.Blocked = true;
            }
        }

        public void Unblock(int pid)
        {
            lock (schedLock)
            {
                var ps = allProcesses.FirstOrDefault(p => p.PID == pid);
                if (ps != null) { ps.Blocked = false; }
            }
        }

        public void PrintProcessTable()
        {
            Console.WriteLine("==== PROCESS TABLE ====");
            Console.WriteLine($"  {"PID",-5} {"Name",-20} {"PC",-18} {"Mode",-10} {"State",-10} {"Priority",-8}");
            lock (schedLock)
            {
                foreach (var ps in allProcesses)
                    Console.WriteLine($"  {ps.PID,-5} {ps.Name,-20} {ps.SavedPC,-18:X} {ps.SavedMode,-10} {(ps.Blocked ? "BLOCKED" : "READY"),-10} {ps.Priority,-8}");
            }
        }
    }

    // ================================================================
    // SYSCALL HANDLER
    // ================================================================
    public class SyscallHandler
    {
        public VirtualFileSystem VFS;
        public MemoryAllocator Allocator;
        public PerformanceCounters Counters;
        public Scheduler Scheduler;

        // Syscall numbers
        const ulong SYS_EXIT = 0;
        const ulong SYS_WRITE = 1;
        const ulong SYS_READ = 2;
        const ulong SYS_OPEN = 3;
        const ulong SYS_CLOSE = 4;
        const ulong SYS_MALLOC = 5;
        const ulong SYS_FREE = 6;
        const ulong SYS_GETPID = 7;
        const ulong SYS_YIELD = 8;
        const ulong SYS_SLEEP = 9;
        const ulong SYS_FORK = 10;

        public ulong Handle(Core c, ulong syscallNum)
        {
            Interlocked.Increment(ref Counters.Syscalls);
            switch (syscallNum)
            {
                case SYS_WRITE:
                    ulong fd = c.R[1]; ulong buf = c.R[2]; ulong count = c.R[3];
                    for (ulong i = 0; i < count; i++) Console.Write((char)c.Mem.Read(buf + i));
                    return count;
                case SYS_MALLOC:
                    ulong size = c.R[1];
                    return Allocator?.Alloc(size) ?? 0;
                case SYS_FREE:
                    ulong addr = c.R[1]; ulong sz = c.R[2];
                    Allocator?.Free(addr, sz);
                    return 0;
                case SYS_GETPID:
                    return 1;
                case SYS_EXIT:
                    c.Running = false;
                    return c.R[1];
                case SYS_YIELD:
                    Thread.Yield();
                    return 0;
                default:
                    return ulong.MaxValue; // ENOSYS
            }
        }
    }

    // ================================================================
    // CLI DEBUGGER
    // ================================================================
    public class Debugger
    {
        brains vm;
        List<Core> cores;

        public Debugger(brains vm, List<Core> cores) { this.vm = vm; this.cores = cores; }

        public void PrintCoreState(Core c)
        {
            Console.WriteLine($"==== CORE {c.CoreID} STATE ====");
            Console.WriteLine($"  PC  : 0x{c.PC:X16}  Mode: {c.Mode}  Running: {c.Running}");
            Console.WriteLine($"  Flags: Z={FlagSet(c, StatusFlags.Zero)} S={FlagSet(c, StatusFlags.Sign)} C={FlagSet(c, StatusFlags.Carry)} O={FlagSet(c, StatusFlags.Overflow)}");
            for (int row = 0; row < 4; row++)
            {
                Console.Write("  ");
                for (int col = 0; col < 8; col++)
                {
                    int idx = row * 8 + col;
                    Console.Write($"r{idx,2}=0x{c.R[idx]:X8}  ");
                }
                Console.WriteLine();
            }
        }

        bool FlagSet(Core c, StatusFlags f) => (c.Flags & f) != 0;

        public void PrintMemory(Core c, ulong start, ulong length)
        {
            Console.WriteLine($"==== MEMORY DUMP 0x{start:X} — 0x{start + length - 1:X} ====");
            for (ulong i = 0; i < length; i += 16)
            {
                Console.Write($"  0x{start + i:X8}  ");
                for (ulong j = 0; j < 16 && i + j < length; j++)
                    Console.Write($"{c.Mem.Read(start + i + j):X2} ");
                Console.Write("  ");
                for (ulong j = 0; j < 16 && i + j < length; j++)
                {
                    byte b = c.Mem.Read(start + i + j);
                    Console.Write(b >= 32 && b < 127 ? (char)b : '.');
                }
                Console.WriteLine();
            }
        }

        public void PrintStack(Core c, int depth = 16)
        {
            Console.WriteLine($"==== STACK (SP=0x{c.R[31]:X}) ====");
            for (int i = 0; i < depth; i++)
            {
                ulong addr = c.R[31] + (ulong)(i * 8);
                Console.WriteLine($"  [SP+{i * 8,3}] 0x{addr:X8}  =  0x{c.Mem.ReadU64(addr):X16}");
            }
        }

        public void PrintFloatRegs(Core c)
        {
            Console.WriteLine("==== FLOAT REGISTERS ====");
            for (int i = 0; i < 32; i += 4)
                Console.WriteLine($"  f{i,2}={c.F[i],12:F6}  f{i + 1,2}={c.F[i + 1],12:F6}  f{i + 2,2}={c.F[i + 2],12:F6}  f{i + 3,2}={c.F[i + 3],12:F6}");
        }

        public void PrintCacheState(Core c)
        {
            Console.WriteLine("==== L1 CACHE STATE ====");
            int validI = c.Mem.L1I.Count(l => l.Valid);
            int validD = c.Mem.L1D.Count(l => l.Valid);
            Console.WriteLine($"  L1I: {validI}/{c.Mem.L1I.Length} lines valid");
            Console.WriteLine($"  L1D: {validD}/{c.Mem.L1D.Length} lines valid  Dirty: {c.Mem.L1D.Count(l => l.Valid && l.Dirty)}");
        }

        public void AddBreakpoint(Core c, ulong addr)
        {
            c.Breakpoints.Add(addr);
            Console.WriteLine($"[DBG] Breakpoint set at 0x{addr:X}");
        }

        public void RemoveBreakpoint(Core c, ulong addr)
        {
            c.Breakpoints.Remove(addr);
            Console.WriteLine($"[DBG] Breakpoint removed at 0x{addr:X}");
        }

        public void Disassemble(Core c, ulong addr, int count)
        {
            Console.WriteLine($"==== DISASSEMBLY @ 0x{addr:X} ====");
            ulong cur = addr;
            for (int i = 0; i < count; i++)
            {
                byte op = c.Mem.Read(cur);
                byte rd = c.Mem.Read(cur + 1);
                byte rs1 = c.Mem.Read(cur + 2);
                byte rs2 = c.Mem.Read(cur + 3);
                ulong imm = 0;
                for (int j = 0; j < 6; j++) imm |= (ulong)c.Mem.Read(cur + 4 + (ulong)j) << (j * 8);
                string dis = ((Opcode)op) switch
                {
                    Opcode.NOP => "NOP",
                    Opcode.HALT => "HALT",
                    Opcode.MOV => $"MOV r{rd}, r{rs1}",
                    Opcode.MOVI => $"MOVI r{rd}, 0x{imm:X}",
                    Opcode.ADD => $"ADD r{rd}, r{rs1}, r{rs2}",
                    Opcode.ADDI => $"ADDI r{rd}, r{rs1}, 0x{imm:X}",
                    Opcode.SUB => $"SUB r{rd}, r{rs1}, r{rs2}",
                    Opcode.MUL => $"MUL r{rd}, r{rs1}, r{rs2}",
                    Opcode.DIV => $"DIV r{rd}, r{rs1}, r{rs2}",
                    Opcode.AND => $"AND r{rd}, r{rs1}, r{rs2}",
                    Opcode.OR => $"OR r{rd}, r{rs1}, r{rs2}",
                    Opcode.XOR => $"XOR r{rd}, r{rs1}, r{rs2}",
                    Opcode.SHL => $"SHL r{rd}, r{rs1}, r{rs2}",
                    Opcode.SHR => $"SHR r{rd}, r{rs1}, r{rs2}",
                    Opcode.CMP => $"CMP r{rs1}, r{rs2}",
                    Opcode.JMP => $"JMP 0x{imm:X}",
                    Opcode.JE => $"JE 0x{imm:X}",
                    Opcode.JNE => $"JNE 0x{imm:X}",
                    Opcode.JL => $"JL 0x{imm:X}",
                    Opcode.JG => $"JG 0x{imm:X}",
                    Opcode.CALL => $"CALL 0x{imm:X}",
                    Opcode.RET => "RET",
                    Opcode.PUSH => $"PUSH r{rs1}",
                    Opcode.POP => $"POP r{rd}",
                    Opcode.LD => $"LD r{rd}, [r{rs1}]",
                    Opcode.ST => $"ST [0x{imm:X}], r{rs1}",
                    Opcode.SYSCALL => "SYSCALL",
                    Opcode.INT => $"INT 0x{imm:X}",
                    _ => $"??? (0x{op:X2})"
                };
                string bp = c.Breakpoints.Contains(cur) ? "* " : "  ";
                Console.WriteLine($"{bp}0x{cur:X8}:  {dis}");
                cur += op == (byte)Opcode.NOP || op == (byte)Opcode.HALT || op == (byte)Opcode.RET || op == (byte)Opcode.FENCE ? 1UL : 10UL;
            }
        }

        public void RunREPL(Core c)
        {
            Console.WriteLine("==== MINDWORKS DEBUGGER REPL ====");
            Console.WriteLine("Commands: reg, mem <addr> <len>, stack, float, cache, dis <addr> <n>, bp <addr>, bpdel <addr>, step, cont, stat, proc, quit");
            while (true)
            {
                Console.Write("dbg> ");
                var line = Console.ReadLine()?.Trim();
                if (string.IsNullOrEmpty(line)) continue;
                var parts = line.Split(' ');
                try
                {
                    switch (parts[0])
                    {
                        case "reg": PrintCoreState(c); break;
                        case "mem":
                            ulong maddr = Convert.ToUInt64(parts[1], 16);
                            ulong mlen = parts.Length > 2 ? ulong.Parse(parts[2]) : 64;
                            PrintMemory(c, maddr, mlen);
                            break;
                        case "stack": PrintStack(c); break;
                        case "float": PrintFloatRegs(c); break;
                        case "cache": PrintCacheState(c); break;
                        case "dis":
                            ulong daddr = parts.Length > 1 ? Convert.ToUInt64(parts[1], 16) : c.PC;
                            int dcount = parts.Length > 2 ? int.Parse(parts[2]) : 10;
                            Disassemble(c, daddr, dcount);
                            break;
                        case "bp":
                            AddBreakpoint(c, Convert.ToUInt64(parts[1], 16)); break;
                        case "bpdel":
                            RemoveBreakpoint(c, Convert.ToUInt64(parts[1], 16)); break;
                        case "step": vm.Step(c); PrintCoreState(c); break;
                        case "cont": c.Running = true; break;
                        case "stat": vm.Counters.Print(); break;
                        case "proc": vm.GetScheduler()?.PrintProcessTable(); break;
                        case "quit": return;
                        default: Console.WriteLine($"Unknown command: {parts[0]}"); break;
                    }
                }
                catch (Exception ex) { Console.WriteLine($"[DBG ERROR] {ex.Message}"); }
            }
        }
    }

    // ================================================================
    // TIMING MODEL
    // ================================================================
    public class TimingModel
    {
        static Dictionary<InstructionType, int> latencies = new Dictionary<InstructionType, int>
        {
            { InstructionType.ALU, 1 },
            { InstructionType.Load, 4 },
            { InstructionType.Store, 3 },
            { InstructionType.Branch, 1 },
            { InstructionType.Jump, 2 },
            { InstructionType.Float, 6 },
            { InstructionType.SIMD, 4 },
            { InstructionType.Vector, 8 },
            { InstructionType.Syscall, 20 },
            { InstructionType.Privileged, 10 },
        };

        public static int GetLatency(Opcode op)
        {
            var type = op switch
            {
                Opcode.LD or Opcode.LDB or Opcode.LDH => InstructionType.Load,
                Opcode.ST or Opcode.STB or Opcode.STH => InstructionType.Store,
                Opcode.FADD or Opcode.FSUB or Opcode.FMUL or Opcode.FDIV => InstructionType.Float,
                Opcode.VADD or Opcode.VSUB or Opcode.VMUL or Opcode.VDOT => InstructionType.Vector,
                Opcode.JMP or Opcode.JE or Opcode.JNE or Opcode.JL or Opcode.JG or Opcode.JLE or Opcode.JGE => InstructionType.Branch,
                Opcode.CALL or Opcode.RET => InstructionType.Jump,
                Opcode.SYSCALL or Opcode.INT => InstructionType.Syscall,
                Opcode.XCHG or Opcode.CAS => InstructionType.Privileged,
                _ => InstructionType.ALU
            };
            return latencies[type];
        }
    }

    // ================================================================
    // MAIN VM — brains
    // ================================================================
    public class brains : IDisposable
    {
        // Core subsystems
        public PerformanceCounters Counters = new PerformanceCounters();
        public L2Cache SharedL2 = new L2Cache();
        public InterruptController PIC = new InterruptController();
        public VirtualFileSystem VFS = new VirtualFileSystem();
        public Scheduler VMScheduler;
        public SyscallHandler Syscalls;
        public GPUDevice GPU = new GPUDevice();
        public Assembler Assembler = new Assembler();
        public BytecodeOptimizer Optimizer;
        public JITCompiler JIT;
        public InstructionScheduler InstrScheduler = new InstructionScheduler();

        Stopwatch profiler = new Stopwatch();
        public long ElapsedMs => profiler.ElapsedMilliseconds;

        List<Core> cores = new List<Core>();
        List<IMemoryRegion> ioRegions = new List<IMemoryRegion>();
        TimerDevice timer;
        UartDevice uart;
        DMADevice dma;

        bool disposed = false;

        public brains()
        {
            GPU.Counters = Counters;
            PIC.Counters = Counters;

            VMScheduler = new Scheduler { Counters = Counters };
            Optimizer = new BytecodeOptimizer { Counters = Counters };
            JIT = new JITCompiler { Counters = Counters };

            // Setup devices
            timer = new TimerDevice { PIC = PIC, IRQ = 0 };
            uart = new UartDevice { PIC = PIC, IRQ = 4 };

            // Register interrupt handlers
            PIC.RegisterHandler(0, () => Console.WriteLine("[IRQ0] Timer tick"), 200);
            PIC.RegisterHandler(4, () => { /* UART handled */ }, 100);
        }

        public Scheduler GetScheduler() => VMScheduler;

        public Core CreateCore()
        {
            var core = new Core
            {
                CoreID = cores.Count,
                Mem = new Memory(SharedL2, Counters),
                KernelStackBase = 0x7FE00000 + (ulong)(cores.Count * 0x100000),
                UserStackBase = 0x7FF00000 + (ulong)(cores.Count * 0x100000),
            };
            // Init SP to user stack base
            core.R[31] = core.UserStackBase;
            core.Allocator = new MemoryAllocator(0x10000000 + (ulong)(cores.Count * 0x10000000), core.Mem)
            { Counters = Counters };
            Syscalls ??= new SyscallHandler { VFS = VFS, Allocator = core.Allocator, Counters = Counters, Scheduler = VMScheduler };
            core.Thread = new Thread(() => RunCore(core)) { IsBackground = true, Name = $"Core-{core.CoreID}" };
            cores.Add(core);

            // Setup DMA for first core
            if (dma == null) { dma = new DMADevice(core.Mem); ioRegions.Add(dma); }
            return core;
        }

        public void MapIO(IMemoryRegion r) => ioRegions.Add(r);

        IMemoryRegion FindIO(ulong addr) => ioRegions.FirstOrDefault(r => addr >= r.Start && addr < r.End);

        void RunCore(Core c)
        {
            const int Quantum = 1000;
            ProcessState currentProcess = null;

            while (c.Running && !c.CTS.Token.IsCancellationRequested)
            {
                // Quantum-based preemption
                currentProcess ??= VMScheduler.Next();

                if (currentProcess != null) VMScheduler.RestoreContext(c, currentProcess);

                for (int q = 0; q < Quantum && c.Running; q++)
                {
                    if (c.Breakpoints.Contains(c.PC))
                    {
                        Console.WriteLine($"[Debugger] Breakpoint hit at 0x{c.PC:X} on Core {c.CoreID}");
                        c.Running = false;
                        break;
                    }

                    // Handle pipeline stalls
                    if (c.StallPipeline && c.StallCycles > 0)
                    {
                        c.StallCycles--;
                        Interlocked.Increment(ref Counters.StallCycles);
                        Interlocked.Increment(ref Counters.Cycles);
                        continue;
                    }
                    c.StallPipeline = false;

                    Step(c);
                    timer.Tick();
                    PIC.DispatchPending();
                }

                if (currentProcess != null)
                {
                    VMScheduler.SaveContext(c, currentProcess);
                    currentProcess = VMScheduler.Next();
                }

                Thread.Yield();
            }
        }

        public void Step(Core c)
        {
            ulong instr = Fetch(c);
            int latency = TimingModel.GetLatency((Opcode)(instr & 0xFF));
            Interlocked.Add(ref Counters.Cycles, latency);
            Execute(c, instr);
            Interlocked.Increment(ref Counters.InstructionsExecuted);
            c.Cycles++;
        }

        ulong Fetch(Core c)
        {
            ulong val = 0;
            for (int i = 0; i < 10; i++)
                val |= (ulong)c.Mem.FetchInstruction(c.PC + (ulong)i) << (i * 8);
            return val;
        }

        void Execute(Core c, ulong instr)
        {
            byte op = (byte)(instr & 0xFF);
            byte rd = (byte)((instr >> 8) & 0xFF);
            byte rs1 = (byte)((instr >> 16) & 0xFF);
            byte rs2 = (byte)((instr >> 24) & 0xFF);
            ulong imm = (instr >> 32) & 0xFFFFFFFFFFFF; // 48-bit immediate

            ulong instrSize = 10UL; // default instruction width

            switch ((Opcode)op)
            {
                case Opcode.NOP: instrSize = 1; break;
                case Opcode.HALT: c.Running = false; instrSize = 1; break;

                case Opcode.MOV: c.R[rd] = c.R[rs1]; break;
                case Opcode.MOVI: c.R[rd] = imm; break;

                case Opcode.ADD: { ulong a = c.R[rs1], b = c.R[rs2]; c.R[rd] = a + b; UpdateFlags(c, c.R[rd], a, b, false); break; }
                case Opcode.ADDI: { ulong a = c.R[rs1]; c.R[rd] = a + imm; UpdateFlags(c, c.R[rd], a, imm, false); break; }
                case Opcode.SUB: { ulong a = c.R[rs1], b = c.R[rs2]; c.R[rd] = a - b; UpdateFlags(c, c.R[rd], a, b, true); break; }
                case Opcode.SUBI: { ulong a = c.R[rs1]; c.R[rd] = a - imm; UpdateFlags(c, c.R[rd], a, imm, true); break; }
                case Opcode.MUL: c.R[rd] = c.R[rs1] * c.R[rs2]; break;
                case Opcode.DIV:
                    if (c.R[rs2] == 0) RaiseException(c, ExceptionType.DivideByZero);
                    else c.R[rd] = c.R[rs1] / c.R[rs2]; break;
                case Opcode.MOD:
                    if (c.R[rs2] == 0) RaiseException(c, ExceptionType.DivideByZero);
                    else c.R[rd] = c.R[rs1] % c.R[rs2]; break;

                case Opcode.AND: c.R[rd] = c.R[rs1] & c.R[rs2]; UpdateZeroSign(c, c.R[rd]); break;
                case Opcode.OR: c.R[rd] = c.R[rs1] | c.R[rs2]; UpdateZeroSign(c, c.R[rd]); break;
                case Opcode.XOR: c.R[rd] = c.R[rs1] ^ c.R[rs2]; UpdateZeroSign(c, c.R[rd]); break;
                case Opcode.NOT: c.R[rd] = ~c.R[rs1]; break;
                case Opcode.SHL: c.R[rd] = c.R[rs1] << (int)(c.R[rs2] & 63); break;
                case Opcode.SHR: c.R[rd] = c.R[rs1] >> (int)(c.R[rs2] & 63); break;
                case Opcode.SAR: c.R[rd] = (ulong)((long)c.R[rs1] >> (int)(c.R[rs2] & 63)); break;

                case Opcode.CMP: { ulong a = c.R[rs1], b = c.R[rs2]; UpdateFlags(c, a - b, a, b, true); break; }
                case Opcode.TEST: UpdateZeroSign(c, c.R[rs1] & c.R[rs2]); break;

                case Opcode.JMP:
                    bool predicted = c.Predictor.Predict(c.PC);
                    c.Predictor.Update(c.PC, true, imm, Counters);
                    if (!predicted) { c.StallPipeline = true; c.StallCycles = 2; } // branch misprediction penalty
                    c.PC = imm;
                    return;

                case Opcode.JE: if ((c.Flags & StatusFlags.Zero) != 0) { c.Predictor.Update(c.PC, true, imm, Counters); c.PC = imm; return; } c.Predictor.Update(c.PC, false, imm, Counters); break;
                case Opcode.JNE: if ((c.Flags & StatusFlags.Zero) == 0) { c.Predictor.Update(c.PC, true, imm, Counters); c.PC = imm; return; } c.Predictor.Update(c.PC, false, imm, Counters); break;
                case Opcode.JL: if ((c.Flags & StatusFlags.Sign) != 0) { c.PC = imm; return; } break;
                case Opcode.JG: if ((c.Flags & StatusFlags.Sign) == 0 && (c.Flags & StatusFlags.Zero) == 0) { c.PC = imm; return; } break;
                case Opcode.JLE: if ((c.Flags & StatusFlags.Sign) != 0 || (c.Flags & StatusFlags.Zero) != 0) { c.PC = imm; return; } break;
                case Opcode.JGE: if ((c.Flags & StatusFlags.Sign) == 0) { c.PC = imm; return; } break;

                case Opcode.CALL:
                    c.Predictor.PushRAS(c.PC + instrSize);
                    c.R[29] = c.PC + instrSize; // save return address
                    Push(c, c.PC + instrSize);
                    c.PC = imm;
                    return;

                case Opcode.RET:
                    instrSize = 1;
                    c.PC = Pop(c);
                    c.Predictor.PopRAS();
                    return;

                case Opcode.PUSH: Push(c, c.R[rs1]); break;
                case Opcode.POP: c.R[rd] = Pop(c); break;

                case Opcode.LD:
                    ulong loadAddr = c.R[rs1] + imm;
                    IORead: var ioR = FindIO(loadAddr);
                    if (ioR != null) { Interlocked.Increment(ref Counters.IOReads); c.R[rd] = ioR.Read(loadAddr); }
                    else { c.StallPipeline = true; c.StallCycles = 3; c.R[rd] = c.Mem.ReadU64(loadAddr); }
                    break;

                case Opcode.LDB:
                    c.R[rd] = c.Mem.Read(c.R[rs1] + imm); break;

                case Opcode.LDH:
                    c.R[rd] = c.Mem.ReadU32(c.R[rs1] + imm); break;

                case Opcode.ST:
                    ulong storeAddr = imm;
                    var ioW = FindIO(storeAddr);
                    if (ioW != null) { Interlocked.Increment(ref Counters.IOWrites); ioW.Write(storeAddr, (byte)c.R[rs1]); }
                    else { c.StallPipeline = true; c.StallCycles = 2; c.Mem.WriteU64(storeAddr, c.R[rs1]); }
                    break;

                case Opcode.STB: c.Mem.Write(imm, (byte)c.R[rs1]); break;
                case Opcode.STH: c.Mem.WriteU32(imm, (uint)c.R[rs1]); break;

                case Opcode.FADD: c.F[rd] = c.F[rs1] + c.F[rs2]; Interlocked.Increment(ref Counters.FloatOps); break;
                case Opcode.FSUB: c.F[rd] = c.F[rs1] - c.F[rs2]; Interlocked.Increment(ref Counters.FloatOps); break;
                case Opcode.FMUL: c.F[rd] = c.F[rs1] * c.F[rs2]; Interlocked.Increment(ref Counters.FloatOps); break;
                case Opcode.FDIV:
                    if (c.F[rs2] == 0) RaiseException(c, ExceptionType.DivideByZero);
                    else c.F[rd] = c.F[rs1] / c.F[rs2];
                    Interlocked.Increment(ref Counters.FloatOps); break;
                case Opcode.FCMP:
                    if (c.F[rs1] == c.F[rs2]) c.Flags |= StatusFlags.Zero; else c.Flags &= ~StatusFlags.Zero;
                    if (c.F[rs1] < c.F[rs2]) c.Flags |= StatusFlags.Sign; else c.Flags &= ~StatusFlags.Sign;
                    Interlocked.Increment(ref Counters.FloatOps); break;
                case Opcode.FCVT: c.F[rd] = (double)(long)c.R[rs1]; Interlocked.Increment(ref Counters.FloatOps); break;

                case Opcode.VADD: c.V[rd] = c.V[rs1] + c.V[rs2]; Interlocked.Increment(ref Counters.SIMDOps); break;
                case Opcode.VSUB: c.V[rd] = c.V[rs1] - c.V[rs2]; Interlocked.Increment(ref Counters.SIMDOps); break;
                case Opcode.VMUL: c.V[rd] = c.V[rs1] * c.V[rs2]; Interlocked.Increment(ref Counters.SIMDOps); break;
                case Opcode.VDOT:
                    // dot product of 4x16-bit lanes
                    ulong dot = 0;
                    for (int i = 0; i < 4; i++) dot += ((c.V[rs1] >> (i * 16)) & 0xFFFF) * ((c.V[rs2] >> (i * 16)) & 0xFFFF);
                    c.V[rd] = dot; Interlocked.Increment(ref Counters.SIMDOps); break;

                case Opcode.XCHG:
                    ulong tmp = c.R[rd]; c.R[rd] = c.R[rs1]; c.R[rs1] = tmp;
                    Interlocked.Increment(ref Counters.AtomicOps); break;

                case Opcode.CAS: // Compare-and-swap: if R[rd] == R[rs1], set R[rd] = R[rs2]
                    if (c.R[rd] == c.R[rs1]) { c.R[rd] = c.R[rs2]; c.Flags |= StatusFlags.Zero; }
                    else c.Flags &= ~StatusFlags.Zero;
                    Interlocked.Increment(ref Counters.AtomicOps); break;

                case Opcode.SYSCALL:
                    ulong sysnum = c.R[0];
                    c.R[0] = Syscalls?.Handle(c, sysnum) ?? ulong.MaxValue;
                    break;

                case Opcode.INT:
                    if (c.Mode != PrivilegeMode.Kernel) RaiseException(c, ExceptionType.GeneralProtection);
                    else PIC.Raise((int)(imm & 0xFF));
                    break;

                case Opcode.IRET:
                    c.PC = c.CSR[0]; // restore PC from CSR
                    c.Mode = (PrivilegeMode)c.CSR[1];
                    instrSize = 1;
                    return;

                case Opcode.FENCE: Thread.MemoryBarrier(); instrSize = 1; break;
                case Opcode.MFENCE: Thread.MemoryBarrier(); instrSize = 1; break;

                case Opcode.CACHE_INV: c.Mem.InvalidateL1(); break;
                case Opcode.CACHE_FLUSH: c.Mem.FlushDirtyL1(); break;

                case Opcode.PREFETCH:
                    // Prefetch hint — just warm the cache
                    try { c.Mem.Read(c.R[rs1] + imm); } catch { }
                    break;

                case Opcode.IN:
                    var ioIn = FindIO(imm);
                    if (ioIn != null) { Interlocked.Increment(ref Counters.IOReads); c.R[rd] = ioIn.Read(imm); }
                    break;

                case Opcode.OUT:
                    var ioOut = FindIO(imm);
                    if (ioOut != null) { Interlocked.Increment(ref Counters.IOWrites); ioOut.Write(imm, (byte)c.R[rs1]); }
                    break;

                default:
                    RaiseException(c, ExceptionType.IllegalInstruction);
                    break;
            }

            c.PC += instrSize;
        }

        void UpdateFlags(Core c, ulong result, ulong a, ulong b, bool isSub)
        {
            c.Flags = StatusFlags.None;
            if (result == 0) c.Flags |= StatusFlags.Zero;
            if ((result & 0x8000000000000000UL) != 0) c.Flags |= StatusFlags.Sign;
            if (isSub) { if (a < b) c.Flags |= StatusFlags.Carry; }
            else { if (result < a) c.Flags |= StatusFlags.Carry; }
            // Overflow: both positive, result negative OR both negative, result positive
            bool aSign = (a & 0x8000000000000000UL) != 0;
            bool bSign = (b & 0x8000000000000000UL) != 0;
            bool rSign = (result & 0x8000000000000000UL) != 0;
            if (!isSub && aSign == bSign && rSign != aSign) c.Flags |= StatusFlags.Overflow;
            // Parity
            ulong pByte = result & 0xFF;
            int bits = 0; while (pByte != 0) { bits += (int)(pByte & 1); pByte >>= 1; }
            if ((bits & 1) == 0) c.Flags |= StatusFlags.Parity;
        }

        void UpdateZeroSign(Core c, ulong val)
        {
            c.Flags &= ~(StatusFlags.Zero | StatusFlags.Sign);
            if (val == 0) c.Flags |= StatusFlags.Zero;
            if ((val & 0x8000000000000000UL) != 0) c.Flags |= StatusFlags.Sign;
        }

        void Push(Core c, ulong value)
        {
            c.R[31] -= 8;
            c.Mem.WriteU64(c.R[31], value);
        }

        ulong Pop(Core c)
        {
            ulong val = c.Mem.ReadU64(c.R[31]);
            c.R[31] += 8;
            return val;
        }

        void RaiseException(Core c, ExceptionType type)
        {
            Console.WriteLine($"[EXCEPTION] Core {c.CoreID}: {type} at PC=0x{c.PC:X}");
            // Save context to CSR
            c.CSR[0] = c.PC;
            c.CSR[1] = (ulong)c.Mode;
            // Switch to kernel mode and jump to handler
            c.Mode = PrivilegeMode.Kernel;
            c.PC = c.ExceptionBase + (ulong)type * 0x10;
            if (type == ExceptionType.IllegalInstruction || type == ExceptionType.GeneralProtection)
                c.Running = false;
        }

        // ============================================================
        // BINARY LOADER V2
        // ============================================================
        public void LoadELF(ELFBinary elf, Core core)
        {
            foreach (var section in elf.Sections)
            {
                if (section.Type != SectionType.BSS && section.Data != null)
                    core.Mem.LoadSegment(section.Data, section.VAddr);
                else if (section.Type == SectionType.BSS && section.Data != null)
                {
                    // Zero initialize BSS
                    for (ulong i = 0; i < (ulong)section.Data.Length; i++) core.Mem.Write(section.VAddr + i, 0);
                }
                if (section.Type == SectionType.Stack) core.R[31] = section.VAddr + (ulong)(section.Data?.Length ?? 0);
            }
            core.PC = elf.EntryPoint;
            Console.WriteLine($"[LOADER] Loaded '{elf.Name}': entry=0x{elf.EntryPoint:X}, {elf.Sections.Count} sections");
        }

        // ============================================================
        // COMPILER PIPELINE: source text -> bytecode
        // ============================================================
        public byte[] CompileSource(string source)
        {
            Console.WriteLine("[COMPILER] Lexing...");
            var lexer = new Lexer(source);
            var tokens = lexer.Tokenize();

            Console.WriteLine("[COMPILER] Parsing...");
            var parser = new Parser(tokens);
            var ast = parser.ParseProgram();

            Console.WriteLine("[COMPILER] Generating IR...");
            var irGen = new IRGenerator();
            var ir = irGen.Generate(ast);

            Console.WriteLine($"[COMPILER] IR: {ir.Count} nodes");

            Console.WriteLine("[COMPILER] Scheduling instructions...");
            ir = InstrScheduler.Schedule(ir);

            Console.WriteLine("[COMPILER] Optimizing...");
            ir = Optimizer.Optimize(ir);

            Console.WriteLine($"[COMPILER] Optimized IR: {ir.Count} nodes, {Counters.BytecodeOptimizations} optimizations applied");

            Console.WriteLine("[COMPILER] JIT compiling...");
            var jitFn = JIT.Compile(ir, "main");

            Console.WriteLine("[COMPILER] Emitting bytecode via assembler...");
            // Convert IR to assembly text then assemble
            var sb = new StringBuilder();
            sb.AppendLine(".org 0x1000");
            int reg = 0;
            var tempRegs = new Dictionary<string, int>();
            foreach (var node in ir)
            {
                if (node.Op == IROpcode.Label) { sb.AppendLine($"{node.Label}:"); continue; }
                int GetReg(string s)
                {
                    if (s == null) return 0;
                    if (!tempRegs.TryGetValue(s, out int r)) { r = reg++ % 28 + 1; tempRegs[s] = r; } // avoid r0=zero, r29-r31=special
                    return r;
                }
                switch (node.Op)
                {
                    case IROpcode.Mov:
                        if (node.Src1 == null) sb.AppendLine($"MOVI r{GetReg(node.Dest)}, {node.Imm}");
                        else sb.AppendLine($"MOV r{GetReg(node.Dest)}, r{GetReg(node.Src1)}");
                        break;
                    case IROpcode.Add: sb.AppendLine($"ADD r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.Sub: sb.AppendLine($"SUB r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.Mul: sb.AppendLine($"MUL r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.Div: sb.AppendLine($"DIV r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.And: sb.AppendLine($"AND r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.Or: sb.AppendLine($"OR r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.Xor: sb.AppendLine($"XOR r{GetReg(node.Dest)}, r{GetReg(node.Src1)}, r{GetReg(node.Src2)}"); break;
                    case IROpcode.Not: sb.AppendLine($"NOT r{GetReg(node.Dest)}, r{GetReg(node.Src1)}"); break;
                    case IROpcode.Jmp: sb.AppendLine($"JMP {node.Label ?? "0"}"); break;
                    case IROpcode.Je: sb.AppendLine($"JE {node.Label ?? "0"}"); break;
                    case IROpcode.Jne: sb.AppendLine($"JNE {node.Label ?? "0"}"); break;
                    case IROpcode.Call: sb.AppendLine($"CALL {node.Label ?? "0"}"); break;
                    case IROpcode.Ret: sb.AppendLine("RET"); break;
                    case IROpcode.Load: sb.AppendLine($"LD r{GetReg(node.Dest)}, r{GetReg(node.Src1)}"); break;
                    case IROpcode.Store: sb.AppendLine($"ST r{GetReg(node.Src1)}, {node.Imm}"); break;
                    case IROpcode.Nop: sb.AppendLine("NOP"); break;
                    case IROpcode.Halt: sb.AppendLine("HALT"); break;
                }
            }
            sb.AppendLine("HALT");
            string asmText = sb.ToString();
            Console.WriteLine("[COMPILER] Assembly generated:");
            Console.WriteLine(asmText);

            return Assembler.Assemble(asmText);
        }

        // ============================================================
        // GPU / SHADER
        // ============================================================
        public ulong RunShader(string code, ulong input) => GPU.RunShaderSource(code, input);
        public void DispatchShader(string code, ulong input) => GPU.Enqueue(() => GPU.RunShaderSource(code, input));
        public void RegisterShader(string name, string src) => GPU.RegisterShader(name, src);
        public void DispatchNamedShader(string name, ulong input) => GPU.Enqueue(() => GPU.RunNamedShader(name, input));
        public void ExecuteGPUQueue() => GPU.ExecuteQueue();

        // ============================================================
        // VM LIFECYCLE
        // ============================================================
        public void StartVM()
        {
            ioRegions.Add(timer);
            ioRegions.Add(uart);
            profiler.Start();
            foreach (var core in cores) core.Thread.Start();
            Console.WriteLine($"[VM] Started {cores.Count} core(s)");
        }

        public void StopVM()
        {
            foreach (var core in cores) { core.Running = false; core.CTS.Cancel(); }
            foreach (var core in cores)
            {
                try { if (core.Thread != null && core.Thread.IsAlive) core.Thread.Join(500); }
                catch (System.Threading.ThreadStateException) { /* Thread was not started; ignore */ }
            }
            profiler.Stop();
            Console.WriteLine("[VM] Stopped");
        }

        public void StartProfiling() => profiler.Restart();
        public void StopProfiling() => profiler.Stop();

        // ============================================================
        // DEBUGGER FACADE
        // ============================================================
        public Debugger GetDebugger() => new Debugger(this, cores);
        public Core GetCore(int id) => id < cores.Count ? cores[id] : null;

        public void PrintStats() => Counters.Print();

        public void Dispose()
        {
            if (!disposed) { StopVM(); disposed = true; }
        }
    }

    // ================================================================
    // EXAMPLE / DEMO ENTRY POINT
    // ================================================================
    public class VMExample
    {
        public static void MainExample(string[] args)
        {
            Console.WriteLine("============================================================");
            Console.WriteLine("        MINDWORKS VM v2.0 — FULL ARCHITECTURE               ");
            Console.WriteLine("============================================================\n");

            using var vm = new brains();

            // Create 2 CPU cores
            var core0 = vm.CreateCore();
            var core1 = vm.CreateCore();

            // ---- ASSEMBLER DEMO ----
            Console.WriteLine(">>> ASSEMBLER DEMO");
            var asm = vm.Assembler;
            string asmSource = @"
; Demo: compute 6 * 7 and store result
.org 0x1000
start:
    MOVI r1, 6          ; r1 = 6
    MOVI r2, 7          ; r2 = 7
    MUL r3, r1, r2      ; r3 = 42
    MOVI r4, 0xFF00     ; output port
    ST r3, 0xFF00       ; store result
    MOVI r0, 1          ; syscall: SYS_EXIT
    SYSCALL
    HALT
";
            byte[] code = asm.Assemble(asmSource);
            Console.WriteLine($"[ASM] Assembled {code.Length} bytes");
            asm.PrintSymbolTable();

            // Load into core0
            var elf = new ELFBinary
            {
                Name = "demo.elf",
                EntryPoint = 0x1000,
                Sections = new List<Section>
                {
                    new Section { Name = ".text", Type = SectionType.Code, VAddr = 0x1000, Data = code },
                    new Section { Name = ".stack", Type = SectionType.Stack, VAddr = 0x7FF00000, Data = new byte[0x10000] },
                }
            };
            vm.LoadELF(elf, core0);

            // ---- VFS DEMO ----
            Console.WriteLine("\n>>> VIRTUAL FILESYSTEM DEMO");
            vm.VFS.Mkdir("/etc");
            vm.VFS.Mkdir("/bin");
            vm.VFS.WriteText("/etc/config.txt", "kernel=mindworks\nversion=2.0\nmax_cores=16");
            vm.VFS.WriteText("/etc/readme.md", "# MindWorks VM\nA full-featured virtual machine.");
            vm.VFS.WriteFile("/bin/demo.elf", code);
            Console.WriteLine($"[VFS] /etc/config.txt: {vm.VFS.ReadText("/etc/config.txt")}");
            Console.WriteLine("[VFS] File tree:");
            vm.VFS.PrintTree();

            // ---- COMPILER PIPELINE DEMO ----
            Console.WriteLine("\n>>> COMPILER PIPELINE DEMO");
            string hiLevelSource = @"
fn add(a, b) {
    return a + b;
}
fn main() {
    let x = 10;
    let y = 20;
    let z = add(x, y);
    while (z > 0) {
        z = z - 1;
    }
    return z;
}
";
            byte[] compiled = vm.CompileSource(hiLevelSource);
            Console.WriteLine($"[COMPILER] Output: {compiled.Length} bytes");

            // ---- GPU / SHADER DEMO ----
            Console.WriteLine("\n>>> GPU SHADER DEMO");
            vm.RegisterShader("plasma", @"
// Plasma effect shader
ITOF
FMUL 3.14159
FSIN
FMUL 255.0
FTOI
OUT
");
            vm.DispatchNamedShader("plasma", 42);
            vm.DispatchShader("ADD 100\nMUL 2\nSHL 1\nOUT", 7);
            vm.ExecuteGPUQueue();

            // ---- INTERRUPT CONTROLLER DEMO ----
            Console.WriteLine("\n>>> INTERRUPT CONTROLLER DEMO");
            vm.PIC.RegisterHandler(10, () => Console.WriteLine("[IRQ10] Custom interrupt fired!"), 50);
            vm.PIC.Raise(10);
            vm.PIC.DispatchPending();

            // ---- DEBUGGER DEMO ----
            Console.WriteLine("\n>>> DEBUGGER DEMO");
            var dbg = vm.GetDebugger();
            dbg.PrintCoreState(core0);
            dbg.PrintCacheState(core0);

            // Run a few steps manually
            Console.WriteLine("\n>>> RUNNING 5 STEPS ON CORE0");
            vm.StartProfiling();
            for (int i = 0; i < 5; i++) vm.Step(core0);
            vm.StopProfiling();

            dbg.PrintCoreState(core0);
            dbg.Disassemble(core0, 0x1000, 8);
            dbg.PrintMemory(core0, 0x1000, 64);

            // ---- SCHEDULER DEMO ----
            Console.WriteLine("\n>>> SCHEDULER DEMO");
            vm.GetScheduler().CreateProcess("proc_a", 0x1000, priority: 1);
            vm.GetScheduler().CreateProcess("proc_b", 0x1000, priority: 2);
            vm.GetScheduler().PrintProcessTable();

            // ---- PRINT STATS ----
            Console.WriteLine($"\n>>> ELAPSED: {vm.ElapsedMs}ms\n");
            vm.PrintStats();

            Console.WriteLine("\n[VM] All demos complete. Press Enter to exit or 'dbg' for debugger REPL.");
            string input = Console.ReadLine();
            if (input?.Trim() == "dbg") dbg.RunREPL(core0);
        }
    }
}