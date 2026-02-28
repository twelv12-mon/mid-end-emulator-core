// ============================================================
// MINDWORKS LAUNCHER
// Unified entry point for both the VM and 3DS emulator
// ============================================================

using System;
using System.IO;

namespace mindworks
{
    class MindWorksLauncher
    {
        static void Main(string[] args)
        {
            Console.WriteLine("==========================================================");
            Console.WriteLine("   MINDWORKS — VM + 3DS EMULATOR                         ");
            Console.WriteLine("==========================================================");
            Console.WriteLine();

            // If a ROM file is passed, go straight to 3DS mode
            if (args.Length > 0)
            {
                string arg0 = args[0];
                if (File.Exists(arg0))
                {
                    string ext = Path.GetExtension(arg0).ToLower();
                    if (ext == ".3ds" || ext == ".cci" || ext == ".elf" || ext == ".axf")
                    {
                        Console.WriteLine($"[LAUNCHER] ROM detected: {arg0}");
                        DS3Main.Main3DS(args);
                        return;
                    }
                }

                // --vm flag: run the mindworks VM demo
                if (arg0 == "--vm")
                {
                    RunVMDemo();
                    return;
                }

                // --test flag: run 3DS display test
                if (arg0 == "--test" || arg0 == "--display-test")
                {
                    DS3Main.Main3DS(Array.Empty<string>());
                    return;
                }
            }

            // Interactive menu
            Console.WriteLine("What would you like to do?");
            Console.WriteLine("  1) Load a 3DS ROM");
            Console.WriteLine("  2) Run 3DS display test (no ROM needed)");
            Console.WriteLine("  3) Run MindWorks VM demo");
            Console.WriteLine("  4) Exit");
            Console.Write("\n> ");

            string choice = Console.ReadLine()?.Trim();
            switch (choice)
            {
                case "1":
                    Console.Write("ROM path (.3ds / .elf): ");
                    string path = Console.ReadLine()?.Trim();
                    if (!string.IsNullOrEmpty(path))
                        DS3Main.Main3DS(new[] { path });
                    else
                        Console.WriteLine("No path entered.");
                    break;

                case "2":
                    DS3Main.Main3DS(Array.Empty<string>());
                    break;

                case "3":
                    RunVMDemo();
                    break;

                default:
                    Console.WriteLine("Goodbye.");
                    break;
            }
        }

        static void RunVMDemo()
        {
            Console.WriteLine("\n[VM] Starting MindWorks VM demo...");
            using var vm = new brains();

            var core0 = vm.CreateCore();
            var core1 = vm.CreateCore();

            // Simple test program
            string asmSrc = @"
.org 0x1000
start:
    MOVI r1, 100
    MOVI r2, 200
    ADD r3, r1, r2
    MOVI r0, 1
    SYSCALL
    HALT
";
            byte[] code = vm.Assembler.Assemble(asmSrc);
            var elf = new ELFBinary
            {
                Name = "vm_demo.elf",
                EntryPoint = 0x1000,
                Sections = new System.Collections.Generic.List<Section>
                {
                    new Section { Name = ".text", Type = SectionType.Code, VAddr = 0x1000, Data = code },
                    new Section { Name = ".stack", Type = SectionType.Stack, VAddr = 0x7FF00000, Data = new byte[0x10000] },
                }
            };
            vm.LoadELF(elf, core0);
            vm.StartProfiling();

            // Step through it
            for (int i = 0; i < 10 && core0.Running; i++)
                vm.Step(core0);

            vm.StopProfiling();
            vm.PrintStats();

            var dbg = vm.GetDebugger();
            dbg.PrintCoreState(core0);
        }
    }
}