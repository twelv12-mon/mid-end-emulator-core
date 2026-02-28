using System;
using System.Text;
using System.Runtime.InteropServices.JavaScript;

namespace mindworks
{
    public static partial class WasmInterop
    {
        // JS will call this with base64-encoded ROM data
        [JSExport]
        public static void Run3DS(string base64Rom)
        {
            try
            {
                byte[] rom = Convert.FromBase64String(base64Rom);
                Console.WriteLine($"[WASM] Received 3DS ROM, {rom.Length} bytes");
                // TODO: actual 3DS emulation isn't supported in WASM build (no SDL/OpenGL)
                // This stub just logs the size; a desktop build should be used instead.
            }
            catch (Exception ex)
            {
                Console.WriteLine("[WASM] Run3DS failed: " + ex);
            }
        }
    }
}
