using System;

namespace mindworks
{
    public static class BrowserPlatform
    {
        public static bool InitDisplay(int topW, int topH, int botW, int botH, string title)
        {
            if (!OperatingSystem.IsBrowser()) return false;
            try
            {
                BrowserBridge.InitCanvas(topW, topH, botW, botH);
                BrowserBridge.SetTitle(title ?? "MindWorks 3DS");
                return true;
            }
            catch (Exception ex) { Console.WriteLine("[WASM] InitDisplay failed: " + ex); return false; }
        }

        public static void PresentFrame(byte[] topRGBA, byte[] botRGBA, int topW, int topH, int botW, int botH)
        {
            if (!OperatingSystem.IsBrowser()) return;
            try { BrowserBridge.PresentFrame(topRGBA, botRGBA, topW, topH, botW, botH); }
            catch (Exception ex) { Console.WriteLine("[WASM] PresentFrame failed: " + ex); }
        }

        public static void InitAudio(int sampleRate)
        {
            if (!OperatingSystem.IsBrowser()) return;
            try { BrowserBridge.InitAudio(sampleRate); }
            catch (Exception ex) { Console.WriteLine("[WASM] InitAudio failed: " + ex); }
        }

        public static void QueueAudio(short[] pcmInterleaved)
        {
            if (!OperatingSystem.IsBrowser()) return;
            try
            {
                // Convert Int16 array to byte[] (little-endian) for JS interop
                if (pcmInterleaved == null || pcmInterleaved.Length == 0) return;
                byte[] bytes = new byte[pcmInterleaved.Length * 2];
                Buffer.BlockCopy(pcmInterleaved, 0, bytes, 0, bytes.Length);
                BrowserBridge.QueueAudio(bytes);
            }
            catch (Exception ex) { Console.WriteLine("[WASM] QueueAudio failed: " + ex); }
        }
    }
}
