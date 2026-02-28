using System.Runtime.InteropServices.JavaScript;

namespace mindworks
{
    // Low-level JS imports (bridge.js)
    public static partial class BrowserBridge
    {
        [JSImport("mw_initCanvas", "./wasm/bridge.js")]
        public static partial void InitCanvas(int topW, int topH, int botW, int botH);

        [JSImport("mw_presentFrame", "./wasm/bridge.js")]
        public static partial void PresentFrame(byte[] topRGBA, byte[] botRGBA, int topW, int topH, int botW, int botH);

        [JSImport("mw_initAudio", "./wasm/bridge.js")]
        public static partial void InitAudio(int sampleRate);

        [JSImport("mw_queueAudio", "./wasm/bridge.js")]
        public static partial void QueueAudio(byte[] pcmInterleaved);

        [JSImport("mw_setTitle", "./wasm/bridge.js")]
        public static partial void SetTitle(string title);
    }
}
