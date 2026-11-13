"""Push-to-talk voice input: sounddevice recorder + faster-whisper (offline, NO LLM).  SIMULATION UI helper.

    v = VoiceInput(log=print)
    v.start_loading()                      # background thread; v.status: "loading" | "ready" | "error"
    v.start_recording()                    # while the button is held ...
    audio = v.stop_recording()             # float32 mono 16 kHz
    v.transcribe_async(audio, callback)    # callback(text, info) from the worker thread
    text, info = v.transcribe_file("x.wav")

Model: faster-whisper "small.en" on CUDA float16 (falls back to int8 on CPU; the device actually used is in v.device).
The model is downloaded on first use into <project>/models/whisper (git-ignored); the download is logged.
The recognised text only goes to the rule-based parser (sim/commands.py); there is no language model in the loop.
"""
import math
import os
import threading
import time
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MODEL_DIR = os.path.join(ROOT, "models", "whisper")
SAMPLE_RATE = 16000
MAX_RECORD_S = 10.0
MIN_RECORD_S = 0.3
VOCAB_PROMPT = ("red cube green cube blue cube cube one cube two cube three zone one zone two zone three stack on top "
                "place put move all cubes home")


_DLL_DIRS = []


def add_cuda_dll_dirs():
    """pip wheels nvidia-cublas-cu12 / nvidia-cudnn-cu12 keep their DLLs in site-packages/nvidia/*/bin (not on PATH):
    register them so ctranslate2 finds cublas64_12.dll / cudnn*.dll. Idempotent."""
    if _DLL_DIRS or not hasattr(os, "add_dll_directory"):
        return list(_DLL_DIRS)
    try:
        import importlib.util
        spec = importlib.util.find_spec("nvidia")
        for base in (list(spec.submodule_search_locations) if spec and spec.submodule_search_locations else []):
            for sub in sorted(os.listdir(base)):
                b = os.path.join(base, sub, "bin")
                if os.path.isdir(b):
                    os.add_dll_directory(b)
                    os.environ["PATH"] = b + os.pathsep + os.environ.get("PATH", "")
                    _DLL_DIRS.append(b)
    except Exception:
        pass
    return list(_DLL_DIRS)


def model_present(size="small.en", cache_dir=MODEL_DIR):
    """True if the faster-whisper model folder exists in the project cache (HF cache layout)."""
    repo = {"small.en": "Systran--faster-whisper-small.en", "base.en": "Systran--faster-whisper-base.en",
            "tiny.en": "Systran--faster-whisper-tiny.en", "medium.en": "Systran--faster-whisper-medium.en"}.get(
        size, "Systran--faster-whisper-" + size)
    snap = os.path.join(cache_dir, "models--" + repo, "snapshots")
    return os.path.isdir(snap) and any(os.path.exists(os.path.join(snap, d, "model.bin")) for d in os.listdir(snap))


def read_wav_16k(path):
    """wav (any rate, 16 bit PCM / float, any channel count) -> float32 mono 16 kHz."""
    with wave.open(path, "rb") as f:
        n, ch, sw, sr = f.getnframes(), f.getnchannels(), f.getsampwidth(), f.getframerate()
        raw = f.readframes(n)
    if sw == 2:
        a = np.frombuffer(raw, "<i2").astype(np.float32) / 32768.0
    elif sw == 4:
        a = np.frombuffer(raw, "<i4").astype(np.float32) / 2147483648.0
    elif sw == 1:
        a = (np.frombuffer(raw, np.uint8).astype(np.float32) - 128.0) / 128.0
    else:
        raise ValueError("unsupported sample width %d" % sw)
    if ch > 1:
        a = a.reshape(-1, ch).mean(1)
    if sr != SAMPLE_RATE:
        t = np.arange(int(len(a) * SAMPLE_RATE / sr)) * (sr / SAMPLE_RATE)
        a = np.interp(t, np.arange(len(a)), a).astype(np.float32)
    return a


class VoiceInput:
    def __init__(self, model_size="small.en", cache_dir=MODEL_DIR, log=print):
        self.model_size, self.cache_dir, self.log = model_size, cache_dir, log
        self.model = None
        self.status = "idle"            # idle | loading | ready | error
        self.device = ""                # "GPU (cuda float16)" | "CPU (int8)"
        self.error = ""
        self.load_s = 0.0
        self.downloaded = False
        self.last_transcribe_s = 0.0
        self.level = 0.0                # live RMS level 0..1 while recording
        self.recording = False
        self.timed_out = False
        self._frames = []
        self._stream = None
        self._lock = threading.Lock()
        self._thread = None
        self.mic_name = ""
        self.mic_error = ""
        self.check_mic()

    # ------------------------------------------------------------------ microphone
    def check_mic(self):
        """True if a default input device exists (the button is disabled otherwise)."""
        try:
            import sounddevice as sd
            d = sd.query_devices(kind="input")
            self.mic_name = d["name"]
            self.mic_error = ""
            return True
        except Exception as e:
            self.mic_name, self.mic_error = "", "no microphone (%s)" % str(e).splitlines()[0][:80]
            return False

    @property
    def mic_ok(self):
        return bool(self.mic_name)

    def start_recording(self):
        """Open a 16 kHz mono input stream; frames are collected until stop_recording() or MAX_RECORD_S."""
        import sounddevice as sd
        with self._lock:
            if self.recording:
                return True
            self._frames, self.level, self.timed_out = [], 0.0, False
            n_max = int(MAX_RECORD_S * SAMPLE_RATE)
            count = [0]

            def cb(indata, frames, tinfo, status):
                x = indata[:, 0].copy()
                if count[0] < n_max:
                    self._frames.append(x[: n_max - count[0]])
                count[0] += len(x)
                self.level = float(min(1.0, math.sqrt(float(np.mean(x ** 2))) * 6.0))
                if count[0] >= n_max:
                    self.timed_out = True

            try:
                self._stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32", callback=cb)
                self._stream.start()
            except Exception as e:
                self.mic_error = "mic open failed: %s" % str(e).splitlines()[0][:100]
                self.log("[voice] " + self.mic_error)
                self._stream = None
                return False
            self.recording = True
            self.t_rec0 = time.perf_counter()
            return True

    def rec_seconds(self):
        return (time.perf_counter() - self.t_rec0) if self.recording else 0.0

    def stop_recording(self):
        """-> float32 array (n,) at 16 kHz (empty if nothing recorded)."""
        with self._lock:
            if not self.recording:
                return np.zeros(0, np.float32)
            self.recording = False
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None
            self.level = 0.0
            return np.concatenate(self._frames) if self._frames else np.zeros(0, np.float32)

    # ------------------------------------------------------------------ model
    def start_loading(self):
        if self._thread is not None:
            return
        self.status = "loading"
        self._thread = threading.Thread(target=self._load, daemon=True, name="voice-load")
        self._thread.start()

    def _make_model(self, device, compute):
        from faster_whisper import WhisperModel
        return WhisperModel(self.model_size, device=device, compute_type=compute, download_root=self.cache_dir)

    def _load(self):
        t0 = time.perf_counter()
        try:
            os.makedirs(self.cache_dir, exist_ok=True)
            self.downloaded = not model_present(self.model_size, self.cache_dir)
            if self.downloaded:
                self.log("[voice] model %s not in %s: downloading (one time, ~480 MB from Hugging Face) ..." % (
                    self.model_size, self.cache_dir))
            add_cuda_dll_dirs()
            try:
                import ctranslate2
                use_cuda = ctranslate2.get_cuda_device_count() > 0
            except Exception:
                use_cuda = False
            model = None
            if use_cuda:
                try:
                    model = self._make_model("cuda", "float16")
                    self._warmup(model)
                    self.device = "GPU (cuda float16)"
                except Exception as e:
                    self.log("[voice] CUDA failed (%s) -> CPU int8" % str(e).splitlines()[0][:120])
                    model = None
            if model is None:
                model = self._make_model("cpu", "int8")
                self.device = "CPU (int8)"
            self.model = model
            self.load_s = time.perf_counter() - t0
            self.status = "ready"
            self.log("[voice] model %s ready on %s in %.1f s%s" % (
                self.model_size, self.device, self.load_s, " (downloaded)" if self.downloaded else ""))
        except Exception as e:
            self.status, self.error = "error", str(e).splitlines()[0][:150] if str(e) else repr(e)
            self.log("[voice] model load failed: %s" % self.error)

    def _warmup(self, model):
        """One tiny decode so a missing CUDA library fails here (-> CPU fallback) and not on the first command."""
        list(model.transcribe(np.zeros(SAMPLE_RATE, np.float32), beam_size=1, language="en")[0])

    @property
    def ready(self):
        return self.status == "ready" and self.model is not None

    def status_text(self):
        if self.status == "ready":
            return "Voice: ready (%s)" % ("GPU" if self.device.startswith("GPU") else "CPU")
        if self.status == "error":
            return "Voice: unavailable (%s)" % self.error[:60]
        return "Voice: loading..." + (" (downloading model)" if self.downloaded else "")

    # ------------------------------------------------------------------ transcription
    def transcribe(self, audio):
        """float32 mono 16 kHz -> (text, info dict). Raises RuntimeError when the model is not ready."""
        if not self.ready:
            raise RuntimeError("voice model not ready")
        audio = np.asarray(audio, np.float32)
        t0 = time.perf_counter()
        segs, info = self._run(audio)
        text = " ".join(s.text.strip() for s in segs).strip()
        self.last_transcribe_s = time.perf_counter() - t0
        return text, {"seconds": self.last_transcribe_s, "audio_s": len(audio) / SAMPLE_RATE, "device": self.device,
                      "language": getattr(info, "language", "en")}

    def _run(self, audio):
        kw = dict(beam_size=5, language="en", vad_filter=True, initial_prompt=VOCAB_PROMPT, condition_on_previous_text=False)
        try:
            segs, info = self.model.transcribe(audio, **kw)
            return list(segs), info
        except Exception as e:
            if self.device.startswith("GPU"):       # e.g. cublas / cudnn DLL missing at run time: one fallback to CPU
                self.log("[voice] GPU transcribe failed (%s) -> reloading on CPU int8" % str(e).splitlines()[0][:100])
                self.model = self._make_model("cpu", "int8")
                self.device = "CPU (int8)"
                segs, info = self.model.transcribe(audio, **kw)
                return list(segs), info
            raise

    def transcribe_file(self, path):
        return self.transcribe(read_wav_16k(path))

    def transcribe_async(self, audio, callback):
        """Worker thread: callback(text, info, error) - text '' + error str on failure (never raises)."""
        def work():
            try:
                text, info = self.transcribe(audio)
                callback(text, info, None)
            except Exception as e:
                self.log("[voice] transcription error: %r" % e)
                callback("", {}, str(e) or repr(e))
        threading.Thread(target=work, daemon=True, name="voice-transcribe").start()
