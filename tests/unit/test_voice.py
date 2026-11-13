"""Voice input tests. The model test is skipped when the faster-whisper model is not downloaded (models/whisper) or
when Windows SAPI TTS is unavailable. Run: python -m pytest tests/unit/test_voice.py"""
import os
import subprocess
import sys
import wave
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim import commands as cmd  # noqa: E402
from sim import voice  # noqa: E402


def _write_wav(path, x, sr):
    with wave.open(str(path), "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(sr)
        f.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())


def test_read_wav_resamples_to_16k(tmp_path):
    sr = 44100
    t = np.arange(sr) / sr
    _write_wav(tmp_path / "a.wav", 0.5 * np.sin(2 * np.pi * 440 * t), sr)
    a = voice.read_wav_16k(str(tmp_path / "a.wav"))
    assert a.dtype == np.float32 and abs(len(a) - 16000) <= 2 and 0.45 < np.abs(a).max() < 0.55


def test_no_microphone_is_handled(monkeypatch):
    import sounddevice as sd
    monkeypatch.setattr(sd, "query_devices", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("no device")))
    v = voice.VoiceInput(log=lambda *a: None)
    assert not v.mic_ok and "no microphone" in v.mic_error
    with pytest.raises(RuntimeError):
        v.transcribe(np.zeros(16000, np.float32))          # model not loaded -> clear error, no crash


def test_status_text_before_load():
    v = voice.VoiceInput(log=lambda *a: None)
    assert v.status_text().startswith("Voice: loading")


SENTENCES = [("Place the red cube in zone one.", [("place", 1, 1, None)]),
             ("Stack red on green.", [("stack", 1, None, 2)]),
             ("Move all cubes to zone two.", [("place", 1, 2, None), ("place", 2, 2, None), ("place", 3, 2, None)])]


@pytest.mark.skipif(not voice.model_present(), reason="faster-whisper small.en not downloaded (models/whisper)")
@pytest.mark.skipif(os.name != "nt", reason="needs Windows SAPI TTS to synthesise test audio")
def test_tts_sentences_transcribe_and_parse(tmp_path):
    v = voice.VoiceInput(log=lambda *a: None)
    v._load()
    assert v.ready
    for k, (text, want) in enumerate(SENTENCES):
        wav = tmp_path / ("s%d.wav" % k)
        ps = ("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
              "$f = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, 16, 1); "
              "$s.SetOutputToWaveFile('%s', $f); $s.Speak('%s'); $s.Dispose()" % (wav, text))
        try:
            subprocess.run(["powershell", "-NoProfile", "-Command", ps], check=True, timeout=60, capture_output=True)
        except Exception:
            pytest.skip("SAPI TTS unavailable")
        heard, info = v.transcribe_file(str(wav))
        r = cmd.parse(heard)
        assert r.ok and [t.key() for t in r.tasks] == want, (text, heard, r.errors)
