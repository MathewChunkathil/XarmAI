"""Non-blocking text-to-speech via Windows SAPI (System.Speech through PowerShell).  SIMULATION UI helper.

    tts.speak("Did you mean: place green cube in zone 2?")   # returns at once; cancels the previous utterance
    tts.stop()

No extra dependencies. Silently does nothing when PowerShell / System.Speech is unavailable (non-Windows, no voice).
The text is passed through stdin (never on the command line), so quotes in the text cannot inject anything.
"""
import subprocess
import sys
import threading

_lock = threading.Lock()
_proc = None
enabled = True
_PS = ("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
       "$s.Rate = 1; $t = [Console]::In.ReadToEnd(); $s.Speak($t)")


def available() -> bool:
    return sys.platform == "win32"


def stop():
    global _proc
    with _lock:
        p, _proc = _proc, None
    if p is not None and p.poll() is None:
        try:
            p.kill()
        except Exception:
            pass


def speak(text: str):
    """Fire and forget; the previous utterance is cut off."""
    if not enabled or not available() or not (text or "").strip():
        return
    stop()

    def run():
        global _proc
        try:
            flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
            p = subprocess.Popen(["powershell", "-NoProfile", "-NonInteractive", "-Command", _PS], stdin=subprocess.PIPE,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags)
            with _lock:
                _proc = p
            p.communicate(input=text.encode("utf-8"), timeout=60)
        except Exception:
            pass
    threading.Thread(target=run, daemon=True).start()
