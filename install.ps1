<#
.SYNOPSIS  One-step installer for the XarmAI simulation (Windows, Python 3.12). Safe to re-run.
.DESCRIPTION
  Creates .venv, installs requirements.txt (+ requirements-gpu.txt if an NVIDIA GPU is found), runs a smoke check.
  Nothing outside this folder is changed unless you ask (-InstallPython, -Ollama).
.PARAMETER InstallPython  If Python 3.12 is missing, install it with winget.
.PARAMETER Ollama         Install Ollama via winget if missing, start it, pull qwen2.5:3b (AI brain for free-form commands).
.PARAMETER Whisper        Pre-download the faster-whisper small.en voice model (~480 MB) into models/whisper.
.PARAMETER All            -Ollama and -Whisper.
.PARAMETER CpuOnly        Do not install the NVIDIA CUDA wheels (voice runs on CPU).
.PARAMETER Test           Run the unit tests (pytest -q tests/unit -x) at the end.
#>
param([switch]$InstallPython, [switch]$Ollama, [switch]$Whisper, [switch]$All, [switch]$CpuOnly, [switch]$Test)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
if ($All) { $Ollama = $true; $Whisper = $true }

function Say($m)  { Write-Host "[xarm] $m" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "[ ok ] $m" -ForegroundColor Green }
function Warn($m) { Write-Host "[warn] $m" -ForegroundColor Yellow }
function Fail($m, $fix) {
    Write-Host "[FAIL] $m" -ForegroundColor Red
    if ($fix) { Write-Host "       What to do: $fix" -ForegroundColor Yellow }
    exit 1
}
function Native([scriptblock]$cmd, $what) {   # run a native command, stop on non-zero exit
    & $cmd
    if ($LASTEXITCODE -ne 0) { Fail "$what failed (exit $LASTEXITCODE)." 'read the messages above, fix, run install.bat again.' }
}

# ---- 1. Python 3.12 ----
Say 'Looking for Python 3.12 ...'
function Find-Python {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $p = & py -3.12 -c "import sys;print(sys.executable)" 2>$null
        if ($LASTEXITCODE -eq 0 -and $p) { return ($p | Select-Object -First 1).Trim() }
    }
    if (Get-Command python -ErrorAction SilentlyContinue) {
        $v = & python -c "import sys;print('%d.%d' % sys.version_info[:2])" 2>$null
        if ($LASTEXITCODE -eq 0 -and $v -eq '3.12') { return (& python -c "import sys;print(sys.executable)").Trim() }
    }
    return $null
}
$pyExe = Find-Python
if (-not $pyExe) {
    if ($InstallPython) {
        if (-not (Get-Command winget -ErrorAction SilentlyContinue)) { Fail 'winget not found.' 'install Python 3.12 from https://www.python.org/downloads/ (tick Add to PATH), then run install.bat again.' }
        Say 'Installing Python 3.12 with winget (per-user) ...'
        & winget install --id Python.Python.3.12 -e --scope user --accept-package-agreements --accept-source-agreements
        $pyExe = Find-Python
        if (-not $pyExe) { Fail 'Python 3.12 installed but not visible yet.' 'close this window, open a new one, run install.bat again.' }
    } else {
        Fail 'Python 3.12 not found.' 'install it from https://www.python.org/downloads/ (tick Add to PATH) or run:  install.bat -InstallPython'
    }
}
Ok "Python 3.12: $pyExe"

# ---- 2. venv ----
$venvPy = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (Test-Path $venvPy) {
    $v = & $venvPy -c "import sys;print('%d.%d' % sys.version_info[:2])" 2>$null
    if ($v -ne '3.12') { Fail ".venv exists but is Python $v, not 3.12." 'delete the .venv folder and run install.bat again.' }
    Ok '.venv already exists'
} else {
    Say 'Creating .venv ...'
    Native { & $pyExe -m venv .venv } 'venv creation'
    Ok '.venv created'
}

# ---- 3. packages ----
Say 'Upgrading pip ...'
Native { & $venvPy -m pip install --disable-pip-version-check -q --upgrade pip } 'pip upgrade'
Say 'Installing requirements.txt (first time takes a few minutes) ...'
Native { & $venvPy -m pip install --disable-pip-version-check -r requirements.txt } 'pip install -r requirements.txt'
Ok 'base packages installed'

$gpu = $false
if (-not $CpuOnly) {
    if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
        & nvidia-smi -L 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) { $gpu = $true }
    }
}
if ($gpu) {
    Say 'NVIDIA GPU found: installing requirements-gpu.txt (CUDA libs for GPU voice, ~1 GB) ...'
    Native { & $venvPy -m pip install --disable-pip-version-check -r requirements-gpu.txt } 'pip install -r requirements-gpu.txt'
    Ok 'GPU voice libraries installed'
} elseif ($CpuOnly) {
    Warn '-CpuOnly: skipped GPU wheels. Voice will run on CPU (slower).'
} else {
    Warn 'No NVIDIA GPU detected (nvidia-smi). Voice will run on CPU (slower).'
}

# ---- 4. optional: Whisper model ----
if ($Whisper) {
    Say 'Downloading faster-whisper small.en (~480 MB) into models/whisper ...'
    $code = @'
import sys, os
sys.path.insert(0, os.getcwd())
from sim import voice
if voice.model_present("small.en"):
    print("already present"); raise SystemExit(0)
from faster_whisper import WhisperModel
WhisperModel("small.en", device="cpu", compute_type="int8", download_root=voice.MODEL_DIR)
print("downloaded" if voice.model_present("small.en") else "MISSING")
raise SystemExit(0 if voice.model_present("small.en") else 1)
'@
    Native { $code | & $venvPy - } 'whisper model download (needs internet)'
    Ok 'whisper model ready'
}

# ---- 5. optional: Ollama ----
if ($Ollama) {
    $guess = Join-Path $env:LOCALAPPDATA 'Programs\Ollama\ollama.exe'
    $olExe = $null
    $c = Get-Command ollama -ErrorAction SilentlyContinue
    if ($c) { $olExe = $c.Source } elseif (Test-Path $guess) { $olExe = $guess }
    if ($olExe) {
        Ok 'Ollama already installed - skipping install'
    } else {
        if (-not (Get-Command winget -ErrorAction SilentlyContinue)) { Fail 'winget not found, cannot install Ollama.' 'install it from https://ollama.com/download then run: install.bat -Ollama' }
        Say 'Installing Ollama with winget ...'
        & winget install --id Ollama.Ollama -e --accept-package-agreements --accept-source-agreements
        $c = Get-Command ollama -ErrorAction SilentlyContinue
        if ($c) { $olExe = $c.Source } elseif (Test-Path $guess) { $olExe = $guess }
        if (-not $olExe) { Fail 'Ollama installed but not found.' 'open a new terminal and run: install.bat -Ollama' }
    }
    $up = $false
    try { Invoke-RestMethod -Uri 'http://localhost:11434/api/tags' -TimeoutSec 3 | Out-Null; $up = $true } catch {}
    if ($up) { Ok 'Ollama server already running' }
    else {
        Say 'Starting Ollama server ...'
        Start-Process -FilePath $olExe -ArgumentList 'serve' -WindowStyle Hidden
        for ($i = 0; $i -lt 20 -and -not $up; $i++) {
            Start-Sleep -Seconds 1
            try { Invoke-RestMethod -Uri 'http://localhost:11434/api/tags' -TimeoutSec 2 | Out-Null; $up = $true } catch {}
        }
        if (-not $up) { Fail 'Ollama server did not start.' 'start the Ollama app from the Start menu, then run install.bat -Ollama again.' }
        Ok 'Ollama server running'
    }
    $have = (& $olExe list 2>$null) -join "`n"
    if ($have -match 'qwen2\.5:3b') { Ok 'model qwen2.5:3b already pulled' }
    else {
        Say 'Pulling qwen2.5:3b (~1.9 GB) ...'
        Native { & $olExe pull qwen2.5:3b } 'ollama pull qwen2.5:3b'
        Ok 'model qwen2.5:3b ready'
    }
}

# ---- 6. smoke check ----
Say 'Smoke check ...'
$smoke = @'
import pybullet, cv2, numpy, yaml, reportlab, sounddevice
import dearpygui.dearpygui as dpg
import faster_whisper, pupil_apriltags
print("imports ok: pybullet", pybullet.getAPIVersion(), "| cv2", cv2.__version__, "| faster_whisper", faster_whisper.__version__)
'@
Native { $smoke | & $venvPy -I - } 'smoke import check'
Ok 'all key packages import'
if ($Test) {
    Say 'Running unit tests ...'
    Native { & $venvPy -I -m pytest -q tests/unit -x } 'unit tests'
    Ok 'unit tests pass'
}

Write-Host ''
Write-Host 'Done. Start the app:  run_app.bat   (or  .venv\Scripts\python -I scripts\sim_app.py )' -ForegroundColor Green
Write-Host 'First start calibrates the camera once (about 2.5 min) unless a calibration cache is present.' -ForegroundColor Green
if (-not $Whisper) { Write-Host 'Voice model downloads on first app start (or run: install.bat -Whisper).' }
if (-not $Ollama)  { Write-Host 'Optional AI brain for free-form commands: install.bat -Ollama' }
