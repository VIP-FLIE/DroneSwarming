# =============================================================================
# Drone VIP environment doctor  --  Windows 11
#
#   .\scripts\doctor.ps1           inspect and recommend
#   .\scripts\doctor.ps1 -Write    also write .devcontainer\.env
#
# If PowerShell refuses to run this ("running scripts is disabled"), use:
#   powershell -ExecutionPolicy Bypass -File .\scripts\doctor.ps1
# =============================================================================
[CmdletBinding()]
param([switch]$Write)

$ErrorActionPreference = 'Continue'

function Write-Head($t) { Write-Host $t -ForegroundColor White }
function Write-Good($t) { Write-Host "  yes   $t" -ForegroundColor Green }
function Write-Bad ($t) { Write-Host "  no    $t" -ForegroundColor Red }
function Write-Warn($t) { Write-Host "  ??    $t" -ForegroundColor Yellow }
function Write-Info($t) { Write-Host "        $t" -ForegroundColor Gray }

$repoRoot = Split-Path -Parent $PSScriptRoot
$envFile  = Join-Path $repoRoot '.devcontainer\.env'

Write-Host ""
Write-Head "=== Drone VIP environment check ==="
Write-Host ""

# --- host --------------------------------------------------------------------
Write-Head "Host"
# try/catch, not -ErrorAction: a missing cmdlet raises CommandNotFoundException,
# which -ErrorAction does not suppress. This keeps the doctor useful even on an
# unexpected PowerShell edition instead of spraying red text.
try   { $os = Get-CimInstance Win32_OperatingSystem -ErrorAction Stop }
catch { $os = $null }
$build = if ($os) { [int]$os.BuildNumber } else { 0 }
Write-Info "OS           : $(if ($os) { $os.Caption } else { 'unknown' })"
Write-Info "Build        : $build"
Write-Info "Architecture : $env:PROCESSOR_ARCHITECTURE"
# Windows 11 is build 22000+. WSLg ships with Windows 11 only.
$isWin11 = $build -ge 22000
if ($isWin11) { Write-Good "Windows 11 (WSLg is available)" }
else { Write-Warn "Windows 10 detected -- WSLg is not available; use the browser GUI" }
Write-Host ""

# --- docker ------------------------------------------------------------------
Write-Head "Docker"
$dockerOk = $false
$dockerInfo = ""
if (Get-Command docker -ErrorAction SilentlyContinue) {
    $dockerInfo = (docker info 2>&1 | Out-String)
    if ($LASTEXITCODE -eq 0) {
        Write-Good "docker is installed and the daemon is running"
        $dockerOk = $true
    } else {
        Write-Bad "docker is installed but the daemon is not responding"
        Write-Info "Start Docker Desktop and wait for the whale icon to settle."
    }
} else {
    Write-Bad "docker not found"
    Write-Info "Install Docker Desktop: https://docs.docker.com/desktop/install/windows-install/"
}

# WSL2 backend. Hyper-V backend cannot do WSLg and is slower for bind mounts.
$wsl2Backend = $false
if ($dockerOk) {
    if ($dockerInfo -match 'docker-desktop' -or $dockerInfo -match 'WSL') {
        Write-Good "Docker Desktop appears to be on the WSL2 backend"
        $wsl2Backend = $true
    } else {
        Write-Warn "could not confirm the WSL2 backend"
        Write-Info "Docker Desktop > Settings > General > 'Use the WSL 2 based engine'."
        Write-Info "It is meaningfully faster than Hyper-V for this project."
    }
}
Write-Host ""

# --- gpu ---------------------------------------------------------------------
Write-Head "GPU"
$hasNvidia = $false
try {
    $gpus = @(Get-CimInstance Win32_VideoController -ErrorAction Stop |
              Select-Object -ExpandProperty Name)
} catch { $gpus = @() }
foreach ($g in $gpus) { Write-Info "detected: $g" }
if ($gpus -match 'NVIDIA') {
    if ($dockerOk -and $dockerInfo -match 'nvidia') {
        Write-Good "NVIDIA GPU is visible to Docker"
        $hasNvidia = $true
    } else {
        Write-Warn "NVIDIA GPU present, but Docker does not expose it yet"
        Write-Info "Docker Desktop > Settings > Resources > enable GPU support,"
        Write-Info "and make sure your NVIDIA driver supports WSL2 (it must be a"
        Write-Info "recent Game Ready / Studio driver -- do NOT install a Linux"
        Write-Info "driver inside WSL, that breaks it)."
    }
} else {
    Write-Warn "no NVIDIA GPU -- software rendering will be used"
    Write-Info "That is fine. Physics is unaffected; only GUI framerate suffers."
}
Write-Host ""

# --- wslg --------------------------------------------------------------------
Write-Head "WSLg (optional native display)"
$wslgOk = $false
if ($isWin11 -and (Get-Command wsl -ErrorAction SilentlyContinue)) {
    $distros = (wsl -l -q 2>&1 | Out-String)
    # WSL emits UTF-16; strip nulls so the match behaves.
    $distros = $distros -replace "`0", ""
    if ($distros -match '\S') {
        Write-Good "WSL is installed"
        Write-Info "Distros: $(($distros -split '\r?\n' | Where-Object {$_ -match '\S'}) -join ', ')"
        Write-Info "Verify WSLg itself works BEFORE using the WSLg config:"
        Write-Info "    wsl -- xclock       (install x11-apps in the distro first)"
        Write-Info "If no window appears, WSLg is broken and the WSLg dev"
        Write-Info "container config will not work either."
        $wslgOk = $true
    } else {
        Write-Warn "no WSL distro installed"
        Write-Info "The WSLg config needs one. The browser GUI does not."
    }
} else {
    Write-Warn "WSLg unavailable on this machine"
}
Write-Host ""

# --- recommendation ----------------------------------------------------------
Write-Head "=== Recommendation ==="
Write-Host ""
Write-Host "  Open the " -NoNewline
Write-Host "Drone VIP - Universal (browser GUI)" -ForegroundColor White -NoNewline
Write-Host " configuration."
Write-Info "Then browse to http://localhost:6080 once it starts."
Write-Info ""
Write-Info "This is the recommended path on Windows even when WSLg works:"
Write-Info "it has no host dependencies and it does not break when Docker"
Write-Info "Desktop moves its WSLg mount paths."
if ($wslgOk) {
    Write-Info ""
    Write-Info "Once that works, you MAY try 'Drone VIP - Windows (WSLg native"
    Write-Info "display)' for native rendering speed."
}
Write-Host ""
Write-Info "In VS Code: F1 > 'Dev Containers: Reopen in Container' > pick from the list."
Write-Host ""

# --- write .env ---------------------------------------------------------------
if ($Write) {
    $lines = @(
        "# Generated by scripts\doctor.ps1 on $((Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ'))",
        "# Machine-specific. Gitignored. Re-run the doctor to regenerate.",
        ""
    )
    if ($hasNvidia) {
        $lines += "DOCKER_RUNTIME=nvidia"
        $lines += "NVIDIA_VISIBLE_DEVICES=all"
        $lines += "NVIDIA_DRIVER_CAPABILITIES=all"
    } else {
        $lines += "# No GPU exposed to Docker; leaving runtime at the default."
    }
    $lines += "DRONE_DISPLAY_MODE=auto"

    # LF endings and no BOM, deliberately. Docker Compose does not strip a
    # trailing CR, so a CRLF .env silently yields values like "nvidia`r"
    # and the container fails to start with a baffling error.
    $text = ($lines -join "`n") + "`n"
    [System.IO.File]::WriteAllText($envFile, $text, (New-Object System.Text.UTF8Encoding($false)))
    Write-Head "Wrote $envFile"
    Get-Content $envFile | ForEach-Object { Write-Host "  $_" }
    Write-Host ""
}

if (-not $dockerOk) { exit 1 }
exit 0
