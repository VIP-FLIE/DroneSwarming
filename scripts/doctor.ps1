# =============================================================================
# Drone VIP environment check. Windows 11.
#
#   .\scripts\doctor.ps1           look, report, change nothing
#   .\scripts\doctor.ps1 -Write    also write .devcontainer\.env
#
# This is optional. The container works without ever running it. All it does is
# check Docker is healthy and turn on GPU support if your machine has a GPU.
#
# If PowerShell refuses to run it:
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

Write-Head "Host"
try   { $os = Get-CimInstance Win32_OperatingSystem -ErrorAction Stop }
catch { $os = $null }
Write-Info "OS           : $(if ($os) { $os.Caption } else { 'unknown' })"
Write-Info "Architecture : $env:PROCESSOR_ARCHITECTURE"
Write-Host ""

# -----------------------------------------------------------------------------
# Docker, with a hard timeout.
#
# `docker info` against Docker Desktop talks to a VM and can block indefinitely
# while that VM is still starting. Running it unguarded is how this script used
# to appear to hang. One call, 25 second ceiling, and it is the only Docker
# call the whole script makes.
# -----------------------------------------------------------------------------
Write-Head "Docker"
$dockerOk   = $false
$dockerInfo = ""
if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Bad "docker not found"
    Write-Info "https://docs.docker.com/desktop/install/windows-install/"
} else {
    Write-Host "  ...   asking Docker how it is doing (up to 25s)" -ForegroundColor Gray
    $job = Start-Job -ScriptBlock { docker info --format '{{json .}}' 2>$null }
    if (Wait-Job $job -Timeout 25) {
        $dockerInfo = (Receive-Job $job | Out-String)
        if ($dockerInfo -match '\S') {
            Write-Good "docker is installed and the daemon is responding"
            $dockerOk = $true
        } else {
            Write-Bad "docker is installed but the daemon is not responding"
            Write-Info "Start Docker Desktop and wait for it to finish starting."
        }
    } else {
        Write-Bad "Docker did not answer within 25 seconds"
        Write-Info "The daemon is usually still starting up. Open Docker Desktop,"
        Write-Info "wait for the whale icon to stop animating, then run this again."
        Write-Info "If it never settles, quit Docker Desktop fully and reopen it."
    }
    Remove-Job $job -Force -ErrorAction SilentlyContinue
}

if ($dockerOk) {
    if ($dockerInfo -match '"MemTotal":(\d+)') {
        $memGb = [math]::Floor([int64]$Matches[1] / 1GB)
        if ($memGb -ge 8) { Write-Good "Docker has $memGb GB of RAM" }
        else {
            Write-Warn "Docker only has $memGb GB of RAM, 8 or more is much better"
            Write-Info "Docker Desktop > Settings > Resources > Memory"
        }
    }
    if ($dockerInfo -notmatch 'wsl' -and $dockerInfo -notmatch 'docker-desktop') {
        Write-Warn "could not confirm the WSL 2 backend"
        Write-Info "Docker Desktop > Settings > General > 'Use the WSL 2 based engine'."
        Write-Info "It is noticeably faster than Hyper-V for this project."
    }
}
Write-Host ""

# -----------------------------------------------------------------------------
Write-Head "GPU"
$hasNvidia = $false
try { $gpus = @(Get-CimInstance Win32_VideoController -ErrorAction Stop |
                Select-Object -ExpandProperty Name) }
catch { $gpus = @() }
foreach ($g in $gpus) { Write-Info "detected: $g" }

if ($gpus -match 'NVIDIA') {
    if ($dockerOk -and $dockerInfo -match 'nvidia') {
        Write-Good "Docker can see your NVIDIA GPU"
        $hasNvidia = $true
    } else {
        Write-Warn "NVIDIA GPU present, but Docker cannot see it yet"
        Write-Info "Docker Desktop > Settings > Resources > enable GPU support, and"
        Write-Info "make sure your NVIDIA driver is a recent Game Ready or Studio one."
        Write-Info "Do NOT install a Linux NVIDIA driver inside WSL, that breaks it."
    }
} else {
    Write-Warn "no NVIDIA GPU found, you will get software rendering"
    Write-Info "Fine for everything except camera-heavy work."
}
Write-Host ""

# -----------------------------------------------------------------------------
Write-Head "=== What to do next ==="
Write-Host ""
Write-Info "Open the repo in VS Code, hit F1, run 'Dev Containers: Reopen in Container'."
Write-Info "There is only one configuration, so there is nothing to choose."
Write-Info "When it is up, open http://localhost:6080 to see the simulation."
Write-Host ""

if ($Write) {
    $lines = @(
        "# Written by scripts\doctor.ps1. Machine-specific, gitignored.",
        "# Safe to delete. The container works without it.",
        ""
    )
    if ($hasNvidia) {
        $lines += "DOCKER_RUNTIME=nvidia"
        $lines += "NVIDIA_VISIBLE_DEVICES=all"
        $lines += "NVIDIA_DRIVER_CAPABILITIES=all"
    } else {
        $lines += "# No GPU visible to Docker, leaving the runtime at the default."
    }
    # LF endings, no BOM. Docker Compose does not strip a trailing CR, so a
    # CRLF .env silently produces values like "nvidia`r" and the container
    # fails to start with a baffling error.
    [System.IO.File]::WriteAllText($envFile, (($lines -join "`n") + "`n"),
                                   (New-Object System.Text.UTF8Encoding($false)))
    Write-Head "Wrote $envFile"
    Get-Content $envFile | ForEach-Object { Write-Host "  $_" }
    Write-Host ""
}

if (-not $dockerOk) { exit 1 }
exit 0
