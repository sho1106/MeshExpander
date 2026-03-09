#Requires -Version 5.1
<#
.SYNOPSIS
    MeshExpander — configure, build, (optionally) fetch Stanford Bunny, run tests.

.DESCRIPTION
    Works on Windows with MSVC (Visual Studio 2019+) or any cmake-compatible toolchain.

.PARAMETER Config
    Build configuration: Release (default) or Debug.

.PARAMETER NoBunny
    Skip Stanford Bunny download and conversion.

.PARAMETER NoTest
    Skip running tests after build.

.PARAMETER Jobs
    Parallel build jobs (default: logical CPU count).

.EXAMPLE
    # Standard first-time setup
    powershell -ExecutionPolicy Bypass -File scripts\build.ps1

    # Debug build, skip bunny, skip tests
    powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Config Debug -NoBunny -NoTest
#>

param(
    [string] $Config   = "Release",
    [switch] $NoBunny,
    [switch] $NoTest,
    [int]    $Jobs     = [Environment]::ProcessorCount
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
function Info  { param($m) Write-Host "[INFO]  $m" -ForegroundColor Cyan }
function Ok    { param($m) Write-Host "[OK]    $m" -ForegroundColor Green }
function Warn  { param($m) Write-Host "[WARN]  $m" -ForegroundColor Yellow }
function Fatal { param($m) Write-Host "[ERROR] $m" -ForegroundColor Red; exit 1 }

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
$ScriptDir = $PSScriptRoot
$Root      = Split-Path $ScriptDir -Parent
$Build     = Join-Path $Root "build"
$DataDir   = Join-Path $Build "tests\data"

Info "Root:   $Root"
Info "Build:  $Build"
Info "Config: $Config   Jobs: $Jobs"

# ---------------------------------------------------------------------------
# Check prerequisites
# ---------------------------------------------------------------------------
if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    Fatal "cmake not found. Install from https://cmake.org/download/ and add to PATH."
}

$cmakeVer = (cmake --version | Select-String -Pattern '[\d]+\.[\d]+\.[\d]+').Matches[0].Value
Info "cmake $cmakeVer"

$parts = $cmakeVer -split '\.'
if ([int]$parts[0] -lt 3 -or ([int]$parts[0] -eq 3 -and [int]$parts[1] -lt 16)) {
    Fatal "cmake >= 3.16 required (found $cmakeVer)"
}

# ---------------------------------------------------------------------------
# Configure
# ---------------------------------------------------------------------------
Info "Configuring..."
$cmakeArgs = @("-S", $Root, "-B", $Build)
# Suppress Eigen/GTest verbose output
& cmake @cmakeArgs 2>&1 | Where-Object { $_ -notmatch "^--" }
if ($LASTEXITCODE -ne 0) { Fatal "cmake configure failed (exit $LASTEXITCODE)" }
Ok "Configure done"

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
Info "Building ($Jobs parallel jobs)..."
& cmake --build $Build --config $Config --parallel $Jobs
if ($LASTEXITCODE -ne 0) { Fatal "Build failed (exit $LASTEXITCODE)" }
Ok "Build done"

# ---------------------------------------------------------------------------
# Stanford Bunny download + conversion (optional)
# ---------------------------------------------------------------------------
if (-not $NoBunny) {
    $Archive  = Join-Path $Root "bunny.tar.gz"
    $BunnyDir = Join-Path $Root "bunny\reconstruction"

    New-Item -ItemType Directory -Force -Path $DataDir | Out-Null

    $stlFull = Join-Path $DataDir "bunny.stl"
    $stlRes4 = Join-Path $DataDir "bunny_res4.stl"

    if ((Test-Path $stlFull) -and (Test-Path $stlRes4)) {
        Ok "Stanford Bunny STL files already present — skipping download"
    } else {
        if (-not (Test-Path $Archive)) {
            Info "Downloading Stanford Bunny (~10 MB)..."
            $url = "http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz"
            try {
                $wc = New-Object System.Net.WebClient
                $wc.DownloadFile($url, $Archive)
                Ok "Downloaded to $Archive"
            } catch {
                Warn "Download failed: $_"
                Warn "Download manually: $url"
                Warn "Then run: python scripts\ply_to_stl.py --all bunny\reconstruction build\tests\data"
                $NoBunny = $true
            }
        } else {
            Ok "Archive already present: $Archive"
        }

        if (-not $NoBunny) {
            Info "Extracting bunny.tar.gz..."
            # tar is available on Windows 10 1803+ and Windows Server 2019+
            if (Get-Command tar -ErrorAction SilentlyContinue) {
                & tar -xzf $Archive -C $Root
                if ($LASTEXITCODE -ne 0) { Fatal "tar extraction failed" }
            } else {
                # Fallback: use 7-Zip if available
                $7z = "C:\Program Files\7-Zip\7z.exe"
                if (Test-Path $7z) {
                    & $7z x $Archive -o"$Root" -y | Out-Null
                    & $7z x (Join-Path $Root "bunny.tar") -o"$Root" -y | Out-Null
                } else {
                    Fatal "No tar or 7-Zip found. Extract $Archive manually to $Root"
                }
            }

            if (-not (Get-Command python3 -ErrorAction SilentlyContinue) -and
                -not (Get-Command python  -ErrorAction SilentlyContinue)) {
                Fatal "Python 3 not found. Install from https://python.org"
            }
            $python = if (Get-Command python3 -ErrorAction SilentlyContinue) { "python3" } else { "python" }

            Info "Converting PLY -> binary STL..."
            & $python (Join-Path $ScriptDir "ply_to_stl.py") --all $BunnyDir $DataDir
            if ($LASTEXITCODE -ne 0) { Fatal "PLY conversion failed" }
            Ok "Bunny STL files ready in $DataDir"
        }
    }
} else {
    Info "Skipping Stanford Bunny (-NoBunny)"
}

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if (-not $NoTest) {
    Info "Running all tests..."
    # 'check' target runs unit_tests then integration_tests
    & cmake --build $Build --config $Config --target check
    if ($LASTEXITCODE -ne 0) { Fatal "Tests failed (exit $LASTEXITCODE)" }
    Ok "All tests passed"
} else {
    Info "Skipping tests (-NoTest)"
    Info "To run tests manually:"
    Info "  cmake --build $Build --config $Config --target check"
}

Write-Host ""
Ok "Done. Build artifacts in: $Build"
