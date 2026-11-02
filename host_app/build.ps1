$ErrorActionPreference = 'Stop'
$buildPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $buildPython)) { throw 'Run start.cmd once to prepare the environment.' }
& $buildPython -m pip install pyinstaller Pillow
if ($LASTEXITCODE -ne 0) { throw 'PyInstaller installation failed.' }
Push-Location $PSScriptRoot
$originalBuildPath = $env:PATH
try {
    # Avoid collecting incompatible ICU/Qt DLLs from other installed applications.
    $env:PATH = "$env:SystemRoot\System32;$env:SystemRoot;$(Join-Path $PSScriptRoot '.venv\Scripts')"
    & $buildPython -m PyInstaller --clean --noconfirm --windowed --onefile --name MPS-FOC-Studio --distpath dist/ov80 --icon assets/feibi.ico --add-data "assets;assets" app.py
    if ($LASTEXITCODE -ne 0) { throw 'Desktop build failed.' }
} finally {
    $env:PATH = $originalBuildPath
    Pop-Location
}
