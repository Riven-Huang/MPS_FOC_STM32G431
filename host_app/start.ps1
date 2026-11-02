$ErrorActionPreference = 'Stop'
$appDirectory = $PSScriptRoot
$appPython = Join-Path $appDirectory '.venv\Scripts\pythonw.exe'
$consolePython = Join-Path $appDirectory '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $consolePython)) {
    Write-Host 'Creating a local Python environment (Python 3.10+ required)...'
    & py -3 -m venv (Join-Path $appDirectory '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Install Python 3.10 or newer with the Python launcher first.' }
}
& $consolePython -c 'import PySide6, pyqtgraph, serial, numpy' 2>$null
if ($LASTEXITCODE -ne 0) {
    & $consolePython -m pip install -r (Join-Path $appDirectory 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Could not install desktop dependencies.' }
}
Start-Process -FilePath $appPython -ArgumentList @(('"' + (Join-Path $appDirectory 'app.py') + '"')) -WorkingDirectory $appDirectory -WindowStyle Hidden
