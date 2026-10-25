# CyberDog app v2: avvia il server locale e apre la pagina di controllo nel browser.
#
#   .\app2\run.ps1                          # il cane vero (192.168.137.115:50052)
#   .\app2\run.ps1 -Dog 127.0.0.1:50052     # l'emulatore sullo stesso PC
#
# Ctrl+C nella finestra ferma il server.
param(
    [string]$Dog = "192.168.137.115:50052",
    [int]$Port = 8794,
    [switch]$NoBrowser
)
$ErrorActionPreference = "Stop"

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $here
$paths = @((Join-Path $root "pylibs"), $root)
if ($env:PYTHONPATH) { $paths += $env:PYTHONPATH }
$env:PYTHONPATH = $paths -join ";"
$env:PYTHONIOENCODING = "utf-8"

$python = Get-Command python -ErrorAction SilentlyContinue
if (-not $python) {
    Write-Host "ERRORE: Python non trovato. Installa Python 3 (python.org) e riprova." -ForegroundColor Red
    exit 1
}

$url = "http://127.0.0.1:$Port/"
Write-Host ""
Write-Host "  CyberDog app v2" -ForegroundColor Cyan
Write-Host "  Apri nel browser:  $url"
Write-Host "  Cane:              $Dog"
Write-Host "  Per fermare il server: Ctrl+C"
Write-Host ""

$opener = $null
if (-not $NoBrowser) {
    # opens the page as soon as the server answers (at most about 15 s)
    $opener = Start-Job -ArgumentList $url -ScriptBlock {
        param($u)
        for ($i = 0; $i -lt 60; $i++) {
            try {
                Invoke-WebRequest -UseBasicParsing -Uri ($u + "api/catalog") -TimeoutSec 1 | Out-Null
                Start-Process $u
                return
            } catch {
                Start-Sleep -Milliseconds 250
            }
        }
    }
}

try {
    & $python.Source (Join-Path $here "server.py") --dog $Dog --port $Port
    $code = $LASTEXITCODE
} finally {
    if ($opener) { Remove-Job -Job $opener -Force -ErrorAction SilentlyContinue }
}
exit $code
