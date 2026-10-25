$ErrorActionPreference = "Stop"

function AwaitOp($op) {
  while ($op.Status -eq 0) { Start-Sleep -Milliseconds 150 }
  if ($op.Status -ne 1) { throw "async op failed, status=$($op.Status)" }
  return $op.GetResults()
}

$prof = [Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]::GetInternetConnectionProfile()
$mgr  = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]::CreateFromConnectionProfile($prof)

if ($mgr.TetheringOperationalState -ne 'On') {
  $res = AwaitOp ($mgr.StartTetheringAsync())
  "StartTethering: " + $res.Status
} else {
  "Tethering already On"
}
"State: " + $mgr.TetheringOperationalState

$cfg = $mgr.GetCurrentAccessPointConfiguration()
"SSID: " + $cfg.Ssid

$env:DOG_SSID     = $cfg.Ssid
$env:DOG_PASS     = $cfg.Passphrase
$env:DOG_PROVIDER = "192.168.137.1"
$env:PYTHONPATH   = Join-Path $PSScriptRoot "pylibs"

& python (Join-Path $PSScriptRoot "make_qr.py")
