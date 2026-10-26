# Italian speech synthesis for tools/voice_pack.py: one WAV per line, with a Windows OneCore voice.
# Run it with Windows PowerShell 5.1 (powershell.exe): PowerShell 7 does not project the WinRT types.
#   powershell.exe -NoProfile -ExecutionPolicy Bypass -File voice_tts.ps1 -Voice Elsa -Job job.json
# job.json is a UTF-8 JSON list of {"text": "...", "out": "C:\...\<id>.wav"}.
param([Parameter(Mandatory = $true)][string]$Voice, [Parameter(Mandatory = $true)][string]$Job)
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Runtime.WindowsRuntime
$null = [Windows.Media.SpeechSynthesis.SpeechSynthesizer, Windows.Media.SpeechSynthesis, ContentType = WindowsRuntime]
$null = [Windows.Media.SpeechSynthesis.SpeechSynthesisStream, Windows.Media.SpeechSynthesis, ContentType = WindowsRuntime]

# IAsyncOperation<T> has no usable members in PowerShell 5.1: wait on it through AsTask<T>
$asTask = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
    $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
    $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
function Await($op, [type]$type) {
    $task = $asTask.MakeGenericMethod($type).Invoke($null, @($op))
    $null = $task.Wait(-1)
    $task.Result
}

$synth = New-Object Windows.Media.SpeechSynthesis.SpeechSynthesizer
$v = [Windows.Media.SpeechSynthesis.SpeechSynthesizer]::AllVoices |
    Where-Object { $_.DisplayName -like "*$Voice*" -and $_.Language -eq 'it-IT' } | Select-Object -First 1
if (-not $v) { throw "Italian voice '$Voice' is not installed" }
$synth.Voice = $v

$items = Get-Content -Raw -Encoding UTF8 $Job | ConvertFrom-Json
foreach ($it in $items) {
    $stream = Await ($synth.SynthesizeTextToStreamAsync($it.text)) ([Windows.Media.SpeechSynthesis.SpeechSynthesisStream])
    $in = [System.IO.WindowsRuntimeStreamExtensions]::AsStreamForRead($stream)
    $f = [IO.File]::Create($it.out + ".tmp")
    try { $in.CopyTo($f) } finally { $f.Close(); $in.Close() }
    Move-Item -Force ($it.out + ".tmp") $it.out
}
"$($v.DisplayName): $(@($items).Count) lines"
