# CyberDog 2 controller launcher.
# Serves the UI at http://127.0.0.1:8793/ and bridges to the dog over gRPC.
$cdog = Split-Path -Parent $PSScriptRoot
$env:PYTHONPATH = Join-Path $cdog "pylibs"
# $env:DOG_IP = "192.168.137.115"   # override if the dog's IP changes
# $env:DOG_REVERSE = "1"            # enable reverse server (video) once a
#                                     cyberdog2.client cert exists
python (Join-Path $PSScriptRoot "server.py")
