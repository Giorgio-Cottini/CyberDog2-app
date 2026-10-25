#!/usr/bin/env bash
# Attende che un client (il cane) compaia sulla subnet dell'hotspot 192.168.137.x
PS='Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue |
 Where-Object { $_.IPAddress -like "192.168.137.*" -and $_.IPAddress -ne "192.168.137.1" -and
   $_.IPAddress -notlike "*.255" -and $_.State -in "Reachable","Stale","Delay","Probe","Permanent" -and
   $_.LinkLayerAddress -ne "" } |
 Select-Object IPAddress,LinkLayerAddress,State | Format-Table -AutoSize | Out-String'
for i in $(seq 1 200); do
  OUT=$(powershell -NoProfile -Command "$PS")
  if echo "$OUT" | grep -qE "192\.168\.137\."; then
    echo "### CANE SULLA RETE al polling #$i (~$((i*3))s) ###"
    echo "$OUT"
    IP=$(echo "$OUT" | grep -oE "192\.168\.137\.[0-9]+" | grep -v "192.168.137.1$" | head -1)
    echo "IP rilevato: $IP"
    echo "--- probe porte su $IP ---"
    for p in 22 8981 8980 50051 8982 5001 9050; do
      powershell -NoProfile -Command "\$r=Test-NetConnection -ComputerName '$IP' -Port $p -WarningAction SilentlyContinue; if(\$r.TcpTestSucceeded){'OPEN  $p'}else{'close $p'}"
    done
    exit 0
  fi
  sleep 3
done
echo "nessun client sull'hotspot in ~10 minuti"
