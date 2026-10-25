function AwaitOp($op){
  while($op.Status -eq 0){ Start-Sleep -Milliseconds 150 }
  if($op.Status -ne 1){ throw "async status $($op.Status)" }
  return $op.GetResults()
}
for($i=0; $i -lt 30; $i++){
  try{
    $prof=[Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]::GetInternetConnectionProfile()
    $mgr=[Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]::CreateFromConnectionProfile($prof)
    if($mgr.TetheringOperationalState -ne 'On'){
      $r=AwaitOp($mgr.StartTetheringAsync())
      "$(Get-Date -Format HH:mm:ss) riacceso: $($r.Status) clients=$($mgr.ClientCount)"
    } else {
      "$(Get-Date -Format HH:mm:ss) On clients=$($mgr.ClientCount)"
    }
  } catch { "$(Get-Date -Format HH:mm:ss) err $($_.Exception.Message)" }
  Start-Sleep -Seconds 30
}
