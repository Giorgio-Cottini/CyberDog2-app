# Connecting a PC to the CyberDog 2

How the PC and the dog find each other on the network, and how a client talks to the dog. The
v1 tools found these facts on a real dog with the Chinese firmware. The protocol notes in
[`protocol/`](protocol) give the details from Xiaomi's source code.

## Network

The scripts in this repository assume the Windows mobile hotspot:

- The PC is the access point and has the address `192.168.137.1`.
- The dog joins the hotspot and gets an address from the PC. On the dog used for this project
  the address was `192.168.137.115`, which is the default in every script. Set `DOG_IP`, or pass
  `-Dog` to `app2/run.ps1`, when your dog gets a different address.
- Windows turns the hotspot off after about 5 minutes without clients.
  [`keepalive.ps1`](../keepalive.ps1) checks it every 30 s for about 15 minutes, and turns it on
  again when it is off.
- [`watch_dog.sh`](../watch_dog.sh) waits for a client on the hotspot and then checks a few
  ports on it. [`scan.py`](../scan.py) scans the TCP ports of one address.

## Wi-Fi setup with a QR code

The dog can read its Wi-Fi settings from a QR code. The official app is not necessary.

1. Start the hotspot and make the QR code. [`run_all.ps1`](../run_all.ps1) does both. It reads
   the hotspot name and password from Windows and calls [`make_qr.py`](../make_qr.py), which
   writes `qr.html` next to the script.
2. Open `qr.html` on a screen.
3. Press and hold the touch area on the dog's head. The dog turns on its camera and reads the
   code.

The QR code holds the JSON text `{"n":"<ssid>","p":"<password>","i":"<provider_ip>"}`,
encrypted with AES-128-ECB and encoded in base64. The key is a fixed value from Xiaomi's
`connector` source, so anyone can decrypt the code. **`qr.html` contains your Wi-Fi password.
Never share it.** `.gitignore` excludes it.

`run_all.ps1` and `keepalive.ps1` use WinRT, so they need Windows PowerShell 5.1
(`powershell.exe`). PowerShell 7 does not give access to the WinRT types they use.
`make_qr.py` needs `pycryptodome` and `qrcode`.

## Control channel

- **Service:** gRPC `grpcapi.GrpcApp` on port 50052 of the dog. The definition is in
  [`proto/cyberdog_app.proto`](../proto/cyberdog_app.proto).
- **Requests:** `sendMsg(SendRequest)` answers with a stream of `RecResponse`. Both messages
  carry a `nameCode` and a JSON string (`params` in the request, `data` in the reply).
  `getFile` sends files, for example camera frames.
- **Security:** mutual TLS 1.3. The dog closes the connection when the client sends no
  certificate. It accepts any client certificate signed by its CA, and it does not check the
  address or the name of the client.
- **Certificates:** Xiaomi publishes the CA certificate and a signed certificate with its key.
  The [README](../README.md#quick-start) explains how to download them into `certs/`.
- **Target name:** the certificate has no subject alternative name, so a client must set the
  TLS target name override to `cyberdog2.server`.

The SDK in [`cyberdog_sdk/`](../cyberdog_sdk) does all of this. For a single request without
the SDK, use [`dog_cli.py`](../dog_cli.py). It takes the address, the port, the name code and
the JSON parameters, and it reads the certificate paths from three variables:

```powershell
$env:DOG_CA         = "certs/ca-cert.pem"
$env:DOG_CLIENTCERT = "certs/client-cert.pem"
$env:DOG_CLIENTKEY  = "certs/client-key.pem"
python dog_cli.py 192.168.137.115 50052 1001 "{}"    # 1001: device information, read only
```

## Results seen on the real dog

| Request | Reply | Effect |
|---|---|---|
| `1001` with `{}` | contains `"non-standing":false` | none, read only |
| `1004` with `{"motion_id":111}` | `{"result":true,"code":0}` | the dog stands up |
| `1004` with `{"motion_id":101}` | `{"result":true,"code":0}` | the dog lies down |

`result:true` means only that the dog accepted the command. A dog that already has the
requested posture does not move. Ask the person who watches the dog what happened.

There is no command that changes the language of the voice. The Chinese firmware speaks
Chinese. The volume (`1012`), the voice actions (`1014`) and the microphone (`1013`) can be
changed, and `1025` stops the current sentence.

## Safety

- Send a motion command only when the dog stands on a flat floor with about 1.5 m of free
  space around it. Never send one when the dog is on a table.
- Never send `9030` (developer unlock), `9031` (reboot), `10006` (power off), `12001` (leg
  calibration) or any `5xxx` (firmware update) unless you know what they do. The SDK refuses
  them unless the caller passes `force=True`.
