# Protocol spec: grpc_fidelity

Extracted from MiRoboticsLab sources and verified against the code by a second reader.

# How the CyberDog 2 GrpcApp server behaves, as a client sees it (emulator spec, verified revision)

## 0. Provenance and conventions

**Source tree.** All paths are relative to `S`, a folder with the [MiRoboticsLab](https://github.com/MiRoboticsLab) repositories cloned side by side under their GitHub names.

- `bridges` is at commit `047a0a48b9f2411ed535555eaa6c2276699378ac` ("Merge pull request #2 from MiRoboticsLab/v1.3.0.0", 2024-01-17).
- `ref/cyberdog_app.cpp` in this repository differs from `bridges/cyberdog_grpc/src/cyberdog_app.cpp` only in line endings (4071 lines each), so the same line numbers apply to both.
- "app.cpp" below means `bridges/cyberdog_grpc/src/cyberdog_app.cpp`. "json.hpp" means `utils/cyberdog_common/include/cyberdog_common/cyberdog_json.hpp`. "qi.hpp" means `manager/cyberdog_manager/include/cyberdog_manager/query_info.hpp`.

**Tags used throughout.**
- **[CODE]**: read directly in the source.
- **[INF]**: inferred, for example from gRPC-core, rapidjson 1.1.0 or C++ semantics, or by reasoning across files.
- **[DOC]**: taken from `blogs/docs/cn/grpc_protocol.md`.
- **[OBS]**: observed on the real dog with the v1 tools ([connection.md](../connection.md)).

**Caveat.** Nobody has verified that the real dog's firmware equals v1.3.0.0. This is the first item in section 12.

**Changes from the reviewed draft.**
- The 4003 list is a union, and its trigger counter was described wrongly.
- The 10007 response keys were incomplete.
- The nav accepted-2 timing, the UWB gating and the 6010 attach semantics were wrong or missing.
- The 1001 version timing was wrong.
- motion_result_cmd is public code, not closed.
- The keepalive rules were incomplete.
- Two line citations were wrong.
- Added: crash vectors, double-write paths, client cancellation, persistent 55001, the full push catalogue, OTA reply shapes, uint8 truncations and parser fidelity.

---

## 1. Process lifecycle and ports

- **Startup waits for the SN.** The node constructor blocks in `ReadySnNode::WaitSn()` (a busy loop) until the serial number is known (app.cpp:95-97; bridges/cyberdog_grpc/include/ready_sn.hpp:49-57). The gRPC server thread and the heartbeat thread start only afterwards (app.cpp:454-459). [CODE]
- **Ports are hard-coded.** The parameters `grpc_server_port` and `grpc_client_port` are declared and then ignored: the server uses `50052` and the reverse client `8981` (app.cpp:121-125). [CODE]
- **Initial state.**
  - `server_ip = "0.0.0.0"` (app.cpp:119).
  - `app_disconnected = true`, `is_internet = false`, `wifi_strength = 0`, `can_process_messages_ = false`, `app_stub_ = nullptr` (app.cpp:84-93).
  - `connect_mark_ = false` (cyberdog_app.hpp:541).

  [CODE]
- **Threads.**
  - ROS callbacks run in a `MultiThreadedExecutor` (main.cpp:26-29).
  - The gRPC server runs in its own thread (`RunServer`) and the heartbeat in another (`HeartBeat`) (app.cpp:454-459).
  - Each reverse-push nameCode gets its own dispatcher thread (section 8.3).

  [CODE]
- **Crash behaviour.** Several request paths can abort the whole process (sections 3 and 6). Whether the node is respawned is not in the cloned repos. [INF]

## 2. Transport and TLS

### 2.1 Server side (the dog listens)

| Item | Value | Source |
|---|---|---|
| Listen address | `0.0.0.0:50052` | app.cpp:572-573, 591 |
| Credentials | `SslServerCredentialsOptions(GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY)`; root = `ca-cert.pem`; key/cert pair = `server-key.pem` / `server-cert.pem` | app.cpp:583-590 |
| Cert files | `<share/params>/toml_config/cyberdog_grpc/{ca-cert,server-cert,server-key}.pem`. If any file is empty, `RunServer` returns and nothing listens | app.cpp:442-453, 579-582 |
| Max send/recv message | `CHUNK_SIZE/4*5` = 5,242,880 bytes (`CHUNK_SIZE` = 4,194,304) | app.cpp:577-578; transmit_files.hpp:30 |
| Channel args | `KEEPALIVE_TIME_MS` = 1000, `KEEPALIVE_TIMEOUT_MS` = 1000, `HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS` = 500, `HTTP2_MIN_SENT_PING_INTERVAL_WITHOUT_DATA_MS` = 1000. Nothing else is set (no `KEEPALIVE_PERMIT_WITHOUT_CALLS`, no thread or quota options) | app.cpp:592-597 |
| gRPC version | C++ 1.44.0, synchronous server | blogs/docs/cn/cyberdog_grpc_cn.md:19 [DOC]; app.cpp:569-602 |

**Certificates** (`bridges/params/toml_config/cyberdog_grpc/`; all public, private key included). [CODE, checked with openssl]

- **CA:** CN=wenlinfeng, self-signed, basicConstraints critical CA:TRUE, valid 2023-08-07 to 2033-08-04. SHA-256 `61:02:83:3B:...:42:BB`.
- **Server cert:** CN=cyberdog2.server, issued by wenlinfeng. It is an **X.509 v1** certificate: no extensions, no SAN. Valid 2023-08-08 to 2033-08-05. SHA-256 `D4:33:1C:5D:...:1F:CB`.
- **Key match:** the modulus of `server-key.pem` matches the server cert.
- **Real dog.** `CyberDog/dog_server.pem` (captured from the dog) and `CyberDog/certs/client-cert.pem` have the same SHA-256 as the repo server cert. [OBS + CODE]

**What a client must do.**

1. **Present a client certificate that chains to the wenlinfeng CA.** The server checks no other identity: `isPeerAvalible` ignores the peer and returns false only if `decision_` is null, which it never is after `SetRequesProcess` (cyberdog_app_server.cpp:26-33; app.cpp:575).
   - The repo server cert and key work as the client identity. The v1 app loads them as `certs/client-key.pem` and `certs/client-cert.pem` (app/server.py:113-116).
   - The dog closes the connection when no client cert is sent ([connection.md](../connection.md)).

   [CODE/OBS]
2. **Override the target name to `cyberdog2.server`** (app/server.py:118-119). The dog cert has no SAN, so gRPC falls back to the CN. [INF]
3. **Raise the receive limit above 4 MiB.** A full 4 MiB FileChunk plus `file_name` and framing exceeds the gRPC default of 4 MiB. 5 MiB is enough; the v1 app uses 32 MiB (server.py:120). [CODE + INF]
4. **Keepalive.** [INF, gRPC-core semantics, not verified on the dog]
   - **While a call is active,** client pings must be at least 500 ms apart, or strikes lead to GOAWAY `too_many_pings`.
   - **With no active call,** `KEEPALIVE_PERMIT_WITHOUT_CALLS` is unset, so the server tolerates roughly one ping every 2 h. Leave client keepalive off, or keep `keepalive_permit_without_calls=0`.
   - **The server's own 1 s pings** (1 s ack timeout) go out only while calls are active, and are capped by `max_pings_without_data` (default 2) on silent streams.

### 2.2 Reverse client (the dog dials the app)

- **Target:** `<server_ip>:8981`, where `server_ip` is the connector's `provider_ip` (app.cpp:694-701).
- **Credentials:** `SslCredentials` with root = CA, private key = **server-key**, cert chain = **server-cert**, and `SetSslTargetNameOverride("cyberdog2.client")` (app.cpp:704-710). [CODE]
- **What the app's reverse server would need.** A cert whose CN or SAN is `cyberdog2.client`, chained to the wenlinfeng CA. The server must also accept the dog's cyberdog2.server cert as the client identity. [INF]
- **Why this is blocked.** No such cert exists in any cloned repo: the only PEMs are the three above and `bes_transmit/server.pem`. The CA key is not public. **The reverse channel cannot be established with public material** (see also app/server.py:560-564). [INF]

## 3. RPC semantics

**Service** `grpcapi.GrpcApp` (protos/cyberdog_app.proto:24-28):
- `sendMsg(SendRequest) returns (stream RecResponse)`
- `heartbeat(Ticks) returns (Result)`
- `getFile(SendRequest) returns (stream FileChunk)`

**Messages** (proto:68-70, 153-156, 158-160, 162-167). [CODE]
- `SendRequest{fixed32 nameCode=1; string params=2}`
- `RecResponse{fixed32 nameCode=1; string data=2}`
- `Result{string result=1}`
- `FileChunk{fixed32 error_code=1; string file_name=2; fixed32 file_size=3; bytes buffer=4}`

**Status codes.**
- `sendMsg` and `getFile` always end with `Status::OK`, whatever the handler did, including writing zero messages (cyberdog_app_server.cpp:35-55). [CODE]
- `heartbeat` on :50052 is not overridden: only `sendMsg` and `getFile` are (cyberdog_app_server.hpp:62-71). It therefore returns `UNIMPLEMENTED`, the generated default. [INF]

**Cancellation.**
- No handler checks `ServerContext::IsCancelled()`; a grep finds no `IsCancelled` in bridges/cyberdog_grpc. When a client deadline expires or the client cancels, the dog-side handler runs to completion and its `Write()` calls return false. [CODE + INF]
- **Consequences:**
  - a 1004 whose client timed out still moves the robot;
  - blocking handlers (section 6, "blocks") hold a server thread forever;
  - a cancelled 6004 keeps a thread for up to 7200 s.

**Parse step** (ProcessMsg, app.cpp:3664-3669).
- `params().c_str()` goes through rapidjson `Parse<0>`, so content after the first NUL byte is ignored.
- On a parse error the dog writes **one** message `{"error_code":323}` with **nameCode 0**, because the response nameCode is set only afterwards (app.cpp:3670). [CODE]
- Inputs that fail to parse [INF, rapidjson 1.1.0, flags 0]:
  - `""` and whitespace-only strings;
  - trailing garbage;
  - `NaN` or `Infinity`;
  - comments;
  - trailing commas;
  - numbers that overflow a double (`1e400`);
  - invalid surrogates;
  - unescaped control characters.
- Python `json.dumps(float('nan'))` produces `NaN`, which the dog rejects. [INF]
- **Duplicate keys:** rapidjson `FindMember` returns the **first** occurrence; Python `json` keeps the last. [INF]
- **This applies to every nameCode**, including 2002 and 5010/5011, which then use the raw string. Invalid JSON there gives 323/nameCode 0 and nothing is published. [CODE]

**Valid JSON that is not an object** (`[]`, `3`, `"x"`):
- Every `CyberdogJson::Get` returns false (json.hpp:480-483 and siblings), so handlers run with defaults or uninitialised values.
- **Raw rapidjson calls** do not go through Get. These are `json_request.HasMember` in 6004/6010 (app.cpp:1434, 1438) and in 6002 (1774, plus `labels[i].HasMember` at 1781-1784), and `roi["x_offset"].GetInt()` without a type check in 6008 (1967-1970). They hit `RAPIDJSON_ASSERT`: abort if the build lacks NDEBUG, undefined behaviour otherwise. [CODE + INF]
- **4001** with a scalar fails `Document2String` (only objects and arrays are accepted, json.hpp:326-333) and replies `{"error_code":323}` with nameCode 4001 (app.cpp:3726-3729). [CODE]

**Crash vectors** (uncaught C++ exceptions on a gRPC thread end in `std::terminate` of the whole process) [CODE + INF]:
- **3002:** `stoul(token_expires_in)` when the key is missing or not numeric (app.cpp:3277-3279).
- **8013:** `std::stoi(message)` when the service message is not numeric (app.cpp:2438).

**Response nameCode.** It equals the request nameCode (app.cpp:3670), with these exceptions:
- a parse error replies with 0;
- 5005 replies with 5004 (app.cpp:1224, 1229);
- dead code in 1010-1012 (section 5).

[CODE]

**Unknown nameCode.** `default: break` (app.cpp:3915-3916) gives **zero messages** and status OK. The same applies to push-only codes sent by a client: 1003, 2001, 3013, 4003, 5007, 6001, 6005, 6006, 6007, 6011, 7003, 7004, 8003, 8008, 8010, 8012. [CODE]

**Threading.**
- The server is a synchronous server with no thread or quota options, so each RPC runs on its own gRPC worker thread with unbounded concurrency. [INF]
- No global lock serialises ProcessMsg, with one exception: `task_init_mutex_` serialises the accept phase of 6004/6010/5010/5011 and is held for up to about 6 s (app.cpp:1338, 1527; action_task.hpp:191-201, 383-400). [CODE]
- ROS service calls use `async_send_request` plus `future.wait_for`, and replies are served by the MultiThreadedExecutor. [CODE]

**Common wait pattern** (most handlers):
1. `wait_for_service(3 s)`. On failure: **322**, or **zero messages** for the handlers marked "0 if absent" in section 6.
2. `wait_for(T)`. On timeout: **321**, or, for the "blocks" handlers, no error at all: they call `future.get()` anyway, which blocks until the ROS service answers, possibly forever. [CODE]

**Double-write paths.** On timeout, two handlers write `{"error_code":321}` and then still write their normal message:
- 6003 sends a second message with empty data (app.cpp:1754-1758);
- 8002 sends a second message `"{}"` (app.cpp:2189-2200).

[CODE]

## 4. JSON parsing rules (`CyberdogJson::Get`, json.hpp)

| C++ target | Accepts | Source |
|---|---|---|
| `bool` | JSON `true`/`false` only (no 0/1) | :623-635 |
| `int32_t` | `IsInt` only: `1.0` is rejected; values outside int32 are rejected | :513-528 |
| `uint32_t` | `IsUint`: negatives and non-integers are rejected | :530-541 |
| `int64_t` / `uint64_t` | `IsInt64` / `IsUint64` | :543-567 |
| `float` | `IsFloat` (a double literal within float range) or `IsInt` (int32) | :583-600 |
| `double` | `IsDouble`, `IsFloat` or `IsInt` (int32 only; a uint32-range literal such as 3000000000 is rejected) | :602-621 |
| `std::string` | `IsString` | :476-490 |
| `rapidjson::Value` | any type, copied as is | :569-581 |
| `std::vector<float>` | `HasMember`, then `GetArray()` **with no `IsArray` check**. Each element must be `IsFloat`; elements are **appended** until the first non-float, then the call returns false | :492-511 |

**Consequences.**

- **Integer literals are rejected inside float arrays.** rapidjson 1.1.0 `IsFloat` requires the double flag, so a literal `0` stops the copy. `"vel_des":[0.3,0,0]` gives `[0.3]`; `[0,0,0]` gives `[]`. Send `0.0`, never `0`. The v1 app sends floats (server.py:373-375 cast to float, so `json.dumps` writes `0.0`). [INF for IsFloat; the loop is CODE]
- **A scalar where an array is expected** (for example `"vel_des":0.3`) hits `GetArray()` on a non-array: an assert or undefined behaviour. An emulator should flag it. [CODE + INF]
- **A failed Get leaves the variable unchanged.** These locals are **uninitialised** when the key is missing or mistyped:
  - 1012 `volume` (app.cpp:3124-3126);
  - 1013 `enable` (3161-3162) and 1014 `enable` (3200-3201);
  - 10003 `data` (2590-2591);
  - 6012 `enable` (2003-2004);
  - 1100 `data` (4017-4018);
  - 12001 `data` (3472-3473);
  - 6004/6010 `type` (1424, read at 1432) and `keep_distance` (1429, read at 1447);
  - 8001 `scan_seconds` (2114-2115), which also sets that handler's wait;
  - 8011 `data` (2414-2415).

  An emulator should treat these as client bugs and log them. [CODE]
- **Output encoding.** Output is compact rapidjson `Writer` output, with keys in insertion order. `Add(uint8)` and `Add(int8)` promote to int (json.hpp:68-225, overload resolution). [CODE + INF]

## 5. Response envelope and error codes

- `data` is a JSON string. A few replies are not JSON objects:
  - `""`: 1002, 4001, 8011, 55001, 3014;
  - raw service strings: 1021, 5001-5005, the 5010/5011 feedback and result, and the 2001/4001 pushes.
- The error form comes from `returnErrorGrpc`: `{"error_code":N}` (app.cpp:978-993; defaults `error_code=-1, namecode=0` at cyberdog_app.hpp:209-211).
- Error codes [CODE; **none of them appear in grpc_protocol.md**]:

| Code | Meaning | Examples |
|---|---|---|
| **321** | ROS call or action timeout; motion call failure (1004 also uses 321 when the service is absent) | 1004 (app.cpp:2945-2946) |
| **322** | Service or action server unavailable (3 s wait) | most handlers |
| **323** | JSON parse error (nameCode 0), JSON encode or format error, or missing required field | 10004, 11001, 11002, 6008, 8002, 4001 |
| **-1** (nameCode 0) | Encode error in 1010-1012 (app.cpp:3068, 3104, 3140). **Unreachable**: `json_response` is always an object, and Document2String fails only on non-object/array documents (json.hpp:326-333) | none in practice |

- **Other in-band status values** (not `error_code`):
  - 4002 `{"result":7|10}`;
  - nav/OTA `{"accepted":2|3}`;
  - 6009 `{"result":10}`;
  - 9030 `{"unlock_result":100}`;
  - 1001 `"volume":-255|-1`.
- **Client rule:**
  - detect errors by the presence of an `"error_code"` key;
  - accept nameCode 0;
  - treat "zero messages plus status OK" as a distinct outcome;
  - for 6003 and 8002, expect a trailing second message after 321.

  [INF]

## 6. nameCode table (sendMsg unless noted)

Legend:
- "0 if absent" means the stream ends with zero messages if the service is unavailable after the wait.
- "blocks" means that after the internal timeout the handler still calls `future.get()` and waits indefinitely.
- All line numbers are in app.cpp unless another file is named.

| nameCode | Handler (app.cpp) | Request fields (parsed type) | Response `data` (exact keys) | Messages / stream behaviour |
|---|---|---|---|---|
| 1001 GET_DEVICE_INFO | HandleGetDeviceInfoRequest :2677-2749 | bool flags `is_sn, is_version, is_uid` (bug: read into is_version), `is_nick_name, is_volume, is_mic_state, is_voice_control, is_wifi, is_bat_info, is_motor_temper, is_audio_state, is_device_model, is_stand, is_lowpower_control`; `uid` (string). `enables` has 20 entries; only 0-13 are set | The manager's `info` string, passed verbatim (7.1) | 0 if `query_divice_info` is absent after 3 s; else exactly 1, after a 3 s wait, then `future.get()` blocks |
| 1002 MOTION_SERVO_REQUEST | motionServoRequestHandle :2900-2922 | `motion_id, cmd_type, value` (int32); `vel_des, rpy_des, pos_des, acc_des, ctrl_point, foot_pose, step_height` (float[]). `cmd_source` is not parsed (stays 0 = APP) | `""` | Exactly 1, immediate; publishes `motion_servo_cmd`. No validation and no gating on motion state |
| 1003 MOTION_SERVO_RESPONSE | push only (:741-751) | none | Push `{"motion_id","order_process_bar","status","result","code"}` (int8 fields promoted to int); `cmd_id` is not forwarded although [DOC] (grpc_protocol.md:44-52) lists it | If a client sends it: 0 messages |
| 1004 MOTION_CMD_REQUEST | motionCMDRequestHandle :2924-2966 → callMotionServoCmd :753-780 | `motion_id` (int32); the same float[] fields; `contact, duration, value` (int32). `cmd_source` (stays 0), `gait_toml` and `toml_data` are not parsed | `{"motion_id":int,"result":bool,"code":int}`, or `{"error_code":321}` | Exactly 1. 321 if the service is absent after 3 s or there is no reply in 20 s |
| 1010 DEVICE_NAME_SWITCH | :3040-3073 | `enable` → `req.data` (bool; defaults to false) | `{"success":bool}` | 322 after 3 s; else 1 message; 3 s wait, then blocks |
| 1011 DEVICE_NAME_SET | :3075-3109 | `nick_name, wake_name` (string) | `{"success":bool}` | As 1010 |
| 1012 DEVICE_VOLUME_SET | :3111-3145 | `volume` (int32 literal → uint8, truncating; uninitialised if absent) | `{"success":bool}` | As 1010 |
| 1013 DEVICE_MIC_SET | :3147-3185 | `enable` (bool, uninitialised if absent) → AudioExecute, client "app_server", state NORMAL or OFFMIC | `{"success":bool}` (= `result`) | As 1010 |
| 1014 DEVICE_AUDIO_SET | :3187-3221 | `enable` (bool, uninitialised) → `audio_action_set` | `{"success":bool}` | As 1010 |
| 1015, 1023 | in neither the proto nor the switch; [DOC] lists them (grpc_protocol.md:702-716, 814-816) | none | none | 0 messages |
| 1020 ACCOUNT_MEMBER_ADD | :2751-2785 | `member` (string) | `{"success":<int code>}` | 0 if absent; blocks |
| 1021 ACCOUNT_MEMBER_SEARCH | :2787-2817 | `account` (string; the srv field is `member`) | Raw `data` string from the service | 0 if absent; blocks |
| 1022 ACCOUNT_MEMBER_DELETE | :2819-2857 | `member` | `{"success":<int>}` | 0 if absent; 0 on a 3 s timeout |
| 1024 ACCOUNT_MEMBER_CHANGE | :2859-2898 | `pre_name, new_name` | `{"success":<int>}` | 0 if absent; 0 on a 3 s timeout |
| 1025 STOP_AUDIO_PLAY | stopAudioHandle :3574-3598 | none | `{}` | 322, or 321 after 3 s |
| 1100 start stair align | :4007-4039 | `data` (bool, uninitialised if absent) | `{"success":bool}` | 322, or 321 after 5 s |
| 1101 stop stair align | :4041-4069 | none | `{"success":bool}` | 322, or 321 after 5 s |
| 2001 VISUAL_BACKEND_MSG | push only (:783-787) | none | Raw `robotend_message` string | Push |
| 2002 VISUAL_FRONTEND_MSG | :3683-3687 | Any **valid** JSON; the original `params` string is published on `frontend_message` | none | **0 messages** (invalid JSON gives 323/nameCode 0 and nothing is published) |
| 3001 AUDIO_AUTHENTICATION_REQUEST | :3223-3258 | none | `{"did","sn"}` | 0 if absent; blocks |
| 3002 AUDIO_AUTHENTICATION_RESPONSE | :3260-3302 | `uid, token_access, token_fresh` (string); `token_expires_in` (**string**, passed to `stoul`) | `{"result":...}` | 0 if absent; blocks. **A missing or non-numeric `token_expires_in` crashes the process** [INF] |
| 3011 VOICEPRINTTRAIN_START | :3304-3331 | `nick_name`, `voiceprint_id` | `"{}"` | 0 if absent; 0 on a 3 s timeout |
| 3012 VOICEPRINTTRAIN_CANCEL | :3333-3358 | none | `"{}"` | As 3011 |
| 3013 | push only (:812-819) | none | `{"code","voiceprint_id"}` | Push |
| 3014 VOICEPRINTS_DATA | :3360-3382 (push at :820-823) | none | `""` | As 3011 |
| 4001 IMAGE_TRANSMISSION_REQUEST | :3724-3734 | Any object or array, re-serialised and published on `img_trans_signal_in` | `""` | 1 immediate; 323 (nameCode 4001) for a scalar JSON. **The WebRTC answer comes back only as a reverse 4001 push** (:826-830) |
| 4002 CAMERA_SERVICE (sendMsg) | :3735-3741 → processCameraMsg :3950-3963 → returnResponse :909-928 | `command` (uint32, **default 2 = START_RECORDING**, truncated to uint8) | `{"result":int}`: 10 if more than 5 transfers are active; 7 if `camera_service` is absent after 5 s or does not reply in 5 s; else the camera result. The camera `msg` is dropped | Exactly 1 |
| 4002 CAMERA_SERVICE (getFile) | ProcessGetFile :3929-3937 → :3965-3978 → ReturnCameraFile (transmit_files.hpp:50-67) | `command` (uint32, default 255, truncated to uint8). Parse errors are ignored (the doc stays `{}`) | FileChunk stream (7.6) | Error chunk, or a file stream |
| 4003 FILES_NOT_DOWNLOAD_COMPLETE | push only (:3980-3999) | none | `{"file_name":[str,...]}` (8.2) | Push |
| 4004 DOWNLOAD_FILE (getFile) | :3938-3942 | `file_name` (string; prefix `/home/mi/Camera/`, **not sanitised**) | FileChunk stream; **the file is deleted after success** | Not subject to the >5 check, but it increments `thread_counts_` (transmit_files.hpp:139) |
| 4005 (disk-space warning) | not implemented; [DOC] only (grpc_protocol.md:1077-1093) | none | none | Never sent |
| other nameCode on getFile | :3943-3946 | none | One chunk `{error_code:255}` | 1 chunk |
| 5001 OTA_STATUS_REQUEST | :995-1035 | the object, re-serialised as the `ota_command_status_query` value | The string value of `"status"` in the OTA reply (`""` if absent); 323 if the reply is not JSON | 0 if `ota_grpc` is absent after 10 s; 10 s wait, then blocks |
| 5002 OTA_VERSION_QUERY | :1079-1119 | `ota_command_version_query` | The `"version"` string | As 5001 |
| 5003 / 5004 | :1121-1155 / :1157-1192 | `ota_command_start_download` / `ota_command_start_upgrade` | Raw reply value | As 5001 |
| 5005 OTA_PROCESS_QUERY | :1194-1234 | `ota_command_process_query` | The `"progress"` string, **with nameCode 5004** (323 also with 5004) | As 5001 |
| 5006 OTA_ESTIMATE_UPGRADE_TIME | :1236-1280 | `ota_command_estimate_upgrade_time_query` | The reply, re-serialised | As 5001 |
| 5007 OTA_NX_REBOOT | not in the switch; push `"{}"` on `ota_upgrade_reboot` (:1069-1076) | none | none | 0 messages |
| 5010 / 5011 OTA action start / continue | handleOTAAction :1282-1409 | Raw `params` as `goal_msg` | `{"accepted":1/2/3}`, then raw `feedback_msg` strings, then the raw `result_msg` | 322 if the action server is absent after 3 s; 321 after 10800 s; **the result is skipped if `result_msg` is empty or `!connect_mark_`** |
| 6001 / 6005 / 6006 / 6007 / 6011 | push only (section 8.3) | none | see 8.3 | Push; queue sizes 2/2/2/2/1 |
| 6002 MAP_SET_LABLE | :1761-1835 | `mapName` (string), `is_outdoor` (bool), `map_id`, `only_delete` (bool), `locationLabelInfo` [{`labelName` string, `physicX`/`physicY` **JSON doubles** (`IsDouble`, so integer literals skip the label)}] | `{"success":int}` (0 ok, 1 failed) | 322 after 3 s; 321 after 10 s |
| 6003 MAP_GET_LABLE | :1673-1759 | `mapName` (string), `map_id` (uint64) | `{"mapName","success":int,"is_outdoor","locationLabelInfo":[{labelName,physicX,physicY}],"map":{resolution,width,height,px,py,pz,qx,qy,qz,qw,"data":[int]}}` (written even when success=1) | 322; on success 1 message; **on a 10 s timeout, 2 messages: `{"error_code":321}`, then one with empty data** (:1754-1758) |
| 6004 NAV_ACTION / 6010 ACCESS_NAV_ACTION | handleNavigationAction :1411-1636 | `type` (uint32 → nav_type uint8; uninitialised if absent), `outdoor` (bool), `goalX`, `goalY`, `theta` (double), `relative_pos` (uint32 → uint8), `keep_distance` (double; uninitialised), `object_tracking` (bool) | `{"accepted":1/2/3}`, then N × `{"feedback_code":int,"feedback_msg":str}`, then `{"result":int}` or `{"error_code":321}` | Long stream (7200 s); see 7.7 |
| 6008 SELECTED_TRACKING_OBJ | :1947-1991 | `{"roi":{x_offset,y_offset,height,width}}` (presence checked, ints not type-checked) | `{"success":bool}` | 322 / 323 / 321 (30 s) |
| 6009 STOP_NAV_ACTION | :2066-2102 | `type` (uint32 → task_id uint8), `map_name` (string) | `{"result":int}` (0 ok, 1 failed); **on a 60 s timeout `{"result":10}`, not an error_code** | 322, else 1 message |
| 6012 ENABLE_POINT_CLOUD | :1993-2029 | `data` (bool, uninitialised if absent) | `{"success":bool,"message":str}` | 322, or 321 after 5 s |
| 6013 POINT_CLOUD_STATE | :2031-2064 | none | `{"success":bool,"message":str}` | 322, or 321 after 3 s |
| 7001 FACE_ENTRY | :2968-3003 | `command, username, oriname, ishost` | `{"result":...}` | 0 if absent; blocks |
| 7002 FACE_RECORDING | :3005-3038 | `command, username` | `{"result":...}` | 0 if absent; blocks |
| 8001 BLUETOOTH_SCAN | :2104-2154 | `scan_seconds` (double; uninitialised if absent) | `{"device_info_list":[{mac,name,addr_type,device_type,firmware_version,battery_level}]}` | 322; 321 after max(6, 1.5×scan_seconds) s |
| 8002 BLUETOOTH_CONNECT | :2156-2201 | `selected_device` {mac,name,addr_type} (323 if present but malformed; sent empty if absent) | `{"result":...}` | 322 / 323; **on an 18 s timeout, 2 messages: `{"error_code":321}` then `"{}"`** |
| 8004 BLUETOOTH_CONNECTED_DEVICES | :2215-2259 | none | `{"device_info_list":[...]}` | 322; 321 after 3 s |
| 8005 BLE_DIVICE_FIRMWARE_VERSION | :2293-2321 | none | `{"success":bool,"message":str}` | 322; 321 after 3 s |
| 8006 BLE_DEVICE_BATTERY_LEVEL | :2261-2291 | none | `{"connected":bool,"persentage":bool}` (bug: the float is written as a Bool) | 322; 321 after 3 s |
| 8007 DELETE_BLE_HISTORY | :2323-2352 | `mac` (sent in `SaveMap.map_url`) | `{"success":bool}` | 322; 321 after 3 s |
| 8009 UPDATE_BLE_FIRMWARE | :2366-2394 | none | `{"success":bool,"message":str}` | 322; 321 after 10 s |
| 8011 SET_BT_TREAD | :2412-2419, :3835-3839 | `data` (int, uninitialised) → `set_bluetooth_tread` Int8 | `""` | Exactly 1, immediate, no service wait |
| 8013 GET_BT_TREAD | :2421-2447 | none | `{"data":int}` from `std::stoi(message)` | 322; 321 after 3 s; **a non-numeric message crashes the process** [INF] |
| 8003 / 8008 / 8010 / 8012 | push only (8.3) | none | see 8.3 | Push |
| 9030 UNLOCK_DEVELOP_ACCESS | :3384-3422 | `httplink` (string) | `{"unlock_result":int}` (100 on a 10 s timeout) | 0 if absent after 3 s |
| 9031 REBOOT_MACHINE | :3423-3460 | **nothing is parsed**; `rebootmachine=1997` is hard-coded | `{"reboot_result":int}` | 0 if absent; 5 s wait, then blocks. **Reboots the robot** |
| 10001 STATUS_REQUEST | statusRequestHandle :2502-2548 | none | `{"motion_status":{"motion_id":int},"task_status":{"task_status":int,"task_sub_status":int},"self_check_status":{"code":int,"description":str},"state_switch_status":{"state":int,"code":int},"charging_status":{"wired_charging":bool,"wireless_charging":bool}}`. **No audio** | Exactly 1, immediate, from the cache (no ROS call) |
| 10002 LOW_POWER_EXIT | :2550-2577 | none | `{"success":bool}` | 322 (3 s), or 321 after 13 s |
| 10003 AUTO_LOW_POWER_ENABLE | :2579-2609 | `data` (bool, uninitialised) | `{"success":bool}` | 322, or 321 after 3 s |
| 10004 SET_WORK_ENVIRONMENT | :2611-2647 | `data` (string, **required**, else 323) | `{"success":bool,"message":str}` | 322 first, then 323, then 321 (3 s) |
| 10005 UPLOAD_SYSLOG | :2649-2675 | none | `{"success":bool}` | 322, or 321 after 59 s |
| 10006 POWER_OFF | :3600-3619 | none | `{"success":true}`, written **before** the request is sent; nothing is awaited | 322, else 1 message, then the dog powers off |
| 10007 LCM_LOG_UPLOAD | :3621-3651 | none | `{"success":bool,"message":str}` | 322, or 321 after 60 s |
| 11001 ENABLE_ELEC_SKIN | :3495-3532 | `data` (bool, **required**, else 323) | `{"success":bool,"message":str}` | 322 first, then 323, then 321 after 5 s |
| 11002 SET_ELEC_SKIN | :3534-3572 | `mode`, `wave_cycle_time` (int32 literals, **both required**, else 323) | `{"success":bool}` | 322 / 323 / 321 (5 s) |
| 12001 DOG_LEG_CALIBRATION | :3461-3493 | `data` (bool, uninitialised) | `{"success":bool,"message":str}` | 322, or 321 after 10 s |
| 55001 (test) | :3896-3902 | none | `""`; publishes `disconnect_app=true`. The connector clears `provider_ip` **and saves the empty provider into wifi.toml** (interaction/connector/src/connector.cpp:374-385, 731-760) | 1 message; the effect persists across reboots |
| anything else | default :3915 | none | none | 0 messages, OK |

**ROS service names** the emulator models (`create_client` calls, app.cpp:120-435):
- **Motion, status, power:** `motion_result_cmd`, `query_divice_info`, `low_power_exit`, `low_power_onoff`, `set_nx_environment`, `upload_syslog`, `poweroff`, `lcm_log_upload`.
- **Camera and OTA:** `camera_service`, `ota_grpc`, `cyberdog_ota_action` (action).
- **Audio:** `nick_name_switch`, `set_nick_name`, `audio_volume_set`, `set_audio_state`, `audio_action_set`, `stop_play`, `get_authenticate_didsn`, `set_authenticate_token`, `voiceprint_train`, `voiceprints_data_notify`.
- **Face:** `cyberdog_face_entry_srv`, `cyberdog_face_recognition_srv`.
- **Accounts:** `account_add`, `account_search`, `account_delete`, `account_change`.
- **Maps, navigation, tracking:** `set_label`, `get_label`, `start_algo_task` (action), `stop_algo_task`, `tracking_object_srv`, `switch_visual_obstacle_avoidance`, `query_visual_obstacle_avoidance_status`.
- **Skin, calibration, stairs:** `enable_elec_skin`, `set_elec_skin`, `dog_leg_calibration`, `start_stair_align`, `stop_stair_align`.
- **Unlock and reboot:** `unlock_develop_access`, `reboot_machine`.
- **Bluetooth:** `scan_bluetooth_devices`, `connect_bluetooth_device`, `get_connected_bluetooth_info`, `ble_device_battery_level`, `ble_device_firmware_version`, `delete_ble_devices_history`, `update_ble_firmware`, `get_bluetooth_tread`.

## 7. Detailed sections

### 7.1 1001 GET_DEVICE_INFO

**Request side** (app.cpp:2677-2749). [CODE]
- **Bug: `is_uid` overwrites `is_version`.** `Get("is_uid", is_version)` at app.cpp:2705 writes into `is_version`. If `is_uid` is present as a bool, it **overrides** `is_version`: `"is_version":true,"is_uid":false` gives no version, and `"is_uid":true` alone requests the version.
- **uid is never returned.** `enables[2]` is never set.
- **"charging" is never returned.** `enables.resize(20)` sets only indices 0-13, so the "charging" key (enables[14], qi.hpp:218-220, 501-504) never appears.
- **Wait.** 3 s, then `future.get()` blocks until the manager answers.

**Producer** (qi.hpp:157-513). [CODE] Keys are emitted in this order, each only when its flag is set:
- `"sn"`: string, queried only while cached `sn_` is empty (2 s + 3 s).
- `"version"`: the JSON value of `version` taken from the OTA action `result_msg`. While the cache `version_` is empty, every such call runs GetVersion (qi.hpp:515-553): up to 5 s for the action server, up to 3 s for the goal handle, then an **unconditional `sleep(5)`**. `version_` is set only by the asynchronous result callback, so the reply can be `"version":{}` (String2Document fails on `""`, json.hpp:297-311).
- `"nick_name"`: `{"current_name","default_name"}`; the default is "铁蛋".
- `"volume"`: int; -255 if the service is unavailable, -1 on timeout.
- `"mic_state"`, `"voice_control"`: bool (false when the service is unavailable or times out).
- `"wifi"`: `{"name","ip","mac":"***********","strength":int}`, from `connector_state` (qi.hpp:821-834; `"****"` when Wi-Fi is down).
- `"bat_info"`: `{"capacity":4500,"power":batt_soc,"voltage":batt_volt,"temperature":batt_temp,"is_charging":wired||wp,"charge_over_current":bool,"discharge_over_current":bool,"discharge_time":120}` (qi.hpp:348-362). In BmsStatus.msg, `batt_soc` is uint8, `batt_volt` uint16 and `batt_temp` int16; units are unverified.
- `"motor_temper"`: `{"hip":[4 × "xx.xx"],"thigh":[4],"crus":[4]}` as strings, or the strings "unavalible" / "unkown".
- `"audio_state"`: bool; false when the manager's stored `uid_` is empty.
- `"device_model"`: "MS2241CN".
- `"stand"`: true iff the last motion_id == 111 (RECOVERYSTAND).
- `"lowpower_control"`: bool. There is no service wait; 3 s reply wait, false on timeout.
- `"non-standing"`: **always present**; true iff motion_id ∈ {0, 101, 102} (qi.hpp:505, 803-819). [OBS: [connection.md](../connection.md) shows `{"non-standing":false}`]

**Timing.** sn, volume, mic_state, voice_control, motor_temper and audio_state each wait up to 2 s + 3 s. lowpower_control waits up to 3 s. version takes up to 13 s. Worst case is about 45 s. The bridge sets no bound. [CODE + INF]

### 7.2 1002 servo and 1003 push

- **Each 1002 publishes once and answers once.** One `motion_servo_cmd` publish and exactly one reply `{nameCode:1002,data:""}` (app.cpp:2900-2922). Motion state comes back only as the **reverse 1003 push**, queue size 2 (app.cpp:147, 741-751), which needs the reverse channel. [CODE]
- **`cmd_type`:** 0 START, 1 DATA, 2 END (bridges/protocol/ros/msg/MotionServoCmd.msg:12-15). [DOC] says "0 or 1: Data, 2: End" (grpc_protocol.md:28). Every array field is `float32[]`, unbounded. [CODE]
- **Ordering is not guaranteed.** Each 1002 is a separate RPC on its own thread, so clients should serialise them (the v1 app does). [INF]
- **Downstream servo handling** lives in motion/motion_manager (motion_decision.hpp:119, 207-213), a separate area.

### 7.3 1004 motion result command

- **Flow.** `wait_for_service(3 s)`, then `wait_for(20 s)` (app.cpp:753-780). Either failure gives `{"error_code":321}`. Success gives `{"motion_id","result","code"}` (app.cpp:2950-2965).
- **Fields.** `duration`, `value` and `contact` are int32 (MotionResultCmd.srv:18-20). [CODE]
- **Cancellation does not cancel the motion** (section 3).
- **Where latency and codes come from.** They are decided by the public `motion_result_cmd` server in motion/motion_manager/src/motion_manager.cpp:55 and the locomotion stack, a separate area. [OBS] 101 and 111 returned `result:true, code:0` (HANDOFF.md:12-13).

### 7.4 1010-1014 and 1025

- **Unavailable service:** 322 after 3 s.
- **Timeout:** after the 3 s `wait_for` times out, the handler **still calls `future.get()`** and blocks. There is no 321 path (app.cpp:3040-3221). [CODE]
- **1025** has a proper 321 path after 3 s (app.cpp:3574-3598).

### 7.5 2002 and 4001

- **2002** is fire-and-forget: zero messages (app.cpp:3683-3687).
- **4001** replies `""` at once (app.cpp:3724-3734). The SDP answer, ICE candidates and the stop reply `{"is_closed","uid"}` travel dog → app as reverse 4001 pushes, queue size 20 (app.cpp:204, 826-830). **WebRTC therefore cannot work without the reverse channel.** [CODE + INF]
- **4001 JSON keys** are documented at grpc_protocol.md:867-952 [DOC]:
  - offer_sdp / answer_sdp {type, sdp};
  - c_sdp {sdpMid, sdpMLineIndex, candidate};
  - uid, height, width, alignment;
  - is_closed.

### 7.6 4002 / 4004 and the FileChunk sequence

**Camera codes** (CameraService.srv) [CODE]:
- Results: 0 SUCCESS, 1 INVALID_ARGS, 2 UNSUPPORTED, 3 TIMEOUT, 4 BUSY, 5 INVALID_STATE, 6 INNER_ERROR, 255 UNDEFINED.
- Commands: 0 SET_PARAMETERS, 1 TAKE_PICTURE, 2 START_RECORDING, 3 STOP_RECORDING, 4 GET_STATE, 5 DELETE_FILE, 6 GET_ALL_FILES, 7 START_LIVE_STREAM, 8 STOP_LIVE_STREAM, 9 START_IMAGE_PUBLISH, 10 STOP_IMAGE_PUBLISH.
- **Only `command` is set.** The bridge never fills `args`, `width`, `height` or `fps` (app.cpp:839-840), so SET_PARAMETERS and DELETE_FILE cannot be parameterised.
- **The camera_service server is not in the cloned repos.**

**getFile 4002 algorithm** (app.cpp:3965-3978; transmit_files.hpp:50-67, 93-99, 111-206):

```
cmd = uint8(command)               # default 255; uint32 truncated
if thread_counts_ > 5:            result = 10
elif camera call fails (5 s svc wait / 5 s reply): result = 7
else result, msg = camera_service(cmd)
if result != 0: write FileChunk{error_code=result}; end       # one chunk, other fields empty
parse msg as "<name>,<size>" (first comma); no comma -> one chunk {error_code=8}; end
size = int prefix after the comma (0 if unparsable -> actual file length)
SendFile(name, prefix="/home/mi/Camera/", size)
```

**SendFile** (shared with 4004, which skips the transfer-count check but still increments the counter):

```
thread_counts_ += 1
open prefix+name; failure -> one chunk {error_code=9}; thread_counts_ -= 1; end
if size == 0: size = actual file length
loop while !eof:                         # C++: read 4 MiB; write gcount
    buf = read(4_194_304)
    write FileChunk{error_code=0, file_name=name, file_size=uint32(size), buffer=buf}
      (Write retried: up to 4 attempts; all failing -> add name to uncomplete_files, stop, no delete)
=> chunk count = floor(len/4 MiB) + 1; the last chunk is EMPTY when len is an exact multiple of 4 MiB (len 0 -> one empty chunk)
remove(prefix+name)                      # FILE IS DELETED ON THE DOG after a successful send (transmit_files.hpp:195)
erase name from uncomplete_files; thread_counts_ -= 1
```

**Error-code table for getFile chunks:**

| Code | Meaning |
|---|---|
| 0 | data chunk |
| 1-6 | camera result |
| 7 | camera service unavailable or timed out |
| 8 | the camera msg is not "name,size" |
| 9 | open failed (file missing) |
| 10 | more than 5 concurrent transfers |
| 255 | camera UNDEFINED, **or** a wrong nameCode on getFile (ambiguous) |

[CODE]

**What a successful 4002 command produces.** [INF; camera_service implementation not public]
- Only commands whose reply `msg` has the form "name,size" yield a file. [DOC] names TAKE_PICTURE and STOP_RECORDING (grpc_protocol.md:954-1033).
- A successful command whose msg has no comma gives error 8.
- A successful command whose msg merely *contains* a comma would make the dog send, and then delete, the file named before the first comma.

**Hazard: 4004 deletes arbitrary files.** `file_name` is concatenated to `/home/mi/Camera/` without sanitising. `../` therefore reads **and then deletes** any file the process can write. **The emulator must reproduce the deletion (sandboxed), and the app must never call 4004 on files it wants to keep.** [CODE + INF]

**4003 push** (app.cpp:553-555, 3980-3999; transmit_files.hpp:69-86, 121-134).
- `file_name` is the sorted **union** of `uncomplete_files` and the output of `ls /home/mi/Camera/` (fgets with a 128-byte buffer).
- It is sent only if non-empty.
- Names from the `autosaved_video_file` topic are added to `uncomplete_files` (app.cpp:4001-4005).
- [DOC] intends the app to fetch each listed file with 4004 (grpc_protocol.md:1034-1076).

[CODE]

### 7.7 6004 / 6010 navigation

**Validation and acceptance.**
- If the action server is absent after 3 s: 322.
- **Goal pose.** `goalX`/`goalY` set a pose only if **both** keys are present; a mistyped value leaves 0. Without `theta` the quaternion w is set to 0 ("illegal").
- `accepted` values:
  - **2:** (6004) the goal was rejected (null handle), or no handle arrived in 3 s **and** no identifying feedback arrived in a further 3 s (action_task.hpp:191-201, 383-402);
  - **3:** (6010) no task of that type is recorded, or the task has finished; for type 11, see the UWB case;
  - **1:** accepted.
- **Once accepted:**
  - a new task (6004) replays the feedback buffered before acceptance;
  - an attach (6010) replays only the latest feedback;
  - then feedback streams, and finally `{"result":r}` or `{"error_code":321}` after 7200 s.
  - Navigation.action results: 0 SUCCESS, 1 ACCEPT, 2 UNAVALIBLE, 3 FAILED, 4 REJECT, 5 CANCEL.

  [CODE]

**6010 attach semantics** (action_task.hpp:146-152, 228-235, 243-249, 441-459; app.cpp:1555-1559, 1573-1574) [CODE]:
- **Feedback moves to the new stream.** AccessTask replaces the task's feedback callback, so an earlier 6004 stream stops receiving feedback. Both streams then wait on the same result.
- **Feedback can stay suppressed.** The attach suppresses feedback until the latest value has been replayed. If the task has produced no feedback yet, the flag is never cleared, and that 6010 stream gets **no feedback** until the result.

**When the reverse channel is down** (`!connect_mark_`):
- **Results are dropped.** The final result is **not written**, and the stream simply ends with OK (app.cpp:1619-1622). [CODE]
- **The stream is cut.** At each 5 s heartbeat-failure event, `disconnectTaskRequest()` (app.cpp:1638-1671) calls `RemoveRequest`. That injects a **fake default Result** (result 0) and wakes the handler (action_task.hpp:237-242, 304-314, 419-428). The handler sees `!connect_mark_` and returns. Every 6004/6010/5010/5011 stream therefore ends within about 5-8 s with no result message. [CODE + INF]
- **The task itself keeps running.** RemoveRequest only unbinds the request; it does not cancel the goal. Stop tasks with 6009. [INF]

**UWB tracking not started from the app** (6010, type 11, no type-11 task recorded) (app.cpp:1546-1595) [CODE]:
- It is accepted (1) only if the cached `task_status_.task_status == 11`; otherwise `{"accepted":3}`.
- While `task_status == 11 && connect_mark_`, the dog polls every 100 ms. On each sub-status change it emits `{"feedback_code":task_sub_status,"feedback_msg":""}`.
- After the loop it writes `{"result":3}` **without** checking `connect_mark_`.
- With the reverse channel down: accepted 1, then result 3 at once.

**Type constants:** 1/2 AB, 3/4 follow, 5/6 mapping, 7/8 localisation, 9/10 docking, 11/12 UWB, 13/14 human tracking (Navigation.action). [CODE]

### 7.8 10001-10003

- **10001 reads only the cache** (app.cpp:2502-2548). The cache is fed by these topics (app.cpp:384-401, 2461-2500) [CODE]:
  - `motion_status`;
  - `algo_task_status`;
  - `self_check_status`;
  - `state_switch_status`;
  - `bms_status` (power_wired_charging, power_wp_charging);
  - `audio_notification_report` (heartbeat only).
- **Cache defaults** (cyberdog_app.hpp:635-659):
  - motion_id 0;
  - task_status 101 (uint8), task_sub_status 0;
  - self_check code -1, description "";
  - state -1, code 0;
  - charging false/false;
  - audio_playing false.
- **10002:** 322 after 3 s; 321 after 13 s.
- **10003:** 322 after 3 s; 321 after 3 s.

### 7.9 11001 / 11002 and miscellaneous

- **11001** checks the service first (322), then requires `data` as a bool (323), then waits 5 s (321).
- **11002** requires `mode` and `wave_cycle_time` as int32 literals. Modes 0-6 are defined in ElecSkin.srv:2.
- **Other handlers** are covered in section 6:
  - 9030 unlock via a cloud `httplink`;
  - 9031 reboot (no fields);
  - 12001 leg calibration;
  - 10006 power off (it replies before acting);
  - 55001 test disconnect (persistent).

## 8. Heartbeat and reverse channel

### 8.1 Connector input

`subscribeConnectStatus` (app.cpp:626-663) receives `connector_state` at 1 Hz (interaction/connector/config/connector.toml:4, 19; connector.cpp:336-345). [CODE]

- **It acts only if `is_connected`** (meaning Wi-Fi is up, not the app) and the server exists.
- **On every message it:**
  - refreshes `connector_update_time_point_`;
  - sets `app_disconnected` to whether `provider_ip` is empty or `"0.0.0.0"`;
  - updates `local_ip` (robot_ip), `is_internet` and `wifi_strength`;
  - if `provider_ip` differs from the stored `server_ip`, runs `destroyGrpc()` followed by `createGrpc()` (app.cpp:683-715). An empty provider creates a stub to `":8981"`.
- **Where `provider_ip` comes from:**
  - the saved Wi-Fi record for the SSID (connector.cpp:408-411, 826-842);
  - the QR code (AES key "b1950ca92ccc9ffe", JSON keys n/p/i; connector.cpp:36-39, 546-574, 602-619);
  - the Connect service (connector.cpp:883-893).
- **The provider** with the Windows hotspot is 192.168.137.1 ([connection.md](../connection.md)). [OBS]
- **`disconnect_app` clears the provider and persists the empty value** (connector.cpp:374-385, 731-760). [CODE]

### 8.2 HeartBeat loop (app.cpp:483-567), WallRate 500 ms

```
static last = first HeartBeat call (end of constructor)
continue_counts = 0
loop every 500 ms:
  if !can_process_messages_: warn; sleep; continue      # before the first createGrpc and between destroy and create
  connector_timeout = duration_cast<seconds>(now - connector_update).count() > 2   # effectively >= 3 s
  if app_stub_ and !connector_timeout:
      ok = stub.heartbeat(Ticks{ip=local_ip, wifi_strength, battery_power=bms.batt_soc, internet,
            sn, motion_status{motion_id}, task_status{task_status (passed as uint8), task_sub_status},
            self_check_status{code,description}, state_switch_status{state,code},
            charging_status{wired,wireless}, audio=audio_playing}, deadline 2.5 s)
            # cyberdog_app_client.cpp:68-118; only the status is checked
  elif connector_timeout: ok = false; app_disconnected = true
  else: sleep; continue
  if !ok:
      if seconds(now - last) >= 5:
          last = now; connect_mark_ = false; continue_counts = 0
          publish app_connection_state=false
          disconnectTaskRequest()                       # unbinds nav/OTA requests (fake results)
          if !app_disconnected: destroyGrpc(); createGrpc()
  else:
      connect_mark_ = true
      if continue_counts++ == 5: publishNotCompleteSendingFiles()   # 4003 push, once per session
      last = now; publish app_connection_state=true
```

**Ticks field types** (proto:30-66) [CODE]:
- `ip` string;
- `wifi_strength` and `battery_power` fixed32;
- `internet` bool;
- `sn` string;
- the nested messages (TaskStatus.task_status fixed32);
- `audio` bool.

### 8.3 Reverse pushes

- **One queue per nameCode.** `send_grpc_msg` creates a `LatestMsgDispather` for each nameCode (app.cpp:942-975). Each has its own thread and a drop-oldest queue (msg_dispatcher.hpp:37, 53-64). [CODE]
- **Pushes are lost while disconnected.** The sender `send_msgs_` (app.cpp:471-481) sends only if `can_process_messages_ && app_stub_ && connect_mark_`. Otherwise the message is **dequeued and silently discarded**, never retried. [CODE]
- **Each push is a reverse sendMsg.** The dog calls `app.sendMsg(SendRequest{nameCode, params})` with a 3 s deadline, drains the response stream and checks only the status (cyberdog_app_client.cpp:27-49). The app's reverse server must implement `sendMsg` (it may return zero messages) and `heartbeat` (status OK; the `Result` content is ignored). [CODE]

**Push catalogue.** [CODE] Queue size is 10 unless listed in app.cpp:147, 204, 297, 303, 321, 332, 340.

| nameCode | Payload | Queue | Source |
|---|---|---|---|
| 1003 | `{"motion_id","order_process_bar","status","result","code"}` | 2 | app.cpp:741-751 |
| 2001 | raw `robotend_message` | 10 | :783-787 |
| 3013 | `{"code","voiceprint_id"}` | 10 | :812-819 |
| 3014 | `"{}"` | 10 | :820-823 |
| 4001 | raw `img_trans_signal_out` (SDP, ICE, `{"is_closed","uid"}`) | 20 | :826-830 |
| 4003 | `{"file_name":[...]}` | 10 | :3980-3999 |
| 5005 | `{"upgrade_progress":0,"download_progress":p,"code":c}` or `{"download_progress":0,"upgrade_progress":p,"code":c}` | 10 | :1037-1067 |
| 5007 | `"{}"` | 10 | :1069-1077 |
| 6001 | `{"resolution","width","height","px","py","pz","qx","qy","qz","qw","data":[int]}` | 2 | :854-875 |
| 6005 | `{"px","py","pz","qx","qy","qz","qw"}` | 2 | :878-891 |
| 6006 | `{"path_point":[{"px","py"}]}` | 2 | :1837-1857 |
| 6007 | `{"body_info_roi":[{"roi":{x_offset,y_offset,height,width},"reid"}],"track_res_roi":{...}}` | 2 | :1906-1945 |
| 6011 | `{"laser_point":[{"px","py"}]}`, only if TF map→laser_frame is available; ranges < 0.05 skipped | 1 | :1859-1904 |
| 7003 / 7004 | `{"result","username"}` | 10 | :790-809 |
| 8003 | `{"disconnected":bool}` | 10 | :2203-2213 |
| 8008 | `{"data":str}` | 10 | :2354-2364 |
| 8010 | `{"status":int,"progress":double,"message":str}` | 10 | :2396-2410 |
| 8012 | `{"data":int}` | 10 | :2449-2459 |

### 8.4 The situation with the Windows hotspot

- **The reverse channel never comes up.** The dog targets 192.168.137.1:8981. Nothing listens there, or the opt-in reverse server of the v1 app cannot present a `cyberdog2.client` cert, so `connect_mark_` stays false. [INF]
- **Consequences:**
  - no pushes at all (section 8.3);
  - nav and OTA streams cut within about 5-8 s with no result;
  - the dog republishes `app_connection_state=false` and recreates its client every 5 s (because `app_disconnected` is false).
- **What still works.** No forward sendMsg or getFile handler is gated on `connect_mark_`, `app_disconnected` or `provider_ip`. No handshake nameCode exists. [CODE]
- **Exceptions:** the final writes of 6004/6010/5010/5011 and the UWB loop. [CODE]

## 9. State the emulator must model

1. **TLS:** server cert, CA, client cert required.
2. **Connector:** `provider_ip` (persisted per SSID, cleared persistently by 55001), `local_ip`, `is_internet`, `wifi_strength`, `connector_update_time` (1 Hz feed), `app_disconnected`, `server_ip`.
3. **Reverse channel:** `can_process_messages_`, `app_stub_`, `connect_mark_`, `heartbeat_last` (5 s window), `continue_counts`.
4. **Status cache** (7.8 defaults), plus BMS (`batt_soc`, `batt_volt`, `batt_temp`, charging and over-current flags).
5. **Motion:**
   - the current `motion_id`, which drives the 1001 `stand` and `non-standing` flags;
   - the servo stream (last cmd_type, vel_des);
   - per-motion_id result latency and code for 1004;
   - 1003 push generation.
6. **Device info:**
   - sn, nick names and switch, volume, mic, voice_control, wifi, motor temperatures, lowpower_control;
   - the `version_` cache with GetVersion timing (5 s + 3 s + `sleep(5)`; `{}` until filled).
7. **Camera:**
   - a state machine giving result codes 0-6/255;
   - the `/home/mi/Camera/` store with **delete-after-download**;
   - `thread_counts_` (incremented by 4002 and 4004);
   - `uncomplete_files`.
8. **Navigation and OTA tasks:**
   - type → goal_hash map (uint8 keys);
   - per-task feedback callback owner (6010 takes it over) and the access-suppression flag;
   - the pre-accept feedback buffer and the latest feedback;
   - shared result;
   - 7200 s / 10800 s timeouts;
   - RemoveRequest injecting a fake result;
   - `task_init_mutex_` serialisation;
   - the UWB special case.
9. **Maps and labels** for 6002 and 6003.
10. **Per-nameCode push queues** with the sizes in 8.3, drop-oldest, discarded when `!connect_mark_`.
11. **Fault-injection switches** for every ROS dependency: `ok`, `absent` (fails after the wait), `timeout` (replies late), `hang` (never replies; exercises the blocking `get()` paths).
12. **Crash switches:** process abort on 3002 with a bad `token_expires_in`, 8013 with a non-numeric message, and non-object JSON reaching the raw rapidjson paths. After an abort, all RPCs fail with UNAVAILABLE until restart (if any).
13. **Server threads leaked** by cancelled-but-blocking handlers. Cancellation never aborts work.

## 10. Emulator pseudo-code (Python, grpcio)

```python
CH = 4_194_304
FLT_MAX = 3.4028234e38
INT32 = lambda v: type(v) is int and -2**31 <= v < 2**31

class Dep:                       # one per ROS service/action, with fault injection
    def __init__(self, name, mode="ok", latency=0.05, reply=None):
        self.name, self.mode, self.latency, self.reply = name, mode, latency, reply
    def wait_for_service(self, t):
        if self.mode == "absent": time.sleep(t); return False
        return True
    def call(self, req, timeout):            # returns (ready, getter)
        if self.mode == "hang":    return False, lambda: block_forever()
        lat = timeout + 1 if self.mode == "timeout" else self.latency
        time.sleep(min(lat, timeout))
        resp = self.reply(req) if self.reply else {}
        if lat > timeout:
            return False, lambda: (time.sleep(lat - timeout), resp)[1]
        return True, lambda: resp

def rj_parse(s):                 # emulate rapidjson Parse<0> on params().c_str()
    s = s.split("\x00", 1)[0]                       # c_str() stops at the first NUL
    if s.strip() == "": raise ParseError
    def no_const(c): raise ParseError               # NaN / Infinity
    def first_wins(pairs):                          # FindMember returns the first duplicate
        d = {}
        for k, v in pairs: d.setdefault(k, v)
        return d
    def fl(x):
        f = float(x)
        if math.isinf(f): raise ParseError          # 1e400 -> kParseErrorNumberTooBig
        return f
    try:
        return json.loads(s, parse_constant=no_const, object_pairs_hook=first_wins, parse_float=fl)
    except ValueError:
        raise ParseError

def get(doc, key, typ, default=MISSING):     # CyberdogJson::Get semantics
    if not isinstance(doc, dict) or key not in doc: return default
    v = doc[key]
    if typ is bool:     return v if isinstance(v, bool) else default
    if typ == "int32":  return v if INT32(v) else default
    if typ == "uint":   return v if type(v) is int and 0 <= v < 2**32 else default
    if typ == "uint64": return v if type(v) is int and 0 <= v < 2**64 else default
    if typ == "float":
        if type(v) is float and abs(v) <= FLT_MAX: return v
        return float(v) if INT32(v) else default
    if typ == "double":
        if type(v) is float: return v
        return float(v) if INT32(v) else default
    if typ is str:      return v if isinstance(v, str) else default
    if typ == "floats":
        if not isinstance(v, list): raise UndefinedBehaviour(key)      # GetArray w/o IsArray
        out = []
        for e in v:
            if type(e) is not float or abs(e) > FLT_MAX: break   # integer literal => stop (IsFloat)
            out.append(e)
        return out
    # python json gives int for "0" and float for "0.0", matching rapidjson flags

def raw_object_access(doc):      # 6002/6004/6010/6008 paths that call HasMember/GetInt directly
    if not isinstance(doc, dict): raise ProcessAbort("RAPIDJSON_ASSERT")

def jdump(o): return json.dumps(o, separators=(",", ":"), ensure_ascii=False)
def msg(nc, data=""):  return RecResponse(nameCode=nc, data=data)
def err(code, nc):     return msg(nc, jdump({"error_code": code}))

class GrpcApp(GrpcAppServicer):
    def sendMsg(self, req, ctx):               # never check ctx.is_active() to abort work: the dog does not
        try: doc = rj_parse(req.params)
        except ParseError: yield err(323, 0); return
        nc = req.nameCode
        h = HANDLERS.get(nc)
        if h is None: return                         # 0 messages, status OK
        yield from h(self, doc, nc, req.params)

    def getFile(self, req, ctx):
        try: doc = rj_parse(req.params)
        except ParseError: doc = {}                  # parse errors ignored
        if req.nameCode == 4002:
            cmd = get(doc, "command", "uint", 255) & 0xFF
            if S.thread_counts > 5: yield FileChunk(error_code=10); return
            ok, res, m = camera_call(cmd)            # 5 s + 5 s
            if not ok: yield FileChunk(error_code=7); return
            if res != 0: yield FileChunk(error_code=res); return
            if "," not in m: yield FileChunk(error_code=8); return
            name, rest = m.split(",", 1)
            yield from send_file(ctx, name, "/home/mi/Camera/", int_prefix_or_0(rest))
        elif req.nameCode == 4004:
            yield from send_file(ctx, get(doc, "file_name", str, ""), "/home/mi/Camera/", 0)
        else:
            yield FileChunk(error_code=255)

    def heartbeat(self, req, ctx):
        ctx.abort(grpc.StatusCode.UNIMPLEMENTED, "")   # not implemented on :50052

def send_file(ctx, name, prefix, size):
    S.thread_counts += 1
    path = S.fs_root + prefix + name                  # NO sanitisation: model traversal, but sandbox fs_root
    try:
        f = open(path, "rb")
    except OSError:
        S.thread_counts -= 1; yield FileChunk(error_code=9); return
    size = size or os.path.getsize(path)
    with f:
        while True:
            buf = f.read(CH)
            if not ctx.is_active():                   # Write() false x4 => interrupted
                S.uncomplete.add(name); S.thread_counts -= 1; return
            yield FileChunk(error_code=0, file_name=name, file_size=size & 0xFFFFFFFF, buffer=buf)
            if len(buf) < CH: break                   # exact multiple of 4 MiB -> extra empty chunk
    os.remove(path)                                   # delete-after-download, as on the dog
    S.uncomplete.discard(name); S.thread_counts -= 1

# --- representative handlers -------------------------------------------------
def h1001(self, d, nc, raw):
    dep = DEP["query_divice_info"]
    if not dep.wait_for_service(3): return                       # 0 messages
    flags = {k: get(d, k, bool, False) for k in FLAG_KEYS}      # is_stand -> enables[12]
    if isinstance(d, dict) and isinstance(d.get("is_uid"), bool):
        flags["is_version"] = d["is_uid"]                        # bug
    flags["is_uid"] = False                                      # enables[2] never set
    ready, g = dep.call((flags, get(d, "uid", str, "")), 3)
    yield msg(nc, g())                                           # blocks until the manager answers

def h1002(self, d, nc, raw):
    cmd = dict(motion_id=get(d,"motion_id","int32",0), cmd_type=get(d,"cmd_type","int32",0),
               value=get(d,"value","int32",0), cmd_source=0,
               **{k: get(d,k,"floats",[]) for k in FLOAT_ARRAYS})
    SIM.publish_servo(cmd)                     # -> robot model / Gazebo bridge
    yield msg(nc, "")

def h1004(self, d, nc, raw):
    dep = DEP["motion_result_cmd"]
    if not dep.wait_for_service(3): yield err(321, nc); return
    ready, g = dep.call(parse_motion(d), 20)   # motion runs even if the client has gone
    if not ready: yield err(321, nc); return
    r = g(); yield msg(nc, jdump({"motion_id": r.motion_id, "result": r.result, "code": r.code}))

def simple(dep, timeout, build, required=None, blocks=False, zero_if_absent=False,
           zero_on_timeout=False, extra_after_timeout=None):
    def h(self, d, nc, raw):
        if not DEP[dep].wait_for_service(3):
            if zero_if_absent: return
            yield err(322, nc); return
        if required and any(get(d, k, t) is MISSING for k, t in required):
            yield err(323, nc); return
        ready, g = DEP[dep].call(d, timeout)
        if not ready and not blocks:
            if zero_on_timeout: return
            yield err(321, nc)
            if extra_after_timeout is not None: yield msg(nc, extra_after_timeout)   # 6003 "", 8002 "{}"
            return
        yield msg(nc, jdump(build(g())))          # g() blocks when blocks=True
    return h

HANDLERS = {
  1001: h1001, 1002: h1002, 1004: h1004,
  1010: simple("nick_name_switch", 3, lambda r: {"success": r.success}, blocks=True),
  1011: simple("set_nick_name",    3, lambda r: {"success": r.success}, blocks=True),
  1012: simple("audio_volume_set", 3, lambda r: {"success": r.success}, blocks=True),
  1013: simple("set_audio_state",  3, lambda r: {"success": r.result},  blocks=True),
  1014: simple("audio_action_set", 3, lambda r: {"success": r.success}, blocks=True),
  1020: simple("account_add", 3, lambda r: {"success": r.code}, blocks=True, zero_if_absent=True),
  1022: simple("account_delete", 3, lambda r: {"success": r.code}, zero_if_absent=True, zero_on_timeout=True),
  1024: simple("account_change", 3, lambda r: {"success": r.code}, zero_if_absent=True, zero_on_timeout=True),
  1025: simple("stop_play", 3, lambda r: {}),
  2002: lambda self, d, nc, raw: (SIM.frontend(raw), iter(()))[1],      # 0 messages
  3002: h3002,            # stoul(token_expires_in): missing/non-numeric -> ProcessAbort
  4001: h4001,            # array/object: publish, yield msg(nc,""); scalar: err(323, nc)
  4002: h4002_sendmsg,    # command default 2, & 0xFF; {"result":N}
  6002: h6002,            # raw_object_access(d); labels need float literals; 10 s
  6003: simple("get_label", 10, build_6003, extra_after_timeout=""),
  6004: hnav(create=True), 6010: hnav(create=False), 6009: h6009,  # 6009 timeout => {"result":10}
  8002: h8002,            # 18 s; timeout => err(321) then msg(nc, "{}")
  8011: lambda self, d, nc, raw: (SIM.bt_tread(get(d, "data", "int32", UNINIT)), iter([msg(nc, "")]))[1],
  8013: h8013,            # stoi(message): non-numeric -> ProcessAbort
  9031: h9031,            # no fields; reboots
  10001: lambda self, d, nc, raw: iter([msg(nc, jdump(S.status_10001()))]),
  10002: simple("low_power_exit", 13, lambda r: {"success": r.success}),
  10003: simple("low_power_onoff", 3, lambda r: {"success": r.success}),
  10004: simple("set_nx_environment", 3, lambda r: {"success": r.success, "message": r.message},
                required=[("data", str)]),
  10005: simple("upload_syslog", 59, lambda r: {"success": r.success}),
  10007: simple("lcm_log_upload", 60, lambda r: {"success": r.success, "message": r.message}),
  11001: simple("enable_elec_skin", 5, lambda r: {"success": r.success, "message": r.message},
                required=[("data", bool)]),
  11002: simple("set_elec_skin", 5, lambda r: {"success": r.success},
                required=[("mode", "int32"), ("wave_cycle_time", "int32")]),
  55001: h55001,          # provider_ip = "" and persist "" for the current SSID; yield msg(nc,"")
  # ... remaining rows of section 6
}

def hnav(create):
    def h(self, d, nc, raw):
        if not DEP["start_algo_task"].wait_for_service(3): yield err(322, nc); return
        raw_object_access(d)
        nav_type = get(d, "type", "uint", UNINIT) & 0xFF
        with S.task_init_lock:                       # serialises all nav/OTA accept phases
            if create:
                task = NAV.start(d, handle_wait=3, feedback_wait=3)   # None -> rejected / no handle
                if task is None: yield msg(nc, jdump({"accepted": 2})); return
                task.bind(stream=self, suppress=False)
            else:
                task = NAV.by_type(nav_type)
                if task is None:
                    if nav_type != 11 or S.task_status != 11:
                        yield msg(nc, jdump({"accepted": 3})); return
                    yield msg(nc, jdump({"accepted": 1}))
                    last = -1
                    while S.task_status == 11 and S.connect_mark:
                        if S.task_sub_status != last:
                            last = S.task_sub_status
                            yield msg(nc, jdump({"feedback_code": last, "feedback_msg": ""}))
                        time.sleep(0.1)
                    yield msg(nc, jdump({"result": 3})); return      # not gated on connect_mark
                task.bind(stream=self, suppress=True)   # steals feedback from earlier stream
            yield msg(nc, jdump({"accepted": 1}))
            if create: task.flush_pre_accept_feedback()
            elif task.latest_feedback is not None: task.replay_latest_and_unsuppress()
            # else: suppression never cleared -> this stream gets no feedback
        for ev in task.events(timeout=7200):     # feedback / result / fake_result (RemoveRequest)
            if ev.kind == "feedback":
                yield msg(nc, jdump({"feedback_code": ev.code, "feedback_msg": ev.msg}))
            elif ev.kind == "timeout": yield err(321, nc); return
            else:                                # real or fake result
                if not S.connect_mark: return    # result swallowed
                yield msg(nc, jdump({"result": ev.result})); return
    return h

# --- reverse channel / heartbeat thread ---------------------------------------
def heartbeat_loop():
    last = time.time(); cc = 0
    while True:
        time.sleep(0.5)
        if not S.can_process: continue
        ct = int(time.time() - S.connector_ts) > 2
        if S.stub and not ct: ok = S.stub.heartbeat(S.ticks(), timeout=2.5)
        elif ct: ok = False; S.app_disconnected = True
        else: continue
        if not ok:
            if int(time.time() - last) >= 5:
                last = time.time(); S.connect_mark = False; cc = 0
                NAV.remove_all_requests()        # inject fake results into live 6004/6010/5010/5011 streams
                if not S.app_disconnected: S.recreate_stub()   # target provider_ip:8981, name cyberdog2.client
        else:
            S.connect_mark = True
            if cc == 5:
                files = S.uncomplete | S.ls_camera()           # union, not "or"
                if files: push(4003, {"file_name": sorted(files)})
            cc += 1; last = time.time()

def push(nc, params):            # LatestMsgDispather + send_msgs_
    q = S.queues.setdefault(nc, Dispatcher(maxlen=QSIZE.get(nc, 10)))   # drop-oldest
    q.put(SendRequest(nameCode=nc, params=params if isinstance(params, str) else jdump(params)))
def dispatcher_worker(q):
    while True:
        m = q.get()
        if S.can_process and S.stub and S.connect_mark:
            S.stub.sendMsg(m, timeout=3)   # drain stream, check status only
        # else: dropped silently
```

**Emulator modes.**
- **(A) "real-dog-as-the-user-has-it"** (the default): the reverse channel fails, `connect_mark_` stays false, and the nav/OTA cut applies.
- **(B) "reverse OK"**: the emulator generates its own test CA plus `cyberdog2.server` and `cyberdog2.client` certs, so the app's push handling can be tested.

**Fidelity requirements.**
- Use compact JSON with keys in the order listed.
- Floats stay floats; bool fields are JSON booleans.
- 8006 `persentage` is a bool.
- Model the two double-write timeouts (6003, 8002).
- Model the crash paths as a process stop.

## 11. Client checklist derived from the above

- **Floats:** send decimal literals (`0.0`) in float arrays and in 6002 label coordinates; send integer literals for int and uint fields (`{"command":1}`, not `1.0`).
- **Params:** never send empty `params`; send `"{}"`. Always send a JSON object.
- **Errors:** detect `error_code`; accept nameCode 0; treat "0 messages" as a failure; tolerate a trailing extra message after 321 on 6003 and 8002.
- **Per-call deadlines:**
  - 1001: at least 15 s, and up to about 45 s with all flags (the version adds up to 13 s while uncached);
  - 1004: at least 25 s;
  - 1010-1014: at least 10 s, with cancellation; they can hang;
  - 6009: at least 65 s;
  - 10005: at least 65 s;
  - 10007: at least 65 s;
  - 8001: at least 1.5 × scan_seconds + 5 s;
  - nav streams: long, but expect them to be cut within about 5-8 s while the reverse channel is down.

  The v1 app currently defaults to 8 s (app/server.py:135).
- **A client timeout does not cancel the action on the dog** (1004 still moves).
- **Serialise 1002.**
- **Downloads:** max receive ≥ 5 MiB; do not enable keepalive pings while idle.
- **Never use 4004 or getFile on files you want to keep** (delete-after-download).
- **Sending 4002 via sendMsg without a valid uint `command` starts recording** (default 2).
- **Never send** 3002 without a numeric-string `token_expires_in`, or 8013 casually: these can crash the bridge.
- **10006 powers the dog off** after replying. **9031 reboots it.** **55001 erases the saved provider IP persistently.**

## 12. Unknowns

- **Firmware version.** Whether the real dog's firmware matches bridges v1.3.0.0 (commit 047a0a4). Check the 1001 version blob and the quirks below.
- **Build options.** The build type (NDEBUG or not, which decides whether RAPIDJSON_ASSERT aborts) and the rapidjson version on the dog (1.1.0 assumed). Whether cyberdog_grpc is respawned after an abort.
- **Probes:**
  - Does an unknown nameCode (for example 9999) return zero messages and status OK?
  - Does empty params give `{"error_code":323}` with nameCode 0?
  - Does heartbeat on :50052 return UNIMPLEMENTED?
  - Does `is_uid:false` together with `is_version:true` suppress the version in 1001?
- **1001 details.** Real latency with and without `is_version`, whether `"version":{}` appears on the first call, the version blob, the units of bat_info, and the motor_temper values.
- **Motion.** motion_result_cmd (1004) latency and result codes per motion_id (public motion_manager, separate area), and whether 1002 is accepted in each motion state.
- **Status topics.** Topic update rates, and the values 10001 reports on the real dog.
- **camera_service** (not in the cloned repos). Result codes and the state machine per command, the msg format for TAKE_PICTURE and STOP_RECORDING (expected "name,size"), and what START_LIVE_STREAM, GET_ALL_FILES and DELETE_FILE return with empty args.
- **Delete-after-download on the real dog.** Confirm only with a throwaway photo, never with wanted files or via 4004 path traversal.
- **Keepalive.** Whether the keepalive settings drop long streams over the Windows hotspot.
- **Integer-literal vel_des.** The physical effect of a truncated array. Test in the emulator or Gazebo only.
- **Reverse channel.** Whether the dog really targets 192.168.137.1:8981, and whether 6004/6010 streams end within about 5-8 s with no result.
- **Low power.** What low-power mode does to forward handling, and what 10002, 10003 and 10004 actually do.
- **Concurrency.** Effective limits on parallel sendMsg and getFile, beyond the counted 6-transfer camera limit.
- **Optional services.** Whether the Bluetooth (8xxx), face (7xxx), account (102x) and voiceprint (301x) services exist on the real dog, or give 322 or zero messages.


## Unknowns that need the real dog

- Whether the real dog's firmware matches bridges v1.3.0.0 (commit 047a0a4). Check the 1001 version blob and the behaviour of the quirks below.
- Probe: does an unknown nameCode (e.g. 1015 or 9999) return zero messages and status OK?
- Probe: does empty params give {"error_code":323} with nameCode 0?
- Probe: does heartbeat on :50052 return UNIMPLEMENTED?
- Probe: does sending is_uid:false together with is_version:true suppress the version in 1001 (is_uid/is_version bug)?
- Real 1001 latency (first call with is_version vs later calls), plus the content of the version blob, the units of bat_info (voltage, temperature) and the motor_temper values.
- motion_result_cmd (1004) latency and result codes per motion_id, and whether 1002 servo commands are accepted in each motion state.
- How fast and how often the motion_status, algo_task_status, self_check_status and state_switch_status topics update, and the values 10001 reports on the real dog.
- camera_service behaviour: the result codes and state machine per command, the msg format for TAKE_PICTURE and STOP_RECORDING (expected 'name,size'), and what START_LIVE_STREAM returns.
- Delete-after-download on the real dog. Confirm only with a throwaway photo, never with wanted files or via 4004 path traversal.
- Whether the server's 1 s keepalive ping with a 1 s timeout drops long streams over the Windows hotspot Wi-Fi.
- The physical effect of integer-literal vel_des (an array truncated to [] or a partial one). Test in the emulator or Gazebo only; never on the real dog.
- Whether the dog's reverse client actually targets 192.168.137.1:8981 and fails TLS, and whether 6004/6010 streams really end within about 5 s with no result message (expected from the disconnectTaskRequest cycle).
- What low-power mode does to forward RPC handling, and what 10002, 10003 and 10004 actually do.
- Effective concurrency limits: how many parallel sendMsg and getFile calls the dog sustains before latency or failures appear, beyond the counted 6-transfer camera limit.
- Whether Bluetooth (8xxx), face (7xxx) and account (102x) services exist on the real dog, or yield 322 or zero messages.

## Closed or missing in the public sources

- The camera_service implementation (cyberdog_camera package, not in the cloned repos). Its result semantics, state machine and file naming are unknown.
- The ota_grpc service and the cyberdog_ota_action action server (not found). The content of the 1001 version blob depends on them.
- The unlock_develop_access backend. 9030 takes a cloud httplink, so it depends on Xiaomi's cloud.
- The official Xiaomi app, and the cyberdog2.client certificate and key it presumably holds. Without them the reverse channel (heartbeat, all pushes, nav and OTA final results, the WebRTC answer) cannot be established.
- The private key of the wenlinfeng CA (not public), so no valid cyberdog2.client cert can be minted.
- The MR813 locomotion controller and the motion_manager internals behind motion_result_cmd and motion_servo_cmd (latency, result codes, gait safety limits).
- Firmware for the R329 audio board (the audio services behind 1010-1014 and 3xxx).
- Library semantics taken from outside the repos: rapidjson 1.1.0 (IsFloat, parse flags, GetArray assert) and gRPC C++ 1.44 (thread model, keepalive and ping policy, hostname check against CN when there is no SAN).

## Verifier corrections (12)

- **Claim:** 4003 push content: the list is the uncomplete set if non-empty, otherwise the output of ls /home/mi/Camera/ (section 7.6; pseudo-code pushes S.uncomplete or S.ls_camera())
  - **Correct:** file_name = sorted union of uncomplete_files and the lines of 'ls /home/mi/Camera/' (fgets with a 128-byte buffer, last character stripped). The push is sent only if the union is non-empty. Names that arrive on the autosaved_video_file topic are also added to uncomplete_files.
  - Source: bridges/cyberdog_grpc/include/transmit_files.hpp:69-86,121-134; bridges/cyberdog_grpc/src/cyberdog_app.cpp:3980-3999,4001-4005
- **Claim:** 4003 fires when the heartbeat has succeeded 6 times in a row
  - **Correct:** 4003 fires on the 6th successful heartbeat counted since thread start or since the last 5 s disconnect event (continue_counts++ == 5). It fires once per such session, and the successes need not be consecutive.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:489,533-539,549-555
- **Claim:** 10007 LCM_LOG_UPLOAD replies {'success':bool}
  - **Correct:** {'success':bool,'message':str}; 322 if the service is absent after 3 s; 321 after 60 s
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:3621-3651 (keys at 3638-3643)
- **Claim:** Uninitialised 6012 enable at app.cpp:2009-2011; 6004 type/keep_distance at app.cpp:1424,1429,1431,1446
  - **Correct:** 6012: bool enable declared at 2003, Get at 2004. 6004/6010: uint32_t type declared at 1424 and read at 1432; double keep_distance declared at 1429 and read at 1447.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:1424,1429,1432,1447,2003-2004
- **Claim:** Nav/OTA accepted 2 means the goal was rejected or no handle arrived within 3 s (action_task.hpp:192)
  - **Correct:** accepted 2 means a null goal handle (rejected), or no handle within 3 s followed by no identifying feedback within a further 3 s (worst case about 6 s). task_init_mutex_ is held for that whole time.
  - Source: bridges/cyberdog_grpc/include/action_task.hpp:191-201,383-402; bridges/cyberdog_grpc/src/cyberdog_app.cpp:1527-1540
- **Claim:** 6010 for a UWB task (type 11) that the app did not start: the dog polls task_status every 100 ms while connect_mark_ is true, then writes result 3
  - **Correct:** The task is accepted (1) only if the cached task_status_.task_status == 11; otherwise the reply is {'accepted':3}. While task_status == 11 and connect_mark_ is true, every sub-status change (polled each 100 ms) emits a fake feedback {'feedback_code':task_sub_status,'feedback_msg':''}. When the loop ends, {'result':3} is written without checking connect_mark_. With the reverse channel down the sequence is accepted 1 then result 3, but only if UWB tracking is currently active.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:1546-1595
- **Claim:** The first 1001 with is_version takes at least 5 s (one-off)
  - **Correct:** Every 1001 with is_version (or with is_uid true, because of the bridge bug) runs GetVersion while version_ is empty: up to 5 s for the OTA action server, up to 3 s for the goal handle, then an unconditional sleep(5). If the result has not arrived, String2Document('') fails and the reply carries 'version':{}.
  - Source: manager/cyberdog_manager/include/cyberdog_manager/query_info.hpp:243-250,515-553; utils/cyberdog_common/include/cyberdog_common/cyberdog_json.hpp:297-311
- **Claim:** 1001: each sub-query waits 2 s for its service and 3 s for the reply
  - **Correct:** 2 s + 3 s applies to sn (only while sn_ is empty), volume, mic_state, voice_control, motor_temper and audio_state (audio_state only when the manager's uid_ is non-empty). lowpower_control has no service wait, only a 3 s reply wait, and gives false on timeout. version follows the GetVersion timing. wifi, bat_info, nick_name, device_model, stand and non-standing come from cache.
  - Source: manager/cyberdog_manager/include/cyberdog_manager/query_info.hpp:221-242,272-335,363-500
- **Claim:** motion_result_cmd latency and codes are decided in the closed motion manager or MR813 stack (section 7.3)
  - **Correct:** motion_result_cmd is served by motion/motion_manager (motion_manager.cpp:55). Its behaviour is readable public code and belongs to a separate area. Locomotion firmware is published in cyberdog_locomotion; that repo was not checked here.
  - Source: motion/motion_manager/src/motion_manager.cpp:55; motion/motion_action/include/motion_action/motion_macros.hpp:101
- **Claim:** The v1 app already uses the repo server cert and key as client identity (app/server.py:118-120)
  - **Correct:** The client key and cert are loaded at server.py:113-116 (certs/client-cert.pem has the same SHA-256 as the repo server-cert.pem). 118-120 hold the override and the receive limit.
  - Source: app/server.py:113-120
- **Claim:** Client keepalive pings need only be at least 500 ms apart; the server pings every 1 s and closes the transport if no ack arrives within 1 s
  - **Correct:** [INF, gRPC-core defaults] While a call is active, client pings must be at least 500 ms apart. With no active call, the server tolerates a client ping only about every 2 h; more frequent pings collect strikes and end in GOAWAY too_many_pings. Server keepalive pings go out only while calls are active, and are also limited by max_pings_without_data (default 2). Clients should leave keepalive off or keep permit_without_calls=0.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:592-597
- **Claim:** An encode failure in 1010/1011/1012 (app.cpp:3066,3102,3138) yields {'error_code':-1} with nameCode 0
  - **Correct:** Keep -1/nameCode 0 only as dead code. In practice 1010-1012 never produce it.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:3066-3069,3102-3105,3138-3141; utils/cyberdog_common/include/cyberdog_common/cyberdog_json.hpp:326-333

## Verifier additions

- 3002 calls stoul(token_expires_in) and 8013 calls std::stoi(service message) with no try/catch. A missing or non-numeric value throws on the gRPC thread, which ends in std::terminate of the whole cyberdog_grpc process [INF]. Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:3277-3279, 2438.
- 8002 BLUETOOTH_CONNECT writes two messages on its 18 s timeout, {'error_code':321} and then '{}' with nameCode 8002, because the 321 branch does not return. Without selected_device it sends an empty request; 323 is returned only when selected_device is present but malformed. Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:2167-2200.
- No handler checks ServerContext::IsCancelled; a grep of bridges/cyberdog_grpc finds none. Client cancellation or deadline expiry therefore never stops dog-side work: a timed-out 1004 still moves the robot, blocking handlers hold a server thread forever, and a cancelled 6004 keeps waiting up to 7200 s. Source: bridges/cyberdog_grpc/src/cyberdog_app_server.cpp:35-55.
- 6010 attach semantics: AccessTask replaces the task's feedback callback, so an earlier 6004 stream stops receiving feedback; both streams then wait on the same result. If the task has no latest feedback yet, the suppression flag is never cleared and the 6010 stream gets no feedback at all. Source: bridges/cyberdog_grpc/include/action_task.hpp:146-152,228-235,243-249,441-459; cyberdog_app.cpp:1555-1559,1573-1574.
- Non-object JSON params reach raw rapidjson calls without type checks: json_request.HasMember in 6004/6010 (cyberdog_app.cpp:1434,1438) and in 6002 including per-label HasMember (1774-1784), and roi[...].GetInt() in 6008 (1967-1970). These hit RAPIDJSON_ASSERT: abort if built without NDEBUG, undefined behaviour otherwise [INF]. 4001 with a scalar fails Document2String and replies 323 with nameCode 4001 (3726-3729; cyberdog_json.hpp:326-333).
- More uninitialised locals: 12001 data (cyberdog_app.cpp:3472-3473); 8001 scan_seconds (2114-2115), which also sets the wait to max(6 s, 1.5 x scan_seconds) (2119-2121); 8011 data (2414-2415).
- 55001 is persistent. The connector clears provider_ip and calls SaveWiFi('', ssid, ''), writing an empty provider into wifi.toml. After a reboot GetWiFiProvider returns '', and the reverse channel is not attempted until a new QR pairing or Connect service call. Source: interaction/connector/src/connector.cpp:374-385,408-411,731-760,826-842,883-893.
- Full reverse-push catalogue (all via send_grpc_msg, all dropped while !connect_mark_): 1003 (cyberdog_app.cpp:741-751); 2001 raw (786); 3013 {code,voiceprint_id} (815-818); 3014 '{}' (822); 4001 raw (829); 4003 (3997); 5005 OTA progress {upgrade_progress,download_progress,code} (1042-1050,1058-1066); 5007 '{}' (1069-1076); 6001 map (857-874); 6005 pose (881-890); 6006 {path_point} (1841-1856); 6007 person (1910-1944); 6011 {laser_point}, sent only if TF map->laser_frame exists (1862-1903); 7003/7004 {result,username} (794-797,804-807); 8003 {disconnected} (2207-2212); 8008 {data} (2356-2363); 8010 {status,progress,message} (2398-2409); 8012 {data} (2451-2458).
- 4005 (disk-space warning, {remain_size}) is documented but never sent by v1.3.0.0 code. Source: blogs/docs/cn/grpc_protocol.md:1077-1093 (no send_grpc_msg(4005) in cyberdog_app.cpp).
- OTA forward reply shapes. 5001: data = the 'status' string inside the OTA reply. 5002: the 'version' string. 5003/5004: the raw reply value. 5005: the 'progress' string, with nameCode 5004. 5006: the reply re-serialised. A non-JSON reply gives 323. 5010/5011 feedback and result are raw feedback_msg/result_msg strings, and the result is also skipped when result_msg is empty. Source: cyberdog_app.cpp:1023-1032,1108-1117,1150-1152,1187-1189,1222-1231,1264-1278,1297-1307,1392-1396.
- Silent uint8 truncations: 4002 command, read as uint32, is passed as uint8 (cyberdog_app.cpp:3736-3738,3930-3933,3952,3967); nav type is stored as uint8 nav_type with uint8 map keys (1451; cyberdog_app.hpp:535); 6009 type becomes task_id uint8 (2081); 1012 volume (3126); heartbeat task_status (cyberdog_app_client.cpp:71).
- The bridge never fills CameraService args/width/height/fps (cyberdog_app.cpp:839-840; bridges/protocol/ros/srv/CameraService.srv:12-17), so SET_PARAMETERS and DELETE_FILE cannot be parameterised over gRPC. A sendMsg 4002 with {'command':1.0} fails IsUint and falls back to 2 = START_RECORDING (cyberdog_app.cpp:3736-3737).
- 4004 skips the >5 transfer check, but SendFile still increments thread_counts_. Active 4004 downloads therefore count toward the limit that makes 4002 return 10. Source: bridges/cyberdog_grpc/include/transmit_files.hpp:139; cyberdog_app.cpp:3938-3942,3957,3972.
- Parser fidelity [INF rapidjson 1.1.0]: params is read through c_str(), so it is cut at the first NUL (cyberdog_app.cpp:3664,3927). Duplicate keys resolve to the first occurrence in rapidjson but the last in Python json. 1e400 is a rapidjson parse error but inf in Python.
- task_init_mutex_ serialises the accept phase of all 6004/6010/5010/5011 calls and is held for up to about 6 s per call. Source: cyberdog_app.cpp:1338,1527; action_task.hpp:191-201,383-400.
- Concrete request fields for rows the spec left as 'per [DOC]': 1020 member, 1021 account, 1022 member, 1024 pre_name/new_name (cyberdog_app.cpp:2766,2802,2834,2874-2875); 3011 nick_name/voiceprint_id (3317-3318); 7001 command/username/oriname/ishost (2980-2983); 7002 command/username (3017-3018). 9031 parses nothing, always sends rebootmachine=1997 and reboots the dog (3435-3436).
- Bluetooth waits: 8001 max(6, 1.5 x scan_seconds) s, 8002 18 s, 8004/8005/8006/8007/8013 3 s, 8009 10 s; 8011 publishes with no wait. 8007 sends 'mac' inside SaveMap.map_url. Source: cyberdog_app.cpp:2119-2121,2182,2226,2272,2304,2334-2336,2377,2412-2419,2432.
- 6002 also reads mapName, is_outdoor, map_id and only_delete, and its timeout is 10 s. Source: cyberdog_app.cpp:1769-1772,1809.
- The v1 app defaults sendMsg to an 8 s deadline (app/server.py:135). That is shorter than 1004's worst case (3 s + 20 s) and 1001's; after a client timeout the dog still executes the request.
- The build type is unknown: no CMAKE_BUILD_TYPE/NDEBUG is set in bridges/cyberdog_grpc/CMakeLists.txt, so whether RAPIDJSON_ASSERT aborts is unknown, and whether cyberdog_grpc is respawned after a crash is not in the cloned repos.