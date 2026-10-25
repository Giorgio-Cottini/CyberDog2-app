# Protocol spec: nav_map

Extracted from MiRoboticsLab sources and verified against the code by a second reader.

# CyberDog 2: maps and navigation protocol spec (gRPC namecodes 6001-6013, 10001, 10004), verified revision

## Sources and citation conventions

`SRC` = a folder with the [MiRoboticsLab](https://github.com/MiRoboticsLab) repositories cloned side by side under their GitHub names. Every citation is `path:line`, relative to SRC unless it names a file of this repository.

**Code versions read:** bridges at commit 047a0a48 (2024-01-17); cyberdog_nav2 at commit 77c65782 (2023-10-07). `ref/cyberdog_app.cpp` in this repository is byte-identical to `bridges/cyberdog_grpc/src/cyberdog_app.cpp` after CRLF normalisation (verified with cmp).

**Short names:**
- APP = `bridges/cyberdog_grpc/src/cyberdog_app.cpp`
- TASK = `bridges/cyberdog_grpc/include/action_task.hpp`
- PROTO = `bridges/cyberdog_grpc/protos/cyberdog_app.proto`
- NAVACT = `bridges/protocol/ros/action/Navigation.action`
- MGR = `cyberdog_nav2/algorithm_manager/src/algorithm_task_manager.cpp`
- MGRH = `cyberdog_nav2/algorithm_manager/include/algorithm_manager/algorithm_task_manager.hpp`
- EXB = `cyberdog_nav2/algorithm_manager/include/algorithm_manager/executor_base.hpp`
- LBL = `cyberdog_nav2/map_label_server/src/labelserver_node.cpp`
- DOC = `blogs/docs/cn/grpc_protocol.md`

**Labels:**
- [CODE]: the code shows it directly.
- [INF]: inference.
- [CLOSED]: the component is not in the public repos.
- [NOT READ]: the component may be public but was not cloned (e.g. SLAM nodes absent from `cyberdog_ws/cyberdog.repos`).
- [DOC]: the official doc only.

---

## 0. Summary for the SDK author

1. **Codes the PC can usefully send** (request/response on the dog's `sendMsg`):
   - 6002 labels and map delete
   - 6003 map and labels
   - 6004 start task
   - 6008 tracking ROI select
   - 6009 stop task
   - 6010 re-attach
   - 6012 / 6013 UWB-follow visual obstacle-avoidance switch and query
   - 10001 status
   - 10004 cloud environment (do not expose)
2. **Push-only: 6001 map, 6005 dog pose, 6006 plan, 6007 tracking ROIs, 6011 laser scan.**
   - These go only dog → app over the reverse channel (dog as client to app:8981).
   - Sent as requests, they hit `default: break` (APP:3915-3916): empty stream.
   - Pushes require `connect_mark_`, i.e. a successful heartbeat (APP:471-481), which never happens with our certificate (section 1.4).
   - So we get no live map, pose, path, scan or ROIs. The saved map arrives only via 6003.
3. **The final `{"result":N}` of a 6004/6010 stream is never written to us**, because it is gated on `connect_mark_` (APP:1619-1622).
   - [INF] Streams are also unbound every ~5-7.5 s by the heartbeat-failure handler (APP:534-548 → APP:1638-1671).
   - Progress must come from: (a) feedback while attached, (b) 6010 re-attach, (c) polling 10001 `task_status`/`task_sub_status`.
4. **`task_status`/`task_sub_status` are not heartbeat-only:** 10001 returns them (APP:2516-2521). Nothing in this area exists only in `Ticks`. What we lose is the reverse channel: pushes, final results, and the UWB not-from-app fake-feedback loop.
5. **DANGER: 6002 can wipe every map.**
   - `only_delete:true` with zero *valid* labels runs `remove_all("/home/mi/mapping/")` (LBL:178-182, :262-266).
   - Labels whose `physicX`/`physicY` are JSON integers are silently dropped by the bridge (APP:1781-1784).
   - Always serialise coordinates as floats with a decimal point. Refuse an empty or invalid label list with `only_delete:true` unless the user confirms "delete map".
6. **DANGER: never send 6004 `{type:0}`.**
   - Task.toml defines ResetNav with Id 0 (Task.toml:52-57). In status 101/6/16 the goal is accepted, and the status becomes raw 0 (MGRH:294).
   - ResetNav Start does nothing (`executor_reset_nav.hpp:54-58`). The manager is wedged: every 6004 is rejected, 6009 type 0 returns FAILED (MGR:197-209), and other 6009 types return a fake 0 (MGR:226-233).
   - Recovery is only through LowPower (MGRH:168-184) or a reboot [INF].
   - `type` is uint8 in NAVACT:2-16, so a uint32 `type` of 256 truncates to 0 [INF]. Always send `type` and validate it client-side against {1,5,7,9,11,13}.
7. **Always send at least `"{}"` as params.** An empty string is a parse error and returns 323 with nameCode 0 (APP:3664-3668).

---

## 1. Transport and delivery model

### 1.1 Service
- `service GrpcApp { rpc sendMsg(SendRequest) returns (stream RecResponse); rpc getFile(...); rpc heartbeat(Ticks) ... }` (PROTO:24-28).
- `SendRequest{fixed32 nameCode; string params}`, where params is a JSON string.
- The server listens on 0.0.0.0:50052 with mTLS `REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY` (APP:577-590).
- Max send/receive is `CHUNK_SIZE/4*5` = 5,242,880 bytes (APP:577-578; `CHUNK_SIZE 4194304`, `bridges/cyberdog_grpc/include/transmit_files.hpp:30`).
- `isPeerAvalible` always returns true (`bridges/cyberdog_grpc/src/cyberdog_app_server.cpp:26-33`).

### 1.2 ProcessMsg
- Params are parsed with rapidjson. A parse error, including an empty string, gives a single `{"error_code":323}` with namecode **0** and ends the stream (APP:3664-3668).
- Otherwise the response namecode is the request namecode (APP:3670).
- Cases: 6002 APP:3790; 6003 :3793; 6004 :3796; 6008 :3799; 6012 :3802; 6013 :3805; 6009 :3808; 6010 :3811; 10001 :3843; 10004 :3852.
- Other codes reach `default: break` (APP:3915-3916). `sendMsg` returns OK with zero messages (`cyberdog_app_server.cpp:35-44`).
- Hidden case 55001 ("for testing") publishes `disconnect_app`=true (APP:3896-3902). The connector then clears and persists an empty provider_ip (`interaction/connector/src/connector.cpp:374-385`). Do not use it.

### 1.3 Error object and delivery modes

**Error object.** `returnErrorGrpc` writes `{"error_code":N}` (APP:978-993):
- 321: timeout
- 322: ROS service or action unavailable after a 3 s wait
- 323: JSON encode/decode error or missing field

Treat `error_code` as terminal. The one exception is the 6003 timeout, which is followed by an extra message.

**Delivery modes:**

| Mode | Meaning |
|---|---|
| S | Single message, then end of stream. |
| M | Multi-message stream: accept, then feedback, then an optional result. It can stay open up to 7200 s (APP:1422). |
| P | Dog → app push via `send_grpc_msg` (APP:942-975), through a per-namecode LatestMsgDispather that drops the oldest entry (`bridges/cyberdog_grpc/include/msg_dispatcher.hpp:53-64`). Queue sizes: map 2, pose 2, plan 2, laser 1, tracking 2 (APP:297, :303, :321, :332, :340). Each push is a separate `sendMsg` from the dog with a 3 s deadline (`cyberdog_app_client.cpp:27-67`). |
| H | Inside `heartbeat(Ticks)`, which the dog calls on the app every 500 ms (APP:483-567; Ticks PROTO:54-66). |
| T | The bridge's ROS topic source behind a P code. |

### 1.4 Why P mode and results are unavailable to us

**[CODE]**
- `send_msgs_` sends only if `can_process_messages_ && app_stub_ && connect_mark_` (APP:475).
- `connect_mark_{false}` (`cyberdog_app.hpp:541`); it is set true only after a successful `sendHeartBeat` (APP:550-552).
- The dog's client uses `SslTargetNameOverride("cyberdog2.client")` (APP:704-711). The app-side server must present a cert valid for `cyberdog2.client`, signed by the dog's CA.
- **Certificates.** Xiaomi publishes one leaf certificate, CN=cyberdog2.server with no SAN. Clients use it as `certs/client-cert.pem`, and the dog presents the same certificate as its server identity (`dog_server.pem`, captured from the dog).
  - Both files have SHA1 49:E8:78:85:10:1E:90:B7:7F:C3:E3:69:1F:CE:41:43:C5:6A:7D:63.
  - The CA is CN=wenlinfeng and its key is not available, so no `cyberdog2.client` cert can be minted.
- The reverse server of the v1 app is opt-in (`app/server.py:281-335`, `:560-567`).

**Heartbeat loop** [CODE] (APP:483-567):
- It runs only while `can_process_messages_` (APP:491).
- If the connector_state topic is older than 2 s, the heartbeat counts as failed and `app_disconnected=true` (APP:493-494, :524-527).
- On failure for 5 s or more (APP:534-548), the loop:
  - sets `connect_mark_=false`;
  - publishes app-connection=false;
  - calls `disconnectTaskRequest()`;
  - if `!app_disconnected`, runs destroyGrpc then createGrpc.

**`disconnectTaskRequest()`** (APP:1638-1671):
- It calls `RemoveRequest(hash)` for every entry of `task_type_hash_map_`.
- `RemoveRequest` removes the feedback callback, installs a fake default result and notifies the waiting handler (TASK:237-242, :419-428).
- The handler sees `!connect_mark_` and returns without writing a result (APP:1619-1622), so the stream ends. The task continues on the dog.
- Entries of finished tasks are erased (APP:1645-1655).

**`can_process_messages_`**:
- It is initialised false (APP:85).
- `createGrpc` sets it true (APP:714); `destroyGrpc` sets it false (APP:686).
- createGrpc is called from `subscribeConnectStatus` only when the connector's provider IP differs from `server_ip` (APP:646-659). `server_ip` is `"0.0.0.0"` at bridge start (APP:119) and changes after createGrpc.
- A provider that is empty or `"0.0.0.0"` sets `app_disconnected=true` (APP:626-663). The connector sets the provider from the QR (`connector.cpp:602-606`) and looks it up on WiFi changes (`connector.cpp:408-411`).

**[INF] Expected behaviour with the Windows hotspot setup ([connection.md](../connection.md))** (provider 192.168.137.1 from the QR):
- createGrpc runs and heartbeats to :8981 fail (2500 ms deadline, `cyberdog_app_client.cpp:68-119`).
- Every ~5-7.5 s, all open 6004/6010 streams close without a result.

**[INF, untested] Provider 0.0.0.0:**
- With provider `0.0.0.0` from a fresh bridge start (i.e. after a dog reboot), createGrpc would never run, and streams would stay attached until the task ends.
- The result itself would still not be written, but the last feedback would arrive.
- After createGrpc has once run, switching to 0.0.0.0 or to empty (e.g. via 55001) still differs from `server_ip`, so createGrpc runs again and the unbinding continues.

---

## 2. Code-by-code reference

### 6001 MAP_DATA_REQUEST (PROTO:108). Mode P (T `map`). Unavailable to us.

**Source** [CODE]:
- Subscription to `map` (`nav_msgs/OccupancyGrid`), KeepLast(1), reliable, transient_local (APP:293-297).
- Publishers: the label server on every `get_label` (LBL:155), and [INF] the mapping node during mapping [NOT READ]. The flow diagram shows the grid and pose pushed during mapping (`blogs/docs/cn/image/grpc_protocol/grpc_slam.svg`).

**Payload** (`processMapMsg`, APP:854-875):
```
{"resolution": <float m/cell>, "width": <int>, "height": <int>,
 "px":..,"py":..,"pz":..,"qx":..,"qy":..,"qz":..,"qw":..,   (map origin pose)
 "data": [<int8 as int>, ...]}                              (row-major, section 3)
```
DOC:1221-1244 documents the same payload.

**As a request:** empty stream. **Replacement:** 6003 (saved map only).

### 6002 MAP_SET_LABLE_REQUEST (PROTO:109). Mode S. `handlLableSetRequest` (APP:1761-1835).

**Request fields:**

| Field | Type rule | Notes |
|---|---|---|
| `mapName` | string (`utils/cyberdog_common/include/cyberdog_common/cyberdog_json.hpp:476-490`) | Must match an existing `/home/mi/mapping/<mapName>.pgm`, otherwise `success:1` (LBL:167-175). Use the name from 6003. |
| `is_outdoor` | bool (IsBool, cyberdog_json.hpp:623-635) | Ignored; the value in the existing json is kept (LBL:221-225). |
| `map_id` | uint64 (cyberdog_json.hpp:556-567) | Unused. |
| `only_delete` | bool, default false | Selects the delete modes. |
| `locationLabelInfo` | array of `{labelName, physicX, physicY}` | An element is kept only if `physicX` and `physicY` pass `IsDouble()`, i.e. JSON numbers with a decimal point or exponent. Otherwise it is **silently dropped** (APP:1781-1784). Units are metres in the `map` frame, stored as float32 (`bridges/protocol/ros/msg/Label.msg`). |

**Semantics** (LBL:159-244):
1. No `<mapName>.pgm`: `success:1` (LBL:171-175).
2. **`only_delete:true` with 0 labels deletes the whole `/home/mi/mapping/` directory** and returns `success:0` (LBL:178-182, :262-266). This is the documented "delete map" (DOC:1283-1309).
3. `<mapName>.json` missing: `success:1` (LBL:186-190).
4. `only_delete:true` with N labels removes those names (coordinates must still be valid doubles).
   - Any name not found gives `success:1`.
   - **Not atomic:** names deleted earlier in the same request stay deleted (LBL:193-207; `label_store.cpp:80-105`, `:215-229`).
5. Otherwise duplicate names give `success:1` (LBL:209-215). If there are none, the server **rewrites the json with exactly the labels in this request**: `map_name`, `is_outdoor` from the old file, and one `{"x":f,"y":f}` per label (LBL:217-243; `label_store.cpp:306-312`).
   - Adding a label means sending the full list.
   - An empty list without `only_delete` clears all labels.

**Response:** `{"success":0|1}` as an int (APP:1821); 0 ok, 1 fail (`bridges/protocol/ros/srv/SetMapLabel.srv`, uint8). Errors: 322 (3 s), 321 (10 s), 323 (APP:1804-1830).

**Example:**
```
{"mapName":"home","is_outdoor":false,"only_delete":false,
 "locationLabelInfo":[{"labelName":"kitchen","physicX":1.5,"physicY":-0.25},
                      {"labelName":"door","physicX":3.0,"physicY":0.0}]}
```

### 6003 MAP_GET_LABLE_REQUEST (PROTO:110). Mode S. `handlLableGetRequest` (APP:1673-1759).

**Request:** `{"mapName":string,"map_id":uint64}`, both optional (APP:1683-1684).
- The label server uses the first `*.pgm` found in `/home/mi/mapping/`. `mapName` is used only when no pgm exists (LBL:93-96).
- [INF] With several maps the choice follows directory iteration order, which is arbitrary. Treat the system as one map at a time.

**Response** (APP:1702-1751), in this key order:
```
{"mapName": str, "success": int, "is_outdoor": bool,
 "locationLabelInfo": [{"labelName": str, "physicX": double, "physicY": double}],
 "map": {"resolution": double, "width": int, "height": int,
         "px": double, "py": double, "pz": double,
         "qx": double, "qy": double, "qz": double, "qw": double,
         "data": [int, ...]}}
```

**`success`** (LBL:76-157, uint8 per `GetMapLabel.srv`):

| Code | Meaning |
|---|---|
| 0 | ok; `map` is filled (LBL:140-147) |
| 1 | json missing (checked first, LBL:102-107) or yaml load failed (LBL:133-138). With no map at all you get 1. |
| 2 | indoor map with empty name (LBL:122-127), or outdoor vision map still building (miloc 300) |
| 3 | outdoor vision map unavailable (miloc 301/302) |
| 4 | miloc service unreachable or threw |
| 5 | label server busy (LBL:86-90) |
| 255 | unknown miloc code (-1 cast to uint8) |

- Codes 2-4 and 255 for outdoor maps come from `CheckVisonMapStatus`, which is public (LBL:119-120, :406-450). Only the miloc status service is [CLOSED/NOT READ].
- When `success != 0`, width and height are 0 and `data` is `[]`, but labels may be present.
- Label coordinates are float32 widened to double (e.g. `0.10000000149011612`).

**Timeout and errors:**
- After a 10 s timeout the handler writes `{"error_code":321}` **and then** a second message with namecode 6003 and empty data (APP:1754-1758).
- 322 if `get_label` is unavailable after 3 s (APP:1677-1681).

**Side effect:** the server publishes the loaded map on `map` (LBL:155).

**Size:** one int per cell. [INF] 400×400 is about 0.5 MB of JSON. The dog caps a message at 5,242,880 bytes (APP:577-578). The client must allow a larger receive size; `app/server.py` sets 32 MB (`server.py:112-121`).

### 6004 NAV_ACTION (PROTO:111). Mode M. `handleNavigationAction(create_new_task=true)` (APP:1411-1636).

Forwards to the ROS 2 action `start_algo_task` (`protocol/action/Navigation`) served by `algorithm_manager` (MGR:46-56).

**Request fields** (APP:1424-1448):

| Field | Type rule | Default | Notes |
|---|---|---|---|
| `type` | uint32 (IsUint, cyberdog_json.hpp:530-541), truncated to uint8 `nav_type` | **uninitialised** (APP:1424) | Always send it; allowed values {1,5,7,9,11,13}; never 0 (section 0 item 6). |
| `outdoor` | bool (IsBool) | false | Must be a JSON literal; it selects lidar (false) or vision (true). |
| `goalX`, `goalY` | double (accepts int or float, cyberdog_json.hpp:602-621) | no pose | A pose is attached only if **both keys exist** (APP:1434-1438). A non-numeric value silently stays 0, giving goal (0,0). Units: metres, `map` frame. |
| `theta` | double, rad | w=0 | Converted to a quaternion (APP:1438-1443) but discarded by the AB executor, which uses x, y, `w=1.0` and frame `map` (`executor_ab_navigation.cpp:314-315`, `:417-420`). DOC range is [-π, π) (DOC:1569). |
| `relative_pos` | uint32 | 0 | 200 auto, 201 behind, 202 left, 203 right (NAVACT:28-32). Ignored by vision tracking (`executor_vision_tracking.cpp:221-226`). |
| `keep_distance` | double | **uninitialised** (APP:1429) | Always send it (e.g. `1.0`). |
| `object_tracking` | bool | false | Type 13: true gives status 3 (object), false gives status 13 (human) (MGRH:286-293). |

`label_id`, `map_name` and `tracking_roi` (NAVACT:23-26) are not settable through gRPC.

**Stream sequence:**
1. Action server not reachable in 3 s: `{"error_code":322}` (APP:1417-1421).
2. Goal sent. SendGoal waits 3 s for the goal response (TASK:181-210). If none arrives, the bridge waits up to 3 s more for feedback (TASK:383-400). The first message may therefore take ~6 s, or ~9 s counting the server wait.
3. Rejected or no response: `{"accepted":2}`, end (APP:1536-1539). Causes are in section 4.2.
4. Accepted: the hash is stored under `nav_type` (APP:1541-1543); then `{"accepted":1}` (APP:1572), then the buffered pre-accept feedback (APP:1576; TASK:250-266).
5. Live feedback `{"feedback_code":int,"feedback_msg":str}`. The manager drops a code equal to the previous one (MGR:331-336). Feedback writes are not gated by `connect_mark_`.
6. Real result: `{"result":int}` is written **only if `connect_mark_`** (APP:1619-1622). For us it never is; the stream just ends.
7. 7200 s without a result: `{"error_code":321}` (APP:1422, :1606-1611).
8. [INF] With heartbeat failure, the stream ends ~5-7.5 s after a failure cycle while the task continues. Re-attach with 6010.

**Result values** (NAVACT:39-45), which we never see: 0 success, 1 accept, 2 unavailable, 3 failed, 4 reject, 5 cancel (TaskCanceled uses abort() with result 5, MGR:375-402).

### 6005 MAP_DOG_POSE_REQUEST (PROTO:112). Mode P (T `dog_pose`, APP:300-303). Unavailable to us.

- `positionchecker` publishes a `geometry_msgs/PoseStamped` lookup of map→base_link every 200 ms, with a 5 s backoff on TF failure (`cyberdog_nav2/positionchecker/src/position_checker_node.cpp:57-80`).
- It starts **only** via the `PoseEnable` SetBool service with data=true (:81-97). The `pose_enable` topic only sets `looping_`; it can stop the loop but not start the thread (:99-102).
- The public executors' realtime-pose enable calls are commented out (e.g. `executor_laser_localization.cpp:178-195`). What enables it on the real dog is unknown [INF].
- Payload `{"px","py","pz","qx","qy","qz","qw"}`, map frame, metres (APP:878-891; DOC:1445-1464).
- There is no pull equivalent; 10001 has no pose.

### 6006 NAV_PLAN_PATH (PROTO:113). Mode P (T `plan`, `nav_msgs/Path`, APP:317-321). Unavailable to us.

- Payload `{"path_point":[{"px":double,"py":double},...]}` (APP:1837-1857; DOC:1627-1649).
- [INF] The frame is `map`.
- The AB executor publishes an empty path after a result (`executor_ab_navigation.cpp:267`).

### 6007 TRACKING_OBJ (PROTO:114). Mode P (T `person`, best effort, APP:335-340). Unavailable to us.

Payload (APP:1906-1945):
```
{"body_info_roi":[{"roi":{"x_offset":int,"y_offset":int,"height":int,"width":int},"reid":str}],
 "track_res_roi":{"x_offset":..,"y_offset":..,"height":..,"width":..}}
```
- ROIs are on a 640×480 image (DOC:1775-1809).
- Without 6007 there is no candidate list.

### 6008 SELECTED_TRACKING_OBJ (PROTO:115). Mode S. `selectTrackingObject` (APP:1947-1991).

**Request:** `{"roi":{"x_offset":int,"y_offset":int,"height":int,"width":int}}`.
- The `roi` object and its keys are required, otherwise 323 (APP:1959-1964).
- Values are read with `GetInt()` without a type check (APP:1967-1970). Send ints only.

**Call:** service `tracking_object_srv` (`protocol/BodyRegion`) with a 30 s timeout (APP:1971). [DOC] It takes 20-30 s (DOC:1835).

**Response:** `{"success":bool}`. Errors: 322, 321.

**Preconditions and effects:**
- A type 13 task must be running with `start_vision_tracking_` set, i.e. after feedback 501 (`executor_vision_tracking.cpp:251`, `:261-265`).
- On success the task emits 502 (:343). On failure it emits 507 and **aborts the task** (:272-341).
- [DOC] For human tracking the ROI must match a box pushed via 6007 (DOC:1813), so human tracking is effectively unusable for us. [INF] Object following (`object_tracking:true`) may accept an arbitrary box.

### 6009 STOP_NAV_ACTION (PROTO:116). Mode S. `handleStopAction` (APP:2066-2102).

**Request:** `{"type":uint32,"map_name":string}`.
- `type` defaults to 0 (APP:2076, Get at :2078-2079). **0 means ResetNav, not no-op.**
- `map_name` matters only for type 5.
- Calls `stop_algo_task` (`protocol/StopAlgoTask`) with a 60 s timeout (APP:2084).

**Response:** `{"result":int}` (APP:2086-2093): 0 SUCCESS, 1 FAILED (`StopAlgoTask.srv`), 10 if the bridge timed out at 60 s. Errors: 322 (3 s), 323.

**Manager semantics** (MGR:190-250):

| `type` | Allowed when `task_status` is | Effect | Status after |
|---|---|---|---|
| 0 ResetNav | 1, 21, 7, 17, 8, 18 (MGR:197-209), else `result:1` | ResetNav Stop: `stop_running_robot_navigation` (sets AB is_exit_, cancels, sleeps 2 s, deactivates; `executor_ab_navigation.cpp:558-604`), then `reset_stop_lidar_localization`, `reset_stop_vision_localization`, then cancel (`executor_reset_nav.hpp:60-91`). Each call has a 2 s service wait and a 60 s timeout (:99-169), so it can exceed the bridge's 60 s and return `result:10` while still running. | 101 (MGR:249) |
| 1 AB | 1 or 21 | If a nav2 goal is active: cancel, wait up to 10 s (`executor_ab_navigation.cpp:611-638`), task result 5. **If no goal has been sent yet (before feedback 300), Stop returns SUCCESS without acting and without setting is_exit_; the executor later sends the goal anyway** (:188-219). | 8 or 18 (MGR:240-247) |
| 5 mapping | 5 or 15 | Non-empty `map_name`: finish=true (save, 15 s call), then the label-server outdoor-flag service writes a **fresh** `<map_name>.json` = `{"is_outdoor":false}` (overwriting existing labels of a same-named map). If that call fails, `result:1` although the map was saved (`executor_laser_mapping.cpp:262-312`, `:369-390`; LBL:297-326). Empty `map_name`: finish=false (:278-280); the effect is [INF] discard. | 101 |
| 7 localization | 7 or 17 | Executor Stop is a no-op (`executor_laser_localization.cpp:206-247`); the status is reset to 101 while relocalization may continue. Use type 0. | 101 (inconsistent) |
| 9 auto dock | 9 | `OnCancel` (`executor_auto_dock.cpp:170-207`) | 101 |
| 11 UWB | 11 | Stop UWB tracking | 101 |
| 13 tracking | 13 or 3 | Stop vision tracking | 101 |
| raw-status match (e.g. 8 in status 8, 9 in status 9) | ReParseStatus passes the raw value through (MGRH:303-326) | Calls the last executor's Stop. [INF] From 8 this does not stop localization. | 101 |
| mismatch | — | Returns without acting and without setting a result, so `result:0`: a fake success (MGR:226-233) | unchanged |

[DOC] DOC:1955-1986 documents 6009, but DOC:1970 says type 0 means "stop all", which the code contradicts.

### 6010 ACCESS_NAV_ACTION (PROTO:117). Mode M. Same handler with `create_new_task=false` (APP:1545-1577).

**Request:** `{"type":uint32}`.

**Lookup** (`task_type_hash_map_`, `std::map<uint8_t,size_t>`, `cyberdog_app.hpp:535`):

| Case | Reply |
|---|---|
| No entry, type != 11 | `{"accepted":3}` (APP:1547-1550) |
| No entry, type 11, cached `task_status == 11` (UWB started elsewhere) | `{"accepted":1}` (APP:1561-1565); the fake feedback loop runs while `connect_mark_` (false for us), then `{"result":3}` is written ungated (APP:1580-1594): "attached, no data" |
| No entry, type 11, status != 11 | `{"accepted":3}` (APP:1567-1569) |
| Entry present, `AccessTask` fails (task finished) | `{"accepted":3}` (APP:1567-1569; TASK:441-460) |
| Success | `{"accepted":1}`, then the latest cached feedback replayed once (APP:1574; TASK:243-249), then live feedback as in 6004 steps 5-8 |

**Edge cases:**
- If no feedback has been produced yet, nothing is replayed. Because of `access_but_before_sending_acception_` (TASK:216-236), later live feedback is not forwarded on that attachment either.
- A second 6010 replaces the task's feedback callback. The earlier attachment stops getting feedback but stays open (TASK:441-460).

### 6011 LASER_SCAN (PROTO:118). Mode P (T `scan`, queue 1, APP:330-332). Unavailable to us.

- TF `map`→**`"laser_frame"` (hardcoded)**. The scan is skipped unless `canTransform` at now within 1 s (APP:1863-1869).
- Ranges below 0.05 m are skipped (APP:1886). [INF] Non-finite ranges are not filtered.
- Each beam is converted to map-frame x, y and pushed as `{"laser_point":[{"px":double,"py":double},...]}` (APP:1877-1903; DOC:1651-1664).

### 6012 ENABLE_POINT_CLOUD (PROTO:119). Mode S. `handleEnablePointCloud` (APP:1993-2029).

**Misnamed: this is the UWB-follow visual obstacle-avoidance switch.**
- Service `switch_visual_obstacle_avoidance` (`std_srvs/SetBool`), 5 s timeout.
- Implemented in `executor_uwb_tracking.cpp:44-59`, `:378-408`. It persists to `UwbTracking.toml` and refuses the change while `deps_lifecycle_activated_` is set.
- **Request:** `{"data":bool}`. `bool enable` is uninitialised if missing (APP:2003-2006); always send it.
- **Response:** `{"success":bool,"message":str}`. Errors: 322, 321.

### 6013 POINT_CLOUD_STATE (PROTO:120). Mode S. `handlePointCloudStateRequest` (APP:2031-2064).

- Service `query_visual_obstacle_avoidance_status` (`std_srvs/Trigger`), 3 s timeout.
- **Request:** `{}`.
- **Response:** `{"success":bool,"message":str}`, where **`success` is the enabled state** read from the toml (`executor_uwb_tracking.cpp:351-374`).

### 10001 STATUS_REQUEST (PROTO:140). Mode S. `statusRequestHandle` (APP:2502-2548).

**Request:** `{}`.

**Response:**
```
{"motion_status":{"motion_id":int},
 "task_status":{"task_status":int,"task_sub_status":int},
 "self_check_status":{"code":int,"description":str},
 "state_switch_status":{"state":int,"code":int},
 "charging_status":{"wired_charging":bool,"wireless_charging":bool}}
```

**Sources of the values:**
- `task_status`/`task_sub_status` are cached from `algo_task_status` (APP:387-389, :2467-2472). The default status is 101 (`cyberdog_app.hpp:641`).
- The manager publishes every 100 ms (MGR:71-79; MGRH:367-372):
  - `task_status` = raw ManagerStatus (table 4.1);
  - `task_sub_status` = `global_feedback_`, which is set on **every** executor feedback before dedup (MGR:327). 10001 therefore shows codes such as 3107 continuously, even though the stream got them only once.
- Reset to 0:
  - always on success (MGR:358);
  - on cancel and abort only when the goal handle is non-null (MGR:380-389, :408-417);
  - `ResetManagerSubStatus` also clears `last_feedback_` (MGRH:333-337).
- **Visibility to polling:**
  - 308, 304 and 3108 are followed immediately by the result and reset, so they are usually missed.
  - 3106 stays for seconds: disable relocalization, lifecycle reset and realsense off all run before the abort (`executor_laser_localization.cpp:137-165`).
  - Mapping code 6 stays for the whole session.
- `charging_status.wireless_charging` is BMS `power_wp_charging` (APP:2491-2492).

DOC:2354-2421 documents 10001.

### 10004 SET_WORK_ENVIRONMENT (PROTO:143). Mode S. `setWorkEnvironmentHandle` (APP:2611-2647).

- **Request:** `{"data":string}`, required; missing or non-string gives 323 (APP:2623-2626). [DOC] Values "pro" or "test" (DOC:2480-2505).
- **Effect:** calls `set_nx_environment` (APP:280), with a 3 s timeout. It writes `env.json` key `environment`, default "pro" (`manager/cyberdog_manager/include/cyberdog_manager/env_contex.hpp:36-85`).
  - The doc names the service `set_work_environment`.
  - The consumer is [CLOSED]; [INF] it selects the cloud backend.
- **Response:** `{"success":bool,"message":str}`. Errors: 322, 321.
- Do not expose it.

---

## 3. Data formats, units, frames

### 3.1 Occupancy grid (6003 `map`, 6001)

**Origin:** loaded by `nav2_map_server::loadMapFromYaml(<name>.yaml)` (LBL:246-260). The yaml and pgm are written by the mapping node [NOT READ].

**Cell values** [CODE]:
- With no `mode` key the mode is Trinary (`cyberdog_nav2/navigation2/nav2_map_server/src/map_io.cpp:146-151`).
- Trinary: 100 occupied, 0 free, -1 unknown (:222-230).
- Scale mode adds 1-99 (:231-243); Raw mode exists (:244-254).
- Accept the whole range -1..100.

**Indexing:**
- `data[row*width+col]`, row-major.
- **Row 0 is the bottom image row**: the pgm is flipped with `data[width*(height-y-1)+x]` (map_io.cpp:258).

**Units and frame:**
- `resolution` is in m/cell (map_io.cpp:187).
- `px,py` is the world position of the lower-left corner of cell (0,0) in `map`.
- The yaw from yaml `origin[2]` is in `qz/qw` (:188-191).

**UI conversions** (yaw 0; otherwise rotate by `yaw=2*atan2(qz,qw)`):
- `col=floor((x-px)/res)`, `row=floor((y-py)/res)`.
- Display pixel `(col, height-1-row)`.
- Cell centre `x=px+(col+0.5)*res`, `y=py+(row+0.5)*res`.

### 3.2 Other formats

- **Labels:** metres in the `map` frame, float32. The file is `{"<labelName>":{"x":f,"y":f},"map_name":..,"is_outdoor":..}` (`label_store.cpp:263-312`), in directory `/home/mi/mapping/` (`label_store.cpp:28`).
- **AB goal:** metres in `map`. The heading is not controllable (w=1, `executor_ab_navigation.cpp:417-420`); the final heading depends on the [NOT READ/CLOSED] controller config. The dock-return path sets yaw tolerance 0.3489 and keeps the file orientation (:326-390); a normal goal restores the tolerance (:391-414).
- **Pose:** `map` frame.
- **Scan:** map-frame metres.
- **Path:** `map` frame [INF].
- **ROI:** pixels on 640×480 [DOC:1775-1835].

---

## 4. Task model (algorithm_manager)

### 4.1 Task types and `task_status`

**NAVACT type enum** (uint8, NAVACT:2-16):
- 0 UNKNOWN
- 1 START_AB, 2 STOP_AB
- 3 START_FOLLOW, 4 STOP_FOLLOW
- 5 START_MAPPING, 6 STOP_MAPPING
- 7 START_LOCALIZATION, 8 STOP_LOCALIZATION
- 9 START_AUTO_DOCKING, 10 STOP_AUTO_DOCKING
- 11 START_UWB_TRACKING, 12 STOP_UWB_TRACKING
- 13 START_HUMAN_TRACKING, 14 STOP_HUMAN_TRACKING

The STOP_* values are unused; stopping is 6009.

**Task.toml entries** (`cyberdog_nav2/algorithm_manager/config/Task.toml`):

| Type | Task | OutDoor | Lines |
|---|---|---|---|
| 1 | NavAB | false, but the match is on id only (MGR:271-275); outdoor=true then needs status 18 | :37-42 |
| 5 | LaserMapping | false (deps camera, map_builder) | :9-14 |
| 5 | VisionMapping | true | :23-28 |
| 7 | LaserLocalization | false | :16-21 |
| 7 | VisionLocalization | true | :30-35 |
| 9 | AutoDock | false (deps include charging_localization) | :60-65 |
| 11 | UwbTracking | false | :1-7 |
| 13 | VisionTracking | false | :44-50 |
| 0 | ResetNav | false; meant for stop only, **but accepted by 6004 (wedge)** | :52-57 |

Any other type (2, 3, 4, 6, 8, 10, 12, 14, ...), or a non-AB type with a mismatched outdoor flag, is rejected: `accepted:2` (MGR:269-288).

**ManagerStatus** (MGRH:39-59), published raw:

| `task_status` | Meaning |
|---|---|
| 100 | uninitialised |
| 101 | idle |
| 102 | launching lifecycle nodes |
| 103 | stopping task |
| 5 | laser mapping |
| 15 | vision mapping |
| 7 | laser relocalization |
| 17 | vision relocalization |
| 6 | laser localization failed |
| 16 | vision localization failed |
| 8 | laser localized (ready for AB) |
| 18 | vision localized |
| 1 | laser AB |
| 21 | vision AB |
| 9 | auto docking |
| 11 | UWB |
| 13 | human tracking |
| 3 | object following |
| 0 | **wedged** after 6004 type 0 (raw cast, MGRH:294) |

### 4.2 Preconditions (the `accepted:2` causes)

- **FSM:** the dog's state machine must be Active or Protected (MGRH:110-124). Entering LowPower stops the task and resets to 101 (MGRH:168-184).
- **Status** (`CheckStatusValid`, MGRH:236-251): type 1 needs 8 (outdoor=false) or 18 (outdoor=true); every other type needs 101, 6 or 16. From 8, send 6009 `{type:0}` first.
- **Task match:** MGR:269-288.
- **Goal response:** none within 3 s plus up to 3 s more for feedback gives `accepted:2` (TASK:181-210, :383-400).
- **Relocalization before AB:** [DOC] required (DOC:1474); [CODE] enforced by status 8/18.
- **Map for relocalization:** `CheckMapAvailable` calls `get_label` and needs `success==0`, so the map must have its `<name>.json`. Otherwise 3102 and abort (`executor_laser_localization.cpp:288-318`). The AB executor's map check is a stub returning true (`executor_ab_navigation.cpp:514-523`).
- **Lidar and sensors:** there is no gRPC switch. Each task activates its lifecycle dependencies (10 s service check, 20 s timeout, EXB:267-310); auto dock uses a 50 s timeout. Failure gives 1002 (types 1/7/9) or 7 (mapping).
- **Standing / motion:**
  - No public posture check exists.
  - Navigation moves the legs through `velocity_adaptor`: cmd_vel → `motion_servo_cmd` with motion_id 309 (the AB executor calls `ChangeGait(309)` via `velocity_adaptor_gait`) and cmd_source 4 (`velocity_adaptor.cpp:26-38`, `:134-140`; `executor_ab_navigation.cpp:171`, `:640-660`).
  - The bridge's 1002 servo stream publishes to the same topic (APP:149-151).
  - [INF] Stand the dog first, and do not stream 1002 while types 1/9/11/13 run. During mapping and relocalization, driving with 1002 is the intended way to move.

### 4.3 Status transitions

| Event | Transition | Source |
|---|---|---|
| Accept | type-specific status (raw cast for 9/11/0) | MGRH:259-295 |
| Success | 7→8, 1→8, 17→18, 21→18; anything else (including 8 after an early AB stop) → 101 | MGR:343-373 |
| Abort | 1→8, 21→18, 7→6, 17→16; else 101 (also 101 if the goal handle is null) | MGR:403-436 |
| Cancel | result 5 (only if the goal handle is non-null); status set by the stop handler | MGR:375-402, :235-249 |

AB success, failure and cancel all end at 8. Only feedback 308 versus 304 distinguishes them.

### 4.4 Feedback codes by task

**Common** (NAVACT:84-86): 1000 preparing dependencies, 1001 ready, 1002 failed.
- They come from the base-class preparation thread, which re-sends every 500 ms (EXB:377-412). They are deduplicated on the stream.
- Used by localization, AB and dock; **not by laser mapping**.

**Mapping (type 5)** (`executor_laser_mapping.cpp:61-136`; NAVACT:66-68):
- `accepted:1`, then **6** started (:133) or **7** failed (:84, :104), then abort to 101.
- No 1000/1001. No result until 6009.

**Relocalization (type 7, laser)** (`executor_laser_localization.cpp:60-204`, `:262-286`; codes in `feedcode_type.hpp:48-64`). Emission order:

| Code | Meaning |
|---|---|
| 3100 | checking map (:72) |
| 3102 | map unavailable (`get_label` != 0), then abort (:76) |
| 3101 | map ok |
| 1000 | preparing (:85) |
| 1002 | deps failed (:90), then abort |
| 1001 | ready (:98) |
| 3103 | starting relocalization services (:108) |
| 3105 | start failed (:112), then abort |
| 3104 | started (:120) |
| 3107 | not localized yet (reloc result 100). It arrives **once** on the stream (dedup) and stays in `task_sub_status`. Drive the dog slowly. |
| 3109 | reloc result 200. It sets the failure flag; WaitRelocalization returns (:353-356) and **3106 follows**. |
| 3106 | timeout after 120 s (:132) or after 3109; then disable, lifecycle reset, realsense off, abort (:137-165). Visible for seconds. |
| **3108** | **success** (:196), immediately followed by the task success (:203) |

3110/3111 are defined but emitted only by the vision localization executor (`executor_vision_localization.cpp:70`).

**AB (type 1)** (`executor_ab_navigation.cpp:97-268`; `feedcode_type.hpp:90-101`):

| Code | Meaning |
|---|---|
| 309 | checking map |
| 311 | map missing (never emitted; the check is a stub, :514-523) |
| 310 | map ok |
| 1000 / 1002 / 1001 | dependency preparation |
| 302 | nav2 connect failed, abort |
| 305 | goal empty or illegal (e.g. dock-return file missing), abort |
| 303 | send goal failed, abort |
| **300** | goal sent |
| 307 | navigating; arrives **once** (dedup; :239) |
| **308** | arrived, then success (:248-249) |
| 304 | nav2 aborted, then abort (:253-255) |

Codes 1-5 and 20 in NAVACT:59-64 are not emitted by this executor.

**Auto dock (type 9)** (`executor_auto_dock.cpp:79-168`):
- 500 from `SetFeedbackCode(500)` plus the 500 ms preparation thread (:84-89).
- 1002 then abort on dependency or stage-2 setup failure (:95-99, :287-326).
- Stage 2 is the `automatic_recharge` action; stage 3 is `seatadjust`, up to 2 tries, waiting for BMS `power_wp_charging` (:108-147, :454-458).
- **Success is reported whether or not charging started** (:149-157).
- [CODE reading, untested] On that success path the preparation thread is never stopped, so 500 may keep appearing in `task_sub_status` and in later tasks' feedback (EXB:377-400, :450-457; MGR:327). Ignore 500 unless the type is 9 or 13.
- AutoDock.toml: stage1 false, stage2 true, stage3 true.
- [DOC] Docking feedback is undefined in DOC:1698.

**Vision tracking (type 13):**
- Executor paths emit 500 (:227), 501 waiting for selection (:251), 502 selected (:343), and 507 then abort on selection failure (:272-341).
- NAVACT:87-96 also defines 503 following, 504 lost and searching, 505 lost, 506/508/509 failures. [INF] These are relayed from the tracking stack.

**UWB (type 11)** (NAVACT:74-82): 10 normal, 11 target lost, 12-14 TF/planner/controller exception, 15 empty target, 16 stair jump, 17 autonomous behaviour, 18 behaviour abnormal.

---

## 5. End-to-end workflows (PC side)

**TaskWatcher (shared re-attach loop)**
1. Run each 6004/6010 call on its own thread or channel call with no or a long deadline. Not under the lock with timeout=8 in `app/server.py:135-141`, which would cut the stream at 8 s and block 10001 and motion calls.
2. When the stream ends without `result`:
   1. Poll 10001.
   2. If `task_status` is still the running status, send 6010 `{type}`.
   3. Treat `accepted:3`, or a status that has left the running value, as ended.
   4. For type 11, `accepted:1` followed at once by `result:3` means "attached, no data".
3. Determine the outcome in this order:
   1. The last terminal feedback (308/304/3108/3106/3102/3105/1002/7/507).
   2. The status transition (4.3).
   3. The task-specific check (dock: `wireless_charging`).
   4. Otherwise report "finished, outcome unknown".
4. Poll 10001 at 2-5 Hz while a task is active.

**Safety:** a, b and c move the dog. Only with the dog on the floor and about 1.5 m clear ([connection.md](../connection.md)).

### (a) Build a map and label it (indoor, lidar)

1. Stand the dog. Check 10001; if the status is not 101/6/16, send 6009 `{type:0}` (only from 1/21/7/17/8/18) and wait for 101.
2. Optional: 6003 to see whether a map exists.
   - Deleting is 6002 `{"mapName":<existing>,"only_delete":true,"locationLabelInfo":[]}`, which wipes all of `/home/mi/mapping/`. Require explicit confirmation.
   - [INF] Whether mapping over an existing map requires deleting first is unknown. The doc does not say the app deletes first.
3. 6004 `{"type":5,"outdoor":false,"keep_distance":0.0}`.
   - Expect `accepted:1`, then **6** (status 5).
   - **7** means failure (status 101).
4. Drive with the motion codes (e.g. 1002). There is no live map preview for us.
5. Save with 6009 `{"type":5,"map_name":"home"}`.
   - Expect `{"result":0}` within 60 s; status 101.
   - `result:1` can mean the outdoor-flag write failed after a successful save; verify with 6003.
   - Re-saving under an existing name **wipes that map's labels** (fresh json, LBL:320-326).
   - An empty name sends finish=false ([INF] discard).
   - [INF] The map builder names pgm/yaml after `map_name` [NOT READ].
6. 6003 `{}`: expect `success:0`, `mapName:"home"`, `map{...}`. Render per 3.1.
7. Labels:
   - Send 6002 with the **complete** list, float coordinates.
   - Verify with 6003.
   - To delete one: `only_delete:true` with that label and float coordinates (e.g. `0.0`).

### (b) AB navigation

1. Status 101/6/16 (else 6009 `{type:0}`). 6003 must return `success:0`, which is also required by relocalization.
2. Relocalize with 6004 `{"type":7,"outdoor":false,"keep_distance":0.0}`.
   - Feedback 3100, 3101, 1000, 1001, 3103, 3104, then possibly 3107 once. While `task_sub_status==3107`, drive slowly forward.
   - **3108** means status 8.
   - Failure: 3102, 3105, 1002, or 3106 (after timeout or 3109); status 6. Retry from 6 without a reset.
   - The limit is 120 s.
3. 6004 `{"type":1,"outdoor":false,"goalX":x,"goalY":y,"theta":0.0,"keep_distance":0.0}`.
   - Requires status 8. Send both goalX and goalY as numbers: a missing key triggers dock-return; a non-number makes the goal (0,0).
   - Feedback 309, 310, 1000, 1001, 300, 307 (once), then **308** or 304. Status 1, then 8.
   - Do not stream 1002 during navigation [INF].
4. Further goals: repeat 3 from status 8.
5. Change goal: only after feedback 300/307, send 6009 `{type:1}` (status 8), then a new 6004.
   - Before 300 the type 1 stop is a no-op and the old goal is still sent later.
   - Use 6009 `{type:0}` instead, then relocalize again.
6. Finish: 6009 `{type:0}`, status 101. It may take longer than 60 s and return `result:10`; keep polling 10001.

### (c) Auto-charge / dock

1. **Near the dock:** 6004 `{"type":9,"outdoor":false,"keep_distance":0.0}`.
   - Requires 101/6/16; from 8, send 6009 `{type:0}` first.
   - Dock detection uses `charging_localization` and the `automatic_recharge` server [CLOSED/NOT READ]. [INF] The dock must be close and in view.
   - Status 9 while running, then 101.
   - Feedback 500 (or 1002 on failure).
   - **Confirm with 10001 `charging_status.wireless_charging`**, which is BMS `power_wp_charging`, the same signal the executor checks. Success is reported even without charging.
2. **Return from afar** [CODE, depends on a file of unknown origin]:
   - In status 8, send 6004 `{"type":1}` **without** goalX/goalY.
   - The executor reads `/SDCARD/ChargePose/charging_pose.txt` (x y z qx qy qz qw) and navigates there with yaw tolerance 0.3489 (`executor_ab_navigation.cpp:326-390`).
   - If the file is missing: 305, abort, status 8.
   - Then 6009 `{type:0}`, then step 1.
   - The file's writer is not public.
3. **Cancel:** 6009 `{type:9}`.

### (d) Cancel and stop

| Situation | Request |
|---|---|
| AB after 300 (status 1) | 6009 `{type:1}`, to 8 |
| AB before 300 | 6009 `{type:0}`, to 101 |
| Localized idle (8) | 6009 `{type:0}`, to 101 |
| Relocalizing (7) | 6009 `{type:0}` (not 7) |
| Mapping (5) | 6009 `{type:5,"map_name":"..."}` to save, or `""` ([INF] discard) |
| Dock (9) | 6009 `{type:9}` |
| UWB (11) | 6009 `{type:11}` |
| Vision tracking (13 or 3) | 6009 `{type:13}` |
| Wedged (0) | No gRPC recovery; LowPower or reboot [INF] |

- Derive `type` from 10001 `task_status`: 1/21→1, 5/15→5, 7/17→0, 8/18→0, 9→9, 11→11, 13/3→13.
- A mismatched type returns `result:0` without acting.
- After any stop, poll 10001 until the expected status appears.
- Emergency motion stop is a motion-area command, not 6009, which can take more than 60 s.

### (e) Tracking

**Vision:**
1. 6004 `{"type":13,"outdoor":false,"object_tracking":true|false,"relative_pos":200,"keep_distance":1.0}`.
2. Wait for 501.
3. Send 6008 with an int ROI on 640×480.
4. Expect 502, or 507 and an abort.
5. Stop with 6009 `{type:13}`.

Human tracking needs a 6007 box we cannot receive (DOC:1813).

**UWB:** 6004 `{"type":11,...}` (needs the UWB tag). Set 6012 before starting, since it is refused while tracking; read with 6013.

---

## 6. What we cannot get

**Push (P) only, unavailable:**

| Code | Data | Topic |
|---|---|---|
| 6001 | map | `map` |
| 6005 | pose | `dog_pose` |
| 6006 | plan | `plan` |
| 6007 | ROIs | `person` |
| 6011 | scan | `scan` |

**Gated on `connect_mark_`:**
- the final 6004/6010 result (APP:1619-1622);
- UWB not-from-app fake feedback (APP:1580-1594);
- hash cleanup after a result (APP:1627-1635), which is harmless;
- [INF] stream longevity.

**Available via 10001 as well as Ticks:** task_status, task_sub_status, motion_id, self_check, state_switch, charging.

**Ticks only (not maps-related):** ip, wifi, soc, internet, sn, audio_playing (APP:519-523; PROTO:54-66).

---

## 7. Client implementation requirements

**Deadlines:**

| Call | Deadline |
|---|---|
| 6003 | ≥ 15 s |
| 6002 | ≥ 15 s |
| 6009 | ≥ 65 s; on `result:10`, keep polling |
| 6008 | ≥ 35 s |
| 6012 / 6013 / 10004 | ≥ 10 s |
| 6004 / 6010 | none or ≥ 7300 s; first message may take ~9 s |

**Concurrency:** streams must not block 10001 or motion calls.

**Receive size:** ≥ 6 MB.

**JSON typing:**
- uint fields as non-negative ints;
- bools as literals;
- label coordinates as floats with a `.`;
- ROI values as ints;
- always send `type`, `keep_distance` and `goalX`+`goalY` (6004 type 1) and `data` (6012);
- never send an empty params string.

**Response handling:**
- `error_code` is terminal.
- Tolerate the extra message after a 6003 timeout.
- Tolerate zero-message streams.
- Ignore stray 500 outside types 9/13.

**Guards:**
- 6002 destructive guard (section 0 item 5).
- Allow 6004 `type` only in {1,5,7,9,11,13} (section 0 item 6).
- Never send 55001.

## 8. Doc versus code

| Doc | Code |
|---|---|
| `blogs/docs/cn/algorithm_manager_cn.md:17`: a mismatched stop fails | Returns 0 (MGR:226-233) |
| `algorithm_manager_cn.md:79`: relocalization results 0/100/1200 | Uses 200 (`executor_laser_localization.cpp:262-286`) |
| DOC:1970: 6009 type 0 stops all | Restricted to 1/21/7/17/8/18 (MGR:197-209) |
| DOC:2480-2505: service `set_work_environment` | Uses `set_nx_environment` (APP:280) |

## 9. Open unknowns (to test on the dog)

- Stream lifetime under heartbeat failure.
- Rest status (expect 101) and FSM state.
- Whether a map exists, the yaml mode, 6003 size and latency.
- Map builder naming, and the effect of finish=false.
- Mapping over an existing map.
- Final AB heading.
- Whether `charging_pose.txt` exists.
- `wireless_charging` on the real dock, and the dock range.
- How much walking 3107 needs.
- The 640×480 ROI space.
- Whether the firmware matches the commits read.
- 6009 type 5 and type 0 latency versus 60 s.
- Whether UWB and vision tracking work without the official app.
- Whether the dog must be standing or auto-stands for nav.
- Whether the auto-dock 500 really persists after success.


## Unknowns that need the real dog

- Does a 6004/6010 stream really end without a result about 5-7.5 s after start (heartbeat-failure unbinding)? Test with type 7 relocalization while polling 10001 and logging stream end times.
- Would a QR/connector provider_ip of 0.0.0.0 stop createGrpc, and with it the stream unbinding, while still allowing WiFi and the :50052 control? (The interaction/connector package was not read.)
- Current 10001 task_status at rest (expected 101) and FSM state; which FSM/motion states make 6004 return accepted:2.
- Does a map already exist in /home/mi/mapping? Also the 6003 response size, latency, map resolution, yaml mode (trinary vs scale) and negate.
- Does the closed map builder name <name>.pgm/.yaml after the map_name given to 6009, so that 6003 finds a matching <name>.json?
- Can mapping start when a previous map exists without deleting it first, and what happens to the old files?
- Real AB end-heading behaviour, given theta is discarded (orientation w=1) and controller_server_ab yaw tolerance is in a closed config.
- Whether /SDCARD/ChargePose/charging_pose.txt exists, and what writes it (dock-return via a 6004 type 1 with no goal).
- Which charging_status flag (wired or wireless) becomes true on the dock, and the dock detection range for type 9.
- How much the dog must walk during relocalization (3107 loop), and success rate within 120 s.
- Whether 6008 ROI coordinates are really in a 640x480 image space on this firmware.
- Whether the deployed firmware (OTA version on the real dog) matches bridges 047a0a48 / cyberdog_nav2 77c65782.
- Latency of 6009 type 5 (map save) relative to the 60 s bridge timeout.
- Whether UWB (type 11) and vision tracking (type 13) work at all without the official app context (UWB tag pairing, camera stack).

## Closed or missing in the public sources

- Reverse-channel certificate: no cert for 'cyberdog2.client' exists and the CA private key (CN=wenlinfeng) is not available, so pushes 6001/6005/6006/6007/6011, results, and UWB fake feedback cannot be received.
- Laser SLAM / mapping servers start_mapping and stop_mapping (map_builder), the writer of the .pgm/.yaml, and the live 'map' publisher during mapping.
- Lidar relocalization node publishing laser_reloc_result and serving reset_stop_lidar_localization.
- Vision/miloc pipeline (outdoor mapping/localization, get_miloc_status, CheckVisonMapStatus codes 3/4).
- Auto-dock servers automatic_recharge, seatadjust and charging_localization; their feedback codes are undocumented.
- Writer of /SDCARD/ChargePose/charging_pose.txt.
- controller_server_ab / nav2 runtime configuration on the dog (goal checker tolerances, costmaps).
- Consumer of params/toml_config/manager/env.json (10004 cloud environment).
- interaction/connector provider_ip logic: public but not read for this spec.
- The official app's UI coordinate conversions and the internal Feishu doc linked as the algorithm task management interface.
- Tracking detector / reid stack behind the 'person' topic and tracking_object_srv internals.

## Verifier corrections (16)

- **Claim:** 6011: the bridge looks up the TF map -> scan.header.frame_id
  - **Correct:** TF map->"laser_frame" (hardcoded). The scan is skipped unless the transform is available at 'now' within 1 s. Ranges < 0.05 m are skipped; non-finite ranges are not.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:1863-1866, :1886
- **Claim:** Mapping (type 5) feedback is 1000, then 1001, then 6 (also workflow (a) step 3: 'Expect accepted:1, feedback 1000, 1001, 6')
  - **Correct:** accepted:1, then 6 (mapping started) or 7 (failed, then abort to 101). No 1000/1001.
  - Source: cyberdog_nav2/algorithm_manager/src/executor_laser_mapping.cpp:61-136 (:84, :104, :133)
- **Claim:** Relocalization: 3109 is a terminal failure; 3107 arrives 'repeated'; 3110/3111 belong to the laser flow; failure set is 3102/3105/3109/3106
  - **Correct:** Laser relocalization terminal outcomes: 3108 then success, or 3106 then abort (after timeout or after 3109), or 3102/3105/1002 then abort. 3107 arrives once on the stream and stays in task_sub_status. AB 307 likewise arrives once.
  - Source: cyberdog_nav2/algorithm_manager/src/executor_laser_localization.cpp:60-204, :262-286, :353-356; cyberdog_nav2/algorithm_manager/src/algorithm_task_manager.cpp:327-340; cyberdog_nav2/algorithm_manager/src/executor_vision_localization.cpp:70
- **Claim:** 10001: final codes (308, 304, 3108, 3106) are visible only for a few milliseconds before the reset
  - **Correct:** 308/304/3108 are transient (usually missed by polling). 3106 is visible for seconds. 6 persists while status is 5.
  - Source: cyberdog_nav2/algorithm_manager/src/executor_ab_navigation.cpp:246-255; executor_laser_localization.cpp:137-165, :196-203; algorithm_task_manager.cpp:71-79
- **Claim:** 6003 success codes 3/4 are vision-map status codes from closed CheckVisonMapStatus; code 2 means only 'map name empty for an indoor map'
  - **Correct:** 0 ok; 1 json missing or yaml load failed; 2 indoor map with empty name OR outdoor vision map still building; 3 outdoor vision map unavailable; 4 miloc service unreachable or threw; 5 label server busy; 255 unknown miloc code.
  - Source: cyberdog_nav2/map_label_server/src/labelserver_node.cpp:76-157, :406-450; bridges/protocol/ros/srv/GetMapLabel.srv
- **Claim:** 6005: dog_pose is published while enabled via the PoseEnable service or the pose_enable topic (position_checker_node.cpp:41-52, :57-80)
  - **Correct:** Published every 200 ms (5 s backoff on TF failure) only after PoseEnable(data=true). The enabler on the real dog is not in the public code [INF].
  - Source: cyberdog_nav2/positionchecker/src/position_checker_node.cpp:57-102; cyberdog_nav2/algorithm_manager/src/executor_laser_localization.cpp:178-195
- **Claim:** 6010: no stored entry gives {"accepted":3}; 'this is also the answer for a type 11 task that the app did not start'
  - **Correct:** type != 11 and no entry: accepted:3. type 11, no entry, status 11: accepted:1 then result:3. type 11, no entry, status != 11: accepted:3. Entry present but AccessTask fails: accepted:3.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:1545-1572, :1580-1594
- **Claim:** The official app deletes the existing map before mapping (DOC:1283-1309)
  - **Correct:** Unknown [INF]. Whether mapping over an existing map requires deleting first is untested.
  - Source: blogs/docs/cn/grpc_protocol.md:1283-1309; blogs/docs/cn/image/grpc_protocol/grpc_slam.svg
- **Claim:** Even type numbers (2..14) have no task entry and are rejected with accepted:2
  - **Correct:** Rejected: any type not in {0,1,5,7,9,11,13}, or a non-AB type whose outdoor flag does not match. Type 0 with outdoor=false is accepted (dangerous).
  - Source: cyberdog_nav2/algorithm_manager/config/Task.toml:1-65; cyberdog_nav2/algorithm_manager/src/algorithm_task_manager.cpp:269-288; bridges/protocol/ros/action/Navigation.action:2-16
- **Claim:** 6009 type 5: an empty map_name discards the map [CODE]
  - **Correct:** Empty map_name sends finish=false; the effect is [INF] (probably discard). With a name: finish=true (15 s call), then the outdoor flag call; if that fails, result:1.
  - Source: cyberdog_nav2/algorithm_manager/src/executor_laser_mapping.cpp:262-312 (:278-280, :300-311)
- **Claim:** 6004 step 2: the handler waits up to 3 s for goal acceptance; accepted:2 if no goal handle arrives within 3 s
  - **Correct:** accepted:1/2 can take up to about 6 s after the 3 s server wait, so worst case about 9 s before the first message.
  - Source: bridges/cyberdog_grpc/include/action_task.hpp:181-210, :383-400; bridges/cyberdog_grpc/src/cyberdog_app.cpp:1417-1421
- **Claim:** The client certs are certs/client-cert.pem and dog_server.pem, both CN=cyberdog2.server
  - **Correct:** One leaf cert (CN=cyberdog2.server, no SAN) present as certs/client-cert.pem and dog_server.pem; CA CN=wenlinfeng; no CA key.
  - Source: certs/client-cert.pem; dog_server.pem (openssl fingerprint)
- **Claim:** Auto dock: feedback codes are not documented [CLOSED servers]; [INF] which charging flag the dock sets is untested, watch both
  - **Correct:** Type 9 feedback: 500 while running; 1002 then abort on preparation failure. Success is reported whether or not charging started. Confirm with 10001 charging_status.wireless_charging (code-derived; real dock untested).
  - Source: cyberdog_nav2/algorithm_manager/src/executor_auto_dock.cpp:84-99, :149-157, :287-326, :454-458; cyberdog_nav2/algorithm_manager/include/algorithm_manager/executor_base.hpp:377-412; bridges/cyberdog_grpc/src/cyberdog_app.cpp:2491-2492
- **Claim:** task_sub_status is reset to 0 when a task succeeds, is cancelled or aborts (MGR:358, :389, :417)
  - **Correct:** The reset is unconditional on success and conditional on a non-null goal handle for cancel and abort. Any later executor feedback overwrites task_sub_status.
  - Source: cyberdog_nav2/algorithm_manager/src/algorithm_task_manager.cpp:327, :343-436
- **Claim:** 6009 type 1 (AB): cancels the nav2 goal; the task result would be 5; status 8/18
  - **Correct:** The type 1 stop is reliable only after feedback 300/307. Before that, use 6009 type 0, whose stop_running_robot_navigation sets is_exit_.
  - Source: cyberdog_nav2/algorithm_manager/src/executor_ab_navigation.cpp:188-219, :558-604; algorithm_task_manager.cpp:235-247
- **Claim:** Line citations: 6008 GetInt at APP:1962-1965; 6012 data at APP:2006-2007; 6009 type default at APP:2077-2079; 6002 success at APP:1820; 6005 enable at position_checker_node.cpp:41-52
  - **Correct:** 6008 GetInt at APP:1967-1970 (roi check :1959-1964). 6012 `bool enable` at APP:2003-2006. 6009 `uint32_t type(0)` at APP:2076, Get at :2078-2079. 6002 success at APP:1821. PoseEnable logic at position_checker_node.cpp:81-102.
  - Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:1959-1970, :2003-2006, :2076-2079, :1821; cyberdog_nav2/positionchecker/src/position_checker_node.cpp:81-102

## Verifier additions

- 6004 {type:0} (or any type that truncates to 0 in the uint8 nav_type, e.g. 256) with outdoor=false in status 101/6/16 is ACCEPTED. Task.toml has ResetNav Id=0, so SetStatus casts the status to raw 0, and ExecutorResetNav::Start only logs 'should never be called' and never completes. The manager is then wedged: every 6004 is rejected (status is not 101/6/16/8/18), 6009 type 0 returns FAILED (0 is not in the allowed set), and 6009 with any other type returns a fake 0 (ReParseStatus(0)=0 does not match). Only LowPower (OnLowPower resets to 101) or a reboot recovers it [INF]. Sources: cyberdog_nav2/algorithm_manager/config/Task.toml:52-57; algorithm_task_manager.hpp:236-251, :294, :303-326, :168-184; executor_reset_nav.hpp:54-58; algorithm_task_manager.cpp:197-233; bridges/protocol/ros/action/Navigation.action:2.
- 6004 goal parsing: a pose is attached only when BOTH goalX and goalY keys exist; values are not validated. A non-numeric goalX/goalY leaves 0, producing goal (0,0). If either key is missing, type 1 takes the dock-return path (/SDCARD/ChargePose/charging_pose.txt). Without theta, orientation.w=0 (later overwritten by the executor). Sources: bridges/cyberdog_grpc/src/cyberdog_app.cpp:1426, :1434-1443; cyberdog_nav2/algorithm_manager/src/executor_ab_navigation.cpp:326-390.
- An early AB stop (6009 type 1 before feedback 300) is a no-op that reports success and sets status 8; the executor still sends the goal later. If that late navigation then succeeds while status is 8, TaskSuccessd does not see status 1 and calls ResetManagerStatus, giving 101. Use 6009 type 0 before 300. Sources: executor_ab_navigation.cpp:188-219, :558-604; algorithm_task_manager.cpp:240-247, :359-372.
- An empty params string is a JSON parse error and returns {"error_code":323} with nameCode 0. Always send at least "{}". Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:3664-3668.
- Laser relocalization's CheckMapAvailable calls the label server get_label and requires success==0, otherwise 3102 and abort. A saved map therefore also needs its <name>.json (created by the outdoor-flag call at save time). Sources: executor_laser_localization.cpp:288-318; labelserver_node.cpp:102-107, :297-326.
- The official doc requires relocalization before AB, done when entering the navigation page. The code enforces this through CheckStatusValid (AB needs 8 or 18). Sources: blogs/docs/cn/grpc_protocol.md:1474; algorithm_task_manager.hpp:241-246.
- AB with outdoor=true requires status 18 (vision localized), although the only AB task entry is OutDoor=false; AB is matched on id only. Sources: algorithm_task_manager.hpp:241-246; algorithm_task_manager.cpp:271-275; Task.toml:37-42.
- Saving a map (6009 type 5 with a name) calls the outdoor-flag service, which writes a fresh {is_outdoor:false} json. Re-saving under an existing name therefore wipes that map's labels. If the outdoor call fails, 6009 returns result 1 although the map was saved. Sources: labelserver_node.cpp:297-326; executor_laser_mapping.cpp:300-311, :369-390.
- 6002 delete of several labels is not atomic. If a later name is not found, success:1 is returned but earlier deletions in the same request stay applied. Source: labelserver_node.cpp:193-207.
- 6003 picks the first *.pgm by directory iteration order. With several maps in /home/mi/mapping the choice is arbitrary [INF: directory_iterator order is unspecified]. Source: labelserver_node.cpp:93-96.
- A second 6010 on the same type replaces the task's feedback callback. The earlier attachment stops receiving feedback but stays open until the result or unbind. Source: bridges/cyberdog_grpc/include/action_task.hpp:441-460.
- Doc-vs-code contradictions: algorithm_manager_cn.md:17 says a mismatched stop fails, but the code returns 0 (algorithm_task_manager.cpp:226-233). algorithm_manager_cn.md:79 lists relocalization results 0/100/1200, but the code uses 200 (executor_laser_localization.cpp:262-286). grpc_protocol.md:1970 says 6009 type 0 stops all, but the code allows it only in status 1/21/7/17/8/18 (algorithm_task_manager.cpp:197-209).
- Human tracking selection (6008) requires an ROI that matches a box pushed via 6007, per the doc. We never receive 6007, so human tracking is effectively unusable for us; object following (object_tracking:true) may accept an arbitrary box [INF]. If selection fails, the executor emits 507 and aborts the task. Sources: blogs/docs/cn/grpc_protocol.md:1813, :1835; executor_vision_tracking.cpp:261-345.
- Standing precondition: no public code in algorithm_manager checks posture. Navigation drives the legs through velocity_adaptor, which converts cmd_vel to motion_servo_cmd with motion_id 309 (set via velocity_adaptor_gait; the AB executor calls ChangeGait(309)) and cmd_source 4. The bridge's own 1002 servo stream publishes to the same motion_servo_cmd topic. [INF] Stand the dog first, and do not stream 1002 while types 1/9/11/13 run. Sources: cyberdog_nav2/velocity_adaptor/src/velocity_adaptor.cpp:26-38, :134-140; executor_ab_navigation.cpp:171, :640-660; bridges/cyberdog_grpc/src/cyberdog_app.cpp:149-151.
- Lidar precondition: no gRPC lidar switch exists in this area. Sensors and SLAM nodes are brought up by each task's lifecycle dependencies (e.g. LaserMapping deps camera and map_builder; ActivateDepsLifecycleNodes uses a 10 s service check and a 20 s timeout). Failure surfaces as 1002 (types 1/7/9) or 7 (mapping). Sources: Task.toml:9-21; executor_base.hpp:267-310.
- Auto-dock preparation thread: on the in-Start success path, StopReportPreparationThread is never called, so the base-class thread keeps calling the feedback callback with 500 every 500 ms. MGR sets global_feedback_ unconditionally, so 10001 task_sub_status may read 500 after docking and during later tasks, and 500 may be interleaved into later tasks' feedback [CODE reading, untested]. The SDK should ignore 500 unless type is 9 or 13. Sources: executor_auto_dock.cpp:84-89, :149-157, :201, :269; executor_base.hpp:142, :377-400, :450-457; algorithm_task_manager.cpp:327.
- Hidden nameCode 55001 ('for testing') publishes disconnect_app=true. The connector then clears provider_ip and persists the empty value. [INF] It does not stop stream unbinding, because createGrpc still runs when the provider ('') differs from the old server_ip, and it may break the app pairing; do not use it. Sources: bridges/cyberdog_grpc/src/cyberdog_app.cpp:3896-3902; interaction/connector/src/connector.cpp:374-385.
- Connector facts behind the 0.0.0.0 idea: subscribeConnectStatus sets app_disconnected=true when the provider is empty or "0.0.0.0", and calls createGrpc only when server_ip != provider. server_ip is "0.0.0.0" only at bridge start and changes after createGrpc. A provider of 0.0.0.0 therefore avoids the heartbeat loop only from a fresh bridge start (dog reboot) [INF]. The connector sets the provider from the QR in DoConnect. Sources: bridges/cyberdog_grpc/src/cyberdog_app.cpp:119, :626-663, :686, :714; interaction/connector/src/connector.cpp:408-411, :602-606.
- The heartbeat loop also fails when the connector_state topic is older than 2 s: it sets hearbeat_result=false and app_disconnected=true, which then skips destroy/createGrpc on the 5 s failure path. Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:493-494, :524-527, :544-547.
- The SLAM nodes (laser map_builder, localization_node, occmap, miloc) are not listed in cyberdog_ws/cyberdog.repos and were not cloned. Label them [NOT READ] rather than definitively closed. Source: cyberdog_ws/cyberdog.repos.
- Per-code push dispatcher queue sizes (irrelevant to us while P is unavailable): map 2, dog_pose 2, plan 2, laser 1, tracking 2. Source: bridges/cyberdog_grpc/src/cyberdog_app.cpp:297, :303, :321, :332, :340.
- 6009 type 0 can exceed the bridge's 60 s wait. ExecutorResetNav::Stop calls stop_running_robot_navigation (which sleeps 2 s and cancels with a 10 s wait), then two reset services, each with a 2 s service wait and a 60 s call timeout. The bridge then returns {"result":10} while the reset continues. Sources: executor_reset_nav.hpp:60-169; executor_ab_navigation.cpp:558-638; bridges/cyberdog_grpc/src/cyberdog_app.cpp:2084-2092.
- Navigation.action type names: 0 UNKNOWN, 1 START_AB, 2 STOP_AB, 3 START_FOLLOW, 4 STOP_FOLLOW, 5 START_MAPPING, 6 STOP_MAPPING, 7 START_LOCALIZATION, 8 STOP_LOCALIZATION, 9 START_AUTO_DOCKING, 10 STOP_AUTO_DOCKING, 11 START_UWB_TRACKING, 12 STOP_UWB_TRACKING, 13 START_HUMAN_TRACKING, 14 STOP_HUMAN_TRACKING. nav_type is uint8. The STOP_* values are not used by the manager; stopping is 6009. Source: bridges/protocol/ros/action/Navigation.action:2-16.
- 6009 with type equal to a raw status that ReParseStatus passes through (e.g. type 8 in status 8, type 9 in status 9) matches and calls activated_executor_->Stop, then resets to 101. For status 8 that is the last executor (AB or localization), not ResetNav, so localization may keep running while status says 101 [INF]. Always use type 0 from 8/18. Source: algorithm_task_manager.cpp:226-249; algorithm_task_manager.hpp:303-326.