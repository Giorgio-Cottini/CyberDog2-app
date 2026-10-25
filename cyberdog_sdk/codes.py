"""Name codes, motion ids and the meaning of result codes (it + en).

Sources: proto/cyberdog_app.proto, MotionID.msg, motion.md §2.8,
grpc_fidelity.md §5 and §7.6.
"""
from __future__ import annotations

from typing import Dict, Tuple


class NameCode:
    GET_DEVICE_INFO = 1001
    MOTION_SERVO_REQUEST = 1002
    MOTION_SERVO_RESPONSE = 1003
    MOTION_CMD_REQUEST = 1004
    DEVICE_NAME_SWITCH = 1010
    DEVICE_NAME_SET = 1011
    DEVICE_VOLUME_SET = 1012
    DEVICE_MIC_SET = 1013
    DEVICE_AUDIO_SET = 1014
    STOP_AUDIO_PLAY = 1025
    VISUAL_BACKEND_MSG = 2001
    VISUAL_FRONTEND_MSG = 2002
    IMAGE_TRANSMISSION_REQUEST = 4001
    CAMERA_SERVICE = 4002
    FILES_NOT_DOWNLOAD_COMPLETE = 4003
    DOWNLOAD_FILE = 4004
    MAP_DATA_REQUEST = 6001
    MAP_SET_LABLE_REQUEST = 6002
    MAP_GET_LABLE_REQUEST = 6003
    NAV_ACTION = 6004
    STOP_NAV_ACTION = 6009
    ACCESS_NAV_ACTION = 6010
    ENABLE_POINT_CLOUD = 6012
    POINT_CLOUD_STATE = 6013
    SET_BT_TREAD = 8011
    GET_BT_TREAD = 8013
    UNLOCK_DEVELOP_ACCESS = 9030
    REBOOT_MACHINE = 9031
    STATUS_REQUEST = 10001
    LOW_POWER_EXIT = 10002
    AUTO_LOW_POWER_ENABLE = 10003
    SET_WORK_ENVIRONMENT = 10004
    POWER_OFF = 10006
    ENABLE_ELEC_SKIN = 11001
    SET_ELEC_SKIN = 11002
    DOG_LEG_CALIBRATION = 12001
    TEST_DISCONNECT = 55001


class MotionId:
    ESTOP = 0
    GETDOWN = 101
    GETDOWN_PASSIVE = 102
    RECOVERYSTAND = 111
    WALK_STAND = 112
    SIT = 143
    POSE_ABS = 201          # FORCECONTROL_DEFINITIVELY
    POSE_REL = 202          # FORCECONTROL_RELATIVEYLY
    WALK_USERTROT = 303     # slow
    WALK_ADAPTIVELY = 304   # auto
    WALK_FLYTROT = 305      # fast
    WALK_MEDIUM = 308


SERVO_START, SERVO_DATA, SERVO_END = 0, 1, 2
CAM_TAKE_PICTURE, CAM_START_LIVE_STREAM, CAM_STOP_LIVE_STREAM = 1, 7, 8

# result codes of 1004 replies and bridge error_code values
RESULT_CODES: Dict[int, Tuple[str, str]] = {
    0: ("OK", "OK"),
    321: ("timeout o chiamata fallita (il movimento può continuare)",
          "timeout or call failure (the motion may still be running)"),
    322: ("servizio non disponibile", "service unavailable"),
    323: ("errore JSON o campo obbligatorio mancante", "JSON error or missing required field"),
    3001: ("priorità servo persa", "servo priority lost"),
    3003: ("macchina a stati non attiva", "state machine not active"),
    3008: ("motion_id sconosciuto", "unknown motion_id"),
    3010: ("regola della durata violata", "duration rule violated"),
    3011: ("occupato: un altro movimento è in corso", "busy: another motion is running"),
    3013: ("comando protetto in stato protetto (batteria scarica)",
           "protected command in Protected state (low battery)"),
    3021: ("arresto di emergenza", "emergency stop"),
    3022: ("transizione vietata", "transition banned"),
    3023: ("smorzamento di emergenza", "emergency damping"),
    3024: ("robot sollevato", "robot lifted"),
    3025: ("surriscaldamento", "over-heat"),
    3026: ("batteria scarica", "low battery"),
    3027: ("timeout della transizione", "transition timeout"),
    3028: ("timeout dell'esecuzione", "execute timeout"),
    3031: ("errore nella definizione della sequenza", "sequence definition error"),
    3032: ("motore offline", "motor offline"),
    3033: ("motore surriscaldato", "motor over-heat"),
    3034: ("motore sovraccarico", "motor overload"),
    3035: ("timeout del feedback LCM", "LCM feedback timeout"),
    3040: ("arresto di emergenza attivo: solo 0 o 111", "estop latched: only 0 or 111"),
    3050: ("errore di orientamento", "orientation error"),
    3051: ("errore di posizione dei piedi", "foot position error"),
    3052: ("bloccato in piedi", "stand stuck"),
    3053: ("motore surriscaldato", "motor over-heat"),
    3054: ("sovracorrente del motore", "motor over-current"),
    3055: ("errore del motore", "motor error"),
    3056: ("in carica", "charging"),
}

# getFile FileChunk.error_code (grpc_fidelity.md §7.6)
FILE_ERRORS: Dict[int, Tuple[str, str]] = {
    -1: ("nessun chunk ricevuto", "no chunk received"),
    0: ("dati", "data"),
    1: ("fotocamera: argomenti non validi", "camera: invalid args"),
    2: ("fotocamera: non supportato", "camera: unsupported"),
    3: ("fotocamera: timeout", "camera: timeout"),
    4: ("fotocamera: occupata", "camera: busy"),
    5: ("fotocamera: stato non valido (riavviare lo stream)", "camera: invalid state (re-arm the stream)"),
    6: ("fotocamera: errore interno", "camera: inner error"),
    7: ("servizio fotocamera non disponibile o in timeout", "camera service unavailable or timed out"),
    8: ("risposta fotocamera non nel formato nome,dimensione", "camera reply is not name,size"),
    9: ("file mancante", "file missing"),
    10: ("più di 5 trasferimenti in corso", "more than 5 concurrent transfers"),
    255: ("fotocamera indefinita o nameCode errato", "camera undefined or wrong nameCode"),
}


def describe(code: int, lang: str = "it", table: Dict[int, Tuple[str, str]] = RESULT_CODES) -> str:
    entry = table.get(code)
    if entry is None:
        return ("codice %d" if lang == "it" else "code %d") % code
    return entry[0] if lang == "it" else entry[1]
