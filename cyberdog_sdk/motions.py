"""Motion catalog (motion.md §6, plus the gaits of §5.1 and the postures of §4).

risk: R0 static, R1 in place / dynamic / two-leg, R2 travels or runs long,
R3 ballistic jumps, R4 flips, RL or special (registration-only ids and
unclassified custom gaits are filed here too, so they never show up in an
R0-R2 menu).
min_ms is MAP min_exec_time; typical_s is the doc's valid-duration lower bound
or the length stated in the text, None when unknown.
needs: "sit" = must be sitting (143 first); "never" = not sendable through the
bridge; "servo" = a gait, stream it with Driver; "pose" = needs pos_des/rpy_des
arrays (Posture or motion(..., rpy_des=...)).
"""
from __future__ import annotations

from typing import Dict, List, NamedTuple, Optional, Tuple


class Motion(NamedTuple):
    id: int
    key: str
    name_it: str
    name_en: str
    risk: str
    lr_safe: bool
    min_ms: int
    typical_s: Optional[float]
    needs: Tuple[str, ...]


_S, _N, _V, _P = ("sit",), ("never",), ("servo",), ("pose",)

_TABLE = [
    # id, key, name_it, name_en, risk, lr_safe, min_ms, typical_s, needs
    # basic / state
    (0, "estop", "arresto di emergenza", "emergency stop", "R0", True, 1000, None, ()),
    (101, "lie_down", "sdraiati", "lie down", "R0", True, 5000, 3.0, ()),
    (102, "lie_down_passive", "sdraiati (passivo)", "lie down (passive)", "R0", False, 5000, None, ()),
    (111, "stand", "alzati", "recovery stand", "R0", True, 15000, 6.0, ()),
    (112, "walk_stand", "in piedi pronto a camminare", "walk-stand", "R0", False, 5000, None, ()),
    (113, "reg_113", "solo registrazione", "registration only", "R4", False, 5000, None, _N),
    (114, "reg_114", "solo registrazione", "registration only", "R4", False, 0, None, _N),
    (115, "reg_115", "solo registrazione", "registration only", "R4", False, 0, None, _N),
    (117, "rl_stand_fast", "alzata RL veloce", "RL stand-up fast", "R2", False, 5000, None, ()),
    (118, "rl_stand_slow", "alzata RL lenta", "RL stand-up slow", "R2", False, 5000, None, ()),
    (119, "rl_stand_medium", "alzata RL media", "RL stand-up medium", "R2", False, 5000, None, ()),
    (124, "roll_left", "sdraiati sul fianco sinistro e rialzati", "lie on left side and recover",
     "R2", False, 20000, None, ()),
    (138, "roll_right", "sdraiati sul fianco destro e rialzati", "lie on right side and recover",
     "R2", False, 20000, None, ()),
    (125, "dog_walking", "passeggiata al guinzaglio", "dog-walking", "R2", False, 1000, None, ()),
    # tricks, mode 62
    (141, "shake_left", "dai la zampa sinistra", "shake left hand", "R0", True, 15000, 8.6, ()),
    (142, "shake_right", "dai la zampa destra", "shake right hand", "R0", True, 15000, 8.6, ()),
    (143, "sit", "seduto", "sit down", "R0", True, 15000, 1.3, ()),
    (148, "sit_swing_left", "seduto, testa a sinistra", "sit + swing head left", "R0", True, 15000, None, _S),
    (149, "sit_swing_right", "seduto, testa a destra", "sit + swing head right", "R0", True, 15000, None, _S),
    (150, "sit_swing_both", "seduto, testa a destra e sinistra", "sit + swing head left-right",
     "R0", True, 15000, None, _S),
    (144, "hip_circles", "cerchi con i fianchi", "hip circles", "R0", True, 15000, 3.9, ()),
    (145, "head_circles", "cerchi con la testa", "head circles", "R0", True, 15000, 3.9, ()),
    (146, "stretch", "stiracchiata", "stretch", "R0", True, 15000, 3.3, ()),
    (173, "squats", "piegamenti sulle zampe", "squats", "R0", True, 15000, None, ()),
    (155, "lift_left_nod", "alza la zampa sinistra e annuisci", "lift left leg and nod",
     "R1", True, 15000, None, ()),
    (156, "lift_right_nod", "alza la zampa destra e annuisci", "lift right leg and nod",
     "R1", True, 15000, None, ()),
    (161, "march_nod", "marcia sul posto annuendo", "march in place nodding", "R1", True, 15000, None, ()),
    (164, "front_leg_lifts", "alza le zampe anteriori a turno", "alternate front leg lifts",
     "R1", True, 15000, None, ()),
    (165, "rear_leg_lifts", "alza le zampe posteriori a turno", "alternate rear leg lifts",
     "R1", True, 15000, None, ()),
    (174, "push_ups", "flessioni", "human-like push-ups", "R1", True, 15000, 14.0, ()),
    (157, "splits_1", "spaccata 1", "leg splits 1", "R1", False, 15000, None, ()),
    (158, "splits_2", "spaccata 2", "leg splits 2", "R1", False, 15000, None, ()),
    (152, "moonwalk", "moonwalk", "moonwalk", "R2", False, 15000, 9.0, ()),
    (159, "walk_nod", "cammina annuendo", "walk and nod", "R2", False, 15000, None, ()),
    (160, "splay_walk", "cammina allargando le zampe", "splay/adduct walk", "R2", False, 15000, None, ()),
    (167, "side_stretch_step", "passi con allungamento laterale", "side-stretch stepping",
     "R2", False, 15000, None, ()),
    (169, "side_swing_step", "passi oscillando di lato", "side-swing stepping", "R2", False, 15000, None, ()),
    (170, "right_leg_step", "passi avanti-indietro zampa destra", "right leg fore-aft stepping",
     "R2", False, 15000, None, ()),
    (171, "left_leg_step", "passi avanti-indietro zampa sinistra", "left leg fore-aft stepping",
     "R2", False, 15000, None, ()),
    (151, "ballet", "balletto", "ballet", "R2", False, 30000, 27.0, ()),
    (140, "dance_collection", "raccolta di balli", "dance collection", "R2", False, 60000, 60.0, ()),
    (176, "dance_1", "ballo 1", "dance 1", "R2", False, 120000, 120.0, ()),
    (177, "dance_2", "ballo 2", "dance 2", "R2", False, 120000, 120.0, ()),
    (178, "dance_3", "ballo 3", "dance 3", "R2", False, 120000, 120.0, ()),
    (501, "dance_4", "ballo 4", "dance 4", "R2", False, 120000, 120.0, ()),
    (502, "dance_5", "ballo 5", "dance 5", "R2", False, 120000, 120.0, ()),
    (503, "dance_6", "ballo 6", "dance 6", "R2", False, 120000, 120.0, ()),
    (504, "dance_7", "ballo 7", "dance 7", "R2", False, 120000, 120.0, ()),
    (505, "dance_8", "ballo 8", "dance 8", "R2", False, 120000, 120.0, ()),
    (153, "jumping_jacks_1", "saltelli a gambe divaricate 1", "jumping jacks 1", "R3", False, 15000, None, ()),
    (154, "jumping_jacks_2", "saltelli a gambe divaricate 2", "jumping jacks 2", "R3", False, 15000, None, ()),
    (162, "front_leg_jumps", "salti avanti-indietro zampe anteriori", "front legs fore-aft jumps",
     "R3", False, 15000, None, ()),
    (163, "rear_leg_jumps", "salti avanti-indietro zampe posteriori", "rear legs fore-aft jumps",
     "R3", False, 15000, None, ()),
    (166, "jump_collection", "raccolta di salti", "jump collection", "R3", False, 15000, None, ()),
    (168, "swing_jumps", "salti oscillando avanti-indietro", "fore-aft swing jumps",
     "R3", False, 15000, None, ()),
    (139, "running_flip", "capriola in corsa", "running flip", "R4", False, 30000, 30.0, ()),
    (172, "backflips", "capriole all'indietro consecutive", "consecutive backflips", "R4", False, 15000, None, ()),
    (179, "front_flip_clap", "capriola in avanti con applauso", "front flip with clap",
     "R4", False, 15000, None, ()),
    (400, "user_sequence", "sequenza utente", "user sequence", "R4", False, 15000, None, _N),
    # mode 64 (two-leg)
    (123, "bow", "inchino / saluto", "bow / greeting", "R1", True, 10000, 8.7, ()),
    (175, "bow_heart", "inchino con cuore", "bow + heart", "R1", True, 15000, None, ()),
    # jumps, mode 16 / 23
    (130, "jump_turn_left", "salto 3D, gira a sinistra 90°", "3D jump, turn left 90°", "R3", False, 1500, 0.79, ()),
    (131, "jump_turn_right", "salto 3D, gira a destra 90°", "3D jump, turn right 90°", "R3", False, 1500, 0.92, ()),
    (132, "jump_forward_60", "salto 3D, avanti 60 cm", "3D jump, forward 60 cm", "R3", False, 2500, 0.79, ()),
    (133, "jump_forward_30", "salto 3D, avanti 30 cm", "3D jump, forward 30 cm", "R3", False, 1500, None, ()),
    (134, "jump_left_20", "salto 3D, sinistra 20 cm", "3D jump, left 20 cm", "R3", False, 1500, None, ()),
    (135, "jump_right_20", "salto 3D, destra 20 cm", "3D jump, right 20 cm", "R3", False, 1500, None, ()),
    (136, "jump_up_30", "salto 3D, in alto 30 cm", "3D jump, up 30 cm", "R3", False, 1500, 0.76, ()),
    (137, "jump_down_step", "salta giù da un gradino", "jump down a step", "R3", False, 5000, None, ()),
    (147, "jump_down_platform", "salta giù da una piattaforma", "jump down from a platform",
     "R3", False, 15000, None, ()),
    (500, "jump_paw", "salto con zampa alzata", "jump with raised paw", "R3", False, 1000, None, ()),
    (126, "jump_up_step", "salta su un gradino", "jump up a step", "R3", False, 5000, None, ()),
    # flips and special
    (120, "flip_stand", "innesco capriola", "flip-stand trigger", "R4", False, 8000, None, ()),
    (121, "backflip", "capriola all'indietro", "backflip", "R4", False, 8000, None, ()),
    (122, "front_flip", "capriola in avanti", "front flip", "R4", False, 8000, None, ()),
    (127, "side_flip_right", "capriola laterale destra", "right side flip", "R4", False, 8000, None, ()),
    (128, "side_flip_left", "capriola laterale sinistra", "left side flip", "R4", False, 8000, None, ()),
    (129, "running_front_flip", "capriola in avanti in corsa", "running front flip", "R4", False, 8000, None, ()),
    (506, "platform_backflip", "capriola indietro da piattaforma", "platform backflip", "R4", False, 1900, None, ()),
    (507, "platform_front_flip", "capriola avanti da piattaforma", "platform front flip",
     "R4", False, 1800, None, ()),
    (508, "skate_backflip", "capriola indietro sullo skateboard", "skateboard backflip",
     "R4", False, 1900, None, ()),
    (509, "skate_front_flip", "capriola avanti sullo skateboard", "skateboard front flip",
     "R4", False, 1800, None, ()),
    (510, "skateboard_1", "skateboard 1", "skateboard 1", "R4", False, 10000, None, ()),
    (511, "skateboard_2", "skateboard 2", "skateboard 2", "R4", False, 10000, None, ()),
    (512, "rl_upright_walk", "camminata eretta RL", "RL upright walk", "R4", False, 10000, None, ()),
    (513, "handstand_split", "verticale con spaccata", "handstand split", "R4", False, 10000, None, ()),
    # gaits (§5.1): stream them with Driver, never through 1004
    (303, "walk_slow", "camminata lenta", "slow walk", "R2", False, 0, None, _V),
    (308, "walk_medium", "camminata media", "medium walk", "R2", False, 0, None, _V),
    (305, "walk_fast", "camminata veloce", "fast walk", "R2", False, 0, None, _V),
    (304, "walk_auto", "trotto adattivo", "auto-frequency trot", "R2", False, 0, None, _V),
    (309, "walk_follow", "andatura di inseguimento", "follow gait", "R2", False, 0, None, _V),
    (301, "bound", "galoppo a balzi", "bound", "R3", False, 0, None, _V),
    (302, "pronk", "saltelli a quattro zampe", "pronk", "R3", False, 0, None, _V),
    (306, "tripod", "andatura a tripode", "tripod gait", "R2", False, -1, None, _V),
    (307, "rl_spin", "rotazione RL", "RL spin", "R4", False, 0, None, _V),
    (310, "ballet_gait", "andatura da balletto", "ballet gait", "R2", False, 0, None, _V),
    (311, "rl_bounce", "rimbalzo RL", "RL bounce", "R4", False, 0, None, _V),
    (340, "stairs_340", "scale", "stairs", "R2", False, -1, None, _V),
    (341, "stairs_341", "scale", "stairs", "R2", False, -1, None, _V),
    (342, "slope_342", "pendenza", "slope", "R2", False, -1, None, _V),
    (343, "tof_stairs_343", "scale con TOF", "TOF stairs", "R2", False, -1, None, _V),
    (344, "visual_stairs", "scale con visione", "visual stairs", "R2", False, -1, None, _V),
    (345, "adaptive_walk", "camminata adattiva", "adaptive walk", "R2", False, -1, None, _V),
    (390, "reg_390", "solo registrazione", "registration only", "R4", False, -1, None, _N),
    (391, "reg_391", "solo registrazione", "registration only", "R4", False, -1, None, _N),
    (392, "reg_392", "solo registrazione", "registration only", "R4", False, -1, None, _N),
    # postures (§4)
    (201, "pose_abs", "postura assoluta (forza)", "absolute force-control posture", "R0", False, 0, None, _P),
    (202, "pose_rel", "postura relativa (forza)", "relative force-control posture", "R0", False, 0, None, _P),
    (203, "reg_203", "solo registrazione", "registration only", "R4", False, -1, None, _N),
    (211, "pose_abs_pos", "postura assoluta (posizione)", "absolute position-control posture",
     "R0", False, -1, None, _P),
    (212, "pose_rel_pos", "postura relativa (posizione)", "relative position-control posture",
     "R0", False, -1, None, _P),
    (213, "handshake_pose", "stretta di mano", "handshake (lifted leg)", "R1", False, -1, None, _P),
    (214, "pose_interp", "interpolazione di postura", "posture interpolation", "R0", False, -1, None, _P),
]
# custom 180-199 [11,80..99]: preset custom gaits 0-19, content unknown
_TABLE += [(i, "custom_%d" % (i - 180), "personalizzato %d" % (i - 180), "custom %d" % (i - 180),
            "R4", False, 15000, None, ()) for i in range(180, 200)]

_BY_ID: Dict[int, Motion] = {row[0]: Motion(*row) for row in _TABLE}
assert len(_BY_ID) == len(_TABLE), "duplicate motion id in the catalog"


def get(motion_id: int) -> Optional[Motion]:
    return _BY_ID.get(motion_id)


def all() -> List[Motion]:     # noqa: A001 (name fixed by contracts.md §3)
    return sorted(_BY_ID.values())


def by_risk(*classes: str) -> List[Motion]:
    return [m for m in all() if m.risk in classes]
