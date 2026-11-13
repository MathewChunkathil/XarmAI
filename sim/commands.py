"""Rule-based command parser (NO LLM, no PyBullet).  SIMULATION ONLY.

    r = parse("place the green cube in zone 1 and stack red on green")
    r.tasks   -> [Task(place, cube 2, zone 1), Task(stack, cube 1, onto 2)]       (ordered so dependencies work)
    r.echo    -> "1) place green (cube 2) in zone 1; 2) stack red (cube 1) on green (cube 2)"
    r.errors  -> clear messages (unknown cube, cube not in scene, zone out of range, missing target, ...)

Made for speech-to-text output (faster-whisper later): lower-case, punctuation stripped, number words
(one/two/three, first/second/third), common mishearings: "zone" -> zoan/zon/soon/sun ... only when a number follows;
"cube to zone" (preposition) vs "cube two" (number: only right after "cube"). Clauses split on and / then / , / .
Supported (cube = red|green|blue or "cube 1..3", the first/second/third cube):
    place|put|move|drop|bring|take|carry|send ... [the] <cube> in|into|to|on|at [zone] <n>
    cube 2 to zone 2                      stack <cube> on [top of] <cube>      put blue on red
    all cubes to zone 3                   stack all in zone 2                  home | reset | park
Conveyor (moving-cube pick, see sim/belt_task.py):
    start|stop [the] conveyor|belt        turn the belt on|off                 set conveyor speed to 4 [cm per second]
    feed [the] red cube [on|onto the conveyor|belt]                            put the red cube on the belt
    pick|catch|grab [the] green cube from|on the conveyor|belt [and place|put it] in zone 2
"""
import re
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

DEFAULT_CUBES = {1: "red", 2: "green", 3: "blue"}

NUM_WORDS = {"one": 1, "two": 2, "three": 3, "first": 1, "second": 2, "third": 3, "won": 1, "1": 1, "2": 2, "3": 3,
             "4": 4, "5": 5, "6": 6, "7": 7, "8": 8, "9": 9, "four": 4, "five": 5, "six": 6}
AFTER_ZONE_NUM = dict(NUM_WORDS, to=2, too=2, tu=2, tue=2, tree=3, free=3, tho=2, wan=1, tri=3, thee=3)   # "zone to" = zone 2 (speech), ONLY after a zone word
ORDINALS = {"first": 1, "second": 2, "third": 3}
ZONE_WORDS = {"zone", "zones", "zoan", "zon", "zoned", "zown", "soon", "sun", "zoom", "joan", "sewn", "loan", "sown", "zohn"}
CUBE_WORDS = {"cube", "cubes", "block", "blocks", "box", "cuboid", "cubed", "kube", "cue", "cubs", "cub"}
COLOR_ALIASES = {"red": 1, "read": 1, "reed": 1, "rid": 1, "wred": 1,
                 "green": 2, "grin": 2, "greene": 2, "grene": 2, "grean": 2, "grain": 2, "greens": 2,
                 "blue": 3, "blew": 3, "blu": 3, "blu'": 3, "blues": 3}
AMBIGUOUS_COLORS = {"read", "reed", "rid", "grin", "grain", "blew", "blu", "blues", "greens"}   # real English words
PLACE_VERBS = {"place", "put", "move", "drop", "bring", "take", "carry", "send", "deliver", "set", "transfer", "relocate",
               "push", "shift", "lay"}
STACK_VERBS = {"stack", "pile", "tower"}
HOME_WORDS = {"home", "reset", "park", "rest"}
PREPS = {"in", "into", "to", "on", "onto", "at", "inside", "within", "over", "upon", "unto"}
ALL_WORDS = {"all", "every", "each", "everything", "both"}
FILLERS = {"the", "a", "an", "please", "robot", "arm", "can", "you", "now", "just", "it", "then", "of", "top", "number",
           "back", "go", "up", "down", "area", "position", "spot", "pick", "that", "this", "cube's", "my", "your", "one"}
CONTEXT_BEFORE_COLOR = {"the", "a", "and", "plus", "on", "onto", "to", "put", "place", "move", "stack", "take", "bring",
                        "with", "all", "then", "pick", "grab", "of", "over", "upon", "cube", "number"} | PLACE_VERBS | STACK_VERBS
SPLIT_WORDS = {"and", "then", "next", "afterwards", "after", "plus", "also"}
BELT_WORDS = {"conveyor", "conveyors", "conveyer", "conveyers", "convoyer", "belt", "belts", "conveyor's", "belt's"}
FEED_VERBS = {"feed", "load", "insert", "spawn", "introduce", "inject"}
PICK_BELT_VERBS = {"pick", "catch", "grab", "grasp", "get", "fetch", "intercept", "snatch", "collect", "lift"}
STOP_WORDS = {"stop", "halt", "pause", "off", "disable", "kill", "shut", "end", "freeze", "deactivate"}
START_WORDS = {"start", "run", "on", "enable", "begin", "activate", "resume", "power", "turn", "switch", "go", "launch", "engage"}
BELT_MAX_CM_S = 6.0
BELT_ACTIONS = ("belt_start", "belt_stop", "belt_speed", "feed", "pick_belt")

_NUMW = r"(?:zero|one|two|three|four|five|six|seven|eight|nine|ten)(?:\s+point\s+(?:zero|one|two|three|four|five|six|seven|eight|nine))?"
_NUM = r"(?:\d+(?:[.,]\d+)?|" + _NUMW + ")"
_UNIT = (r"(?:\s*(?P<unit>centimet(?:er|re)s?|cm|millimet(?:er|re)s?|mm|met(?:er|re)s?|m)\b)?"
         r"(?:\s*(?:/|per|a|each)\s*(?:s|sec|secs|second|seconds)\b)?")
SPEED_RE = re.compile(
    r"\b(?:(?:set|change|make|adjust|put|turn|run|speed)\s+)*(?:the\s+)?"
    r"(?:(?:(?:conveyor|conveyer|belt)(?:'s)?\s+(?:speed|velocity))|(?:(?:speed|velocity)\s+of\s+the\s+(?:conveyor|conveyer|belt))|"
    r"(?:(?:set|change|make|adjust)\s+(?:the\s+)?speed))"
    r"\s*(?:(?:to|at|=|of|is|be|equals?|up\s+to|down\s+to)\s+)*(?:to\s+)?(?P<num>" + _NUM + r")" + _UNIT, re.I)
_WORD_NUM = {"zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10}


@dataclass
class Task:
    action: str                         # "place" | "stack" | "home" | "belt_start" | "belt_stop" | "belt_speed" | "feed" | "pick_belt"
    cube: Optional[int] = None
    zone: Optional[int] = None          # place / pick_belt: target zone
    onto: Optional[int] = None          # stack: the cube it goes on
    text: str = ""                      # the clause it came from
    value: Optional[float] = None       # belt_speed: cm/s

    def key(self):
        return (self.action, self.cube, self.zone, self.onto)

    def describe(self, names: Optional[Dict[int, str]] = None) -> str:
        n = names or DEFAULT_CUBES
        c = "%s (cube %d)" % (n.get(self.cube, "?"), self.cube) if self.cube else ""
        if self.action == "home":
            return "home"
        if self.action == "belt_start":
            return "start the conveyor"
        if self.action == "belt_stop":
            return "stop the conveyor"
        if self.action == "belt_speed":
            return "set conveyor speed to %g cm/s" % self.value
        if self.action == "feed":
            return "feed %s onto the conveyor" % c
        if self.action == "pick_belt":
            return "pick %s from the conveyor, place in zone %s" % (c, self.zone)
        if self.action == "stack":
            return "stack %s on %s (cube %d)" % (c, n.get(self.onto, "?"), self.onto)
        return "place %s in zone %d" % (c, self.zone)

    def short(self, names: Optional[Dict[int, str]] = None) -> str:
        """File-name friendly, e.g. green_to_zone2 / red_on_green / home."""
        n = names or DEFAULT_CUBES
        if self.action == "home":
            return "home"
        if self.action in ("belt_start", "belt_stop"):
            return self.action.replace("_", "")
        if self.action == "belt_speed":
            return "belt_speed%g" % self.value
        if self.action == "feed":
            return "feed_%s" % n.get(self.cube, "cube%s" % self.cube)
        if self.action == "pick_belt":
            return "%s_belt_to_zone%s" % (n.get(self.cube, "cube%s" % self.cube), self.zone)
        if self.action == "stack":
            return "%s_on_%s" % (n.get(self.cube, "cube%s" % self.cube), n.get(self.onto, "cube%s" % self.onto))
        return "%s_to_zone%d" % (n.get(self.cube, "cube%s" % self.cube), self.zone)


@dataclass
class ParseResult:
    tasks: List[Task] = field(default_factory=list)
    echo: str = ""
    errors: List[str] = field(default_factory=list)
    normalized: str = ""
    warnings: List[str] = field(default_factory=list)
    speeds: Dict[int, float] = field(default_factory=dict)      # speed marker k -> cm/s (internal)

    @property
    def ok(self):
        return bool(self.tasks) and not self.errors


def _speed_cm_s(num: str, unit: Optional[str]) -> float:
    n = num.strip().lower()
    if re.fullmatch(r"\d+(?:[.,]\d+)?", n):
        v = float(n.replace(",", "."))
    else:
        parts = n.split()
        v = float(_WORD_NUM[parts[0]])
        if len(parts) == 3:
            v += _WORD_NUM[parts[2]] / 10.0
    u = (unit or "cm").lower()
    if u.startswith("mm") or u.startswith("milli"):
        return v / 10.0
    if u.startswith("centi") or u == "cm":
        return v
    return v * 100.0                                            # metres


def extract_speed(text: str, res: "ParseResult") -> str:
    """Replace 'set conveyor speed to 4 cm per second' by a marker clause (decimals survive: done before normalize())."""
    out, pos, k = [], 0, 0
    t = (text or "").lower()
    for m in SPEED_RE.finditer(t):
        res.speeds[k] = _speed_cm_s(m.group("num"), m.group("unit"))
        out.append(t[pos:m.start()] + " , zzspeed%d , " % k)
        pos, k = m.end(), k + 1
    out.append(t[pos:])
    return "".join(out)


# --------------------------------------------------------------------------------------- normalisation
def normalize(text: str) -> str:
    """lower case, '&' -> and, punctuation -> clause separators (',' ';' '.'), other symbols dropped."""
    t = (text or "").lower().replace("&", " and ").replace("’", "'")
    t = re.sub(r"(?<=\d)\.(?=\d)", "", t)              # 1.5 -> 15 (never meaningful here)
    t = re.sub(r"[.,;!?\n]+", " , ", t)
    t = re.sub(r"[^a-z0-9,' ]", " ", t)
    return re.sub(r"\s+", " ", t).strip()


def _is_num(tok: str, table=NUM_WORDS) -> Optional[int]:
    return table.get(tok)


Item = Tuple[str, object]       # ("cube", n|None) ("zone", n) ("prep", w) ("verb", w) ("all", None) ("home", None) ("unk", w)


def _tokenize_clause(toks: List[str]) -> List[Item]:
    """tokens -> typed items. Context rules for the speech-to-text mishearings live here."""
    items: List[Item] = []
    i, n = 0, len(toks)
    has_belt = any(t in BELT_WORDS for t in toks)
    while i < n:
        w = toks[i]
        nxt = toks[i + 1] if i + 1 < n else ""
        nn = toks[i + 2] if i + 2 < n else ""
        if w.startswith("zzspeed") and w[7:].isdigit():
            items.append(("speed", int(w[7:])))
            i += 1
            continue
        if w in BELT_WORDS:
            items.append(("belt", None))
            i += 1
            continue
        if has_belt and w in PICK_BELT_VERBS:
            items.append(("verb", "pickbelt"))
            i += 1
            continue
        if has_belt and (w in STOP_WORDS or w in START_WORDS) and w not in ("on", "go", "end"):
            items.append(("onoff", "stop" if w in STOP_WORDS else "start"))
            i += 1
            continue
        if w in FEED_VERBS:
            items.append(("verb", "feed"))
            i += 1
            continue
        # zone word (or mishearing) ONLY counts as "zone" when a number follows
        if w in ZONE_WORDS and w not in ("sun", "soon", "zoom", "joan", "sewn", "loan", "sown") and _is_num(nxt, AFTER_ZONE_NUM) is None:
            items.append(("zone", None))                  # "zone" without a number -> error later
            i += 1
            continue
        if w in ZONE_WORDS and _is_num(nxt, AFTER_ZONE_NUM) is not None:
            items.append(("zone", AFTER_ZONE_NUM[nxt]))
            i += 2
            continue
        if w in ("zone", "zones") and nxt in ("number", "no") and _is_num(nn, AFTER_ZONE_NUM) is not None:
            items.append(("zone", AFTER_ZONE_NUM[nn]))
            i += 3
            continue
        if w in CUBE_WORDS:
            j = i + 1
            if nxt == "number":
                j += 1
            tok = toks[j] if j < n else ""
            nxt2 = toks[j + 1] if j + 1 < n else ""
            num = _is_num(tok)
            if num is None and tok in ("tree", "free"):
                num = 3
            if num is None and tok in ("to", "too", "tu") and (nxt2 in PREPS or nxt2 in ZONE_WORDS and False):
                num = 2                                      # "cube to to zone 1" = cube two to zone 1
            if num is None and tok in ("to", "too") and j > i + 0 and nxt2 == "" and False:
                num = 2
            if num is not None and not (tok in ORDINALS and False):
                items.append(("cube", num))
                i = j + 1
                continue
            if items and items[-1][0] == "unk":              # "yellow cube": a colour / word we do not know
                items[-1] = ("badcube", items[-1][1])
            else:
                items.append(("cube", None))
            i += 1
            continue
        if w in ORDINALS and (nxt in CUBE_WORDS or nxt == "one"):
            items.append(("cube", ORDINALS[w]))
            i += 2
            continue
        if w in COLOR_ALIASES and w in AMBIGUOUS_COLORS and not (
                nxt in CUBE_WORDS or nxt == "one" or i == 0 or toks[i - 1] in CONTEXT_BEFORE_COLOR):
            items.append(("unk", w))                         # "I read that" / "the wind blew": not a colour here
            i += 1
            continue
        if w in COLOR_ALIASES:
            items.append(("cube", COLOR_ALIASES[w]))
            i += 2 if nxt in CUBE_WORDS or nxt == "one" else 1
            continue
        if w in ALL_WORDS:
            items.append(("all", None))
            i += 2 if nxt in CUBE_WORDS | {"of", "the"} and nxt in CUBE_WORDS else 1
            continue
        if w in STACK_VERBS:
            items.append(("verb", "stack"))
        elif w in PLACE_VERBS:
            items.append(("verb", "place"))
        elif w in HOME_WORDS:
            items.append(("home", w))
        elif w in PREPS:
            items.append(("prep", w))
        elif w in ("top",) and items and items[-1] == ("prep", "on"):
            pass                                             # "on top of"
        elif w in FILLERS:
            pass
        elif re.fullmatch(r"[a-z]+", w) or w.isdigit():
            items.append(("unk", w))
        i += 1
    return items


def _cube_list_join(tokens: List[str]) -> List[str]:
    """'red and green' (cube list) must not be split into two clauses: replace that 'and' by 'plus_'."""
    def is_cube_end(k):         # token k ends a cube reference
        t = tokens[k]
        if t in COLOR_ALIASES:
            return True
        if t in CUBE_WORDS:
            return True
        return _is_num(t) is not None and k > 0 and (tokens[k - 1] in CUBE_WORDS or tokens[k - 1] == "number")

    def is_cube_start(k):
        t = tokens[k]
        if t in ("the", "a"):
            return k + 1 < len(tokens) and (tokens[k + 1] in COLOR_ALIASES or tokens[k + 1] in CUBE_WORDS)
        return t in COLOR_ALIASES or t in CUBE_WORDS or t in ORDINALS
    out = list(tokens)
    for k, t in enumerate(tokens):
        if t == "and" and 0 < k < len(tokens) - 1 and is_cube_end(k - 1) and is_cube_start(k + 1):
            # only a list if the clause has no 'on/onto' (stack) relation before it - then it is a new clause
            before = tokens[:k]
            if not any(b in ("on", "onto", "over", "upon") or b in STACK_VERBS for b in before[-6:]) or True:
                out[k] = "plus_"
    return out


def _split_clauses(norm: str) -> List[List[str]]:
    toks = norm.split()
    toks = _cube_list_join(toks)
    clauses, cur = [], []
    for t in toks:
        if t == "," or t in SPLIT_WORDS:
            if cur:
                clauses.append(cur)
            cur = []
        else:
            cur.append(t)
    if cur:
        clauses.append(cur)
    return [[("and" if t == "plus_" else t) for t in c] for c in clauses]


# --------------------------------------------------------------------------------------- parsing
def parse(text: str, cubes: Optional[Dict[int, str]] = None, zones: Sequence[int] = (1, 2, 3),
          in_scene: Optional[Iterable[int]] = None) -> ParseResult:
    """Text -> ordered tasks + echo + errors. cubes {n: name}; zones = valid zone numbers;
    in_scene = cube numbers currently in the scene (None = do not check)."""
    names = dict(cubes or DEFAULT_CUBES)
    scene = None if in_scene is None else set(int(c) for c in in_scene)
    res = ParseResult()
    res.normalized = normalize(extract_speed(text, res))
    if not res.normalized.strip(" ,"):
        res.errors.append("empty command")
        return res
    raw: List[Task] = []
    for clause in _split_clauses(res.normalized):
        text_c = " ".join(clause)
        items = _tokenize_clause(clause)
        _interpret(items, text_c, names, tuple(zones), scene, raw, res)
    for t in raw:                                    # "pick X from the belt" never got a zone
        if t.action == "pick_belt" and t.zone is None:
            res.errors.append("pick from the conveyor needs a target zone (say: ... and place it in zone 1-%d) in '%s'" % (max(zones), t.text))
    raw = [t for t in raw if not (t.action == "pick_belt" and t.zone is None)]
    if not raw and not res.errors:
        res.errors.append("no command found in '%s'" % text.strip())
    # validation against zones / scene
    valid: List[Task] = []
    for t in raw:
        errs = _validate(t, names, tuple(zones), scene)
        if errs:
            res.errors.extend(errs)
        else:
            valid.append(t)
    ordered, cyc = _order(valid)
    if cyc:
        res.errors.append("circular stacking request: " + ", ".join(t.short(names) for t in cyc))
    res.tasks = ordered
    res.echo = "; ".join("%d) %s" % (i + 1, t.describe(names)) for i, t in enumerate(ordered)) or ""
    stack_h = _stack_heights(ordered)
    if stack_h > 3:
        res.errors.append("stack of %d cubes requested; the maximum is 3" % stack_h)
    return res


def _interpret(items: List[Item], text_c: str, names, zones, scene, out: List[Task], res: ParseResult):
    kinds = [k for k, _ in items]
    cubes = [v for k, v in items if k == "cube"]
    zlist = [v for k, v in items if k == "zone"]
    has_all = "all" in kinds
    verbs = [v for k, v in items if k == "verb"]
    preps = [v for k, v in items if k == "prep"]
    unknown = [v for k, v in items if k == "unk"]
    sp = [v for k, v in items if k == "speed"]
    if sp:
        v = res.speeds[sp[0]]
        if not (0.0 <= v <= BELT_MAX_CM_S + 1e-9):
            res.errors.append("conveyor speed %g cm/s is outside 0-%g cm/s" % (v, BELT_MAX_CM_S))
        else:
            out.append(Task("belt_speed", value=v, text=text_c))
        return
    if any(k == "home" for k in kinds) and not cubes and not zlist:
        out.append(Task("home", text=text_c))
        return
    if not items or all(k == "unk" for k in kinds):
        res.errors.append("did not understand '%s'" % text_c)
        return
    bad = [v for k, v in items if k == "badcube"]
    if bad:
        res.errors.append("unknown cube '%s' (valid: %s)" % (bad[0], ", ".join("%s = cube %d" % (v, k) for k, v in sorted(names.items()))))
        return
    if "zone" in kinds and (not zlist or any(z is None for z in zlist)):
        res.errors.append("zone needs a number (1-%d) in '%s'" % (max(zones), text_c))
        return
    if not verbs and not preps and not has_all and not zlist and not cubes and "belt" not in kinds:
        res.errors.append("did not understand '%s'" % text_c)
        return
    if None in cubes and len(cubes) == 1 and not has_all:
        res.errors.append("which cube? say red, green or blue (or cube 1-3) in '%s'" % text_c)
        return
    if None in cubes:
        cubes = [c for c in cubes if c is not None]
    stack_verb = "stack" in verbs
    on_idx = [i for i, (k, v) in enumerate(items) if k == "prep" and v in ("on", "onto", "over", "upon")]
    scene_cubes = sorted(scene) if scene is not None else sorted(names)

    # ---- conveyor: start / stop, feed, pick from belt
    pending = out[-1] if out and out[-1].action == "pick_belt" and out[-1].zone is None else None
    if pending is not None and zlist and (not cubes or cubes == [pending.cube]) and not has_all:
        pending.zone = zlist[-1]                      # "... from the belt and place it in zone 2": the zone clause joins the pick
        return
    belt = "belt" in kinds
    if belt or "feed" in verbs:
        onoff = [v for k, v in items if k == "onoff"]
        pickv = "pickbelt" in verbs
        if has_all and not cubes:
            cubes = list(scene_cubes if scene is not None else sorted(names))
        if not cubes and pickv and zlist and out and out[-1].cube is not None:
            cubes = [out[-1].cube]                    # "... and pick it from the belt": 'it' = the cube of the previous task
        if not cubes:
            if not belt:
                res.errors.append("which cube? say red, green or blue in '%s'" % text_c)
            elif "stop" in onoff:
                out.append(Task("belt_stop", text=text_c))
            elif "start" in onoff or "on" in preps:
                out.append(Task("belt_start", text=text_c))
            else:
                res.errors.append("conveyor: say start / stop / speed / feed / pick in '%s'" % text_c)
            return
        if pickv or (belt and zlist and "feed" not in verbs):
            for c in cubes:
                out.append(Task("pick_belt", c, zone=zlist[-1] if zlist else None, text=text_c))   # zone None: a later clause may add it
            return
        for c in cubes:
            out.append(Task("feed", c, text=text_c))
        return
    # ---- "all ..." forms
    if has_all and not cubes:
        if not zlist:
            res.errors.append("no target zone given in '%s'" % text_c)
            return
        z = zlist[-1]
        if stack_verb:
            seq = scene_cubes
            for i, c in enumerate(seq):
                if i == 0:
                    out.append(Task("place", c, zone=z, text=text_c))
                else:
                    out.append(Task("stack", c, onto=seq[i - 1], text=text_c))
        else:
            for c in scene_cubes:
                out.append(Task("place", c, zone=z, text=text_c))
        return
    # ---- stack A on B
    cube_after_on = None
    if on_idx and len(cubes) >= 2:
        cube_after_on = True
    if (stack_verb or cube_after_on) and len(cubes) >= 2 and not (zlist and not cube_after_on):
        a, b = cubes[0], cubes[1]
        if len(cubes) > 2 and stack_verb:        # "stack red on green on blue" -> chain top-down
            chain = cubes
            for i in range(len(chain) - 1):
                out.append(Task("stack", chain[i], onto=chain[i + 1], text=text_c))
            return
        out.append(Task("stack", a, onto=b, text=text_c))
        return
    # ---- place cube(s) in zone
    if cubes and zlist:
        for c in cubes:
            out.append(Task("place", c, zone=zlist[-1], text=text_c))
        return
    if cubes and not zlist:
        what = "stack onto which cube?" if stack_verb else "no target (zone 1-%d or another cube) given" % max(zones)
        res.errors.append("%s in '%s'" % (what, text_c))
        return
    if zlist and not cubes:
        res.errors.append("which cube should go to zone %s? in '%s'" % (zlist[-1], text_c))
        return
    res.errors.append("did not understand '%s'" % text_c)


def _validate(t: Task, names, zones, scene) -> List[str]:
    errs = []
    for label, c in (("cube", t.cube), ("target cube", t.onto)):
        if c is None:
            continue
        if c not in names:
            errs.append("unknown cube %s (valid: %s)" % (c, ", ".join("%s=%d" % (v, k) for k, v in sorted(names.items()))))
        elif scene is not None and c not in scene and t.action not in ("feed", "pick_belt"):
            errs.append("%s cube (cube %d) is not in the scene" % (names[c], c))
    if t.action in ("place", "pick_belt") and t.zone not in zones:
        errs.append("zone %s does not exist (zones %s)" % (t.zone, ", ".join(str(z) for z in zones)))
    if t.action == "stack" and t.cube == t.onto:
        errs.append("cannot stack %s on itself" % names.get(t.cube, t.cube))
    return errs


def _order(tasks: List[Task]):
    """Stable topological order. Edge i -> j when task i MOVES cube X and task j stacks something ONTO X:
    X must have reached its place before the other cube is put on it. -> (ordered, cyclic leftovers)."""
    n = len(tasks)
    deps = {j: set() for j in range(n)}
    for i, ti in enumerate(tasks):
        if ti.cube is None:
            continue
        for j, tj in enumerate(tasks):
            if i != j and tj.action == "stack" and tj.onto == ti.cube and ti.action in ("place", "stack"):
                # i must come first, unless i itself is a later stack that depends on j (handled by cycle detection)
                deps[j].add(i)
    done, order = set(), []
    while len(order) < n:
        progress = False
        for j in range(n):
            if j in done:
                continue
            if deps[j] <= done:
                order.append(j)
                done.add(j)
                progress = True
                break
        if not progress:
            return [tasks[k] for k in order], [tasks[k] for k in range(n) if k not in done]
    return [tasks[k] for k in order], []


def _stack_heights(tasks: List[Task]) -> int:
    """Longest chain of 'stack X onto Y' links in the command (levels incl. the base cube)."""
    onto = {}
    for t in tasks:
        if t.action == "stack":
            onto[t.cube] = t.onto
    best = 1
    for c in list(onto):
        h, cur, seen = 1, c, set()
        while cur in onto and cur not in seen:
            seen.add(cur)
            cur = onto[cur]
            h += 1
        best = max(best, h)
    return best if onto else 0
