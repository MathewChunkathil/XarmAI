"""Free-form command understanding (NO PyBullet).  SIMULATION ONLY.

    eng = IntentEngine(load_config())
    p = eng.understand("tidy everything into the second area", ctx, from_voice=True)
    p.tasks, p.question, p.source, p.errors, p.needs_confirm

Safety design (IMPLEMENTATION_PLAN D3 / 9.1): an LLM is only a *proposal generator*.
  * it can only name whitelisted actions (the same ones as commands.Task); everything else is rejected;
  * its output is converted to commands.Task and runs through commands.finalize() = the SAME cube / zone / scene /
    stack-height / ordering checks as the rule parser;
  * nothing is executed here: the app runs the tasks only after the user confirms (button or "yes");
  * the rule parser goes first and is the fallback (Ollama offline -> fuzzy word correction -> hint).
Provider interface: IntentProvider.interpret(text, ctx) -> (list of task dicts, say, unsure).
"""
import difflib
import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from sim import commands as cm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(ROOT, "config", "ai.yaml")
ACTIONS = ("place", "stack", "home", "belt_start", "belt_stop", "belt_speed", "feed", "pick_belt")
DEFAULTS = {"provider": "ollama", "model": "qwen2.5:3b", "url": "http://localhost:11434", "timeout": 30.0,
            "confirm_voice": True, "confirm_text_rules": False}


def load_config(path: str = CONFIG_PATH) -> dict:
    cfg = dict(DEFAULTS)
    try:
        txt = open(path, encoding="utf-8").read()
    except OSError:
        return cfg
    try:
        import yaml
        data = yaml.safe_load(txt) or {}
    except Exception:                       # tiny reader: "key: value  # comment"
        data = {}
        for ln in txt.splitlines():
            ln = ln.split(" #")[0].strip()
            if ":" in ln and not ln.startswith("#"):
                k, v = ln.split(":", 1)
                data[k.strip()] = v.strip()
    for k, v in data.items():
        if k not in DEFAULTS:
            continue
        d = DEFAULTS[k]
        if isinstance(d, bool):
            v = v if isinstance(v, bool) else str(v).strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(d, float):
            v = float(v)
        cfg[k] = v
    return cfg


@dataclass
class Context:
    names: Dict[int, str]
    zones: Tuple[int, ...] = (1, 2, 3)
    in_scene: Optional[set] = None
    belt_running: bool = False
    belt_speed_cm_s: float = 3.0


@dataclass
class Proposal:
    tasks: List[cm.Task] = field(default_factory=list)
    echo: str = ""
    question: str = ""                      # "Did you mean: ...?"  (empty when no confirmation is needed)
    source: str = "none"                    # rules | ollama | fuzzy | none
    errors: List[str] = field(default_factory=list)
    needs_confirm: bool = False
    say: str = ""                           # chat answer when there are no tasks ("what can you do")
    raw: Optional[dict] = None              # what the provider returned (log / debug)
    offline: bool = False

    @property
    def ok(self):
        return bool(self.tasks) and not self.errors


class ProviderError(Exception):
    """Provider unreachable / bad reply: the engine falls back to fuzzy matching."""


class IntentProvider:
    def interpret(self, text: str, ctx: Context) -> Tuple[List[dict], str, bool]:
        raise NotImplementedError

    def status(self) -> Tuple[bool, str]:
        return False, "no provider"


# --------------------------------------------------------------------------------------- Ollama
SCHEMA = {
    "type": "object",
    "properties": {
        "tasks": {"type": "array", "items": {
            "type": "object",
            "properties": {"action": {"type": "string", "enum": list(ACTIONS)},
                           "cube": {"type": ["integer", "null"]}, "zone": {"type": ["integer", "null"]},
                           "onto": {"type": ["integer", "null"]}, "value": {"type": ["number", "null"]}},
            "required": ["action"]}},
        "say": {"type": "string"},
        "unsure": {"type": "boolean"}},
    "required": ["tasks", "say", "unsure"]}

_EXAMPLES = """Examples (cubes 1=red 2=green 3=blue, zones 1-3, all three cubes in scene):
"move the green one over to the second zone" -> {"tasks":[{"action":"place","cube":2,"zone":2}],"say":"Move the green cube to zone 2?","unsure":false}
"tidy everything into the second area" -> {"tasks":[{"action":"place","cube":1,"zone":2},{"action":"place","cube":2,"zone":2},{"action":"place","cube":3,"zone":2}],"say":"Put all cubes in zone 2?","unsure":false}
"put blue on top of red" -> {"tasks":[{"action":"stack","cube":3,"onto":1}],"say":"Stack the blue cube on the red cube?","unsure":false}
"make a tower" -> {"tasks":[{"action":"stack","cube":2,"onto":1},{"action":"stack","cube":3,"onto":2}],"say":"Build a tower: green on red, then blue on green?","unsure":false}
"get the red one off the belt and drop it in 3" -> {"tasks":[{"action":"pick_belt","cube":1,"zone":3}],"say":"Catch the red cube from the conveyor and place it in zone 3?","unsure":false}
"put the red cube on the belt and catch it into zone 2" -> {"tasks":[{"action":"feed","cube":1},{"action":"pick_belt","cube":1,"zone":2}],"say":"Feed red onto the belt and catch it into zone 2?","unsure":false}
"speed the belt up a bit" (belt currently 3 cm/s) -> {"tasks":[{"action":"belt_speed","value":4.5}],"say":"Set belt speed to 4.5 cm/s?","unsure":false}
"turn the conveyor on" -> {"tasks":[{"action":"belt_start"}],"say":"Start the conveyor?","unsure":false}
"go back to rest" -> {"tasks":[{"action":"home"}],"say":"Move the arm home?","unsure":false}
"what can you do" -> {"tasks":[],"say":"I can move cubes to zones 1-3, stack them, run the conveyor and catch cubes from it.","unsure":false}
"blah banana" -> {"tasks":[],"say":"Sorry, I did not get that.","unsure":true}"""


def build_system_prompt(ctx: Context) -> str:
    names = ", ".join("%d=%s" % (k, v) for k, v in sorted(ctx.names.items()))
    scene = sorted(ctx.in_scene) if ctx.in_scene is not None else sorted(ctx.names)
    return (
        "You translate a spoken or typed command for a small robot arm (simulation) into a JSON list of tasks. "
        "You only PROPOSE tasks; a human confirms them before anything moves. Reply with JSON only.\n"
        "Allowed actions (nothing else exists):\n"
        "- place: cube -> zone (move one cube to a zone)\n"
        "- stack: cube, onto (put cube on top of cube 'onto'; max tower height 3)\n"
        "- home: move the arm to its rest pose\n"
        "- belt_start / belt_stop: conveyor on / off\n"
        "- belt_speed: value in cm/s, between 0 and %g\n"
        "- feed: cube (put that cube on the start of the conveyor)\n"
        "- pick_belt: cube, zone (catch that cube from the moving conveyor and place it in the zone)\n"
        "Cubes (use the NUMBER): %s. Valid zones: %s (zone/area/spot number words: first=1, second=2, third=3).\n"
        "Cubes currently in the scene: %s. Conveyor: %s at %.1f cm/s.\n"
        "Task order matters: they run in the order given. Use 'say' for a short confirmation question describing what "
        "you will do, or a short answer when there is nothing to do. If the request is not about this robot or cannot be "
        "mapped to the allowed actions, return no tasks and unsure=true. Never invent cubes or zones.\n"
        % (cm.BELT_MAX_CM_S, names, ", ".join(str(z) for z in ctx.zones), ", ".join(str(c) for c in scene),
           "running" if ctx.belt_running else "stopped", ctx.belt_speed_cm_s)) + _EXAMPLES


class OllamaProvider(IntentProvider):
    def __init__(self, model="qwen2.5:3b", url="http://localhost:11434", timeout=30.0):
        self.model, self.url, self.timeout = model, url.rstrip("/"), float(timeout)

    def _req(self, path, body=None, timeout=None):
        data = None if body is None else json.dumps(body).encode("utf-8")
        rq = urllib.request.Request(self.url + path, data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(rq, timeout=timeout or self.timeout) as r:
            return json.loads(r.read().decode("utf-8"))

    def status(self) -> Tuple[bool, str]:
        try:
            tags = self._req("/api/tags", timeout=2.0)
        except Exception:
            return False, "Ollama offline - rules only"
        have = [m.get("name", "") for m in tags.get("models", [])]
        if any(n == self.model or n.split(":")[0] == self.model for n in have):
            return True, "Brain: Ollama %s ready" % self.model
        return False, "Ollama up, model %s missing - rules only" % self.model

    def warm(self):
        """Load the model into memory (first call can take >10 s); errors ignored."""
        try:
            self._req("/api/chat", {"model": self.model, "stream": False, "keep_alive": "30m", "options": {"num_predict": 1},
                                    "messages": [{"role": "user", "content": "hi"}]}, timeout=90)
        except Exception:
            pass

    def interpret(self, text, ctx):
        body = {"model": self.model, "stream": False, "format": SCHEMA, "keep_alive": "30m",
                "options": {"temperature": 0, "num_predict": 400},
                "messages": [{"role": "system", "content": build_system_prompt(ctx)},
                             {"role": "user", "content": text}]}
        try:
            out = self._req("/api/chat", body)
        except (urllib.error.URLError, OSError, ValueError) as e:      # HTTPError (model missing) is a URLError
            raise ProviderError("ollama: %s" % e)
        try:
            d = json.loads(out["message"]["content"])
        except Exception:
            raise ProviderError("ollama: reply is not JSON")
        return _unpack(d)


def _unpack(d) -> Tuple[List[dict], str, bool]:
    if not isinstance(d, dict):
        raise ProviderError("reply is not an object")
    tasks = d.get("tasks") or []
    if not isinstance(tasks, list):
        raise ProviderError("tasks is not a list")
    return [t for t in tasks], str(d.get("say") or "").strip(), bool(d.get("unsure"))


# --------------------------------------------------------------------------------------- dict -> Task (+ validation)
def _as_cube(v, ctx: Context) -> Optional[int]:
    if v is None or isinstance(v, bool):
        return None
    inv = {n.lower(): k for k, n in ctx.names.items()}
    if isinstance(v, str):
        s = v.strip().lower()
        return inv.get(s) if s in inv else (int(s) if s.isdigit() else None)
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _as_int(v) -> Optional[int]:
    try:
        return None if v is None or isinstance(v, bool) else int(v)
    except (TypeError, ValueError):
        return None


def tasks_from_dicts(items: List[dict], ctx: Context, text: str = "") -> cm.ParseResult:
    """Whitelist check + SAME validation / ordering as the rule parser (commands.finalize)."""
    res = cm.ParseResult()
    raw: List[cm.Task] = []
    for d in items:
        if not isinstance(d, dict):
            res.errors.append("bad task entry %r" % (d,))
            continue
        a = str(d.get("action", "")).strip().lower()
        if a not in ACTIONS:
            res.errors.append("unknown action %r (allowed: %s)" % (d.get("action"), ", ".join(ACTIONS)))
            continue
        cube, zone, onto = _as_cube(d.get("cube"), ctx), _as_int(d.get("zone")), _as_cube(d.get("onto"), ctx)
        t = cm.Task(a, text=text)
        if a == "place":
            if cube is None or zone is None:
                res.errors.append("place needs a cube and a zone")
                continue
            t.cube, t.zone = cube, zone
        elif a == "stack":
            if cube is None or onto is None:
                res.errors.append("stack needs a cube and a target cube")
                continue
            t.cube, t.onto = cube, onto
        elif a == "feed":
            if cube is None:
                res.errors.append("feed needs a cube")
                continue
            t.cube = cube
        elif a == "pick_belt":
            if cube is None or zone is None:
                res.errors.append("pick from the conveyor needs a cube and a zone")
                continue
            t.cube, t.zone = cube, zone
        elif a == "belt_speed":
            try:
                v = float(d.get("value"))
            except (TypeError, ValueError):
                res.errors.append("belt_speed needs a value in cm/s")
                continue
            if not (0.0 <= v <= cm.BELT_MAX_CM_S + 1e-9):
                res.errors.append("conveyor speed %g cm/s is outside 0-%g cm/s" % (v, cm.BELT_MAX_CM_S))
                continue
            t.value = v
        raw.append(t)
    return cm.finalize(raw, ctx.names, tuple(ctx.zones), ctx.in_scene, res)


# --------------------------------------------------------------------------------------- fuzzy fallback
def _vocab() -> List[str]:
    v = set()
    for s in (cm.NUM_WORDS, cm.ZONE_WORDS, cm.CUBE_WORDS, cm.COLOR_ALIASES, cm.PLACE_VERBS, cm.STACK_VERBS, cm.HOME_WORDS,
              cm.PREPS, cm.ALL_WORDS, cm.FILLERS, cm.SPLIT_WORDS, cm.BELT_WORDS, cm.FEED_VERBS, cm.PICK_BELT_VERBS,
              cm.STOP_WORDS, cm.START_WORDS):
        v.update(w for w in s if w.isalpha())
    v.update(("speed", "centimetres", "per", "second", "from", "conveyor", "zone", "cube"))
    return sorted(v)


_VOCAB = _vocab()
_VOCAB_SET = set(_VOCAB)


def fuzzy_correct(text: str) -> str:
    out = []
    for tok in cm.normalize(text).split():
        if tok in _VOCAB_SET or not tok.isalpha() or len(tok) < 4:
            out.append(tok)
            continue
        m = difflib.get_close_matches(tok, _VOCAB, n=1, cutoff=0.8)
        out.append(m[0] if m else tok)
    return " ".join(out)


# --------------------------------------------------------------------------------------- yes / no
_YES = {"yes", "yeah", "yep", "yup", "yea", "sure", "ok", "okay", "k", "confirm", "confirmed", "correct", "right",
        "affirmative", "proceed", "execute", "absolutely", "definitely", "certainly", "do", "go", "ahead"}
_NO = {"no", "nope", "nah", "cancel", "stop", "wrong", "don't", "dont", "abort", "negative", "never", "nevermind",
       "incorrect", "not", "wait"}
_NEUTRAL = {"it", "that", "that's", "thats", "is", "please", "thanks", "thank", "you", "the", "so", "now", "just", "this",
            "one", "mind", "again", "i", "of", "course", "make", "sounds", "good", "great", "perfect", "exactly", "fine",
            "lets", "let's", "a", "all"}


def classify_reply(text: str) -> str:
    """'yes' | 'no' | 'other'. Only short replies made of yes/no/neutral words count; anything else is a new command
    (so "stop the conveyor" is NOT a cancel)."""
    t = (text or "").lower().replace("’", "'")
    toks = [w.strip("'") for w in re.findall(r"[a-z']+", t)]
    toks = [w for w in toks if w]
    if not toks or len(toks) > 7:
        return "other"
    if not all(w in _YES or w in _NO or w in _NEUTRAL for w in toks):
        return "other"
    if any(w in _NO for w in toks):
        return "no"
    if any(w in _YES for w in toks):
        return "yes"
    return "other"


# --------------------------------------------------------------------------------------- engine
def spoken(tasks, names) -> str:
    return "; ".join(t.describe(names) for t in tasks)


class IntentEngine:
    def __init__(self, cfg: Optional[dict] = None, provider: Optional[IntentProvider] = None):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        if provider is None and str(self.cfg["provider"]).lower() == "ollama":
            provider = OllamaProvider(self.cfg["model"], self.cfg["url"], self.cfg["timeout"])
        self.provider = provider

    def status(self) -> Tuple[bool, str]:
        if self.provider is None:
            return False, "No LLM brain - rules only"
        return self.provider.status()

    def warm(self):
        if hasattr(self.provider, "warm"):
            self.provider.warm()

    @staticmethod
    def _confirm_q(tasks, names) -> str:
        return "Did you mean: %s?" % spoken(tasks, names)

    def understand(self, text: str, ctx: Context, from_voice: bool = False) -> Proposal:
        names, zones, scene = ctx.names, tuple(ctx.zones), ctx.in_scene
        # 1) rule parser
        r = cm.parse(text, cubes=names, zones=zones, in_scene=scene)
        if r.ok:
            need = (from_voice and bool(self.cfg["confirm_voice"])) or (not from_voice and bool(self.cfg["confirm_text_rules"]))
            return Proposal(r.tasks, r.echo, self._confirm_q(r.tasks, names) if need else "", "rules", [], need)
        rule_errors = list(r.errors)
        # 2) LLM proposal
        offline = False
        if self.provider is not None:
            try:
                items, say, unsure = self.provider.interpret(text, ctx)
                p = self._from_provider(items, say, unsure, text, ctx)
                if p is not None:
                    return p
            except ProviderError:
                offline = True
        # 3) fuzzy word correction with the rule parser
        fixed = fuzzy_correct(text)
        if fixed != cm.normalize(text):
            r2 = cm.parse(fixed, cubes=names, zones=zones, in_scene=scene)
            if r2.ok:
                return Proposal(r2.tasks, r2.echo, self._confirm_q(r2.tasks, names), "fuzzy", [], True, offline=offline)
        hint = ("Try e.g. 'put the green cube in zone 2', 'stack red on blue', 'start the conveyor'."
                if offline or self.provider is None else "")
        return Proposal(errors=rule_errors + ([hint] if hint else []), source="none", offline=offline)

    def _from_provider(self, items, say, unsure, text, ctx) -> Optional[Proposal]:
        raw = {"tasks": items, "say": say, "unsure": unsure}
        if not items:
            if say:
                return Proposal(source="ollama", say=say, raw=raw)
            return None
        res = tasks_from_dicts(items, ctx, text)
        if res.errors:
            return Proposal(errors=["AI proposal rejected: " + "; ".join(res.errors)], source="ollama", raw=raw)
        return Proposal(res.tasks, res.echo, self._confirm_q(res.tasks, ctx.names), "ollama", [], True, say=say, raw=raw)
