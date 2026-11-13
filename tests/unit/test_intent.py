"""Intent engine: fake provider (no network) + one live Ollama test (skipped when the model is not reachable)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from sim import intent as it  # noqa: E402

NAMES = {1: "red", 2: "green", 3: "blue"}


def ctx(**k):
    return it.Context(NAMES, (1, 2, 3), k.pop("in_scene", {1, 2, 3}), **k)


class Fake(it.IntentProvider):
    def __init__(self, items=None, say="", unsure=False, fail=False):
        self.items, self.say, self.unsure, self.fail, self.calls = items or [], say, unsure, fail, 0

    def interpret(self, text, c):
        self.calls += 1
        if self.fail:
            raise it.ProviderError("offline")
        return self.items, self.say, self.unsure


def eng(p):
    return it.IntentEngine({"provider": "none"}, provider=p)


def test_rules_first_no_llm_call():
    f = Fake()
    p = eng(f).understand("put the green cube in zone 2", ctx())
    assert p.ok and p.source == "rules" and not p.needs_confirm and f.calls == 0


def test_voice_always_confirms_rules():
    p = eng(Fake()).understand("put the green cube in zone 2", ctx(), from_voice=True)
    assert p.ok and p.source == "rules" and p.needs_confirm and p.question.startswith("Did you mean:")


def test_vague_text_becomes_proposal():
    f = Fake([{"action": "place", "cube": 1, "zone": 2}, {"action": "place", "cube": 2, "zone": 2}, {"action": "place", "cube": 3, "zone": 2}])
    p = eng(f).understand("tidy everything into the second area", ctx())
    assert p.ok and p.source == "ollama" and p.needs_confirm and f.calls == 1
    assert [(t.action, t.cube, t.zone) for t in p.tasks] == [("place", 1, 2), ("place", 2, 2), ("place", 3, 2)]
    assert "zone 2" in p.question


def test_stack_order_validated():
    f = Fake([{"action": "stack", "cube": 3, "onto": 2}, {"action": "stack", "cube": 2, "onto": 1}])
    p = eng(f).understand("make a tower", ctx())
    assert p.ok and [t.cube for t in p.tasks] == [2, 3]       # dependency order (green on red first)


def test_belt_speed_and_pick():
    p = eng(Fake([{"action": "belt_speed", "value": 4.5}])).understand("speed the belt up a bit", ctx())
    assert p.ok and p.tasks[0].value == 4.5
    p = eng(Fake([{"action": "pick_belt", "cube": 1, "zone": 3}])).understand("get the red one off the belt into 3", ctx())
    assert p.ok and p.tasks[0].action == "pick_belt" and p.tasks[0].zone == 3


@pytest.mark.parametrize("items", [
    [{"action": "place", "cube": 1, "zone": 9}],                                   # bad zone
    [{"action": "place", "cube": 7, "zone": 1}],                                   # bad cube
    [{"action": "open_gripper"}],                                                  # not whitelisted
    [{"action": "shell", "cmd": "rm -rf /"}],
    [{"action": "belt_speed", "value": 50}],                                       # out of range
    [{"action": "place", "cube": 1}],                                              # missing zone
    [{"action": "stack", "cube": 1, "onto": 1}],                                   # on itself
    [{"action": "stack", "cube": 2, "onto": 1}, {"action": "stack", "cube": 3, "onto": 2},
     {"action": "stack", "cube": 4, "onto": 3}],                                   # tower > 3 (and cube 4 unknown)
    ["place the cube"],
])
def test_invalid_llm_output_rejected(items):
    p = eng(Fake(items)).understand("do something odd", ctx())
    assert not p.tasks and p.errors and not p.ok


def test_stack_height_over_3_rejected():
    names = {1: "a", 2: "b", 3: "c", 4: "d"}
    c = it.Context(names, (1, 2, 3), {1, 2, 3, 4})
    items = [{"action": "stack", "cube": 2, "onto": 1}, {"action": "stack", "cube": 3, "onto": 2}, {"action": "stack", "cube": 4, "onto": 3}]
    res = it.tasks_from_dicts(items, c)
    assert any("maximum is 3" in e for e in res.errors)


def test_cube_not_in_scene_rejected():
    p = eng(Fake([{"action": "place", "cube": 3, "zone": 1}])).understand("shuffle it", ctx(in_scene={1}))
    assert not p.tasks and p.errors


def test_no_tasks_say_is_answer():
    p = eng(Fake([], say="I can move cubes.")).understand("what can you do", ctx())
    assert not p.tasks and p.say and not p.errors and not p.needs_confirm


def test_ollama_offline_fuzzy_fallback():
    p = eng(Fake(fail=True)).understand("pleace the grean cubee into zone twoo", ctx())
    assert p.ok and p.source == "fuzzy" and p.needs_confirm and p.tasks[0].cube == 2 and p.tasks[0].zone == 2


def test_offline_unparseable_gives_hint():
    p = eng(Fake(fail=True)).understand("sing me a song", ctx())
    assert not p.tasks and p.errors and p.offline and any("Try e.g." in e for e in p.errors)


def test_no_provider_rules_only():
    p = it.IntentEngine({"provider": "none"}).understand("sing me a song", ctx())
    assert not p.ok and p.errors


@pytest.mark.parametrize("text,exp", [
    ("Yes.", "yes"), ("yeah, do it", "yes"), ("Yep!", "yes"), ("Okay", "yes"), ("OK, go ahead", "yes"), ("Sure.", "yes"),
    ("That's correct", "yes"), ("right", "yes"), ("Affirmative", "yes"), ("confirm", "yes"), ("yes please", "yes"),
    ("No.", "no"), ("nope", "no"), ("Cancel!", "no"), ("stop", "no"), ("that's wrong", "no"), ("don't", "no"),
    ("abort", "no"), ("no, cancel that", "no"), ("not right", "no"),
    ("put the red cube in zone 1", "other"), ("stop the conveyor", "other"), ("", "other"),
    ("yes but make it zone 3", "other"), ("no wait put it in zone 3 instead", "other"),
])
def test_classify_reply(text, exp):
    assert it.classify_reply(text) == exp


def test_config_loads():
    c = it.load_config()
    assert c["model"] and c["url"].startswith("http") and isinstance(c["confirm_voice"], bool)


# ------------------------------------------------------------------------------ live model (optional)
def _live():
    cfg = it.load_config()
    ok, _ = it.OllamaProvider(cfg["model"], cfg["url"], cfg["timeout"]).status()
    return ok


@pytest.mark.skipif(not _live(), reason="Ollama / model not reachable")
@pytest.mark.parametrize("text,check", [
    ("could you move the green one over to the second zone", lambda t: [(x.action, x.cube, x.zone) for x in t] == [("place", 2, 2)]),
    ("tidy everything into the second area", lambda t: sorted((x.cube, x.zone) for x in t if x.action == "place") == [(1, 2), (2, 2), (3, 2)]),
    ("get the red one off the belt and drop it in 3", lambda t: [(x.action, x.cube, x.zone) for x in t] == [("pick_belt", 1, 3)]),
    ("speed the belt up a bit", lambda t: len(t) == 1 and t[0].action == "belt_speed" and 3.0 < t[0].value <= 6.0),
])
def test_live_ollama(text, check):
    cfg = it.load_config()
    p = it.IntentEngine(cfg).understand(text, ctx(belt_running=True, belt_speed_cm_s=3.0), from_voice=True)
    print(text, "->", p.source, p.echo, p.raw)
    assert p.ok, (p.errors, p.raw)
    assert check(p.tasks), (p.echo, p.raw)
