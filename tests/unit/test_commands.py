"""Rule-based command parser tests (speech-to-text style phrasings). Run: python -m pytest tests/unit/test_commands.py"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim.commands import parse, normalize  # noqa: E402

P = lambda c, z: ("place", c, z, None)        # noqa: E731
S = lambda c, o: ("stack", c, None, o)        # noqa: E731
HOME = ("home", None, None, None)

OK = [
    ("place the green cube in zone 1", [P(2, 1)]),
    ("Place the green cube in zone 1.", [P(2, 1)]),
    ("put the red cube into zone 3", [P(1, 3)]),
    ("move the blue cube to zone 2", [P(3, 2)]),
    ("drop the green cube on zone 1", [P(2, 1)]),
    ("bring the blue cube at zone three", [P(3, 3)]),
    ("cube 2 to zone 2", [P(2, 2)]),
    ("cube two to zone 2", [P(2, 2)]),
    ("cube to to zone 3", None),                       # no cube id: error (see ERR)
    ("move cube number 3 to zone 1", [P(3, 1)]),
    ("put the first cube in zone one", [P(1, 1)]),
    ("send the third cube to zone two", [P(3, 2)]),
    ("put the green cube in zoan 2", [P(2, 2)]),       # speech mishearings of "zone" before a number
    ("put the green cube in zon 2", [P(2, 2)]),
    ("put the green cube in soon 2", [P(2, 2)]),
    ("put the green cube in sun 3", [P(2, 3)]),
    ("put the red cube in zone to", [P(1, 2)]),        # "zone to" = zone 2 (only after a zone word)
    ("stack red on green", [S(1, 2)]),
    ("stack the red cube on top of the green cube", [S(1, 2)]),
    ("stack red onto blue", [S(1, 3)]),
    ("put blue on red", [S(3, 1)]),
    ("put the blue cube on top of the red cube", [S(3, 1)]),
    ("place the green cube in zone 1 and stack red on green", [P(2, 1), S(1, 2)]),
    ("place the green cube in zone 1, then stack red on green.", [P(2, 1), S(1, 2)]),
    ("stack red on green and place green in zone 1", [P(2, 1), S(1, 2)]),   # dependency re-ordering
    ("stack red on green, stack green on blue", [S(2, 3), S(1, 2)]),        # green must move first
    ("all cubes to zone 3", [P(1, 3), P(2, 3), P(3, 3)]),
    ("move everything to zone 2", [P(1, 2), P(2, 2), P(3, 2)]),
    ("stack all in zone 2", [P(1, 2), S(2, 1), S(3, 2)]),
    ("put the red and green cube in zone 2", [P(1, 2), P(2, 2)]),
    ("home", [HOME]),
    ("Reset!", [HOME]),
    ("go home", [HOME]),
    ("put red in zone 1, green in zone 2 and blue in zone 3", [P(1, 1), P(2, 2), P(3, 3)]),
    ("PLACE THE RED CUBE IN ZONE 2", [P(1, 2)]),
    ("place red cube in zone 2 then home", [P(1, 2), HOME]),
]
ERR = [
    ("place the yellow cube in zone 1", "unknown cube 'yellow'"),
    ("place cube 4 in zone 1", "unknown cube 4"),
    ("place the red cube in zone 5", "zone 5 does not exist"),
    ("place the red cube in zone", "zone needs a number"),
    ("stack blue on blue", "cannot stack"),
    ("stack red", "stack onto which cube"),
    ("move the red cube", "no target"),
    ("cube to zone 2", "which cube"),
    ("the weather is nice today", "did not understand"),
    ("", "empty command"),
    ("put the red cube in the sun", "no target"),
]


@pytest.mark.parametrize("text,expect", [t for t in OK if t[1] is not None])
def test_phrasings(text, expect):
    r = parse(text)
    assert not r.errors, (text, r.errors)
    assert [t.key() for t in r.tasks] == expect, (text, r.normalized, r.echo)
    assert r.echo and r.ok


@pytest.mark.parametrize("text,msg", ERR)
def test_errors(text, msg):
    r = parse(text)
    assert not r.ok and any(msg in e for e in r.errors), (text, r.errors)


def test_cube_two_vs_to():
    # "to" before a zone is a preposition, "two" right after "cube" is the number
    assert parse("put cube two to zone 1").tasks[0].cube == 2
    assert parse("put cube to zone 1").errors
    assert parse("cube two to zone 3").tasks[0].key() == P(2, 3)


def test_zone_words_only_before_number():
    r = parse("put the red cube in the sun")        # 'sun' without a number is NOT zone
    assert r.errors
    r = parse("put the red cube in sun two")
    assert r.tasks[0].key() == P(1, 2)


def test_scene_validation():
    r = parse("stack red on green", in_scene=[1, 3])
    assert any("green cube (cube 2) is not in the scene" in e for e in r.errors)
    r = parse("all cubes to zone 1", in_scene=[1, 3])
    assert [t.cube for t in r.tasks] == [1, 3]
    r = parse("stack all in zone 2", in_scene=[2, 3])
    assert [t.key() for t in r.tasks] == [P(2, 2), S(3, 2)]


def test_zone_range_configurable():
    assert parse("place red in zone 4", zones=(1, 2, 3, 4)).tasks[0].zone == 4
    assert parse("place red in zone 4").errors


def test_circular_stacking_error():
    r = parse("stack red on green and stack green on red")
    assert any("circular" in e for e in r.errors)


def test_stack_too_high():
    r = parse("stack red on green, stack green on blue, stack blue on red")
    assert r.errors            # circular or too high: never silently accepted


def test_echo_and_names():
    r = parse("place the green cube in zone 1 and stack red on green")
    assert r.echo == "1) place green (cube 2) in zone 1; 2) stack red (cube 1) on green (cube 2)"
    assert r.tasks[1].short() == "red_on_green" and r.tasks[0].short() == "green_to_zone1"


def test_custom_names():
    r = parse("place cube 1 in zone 1", cubes={1: "cyan"})
    assert r.tasks[0].short({1: "cyan"}) == "cyan_to_zone1"


def test_normalize():
    assert normalize("Place THE Green-cube, in Zone 1!") == "place the green cube , in zone 1 ,"


# ---- whisper-style outputs (voice input): mishearings, capitalisation, trailing punctuation
WHISPER_OK = [
    ("Zone 1.", None),                                            # fragment: no cube -> error, never a crash
    ("Place the red cube in Zone 1.", [P(1, 1)]),
    ("Put the Greene cube in zone two.", [P(2, 2)]),
    ("Move the read cube to zone three", [P(1, 3)]),             # read -> red (after a verb + "the")
    ("Put the blew cube in zone 2.", [P(3, 2)]),                  # blew -> blue in cube context
    ("Move cube to to zone one", [P(2, 1)]),                      # cube two to zone 1
    ("Put cube free in zone 2.", [P(3, 2)]),                      # cube three
    ("Move cube tree to zone one", [P(3, 1)]),
    ("Place cube won in zone too", [P(1, 2)]),                    # cube 1, zone 2
    ("stack read on grin", [S(1, 2)]),                            # ambiguous words but verb / clause start context
    ("Stack the blue cube on top of the read cube.", [S(3, 1)]),
    ("Move all cubes to Zone 3.", [P(1, 3), P(2, 3), P(3, 3)]),
    ("Place the green cube in zone one, and stack red on green.", [P(2, 1), S(1, 2)]),
    ("Go home.", [HOME]),
]


@pytest.mark.parametrize("text,tasks", [c for c in WHISPER_OK if c[1] is not None])
def test_whisper_outputs(text, tasks):
    r = parse(text)
    assert r.ok, (text, r.errors)
    assert [t.key() for t in r.tasks] == tasks


def test_whisper_fragment_is_error():
    assert parse("Zone 1.").errors


def test_ambiguous_colour_words_need_context():
    r = parse("I read the manual")
    assert not r.ok and not r.tasks
    assert parse("the wind blew").errors
    assert parse("blue cube to zone 2").ok
