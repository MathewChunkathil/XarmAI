"""Conveyor phrasings of the rule-based command parser. Run: python -m pytest tests/unit/test_commands_conveyor.py"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim.commands import parse  # noqa: E402

START = ("belt_start", None, None, None)
STOP = ("belt_stop", None, None, None)


def key(t):
    return (t.action, t.cube, t.zone, t.onto)


def speed(v):
    return ("belt_speed", None, None, None, v)


def full(t):
    return key(t) + ((t.value,) if t.action == "belt_speed" else ())


def FEED(c):
    return ("feed", c, None, None)


def PICK(c, z):
    return ("pick_belt", c, z, None)


OK = [
    # start / stop
    ("start the conveyor", [START]),
    ("Start the belt.", [START]),
    ("turn on the conveyor", [START]),
    ("turn the belt on", [START]),
    ("run the conveyor", [START]),
    ("stop the conveyor", [STOP]),
    ("stop the belt", [STOP]),
    ("turn off the belt", [STOP]),
    ("switch off the conveyor", [STOP]),
    ("turn the conveyor off", [STOP]),
    # speed
    ("set conveyor speed to 4 cm per second", [speed(4.0)]),
    ("set the conveyor speed to 4 centimeters per second", [speed(4.0)]),
    ("set belt speed to 2.5 cm/s", [speed(2.5)]),
    ("conveyor speed 3", [speed(3.0)]),
    ("set the conveyor speed to four", [speed(4.0)]),
    ("set conveyor speed to two point five", [speed(2.5)]),
    ("set the speed of the belt to 40 mm per second", [speed(4.0)]),
    ("set conveyor speed to 0.05 meters per second", [speed(5.0)]),
    ("set conveyor speed to 0", [speed(0.0)]),
    # feed
    ("feed the red cube", [FEED(1)]),
    ("feed the red cube on the conveyor", [FEED(1)]),
    ("feed the red cube onto the conveyor", [FEED(1)]),
    ("feed the green cube onto the belt", [FEED(2)]),
    ("feed cube 3 on the conveyor", [FEED(3)]),
    ("put the blue cube on the belt", [FEED(3)]),
    ("place the red cube on the conveyor", [FEED(1)]),
    # pick from belt
    ("pick the green cube from the conveyor and place it in zone 2", [PICK(2, 2)]),
    ("pick the green cube from the belt and put it in zone 2", [PICK(2, 2)]),
    ("pick green from the conveyor in zone 2", [PICK(2, 2)]),
    ("catch the blue cube on the belt and put it in zone 3", [PICK(3, 3)]),
    ("catch the blue cube on the conveyor in zone three", [PICK(3, 3)]),
    ("grab the red cube from the belt to zone 1", [PICK(1, 1)]),
    ("take the red cube from the conveyor to zone 1", [PICK(1, 1)]),
    ("pick the red cube from the belt and place the red cube in zone 3", [PICK(1, 3)]),
    ("pick the green cube from the conveyor, then place it in zone 1", [PICK(2, 1)]),
    # sequences (voice style)
    ("feed the red cube and pick the red cube from the belt and place it in zone 2", [FEED(1), PICK(1, 2)]),
    ("start the conveyor, set conveyor speed to 4 cm/s, feed the red cube and pick it from the belt in zone 2",
     [START, speed(4.0), FEED(1), PICK(1, 2)]),
    ("start the belt and feed the blue cube", [START, FEED(3)]),
]


@pytest.mark.parametrize("text,want", OK)
def test_conveyor_ok(text, want):
    r = parse(text, in_scene=[1])
    assert not r.errors, (text, r.errors)
    got = [full(t) for t in r.tasks]
    assert got == [tuple(x) for x in want], (text, got)


ERR = [
    ("pick the red cube from the belt", "zone"),                      # no zone
    ("set conveyor speed to 9 cm/s", "outside"),                      # above the 6 cm/s maximum
    ("feed", "which cube"),
    ("conveyor", "say start"),
    ("feed the yellow cube", "unknown cube"),
    ("pick the red cube from the belt in zone 7", "zone 7"),
]


@pytest.mark.parametrize("text,frag", ERR)
def test_conveyor_errors(text, frag):
    r = parse(text, in_scene=[1])
    assert r.errors and any(frag in e for e in r.errors), (text, r.errors)


def test_feed_and_pick_do_not_need_the_cube_in_the_scene():
    r = parse("feed the green cube and pick the green cube from the belt in zone 1", in_scene=[1])     # green is not in the scene
    assert not r.errors and [t.action for t in r.tasks] == ["feed", "pick_belt"]


def test_static_commands_unchanged_by_conveyor_words():
    r = parse("place the red cube in zone 1")
    assert [key(t) for t in r.tasks] == [("place", 1, 1, None)]
    r = parse("stack red on green")
    assert [key(t) for t in r.tasks] == [("stack", 1, None, 2)]
    r = parse("home")
    assert [t.action for t in r.tasks] == ["home"]


def test_describe_and_short():
    r = parse("feed the red cube and pick the red cube from the belt and place it in zone 2")
    assert [t.short() for t in r.tasks] == ["feed_red", "red_belt_to_zone2"]
    assert "zone 2" in r.tasks[1].describe()
