import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "optimus_voice"))
from intent_parser import IntentParser, split_wake_word  # noqa: E402

WPS = {
    "home": {"aliases": ["base", "dock"]},
    "loading_bay": {"aliases": ["loading area", "point 1"]},
    "storage_rack": {"aliases": ["rack", "point 2"]},
    "packing_station": {"aliases": ["packing"]},
}
P = IntentParser(WPS)


def cmd(text):
    woke, rest = split_wake_word(text)
    assert woke, text
    return P.parse(rest)


def test_wake():
    assert split_wake_word("hey optimus go to the rack") == (True, "go to the rack")
    assert split_wake_word("optimist stop") == (True, "stop")
    assert split_wake_word("go to the rack")[0] is False
    assert split_wake_word("Optimus") == (True, "")
    # Ordinary speech must not wake the robot.
    assert split_wake_word("that is the optimal route")[0] is False


def test_goto():
    i = cmd("Optimus, go to the loading bay")
    assert (i.kind, i.args["name"]) == ("goto", "loading_bay")
    assert cmd("optimus navigate to point one").args["name"] == "loading_bay"
    assert cmd("optimus move to storage rak").args["name"] == "storage_rack"
    assert cmd("optimus packing").args["name"] == "packing_station"
    i = cmd("optimus go to the moon")
    assert i.kind == "goto" and i.args["name"] is None


def test_patrol_home_stop():
    assert cmd("optimus start auto navigation").kind == "patrol"
    assert cmd("optimus start autonomous navigation").kind == "patrol"
    assert cmd("optimus begin patrol in a loop").args["loop"] is True
    assert cmd("optimus go back home").kind == "home"
    assert cmd("optimus return to the dock").kind == "home"
    assert cmd("optimus stop patrol").kind == "stop"
    assert cmd("optimus emergency stop").kind == "stop"


def test_move_turn():
    assert cmd("optimus move forward two meters").args["distance"] == 2.0
    assert cmd("optimus go back 50 centimeters").args["distance"] == -0.5
    assert cmd("optimus move forward one point five meters").args["distance"] == 1.5
    assert cmd("optimus move forward").args["distance"] == 0.5
    assert cmd("optimus turn left").args["degrees"] == 90
    assert cmd("optimus turn right forty five degrees").args["degrees"] == -45
    assert cmd("optimus turn around").args["degrees"] == 180


def test_misc():
    assert cmd("optimus where are you").kind == "status"
    assert cmd("optimus battery status").kind == "battery"
    assert cmd("optimus what places can you go to").kind == "list"
    assert cmd("optimus save this location as charging corner").args["name"] == "charging_corner"
    assert cmd("optimus who are you").kind == "greet"
    assert cmd("optimus make me a sandwich").kind == "unknown"
