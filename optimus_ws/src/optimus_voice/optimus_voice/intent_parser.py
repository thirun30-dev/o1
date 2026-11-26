"""Wake-word detection and command parsing for Optimus.

Pure Python (no ROS imports) so it can be unit-tested anywhere.
"""
import difflib
import re
from dataclasses import dataclass, field

# Spellings speech recognisers commonly produce for "Optimus".
# "optimal" is deliberately absent: it is an everyday word ("the optimal
# route"), so it woke the robot during ordinary conversation.
WAKE_WORDS = (
    "optimus", "optimas", "optimist", "optimums", "optimum",
    "opti mus", "up to miss", "obtain us",
)

_NUM_WORDS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
    "eleven": 11, "twelve": 12, "fifteen": 15, "twenty": 20,
    "thirty": 30, "forty": 40, "forty five": 45, "fifty": 50,
    "sixty": 60, "ninety": 90, "hundred": 100, "one eighty": 180,
    "one hundred eighty": 180, "three sixty": 360,
}


@dataclass
class Intent:
    kind: str                    # stop, goto, home, patrol, move, turn, status,
                                 # battery, list, save, greet, unknown
    args: dict = field(default_factory=dict)
    text: str = ""


def normalise(text: str) -> str:
    text = text.lower().strip()
    text = re.sub(r"[^a-z0-9.\s]", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def words_to_numbers(text: str) -> str:
    """'one point five' -> '1.5', 'half a' -> '0.5', 'ninety' -> '90'."""
    text = re.sub(r"\bone and a half\b", "1.5", text)
    text = re.sub(r"\b(a )?half( a)?\b", "0.5", text)
    # longest phrases first so 'one eighty' wins over 'one'
    for w in sorted(_NUM_WORDS, key=len, reverse=True):
        text = re.sub(rf"\b{w}\b", str(_NUM_WORDS[w]), text)
    text = re.sub(r"\b(\d+) point (\d+)\b", r"\1.\2", text)
    return text


def split_wake_word(text: str):
    """Return (woke, command_after_wake_word)."""
    t = normalise(text)
    best = None
    for w in WAKE_WORDS:
        m = re.search(rf"\b{w}\b", t)
        if m and (best is None or m.start() < best.start()):
            best = m
    if best is None:
        return False, ""
    rest = t[best.end():].strip(" ,.")
    rest = re.sub(r"^(please|can you|could you|now)\s+", "", rest)
    return True, rest


def _num(s, default):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


class IntentParser:
    def __init__(self, waypoints: dict):
        # waypoints: {name: {"aliases": [...], ...}}
        self.set_waypoints(waypoints)

    def set_waypoints(self, waypoints: dict):
        self.lookup = {}
        for name, wp in (waypoints or {}).items():
            spoken = [name.replace("_", " ")] + list(wp.get("aliases", []))
            for s in spoken:
                self.lookup[words_to_numbers(normalise(s))] = name

    def match_waypoint(self, phrase: str):
        phrase = words_to_numbers(normalise(phrase))
        phrase = re.sub(r"^(the|a|our|my)\s+", "", phrase).strip()
        if not phrase:
            return None
        if phrase in self.lookup:
            return self.lookup[phrase]
        close = difflib.get_close_matches(phrase, self.lookup.keys(), n=1, cutoff=0.6)
        return self.lookup[close[0]] if close else None

    def parse(self, text: str) -> Intent:
        raw = text
        t = words_to_numbers(normalise(text))
        if not t:
            return Intent("unknown", text=raw)

        if re.search(r"\b(stop|halt|freeze|cancel|abort|emergency|wait)\b", t):
            return Intent("stop", text=raw)

        if re.search(r"\b(battery|charge level|power level)\b", t):
            return Intent("battery", text=raw)

        m = re.search(r"\b(save|remember|mark|store)\b.*?\bas\b (.+)$", t)
        if m:
            name = re.sub(r"^(the|a)\s+", "", m.group(2)).strip().replace(" ", "_")
            return Intent("save", {"name": name}, raw)

        if re.search(r"\b(where are you|your (location|position)|status|report)\b", t):
            return Intent("status", text=raw)

        if re.search(r"\b(list|which|what)\b.*\b(places|points|waypoints|locations|go)\b", t):
            return Intent("list", text=raw)

        if re.search(r"\b(hello|hi|hey|who are you|introduce)\b", t):
            return Intent("greet", text=raw)

        if re.search(r"\b(auto ?(matic)? ?nav(igation)?|autonomous|patrol|tour|round)\b", t):
            loop = bool(re.search(r"\b(loop|forever|continuous(ly)?|keep)\b", t))
            return Intent("patrol", {"loop": loop}, raw)

        if re.search(r"\b(go|return|head|come|get)\b.*\b(home|dock|base|charging station|charger)\b", t):
            return Intent("home", text=raw)

        m = re.search(
            r"\b(move|go|drive|roll)\s+(forward|ahead|straight|back(ward)?s?|reverse)"
            r"(\s+(by\s+)?(?P<n>\d+(\.\d+)?)\s*(?P<u>meters?|metres?|m|centimeters?|centimetres?|cm)?)?", t)
        if m:
            dist = _num(m.group("n"), 0.5)
            if m.group("u") and m.group("u").startswith(("c",)):
                dist /= 100.0
            if m.group(2).startswith(("back", "rev")):
                dist = -dist
            return Intent("move", {"distance": dist}, raw)

        m = re.search(r"\b(turn|rotate|spin)\s+(?P<d>left|right|around)"
                      r"(\s+(by\s+)?(?P<n>\d+(\.\d+)?)\s*(degrees?)?)?", t)
        if m:
            d = m.group("d")
            deg = 180.0 if d == "around" else _num(m.group("n"), 90.0)
            if d == "right":
                deg = -deg
            return Intent("turn", {"degrees": deg}, raw)

        m = re.search(r"\b(go|move|navigate|drive|head|proceed|take me|travel)\b\s*(over\s+)?(to|towards|into)\s+(?P<p>.+)$", t)
        if m:
            name = self.match_waypoint(m.group("p"))
            return Intent("goto", {"name": name, "spoken": m.group("p")}, raw)

        # Bare place name: "Optimus, kitchen"
        name = self.match_waypoint(t)
        if name:
            return Intent("goto", {"name": name, "spoken": t}, raw)

        return Intent("unknown", text=raw)
