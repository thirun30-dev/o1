"""Optimus's spoken replies. Original lines; edit freely."""
import random

LINES = {
    "online": ["Optimus online. All systems operational. Awaiting your orders."],
    "wake": ["I am listening.", "Standing by.", "Give the order."],
    "goto": ["Understood. Setting course for {name}.",
             "Acknowledged. Moving to {name}.",
             "On my way to {name}."],
    "arrived": ["I have reached {name}.", "Destination reached. {name}."],
    "failed": ["I could not reach {name}. The path may be blocked.",
               "Mission to {name} failed. Requesting new orders."],
    "unknown_place": ["I do not know a place called {spoken}. Known locations are: {places}."],
    "home": ["Returning to base."],
    "patrol": ["Beginning autonomous patrol. {n} checkpoints ahead.",
               "Autonomous navigation engaged. {n} checkpoints on the route."],
    "patrol_done": ["Patrol complete. All checkpoints covered."],
    "patrol_loop": ["Patrol loop complete. Starting the next round."],
    "stop": ["Halting all motion.", "Stopping now."],
    "move": ["Moving {dir} {dist} meters."],
    "turn": ["Turning {dir} {deg} degrees."],
    "done": ["Manoeuvre complete."],
    "status_pose": ["I am at x {x}, y {y}, nearest to {near}."],
    "status_none": ["I cannot determine my position. Is localization running?"],
    "battery": ["Battery at {pct} percent, {volt} volts."],
    "battery_none": ["I have no battery data."],
    "list": ["I can go to: {places}."],
    "saved": ["Location saved as {name}."],
    "save_fail": ["I could not save this location. I do not know where I am."],
    "greet": ["I am Optimus, an autonomous mobile robot. Ready to roll out on your command."],
    "not_ready": ["Navigation is not ready. Please start Nav2."],
    "unknown": ["I did not understand that order.", "Please repeat the command."],
}


def line(key, **kw):
    return random.choice(LINES[key]).format(**kw)
