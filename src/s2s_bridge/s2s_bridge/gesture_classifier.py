"""Rule-based gesture classifier for Craig (InMoov virtual robot).

Maps an LLM response text to a single gesture name from the InMoov
motion manager gesture catalog.  No external dependencies — pure stdlib.

Gesture catalog (must match gestures.yaml):
  neutral, raise_left_hand, raise_right_hand,
  wave_left, wave_right, nod_yes, look_left, look_right,
  jaw_open, jaw_close, hands_open, hands_closed

The classifier is intentionally simple and rule-transparent so that
changing Craig's personality is just editing keyword lists.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Sequence


# ---------------------------------------------------------------------------
# Rule table
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GestureRule:
    """A prioritised keyword rule.  Higher priority wins."""

    gesture: str
    keywords: Sequence[str]
    priority: int = 0  # higher = more specific / wins ties


# Rules are evaluated in priority-descending order; first match wins.
_RULES: list[GestureRule] = [

    # --- Greetings & farewells (high priority: short utterances) -----------
    GestureRule("wave_right", [
        r"\bhello\b", r"\bhi\b", r"\bhey\b",
        r"\bgood morning\b", r"\bgood afternoon\b", r"\bgood evening\b",
        r"\bgreetings?\b", r"\bwelcome\b",
    ], priority=10),

    GestureRule("wave_left", [
        r"\bgoodbye\b", r"\bbye\b", r"\bsee you\b", r"\bfarewell\b",
        r"\btake care\b", r"\buntil next time\b",
    ], priority=10),

    # --- Agreement / confirmation ------------------------------------------
    GestureRule("nod_yes", [
        r"\byes\b", r"\babsolutely\b", r"\bcertainly\b", r"\bof course\b",
        r"\bexactly\b", r"\bindeed\b", r"\bcorrect\b", r"\bright\b",
        r"\bagree\b", r"\bsure\b", r"\baffirmative\b",
        r"\bgreat idea\b", r"\bgood point\b",
        r"\bI think so\b", r"\bI believe so\b",
    ], priority=5),

    # --- Pointing / directing attention ------------------------------------
    GestureRule("look_left", [
        r"\bto the left\b", r"\bon the left\b", r"\bleft side\b",
        r"\bover there\b.*\bleft\b", r"\blook left\b",
    ], priority=7),

    GestureRule("look_right", [
        r"\bto the right\b", r"\bon the right\b", r"\bright side\b",
        r"\bover there\b.*\bright\b", r"\blook right\b",
    ], priority=7),

    # --- Offering / presenting (raise arm) --------------------------------
    GestureRule("raise_right_hand", [
        r"\bhere you go\b", r"\ballow me\b", r"\blet me show\b",
        r"\bI('d| would) like to present\b",
        r"\blisten\b", r"\bpay attention\b", r"\bone moment\b",
        r"\bjust a moment\b", r"\bwait\b",
    ], priority=6),

    GestureRule("raise_left_hand", [
        r"\bon the other hand\b", r"\balternatively\b",
        r"\bor perhaps\b", r"\banother option\b",
    ], priority=4),

    # --- Positive emotional expressions -----------------------------------
    GestureRule("hands_open", [
        r"\bI don'?t know\b", r"\bI('m| am) not sure\b",
        r"\bwho knows\b", r"\bhard to say\b", r"\buncertain\b",
        r"\bopen question\b", r"\bopen to\b",
        r"\bwonderful\b", r"\bamazing\b", r"\bfantastic\b",
    ], priority=6),  # higher than nod_yes so "not sure" beats bare "sure"

    GestureRule("hands_closed", [
        r"\bimportant\b", r"\bkey point\b", r"\bremember\b",
        r"\bcrucial\b", r"\bessential\b", r"\bfundamental\b",
        r"\bno\b", r"\bnot\b", r"\bdefinitely not\b",
    ], priority=2),

    # --- Default fallback (lowest priority) --------------------------------
    GestureRule("neutral", [
        r".*",  # matches everything — used as explicit fallback
    ], priority=-1),
]

# Pre-compile all regexes once
_COMPILED_RULES: list[tuple[GestureRule, list[re.Pattern[str]]]] = [
    (rule, [re.compile(kw, re.IGNORECASE | re.DOTALL) for kw in rule.keywords])
    for rule in sorted(_RULES, key=lambda r: -r.priority)
]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def classify(text: str, available_gestures: frozenset[str] | None = None) -> str:
    """Return a gesture name for the given LLM response *text*.

    Args:
        text: Full or partial LLM response string.
        available_gestures: Optional set of gesture names known to the motion
            manager.  If provided, gestures not in this set are skipped.
            Defaults to None (no filtering).

    Returns:
        A gesture name string.  Falls back to ``"neutral"`` if nothing matches.
    """
    if not text or not text.strip():
        return "neutral"

    for rule, patterns in _COMPILED_RULES:
        if available_gestures is not None and rule.gesture not in available_gestures:
            continue
        if any(pat.search(text) for pat in patterns):
            return rule.gesture

    return "neutral"


def classify_with_confidence(text: str) -> tuple[str, str]:
    """Return (gesture_name, matched_pattern) for debugging/logging."""
    if not text or not text.strip():
        return "neutral", "<empty>"

    for rule, patterns in _COMPILED_RULES:
        for pat in patterns:
            m = pat.search(text)
            if m:
                return rule.gesture, m.group(0)

    return "neutral", "<fallback>"
