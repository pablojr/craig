"""Unit tests for the gesture classifier — no ROS required."""

import sys
import os

# Allow running directly from this directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from s2s_bridge.gesture_classifier import classify, classify_with_confidence


class TestClassify:
    # --- Greetings ---
    def test_hello_gives_wave_right(self):
        assert classify("Hello! How are you today?") == "wave_right"

    def test_hi_gives_wave_right(self):
        assert classify("Hi there!") == "wave_right"

    def test_good_morning_gives_wave_right(self):
        assert classify("Good morning! Nice to meet you.") == "wave_right"

    # --- Farewells ---
    def test_goodbye_gives_wave_left(self):
        assert classify("Goodbye! See you next time.") == "wave_left"

    def test_see_you_gives_wave_left(self):
        assert classify("Take care and see you soon.") == "wave_left"

    # --- Agreement ---
    def test_yes_gives_nod(self):
        assert classify("Yes, absolutely! That is correct.") == "nod_yes"

    def test_certainly_gives_nod(self):
        assert classify("Certainly, I agree with that point.") == "nod_yes"

    def test_of_course_gives_nod(self):
        assert classify("Of course! I'd be happy to help.") == "nod_yes"

    # --- Directional ---
    def test_look_left(self):
        assert classify("The exit is to the left.") == "look_left"

    def test_look_right(self):
        assert classify("You can find that on the right side.") == "look_right"

    # --- Offering ---
    def test_here_you_go_raises_right(self):
        assert classify("Here you go, I hope this helps!") == "raise_right_hand"

    def test_wait_raises_right(self):
        assert classify("Wait, let me think about that.") == "raise_right_hand"

    # --- Uncertainty ---
    def test_dont_know_opens_hands(self):
        assert classify("I don't know, it's really hard to say.") == "hands_open"

    def test_not_sure_opens_hands(self):
        assert classify("I'm not sure about that.") == "hands_open"

    # --- Empty / short text ---
    def test_empty_gives_neutral(self):
        assert classify("") == "neutral"

    def test_whitespace_gives_neutral(self):
        assert classify("   ") == "neutral"

    def test_short_gives_neutral_or_match(self):
        # Very short text — may or may not match; just check it doesn't crash
        result = classify("OK")
        assert isinstance(result, str)

    # --- Available gestures filter ---
    def test_filter_unavailable_gesture(self):
        # wave_right not available → should fall back to something else or neutral
        available = frozenset({"neutral", "nod_yes", "wave_left"})
        result = classify("Hello!", available_gestures=available)
        assert result in available

    # --- With confidence ---
    def test_confidence_returns_tuple(self):
        gesture, matched = classify_with_confidence("Hello, good morning!")
        assert isinstance(gesture, str)
        assert isinstance(matched, str)
        assert gesture == "wave_right"

    def test_confidence_empty(self):
        gesture, matched = classify_with_confidence("")
        assert gesture == "neutral"
        assert matched == "<empty>"


if __name__ == "__main__":
    import pytest
    pytest.main([__file__, "-v"])
