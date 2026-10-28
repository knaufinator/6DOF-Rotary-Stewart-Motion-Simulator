"""Static checks for E-stop paths that cannot be run without an ESP32-S3 board."""

from pathlib import Path
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "main" / "main.cpp").read_text(encoding="utf-8")


def between(start: str, end: str) -> str:
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


class EStopSourceContract(unittest.TestCase):
    def test_boot_starts_paused_and_stops_new_engine(self):
        self.assertIn("isPausedEStop{true}", SOURCE)
        setup = between("void setupMotorPins() {\n    // PCBv2", "StepDriver_report();")
        self.assertIn("if (estop_motion_is_blocked())", setup)
        self.assertIn("StepDriver_stop();", setup)

    def test_release_does_not_restart_engine(self):
        release = between("// Releasing the switch never restarts motion.", "prev_state = current_state;")
        self.assertNotIn("StepDriver_resume", release)
        self.assertNotIn("GPTimerScheduler::start", release)

    def test_reset_checks_both_inputs_and_discards_targets_before_resume(self):
        reset = between('if (strcmp(data, "ESTOP:RESET") == 0) {', 'if (strcmp(data, "ESTOP?") == 0) {')
        self.assertIn("estop_reset_allowed(", reset)
        self.assertIn("gpio_get_level((gpio_num_t)ESTOP_PIN)", reset)
        self.assertIn("estop_button.debounced_state", reset)
        self.assertLess(reset.index("motor_emergencyStop(i)"), reset.index("StepDriver_resume()"))
        self.assertIn("if (estop_input_active())", reset)

    def test_motion_diagnostics_are_gated(self):
        dispatch = between("void process_data(char * data) {", "// ── DBG:1")
        for command in ("RATETEST", "FREQTEST:", "PINTEST", "SIGTEST:",
                        "MTEST", "ESTOP:SOFT", "ZERO"):
            self.assertIn(command, dispatch)
        self.assertIn("!estop_motion_is_blocked()", SOURCE)
        self.assertIn("while (esp_timer_get_time() < end_time && !estop_motion_is_blocked())", SOURCE)


if __name__ == "__main__":
    unittest.main()
