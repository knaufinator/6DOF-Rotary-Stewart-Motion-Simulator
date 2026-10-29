"""Static contract for the R14 ESP32-S3 lockout until physical IO is qualified."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "main" / "main.cpp").read_text(encoding="utf-8")
CMAKE = (ROOT / "main" / "CMakeLists.txt").read_text(encoding="utf-8")
SAFETY = (ROOT / "include" / "R14RuntimeSafety.h").read_text(encoding="utf-8")


class R14FailClosedSourceContract(unittest.TestCase):
    def test_s3_r14_build_defaults_to_lockout(self):
        self.assertIn('option(R14_FAIL_CLOSED "Keep ESP32-S3 STEP outputs disarmed for R14 bringup" ON)', CMAKE)
        self.assertIn("R14_FAIL_CLOSED=1", CMAKE)
        self.assertIn("static r14::MotionAdmission<false> r14MotionAdmission(1, 1, 1);", MAIN)
        self.assertNotIn("MotionAdmission<true>", MAIN)

    def test_boot_never_starts_step_timer_in_locked_build(self):
        setup = MAIN.split("void setupMotorPins() {\n    // PCBv2", 1)[1]
        locked = setup.split("#if defined(R14_FAIL_CLOSED)", 1)[1].split("#else", 1)[0]
        self.assertIn("gpio_set_level(stepPins[i], 0)", locked)
        self.assertNotIn("StepDriver_start()", locked)
        self.assertIn("if (!StepDriver_start())", setup.split("#else", 1)[1])

    def test_alarm_ingress_latches_and_stops_without_servo_off(self):
        callback = MAIN.split('extern "C" void r14_report_normalized_drive_alarm(int axis) {', 1)[1].split("\n}", 1)[0]
        self.assertIn("r14::report_alarm_and_stop(r14MotionAdmission, r14DriveAlarms, axis, R14FaultStopOps{})", callback)
        for required in ("alarms.assert_alarm(axis)", "admission.report_alarm(axis)",
                         'finish_fault_stop(ops, "DRIVE_ALARM")'):
            self.assertIn(required, SAFETY)
        ops = MAIN.split("struct R14FaultStopOps {", 1)[1].split("\n};", 1)[0]
        for required in ("isPausedEStop.store(true)", "gpio_set_level(GPIO_NUM_47, 1)",
                         "gpio_set_level(GPIO_NUM_48, 0)", "StepDriver_isInitialized()",
                         "StepDriver_stop()", "StepDriver_emergencyStop()"):
            self.assertIn(required, ops)
        self.assertNotIn("SRV_ON", callback)
        self.assertNotIn("servo_disable", callback)

    def test_local_requests_and_reconnect_are_task_ingress_only(self):
        self.assertIn('extern "C" bool r14_request_local_arm(uint64_t now_us)', MAIN)
        self.assertIn('extern "C" bool r14_request_local_home(uint64_t now_us)', MAIN)
        self.assertIn("r14MotionAdmission.request_operator_arm(now_us)", MAIN)
        self.assertIn("r14MotionAdmission.request_operator_home(now_us)", MAIN)
        reconnect = MAIN.split('extern "C" void r14_report_transport_reconnect(void) {', 1)[1].split("\n}", 1)[0]
        self.assertIn("r14::report_reconnect_and_stop(r14MotionAdmission, R14FaultStopOps{})", reconnect)
        self.assertIn("admission.report_reconnect_or_reset()", SAFETY)
        self.assertIn('finish_fault_stop(ops, "TRANSPORT_RECONNECT")', SAFETY)
        parser = MAIN.split("void process_data(char * data) {", 1)[1]
        self.assertNotIn("r14_request_local_arm(", parser)
        self.assertNotIn("r14_request_local_home(", parser)

    def test_serial_reset_and_motion_diagnostics_cannot_open_locked_gate(self):
        reset = MAIN.split('if (strcmp(data, "ESTOP:RESET") == 0) {', 1)[1]
        locked = reset.split("#if defined(R14_FAIL_CLOSED) && PCB_VERSION == 2", 1)[1].split("#endif", 1)[0]
        self.assertIn("reset denied", locked)
        self.assertIn("return;", locked)
        self.assertNotIn("StepDriver_resume()", locked)
        blocked = MAIN.split("static bool estop_motion_is_blocked() {", 1)[1].split("\n}", 1)[0]
        self.assertIn("r14::admission_blocks_motion(r14MotionAdmission, r14DriveAlarms)", blocked)
        self.assertIn("!admission.normal_motion_permitted() || alarms.faulted()", SAFETY)
        dispatch = MAIN.split("void process_data(char * data) {", 1)[1].split("// ── DBG:1", 1)[0]
        for command in ("FREQTEST:", "PINTEST", "SIGTEST:", "MTEST", "ESTOP:SOFT", "ZERO"):
            self.assertIn(command, dispatch)
        self.assertIn("estop_motion_is_blocked()", dispatch)

    def test_live_motor_and_decoder_bindings_use_tested_seams(self):
        setters = [line for line in MAIN.splitlines() if "bool    motor_setTarget(" in line]
        self.assertEqual(len(setters), 2)
        for setter in setters:
            self.assertIn("controller::guarded_motor_target(estop_motion_is_blocked()", setter)
        decoder = MAIN.split("void process_binary_packet(const uint8_t *payload, int len) {", 1)[1].split("\n}", 1)[0]
        self.assertIn("controller::decode_motion_payload(payload, len, raw)", decoder)
        self.assertLess(decoder.index("decode_motion_payload"), decoder.index("mapRawToPosition"))
        self.assertIn("applyMotionValues(mapped)", decoder)


if __name__ == "__main__":
    unittest.main()
