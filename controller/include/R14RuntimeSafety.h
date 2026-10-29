#pragma once

// Task-context safety composition, shared by main.cpp and the host runner.
// Ops supplies the existing ESP-IDF/StepDriver bindings in firmware. No method
// here starts/resumes a timer, clears a fault, or grants motion permission.
namespace r14 {
template<class Admission, class Alarms>
bool admission_blocks_motion(const Admission& admission, const Alarms& alarms) {
    return !admission.normal_motion_permitted() || alarms.faulted();
}

template<class Ops>
void finish_fault_stop(Ops ops, const char* reason) {
    ops.pause();
    ops.bench_stop(reason);
    ops.inhibit();
    if (ops.initialized()) {
        ops.stop();
        ops.discard_targets();
    }
}

template<class Admission, class Alarms, class Ops>
void report_alarm_and_stop(Admission& admission, Alarms& alarms, int axis, Ops ops) {
    alarms.assert_alarm(axis);
    admission.report_alarm(axis);
    finish_fault_stop(ops, "DRIVE_ALARM");
}

template<class Admission, class Ops>
void report_reconnect_and_stop(Admission& admission, Ops ops) {
    admission.report_reconnect_or_reset();
    finish_fault_stop(ops, "TRANSPORT_RECONNECT");
}
} // namespace r14
