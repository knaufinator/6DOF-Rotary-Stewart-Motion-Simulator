#pragma once
// Task-context bring-up diagnostics. No command in this interface arms motion.
void r14_hardware_start();
bool r14_hardware_command(const char* command, int source);
