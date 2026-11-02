#pragma once

// One application entry/command router. Exactly one hardware runtime is linked.
void controller_profile_run();
void controller_profile_command(char* command, int source);
void controller_command_dispatch(const char* command, int source);
