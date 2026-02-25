/*
 * crash_handler.h — Windows SEH crash logger
 *
 * Catches unhandled exceptions and access violations.
 * Writes a .log file and a .dmp minidump next to the executable.
 * Shows a message box with the crash log path before exiting.
 *
 * Usage: call InstallCrashHandler() at the top of main().
 */
#pragma once
#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <dbghelp.h>
#include <shlobj.h>
#include <cstdio>
#include <ctime>
#include <string>

#pragma comment(lib, "dbghelp.lib")

// ── Helpers ──────────────────────────────────────────────────────────

static std::string CrashGetExeDir() {
    char buf[MAX_PATH] = {};
    GetModuleFileNameA(NULL, buf, MAX_PATH);
    std::string path(buf);
    auto pos = path.rfind('\\');
    return pos != std::string::npos ? path.substr(0, pos) : ".";
}

static std::string CrashTimestamp() {
    time_t t = time(nullptr);
    struct tm lt;
    localtime_s(&lt, &t);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &lt);
    return buf;
}

static const char* CrashCodeName(DWORD code) {
    switch (code) {
        case EXCEPTION_ACCESS_VIOLATION:         return "ACCESS_VIOLATION";
        case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:    return "ARRAY_BOUNDS_EXCEEDED";
        case EXCEPTION_BREAKPOINT:               return "BREAKPOINT";
        case EXCEPTION_DATATYPE_MISALIGNMENT:    return "DATATYPE_MISALIGNMENT";
        case EXCEPTION_FLT_DIVIDE_BY_ZERO:       return "FLT_DIVIDE_BY_ZERO";
        case EXCEPTION_FLT_OVERFLOW:             return "FLT_OVERFLOW";
        case EXCEPTION_ILLEGAL_INSTRUCTION:      return "ILLEGAL_INSTRUCTION";
        case EXCEPTION_INT_DIVIDE_BY_ZERO:       return "INT_DIVIDE_BY_ZERO";
        case EXCEPTION_INT_OVERFLOW:             return "INT_OVERFLOW";
        case EXCEPTION_PRIV_INSTRUCTION:         return "PRIV_INSTRUCTION";
        case EXCEPTION_STACK_OVERFLOW:           return "STACK_OVERFLOW";
        default:                                 return "UNKNOWN";
    }
}

// ── Stack walker ─────────────────────────────────────────────────────

static void CrashWriteStack(FILE* f, CONTEXT* ctx) {
    HANDLE proc = GetCurrentProcess();
    HANDLE thread = GetCurrentThread();

    SymInitialize(proc, NULL, TRUE);
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME);

    STACKFRAME64 frame = {};
    frame.AddrPC.Mode    = AddrModeFlat;
    frame.AddrFrame.Mode = AddrModeFlat;
    frame.AddrStack.Mode = AddrModeFlat;

#ifdef _M_X64
    DWORD machine = IMAGE_FILE_MACHINE_AMD64;
    frame.AddrPC.Offset    = ctx->Rip;
    frame.AddrFrame.Offset = ctx->Rbp;
    frame.AddrStack.Offset = ctx->Rsp;
#else
    DWORD machine = IMAGE_FILE_MACHINE_I386;
    frame.AddrPC.Offset    = ctx->Eip;
    frame.AddrFrame.Offset = ctx->Ebp;
    frame.AddrStack.Offset = ctx->Esp;
#endif

    char sym_buf[sizeof(SYMBOL_INFO) + MAX_SYM_NAME * sizeof(TCHAR)];
    SYMBOL_INFO* sym = (SYMBOL_INFO*)sym_buf;
    sym->SizeOfStruct = sizeof(SYMBOL_INFO);
    sym->MaxNameLen   = MAX_SYM_NAME;

    IMAGEHLP_LINE64 line = {};
    line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);

    fprintf(f, "\nStack trace:\n");
    for (int depth = 0; depth < 64; depth++) {
        if (!StackWalk64(machine, proc, thread, &frame, ctx,
                         NULL, SymFunctionTableAccess64, SymGetModuleBase64, NULL))
            break;
        if (frame.AddrPC.Offset == 0) break;

        DWORD64 disp_sym = 0;
        DWORD   disp_line = 0;
        bool got_sym  = SymFromAddr(proc, frame.AddrPC.Offset, &disp_sym, sym) == TRUE;
        bool got_line = SymGetLineFromAddr64(proc, frame.AddrPC.Offset, &disp_line, &line) == TRUE;

        if (got_sym && got_line) {
            fprintf(f, "  #%-2d 0x%016llX  %s  (%s:%lu)\n",
                depth, (unsigned long long)frame.AddrPC.Offset,
                sym->Name, line.FileName, line.LineNumber);
        } else if (got_sym) {
            fprintf(f, "  #%-2d 0x%016llX  %s\n",
                depth, (unsigned long long)frame.AddrPC.Offset, sym->Name);
        } else {
            fprintf(f, "  #%-2d 0x%016llX  <unknown>\n",
                depth, (unsigned long long)frame.AddrPC.Offset);
        }
    }

    SymCleanup(proc);
}

// ── Minidump writer ──────────────────────────────────────────────────

static void CrashWriteMinidump(const std::string& path, EXCEPTION_POINTERS* ep) {
    HANDLE f = CreateFileA(path.c_str(), GENERIC_WRITE, 0, NULL,
                           CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (f == INVALID_HANDLE_VALUE) return;

    MINIDUMP_EXCEPTION_INFORMATION mei = {};
    mei.ThreadId          = GetCurrentThreadId();
    mei.ExceptionPointers = ep;
    mei.ClientPointers    = FALSE;

    MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), f,
        (MINIDUMP_TYPE)(MiniDumpWithDataSegs | MiniDumpWithFullMemoryInfo |
                        MiniDumpWithThreadInfo | MiniDumpWithUnloadedModules),
        &mei, NULL, NULL);
    CloseHandle(f);
}

// ── SEH filter ───────────────────────────────────────────────────────

static LONG WINAPI CrashHandler(EXCEPTION_POINTERS* ep) {
    std::string dir  = CrashGetExeDir();
    std::string ts   = CrashTimestamp();
    std::string log_path = dir + "\\crash_" + ts + ".log";
    std::string dmp_path = dir + "\\crash_" + ts + ".dmp";

    DWORD code = ep->ExceptionRecord->ExceptionCode;

    // Write text log
    FILE* f = nullptr;
    fopen_s(&f, log_path.c_str(), "w");
    if (f) {
        fprintf(f, "Stewart Platform — CRASH REPORT\n");
        fprintf(f, "================================\n");
        fprintf(f, "Time:      %s\n", ts.c_str());
        fprintf(f, "Exception: 0x%08lX (%s)\n", code, CrashCodeName(code));
        fprintf(f, "Address:   0x%016llX\n",
            (unsigned long long)ep->ExceptionRecord->ExceptionAddress);

        if (code == EXCEPTION_ACCESS_VIOLATION && ep->ExceptionRecord->NumberParameters >= 2) {
            const char* op = ep->ExceptionRecord->ExceptionInformation[0] == 1 ? "write" : "read";
            fprintf(f, "AV:        %s at 0x%016llX\n", op,
                (unsigned long long)ep->ExceptionRecord->ExceptionInformation[1]);
        }

        CrashWriteStack(f, ep->ContextRecord);

        fprintf(f, "\nMinidump:  %s\n", dmp_path.c_str());
        fclose(f);
    }

    // Write minidump
    CrashWriteMinidump(dmp_path, ep);

    // Show message box
    char msg[512];
    snprintf(msg, sizeof(msg),
        "Stewart Platform crashed!\n\n"
        "Exception: 0x%08lX (%s)\n\n"
        "Crash log:  %s\n"
        "Minidump:   %s\n\n"
        "Please send these files for debugging.",
        code, CrashCodeName(code),
        log_path.c_str(), dmp_path.c_str());

    MessageBoxA(NULL, msg, "Stewart Platform — Crash", MB_OK | MB_ICONERROR | MB_SYSTEMMODAL);

    return EXCEPTION_EXECUTE_HANDLER;
}

// ── Public API ───────────────────────────────────────────────────────

inline void InstallCrashHandler() {
    SetUnhandledExceptionFilter(CrashHandler);
}

#else
// Non-Windows stub
inline void InstallCrashHandler() {}
#endif
