/*
 * Opens a routing package and routes between two points.
 *
 *   cargo build --release -p wayfarer-ffi
 *   cc examples/route.c -Iinclude <target>/release/libwayfarer_ffi.a \
 *      -framework CoreFoundation -o route     # macOS; on Linux add -lpthread -ldl -lm
 *   cl examples\route.c /Iinclude <target>\release\wayfarer_ffi.dll.lib    # Windows (DLL)
 *   cl /MD examples\route.c /Iinclude <target>\release\wayfarer_ffi.lib \
 *      ws2_32.lib userenv.lib ntdll.lib bcrypt.lib advapi32.lib    # Windows (static)
 *   ./route routing.mrt 52.0 21.0002 52.0 21.0018
 *
 * Licensed under the Apache License, Version 2.0.
 */
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "wayfarer.h"

static int run(int argc, char **argv) {
    if (argc != 6) {
        fprintf(stderr, "usage: %s <routing.mrt> <lat> <lng> <lat> <lng>\n", argv[0]);
        return 2;
    }
    char *text = NULL;
    WayfarerPackage *package = NULL;
    int32_t status = wayfarer_open(argv[1], NULL, &package, &text);
    if (status != WAYFARER_OK) {
        fprintf(stderr, "open failed (%d): %s\n", status, text);
        wayfarer_string_free(text);
        return 1;
    }
    wayfarer_string_free(text);

    char request[512];
    snprintf(request, sizeof request,
             "{\"locations\": [{\"point\": {\"lat\": %s, \"lng\": %s}},"
             " {\"point\": {\"lat\": %s, \"lng\": %s}}],"
             " \"profile\": \"wheeled\"}",
             argv[2], argv[3], argv[4], argv[5]);
    status = wayfarer_route(package, request, &text);
    printf("%s\n", text);
    wayfarer_string_free(text);
    wayfarer_close(package);
    return status == WAYFARER_OK ? 0 : 1;
}

#ifdef _WIN32
/* Windows passes main() its arguments in the ANSI code page, which can't
 * spell every path (a Polish folder name on a Western European system, say),
 * while wayfarer_open takes UTF-8. wmain gets them as UTF-16; convert each
 * one to UTF-8. (wayfarer_open_w takes the UTF-16 path as it is.) With MinGW,
 * link with -municode. */
int wmain(int argc, wchar_t **wide) {
    char **argv = calloc((size_t)argc + 1, sizeof *argv);
    if (argv == NULL) {
        return 1;
    }
    for (int i = 0; i < argc; i++) {
        int bytes = WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, NULL, 0, NULL, NULL);
        argv[i] = bytes > 0 ? malloc((size_t)bytes) : NULL;
        if (argv[i] == NULL
            || WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, argv[i], bytes, NULL, NULL) != bytes) {
            fprintf(stderr, "argument %d is not valid UTF-16\n", i);
            return 2;
        }
    }
    int code = run(argc, argv);
    for (int i = 0; i < argc; i++) {
        free(argv[i]);
    }
    free(argv);
    return code;
}
#else
int main(int argc, char **argv) { return run(argc, argv); }
#endif
