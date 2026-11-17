/*
 * Opens a routing package and routes between two points.
 *
 *   cargo build --release -p wayfarer-ffi
 *   cc examples/route.c -Iinclude <target>/release/libwayfarer_ffi.a \
 *      -framework CoreFoundation -o route     # macOS; on Linux add -lpthread -ldl -lm
 *   cl examples\route.c /Iinclude <target>\release\wayfarer_ffi.dll.lib    # Windows
 *   ./route routing.mrt 52.0 21.0002 52.0 21.0018
 *
 * Licensed under the Apache License, Version 2.0.
 */
#include <stdio.h>
#include <stdlib.h>

#include "wayfarer.h"

int main(int argc, char **argv) {
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
