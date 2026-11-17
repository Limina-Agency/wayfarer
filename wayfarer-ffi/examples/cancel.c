/*
 * Cancels routes: from another thread while a worker routes in a loop, and
 * by a token's timeout.
 *
 *   cargo build --release -p wayfarer-ffi
 *   cc examples/cancel.c -Iinclude <target>/release/libwayfarer_ffi.a \
 *      -framework CoreFoundation -o cancel    # macOS; on Linux add -lpthread -ldl -lm
 *   cl examples\cancel.c /Iinclude <target>\release\wayfarer_ffi.dll.lib    # Windows
 *   ./cancel routing.mrt 52.0 21.0002 52.0 21.0018
 *
 * Licensed under the Apache License, Version 2.0.
 */
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <time.h>
#endif

#include "wayfarer.h"

struct worker {
    const WayfarerPackage *package;
    const WayfarerCancelToken *cancel;
    const char *request;
    long routes;
    int32_t status;
    char *text;
};

/* Routes until a call fails: once the token is cancelled, the route in flight
 * stops and every later one fails at once. */
static void route_until_cancelled(struct worker *worker) {
    for (worker->routes = 0; worker->routes < 10000000; worker->routes++) {
        char *text = NULL;
        worker->status = wayfarer_route_with_cancel(worker->package, worker->request,
                                                            worker->cancel, &text);
        if (worker->status != WAYFARER_OK) {
            worker->text = text;
            return;
        }
        wayfarer_string_free(text);
    }
}

/* A sleep and one worker thread, on Windows and on POSIX systems. */
#ifdef _WIN32
typedef HANDLE worker_thread;

static void sleep_ms(long ms) { Sleep((DWORD)ms); }

static DWORD WINAPI worker_main(LPVOID argument) {
    route_until_cancelled(argument);
    return 0;
}

static int worker_start(worker_thread *thread, struct worker *worker) {
    *thread = CreateThread(NULL, 0, worker_main, worker, 0, NULL);
    return *thread ? 0 : -1;
}

static void worker_join(worker_thread thread) {
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
}
#else
typedef pthread_t worker_thread;

static void sleep_ms(long ms) {
    struct timespec delay = {ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&delay, NULL);
}

static void *worker_main(void *argument) {
    route_until_cancelled(argument);
    return NULL;
}

static int worker_start(worker_thread *thread, struct worker *worker) {
    return pthread_create(thread, NULL, worker_main, worker);
}

static void worker_join(worker_thread thread) { pthread_join(thread, NULL); }
#endif

int main(int argc, char **argv) {
    if (argc != 6) {
        fprintf(stderr, "usage: %s <routing.mrt> <lat> <lng> <lat> <lng>\n", argv[0]);
        return 2;
    }
    char *text = NULL;
    WayfarerPackage *package = NULL;
    int32_t status = wayfarer_open(argv[1], NULL, &package, &text);
    wayfarer_string_free(text);
    if (status != WAYFARER_OK) {
        fprintf(stderr, "open failed (%d)\n", status);
        return 1;
    }
    char request[512];
    snprintf(request, sizeof request,
             "{\"locations\": [{\"point\": {\"lat\": %s, \"lng\": %s}},"
             " {\"point\": {\"lat\": %s, \"lng\": %s}}],"
             " \"profile\": \"wheeled\"}",
             argv[2], argv[3], argv[4], argv[5]);
    int failed = 0;

    /* Cancelled from this thread while the worker routes with the token. */
    WayfarerCancelToken *cancel = wayfarer_cancel_token_new(0);
    struct worker worker = {package, cancel, request, 0, WAYFARER_OK, NULL};
    worker_thread thread;
    if (worker_start(&thread, &worker) != 0) {
        fprintf(stderr, "starting the worker thread failed\n");
        return 1;
    }
    sleep_ms(20);
    wayfarer_cancel_token_cancel(cancel);
    worker_join(thread);
    printf("cancelled after %ld routes (%d): %s\n", worker.routes, worker.status,
           worker.text ? worker.text : "");
    if (worker.status != WAYFARER_CANCELLED || !worker.text
        || !strstr(worker.text, "\"cancelled\"")) {
        failed = 1;
    }
    wayfarer_string_free(worker.text);
    /* Free the token only once no call using it is in flight. */
    wayfarer_cancel_token_free(cancel);

    /* A token whose 1 ms timeout has passed. */
    WayfarerCancelToken *deadline = wayfarer_cancel_token_new(1);
    sleep_ms(2);
    status = wayfarer_route_with_cancel(package, request, deadline, &text);
    printf("after the deadline (%d): %s\n", status, text);
    if (status != WAYFARER_CANCELLED || !strstr(text, "\"deadline_exceeded\"")) {
        failed = 1;
    }
    wayfarer_string_free(text);
    wayfarer_cancel_token_free(deadline);

    wayfarer_close(package);
    return failed;
}
