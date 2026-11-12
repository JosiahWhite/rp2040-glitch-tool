#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REQUEST_LIMIT  4096u
#define RESPONSE_LIMIT 32768u
#define ARG_LIMIT      16u

typedef struct {
    char        line[REQUEST_LIMIT];
    char       *argv[ARG_LIMIT];
    size_t      length;
    unsigned    argc;
    uint32_t    id;
    bool        overflow, ready;
    const char *error;
} request_t;

/* Feed at most one complete request; caller retains ready until dispatched. */
void request_reset(request_t *request);
bool request_feed(request_t *request, unsigned char character);
bool request_u32(const request_t *request, unsigned index, uint32_t *value);
bool parse_u32(const char *text, uint32_t *value);
bool parse_hex(const char *text, uint8_t *out, size_t capacity, size_t *length);

typedef struct {
    char   data[RESPONSE_LIMIT];
    size_t length, sent;
    bool   failed;
} response_t;
void response_begin(response_t *response, uint32_t id);
bool response_append(response_t *response, const char *format, ...);
bool response_hex(response_t *response, const uint8_t *data, size_t length);
void response_end(response_t *response);
void response_error(response_t *response, uint32_t id, const char *code, int step, uint32_t detail);
