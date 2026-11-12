#include "protocol.h"
#include <assert.h>
#include <string.h>

static void frame(request_t *r, const char *s) {
    request_reset(r);
    for (; *s; ++s) request_feed(r, (unsigned char)*s);
}
int main(void) {
    static request_t r;
    static response_t out;
    uint32_t n = 9;
    uint8_t bytes[2] = {42, 42};
    size_t length;
    assert(parse_u32("4294967295", &n) && n == UINT32_MAX);
    assert(!parse_u32("4294967296", &n));
    assert(!parse_u32("-1", &n) && !parse_u32("+1", &n));
    assert(!parse_u32("0x10", &n) && !parse_u32("1x", &n));
    assert(parse_hex("00ff", bytes, 2, &length) && length == 2 && bytes[1] == 255);
    assert(!parse_hex("01FG", bytes, 2, &length) && bytes[0] == 0);
    assert(!parse_hex("0", bytes, 2, &length));
    assert(!parse_hex("000000", bytes, 2, &length));
    assert(parse_hex("-", bytes, 2, &length) && length == 0);
    frame(&r, "1 stat");
    assert(!r.ready);
    for (const char *p = "us\r\n"; *p; ++p) request_feed(&r, *p);
    assert(r.ready && !r.error && r.id == 1 && r.argc == 2 && !strcmp(r.argv[1], "status"));
    assert(request_u32(&r, 0, &n) && n == 1);
    assert(!request_u32(&r, 1, &n) && n == 1);
    assert(!request_u32(&r, r.argc, &n) && n == 1);
    frame(&r, "2 status extra \n"); assert(r.error);
    frame(&r, "0 info\n"); assert(r.error);
    frame(&r, "3  info\n"); assert(r.error);
    frame(&r, "4 info\rjunk\n"); assert(r.error);
    frame(&r, "5 ");
    for (unsigned i = 0; i < REQUEST_LIMIT; ++i) request_feed(&r, 'x');
    assert(!r.ready);
    request_feed(&r, '\n');
    assert(r.ready && r.error && r.id == 5);
    frame(&r, "6 info\n"); assert(!r.error && r.id == 6);
    response_begin(&out, 6);
    response_append(&out, "{\"data\":\"");
    const uint8_t value[] = {0, 255};
    response_hex(&out, value, 2);
    response_append(&out, "\"}"); response_end(&out);
    assert(!strcmp(out.data, "{\"id\":6,\"ok\":true,\"result\":{\"data\":\"00ff\"}}\n"));
    response_error(&out, 7, "NACK", -1, 1);
    assert(!strcmp(out.data, "{\"id\":7,\"ok\":false,\"error\":{\"code\":\"NACK\",\"step\":null,\"detail\":1}}\n"));
    return 0;
}
