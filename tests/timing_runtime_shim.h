/* Digital RP2040 register/FIFO model for the production timing_hw.c runtime.
 * Programs come from pioasm JSON, not a second hand-written waveform engine. */
#ifndef TIMING_RUNTIME_SHIM_H
#define TIMING_RUNTIME_SHIM_H
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define __not_in_flash_func(name) name

typedef unsigned uint;
enum pio_src_dest { pio_x=1, pio_y=2, pio_osr=7 };
enum { clk_sys, GPIO_IN, GPIO_FUNC_SIO, GPIO_OVERRIDE_NORMAL, PIO_FIFO_JOIN_TX };
#define DMA_SIZE_32 2u
#define NUM_DMA_CHANNELS 12u
#define DMA_CH0_CTRL_TRIG_EN_BITS 1u
#define DMA_CH0_CTRL_TRIG_READ_ERROR_BITS (1u << 30)
#define DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS (1u << 29)
#define PIO_FDEBUG_TXSTALL_LSB 24u
#define PIO_FDEBUG_RXSTALL_LSB 16u

typedef struct {
    uint32_t read_addr, write_addr, transfer_count, ctrl_trig;
    uint32_t al1_ctrl, al1_read_addr, al1_write_addr, al1_transfer_count_trig;
    uint32_t al2_ctrl, al2_transfer_count, al2_read_addr, al2_write_addr_trig;
    uint32_t al3_ctrl, al3_write_addr, al3_transfer_count, al3_read_addr_trig;
} dma_channel_hw_t;
typedef struct { uint32_t ctrl; } dma_channel_config;
typedef struct {
    uint32_t instr_mem[32], txf[4], rxf[4], fdebug, input_sync_bypass;
    struct { uint32_t instr; } sm[4];
} pio_hw_t;
typedef pio_hw_t *PIO;
typedef struct {
    uint out_base, out_count, set_base, side_base, in_base, jmp_pin, wrap, wrap_target;
    bool autopull, sideset;
} pio_sm_config;
typedef struct pio_program {
    const uint16_t *instructions;
    uint length, origin, wrap, wrap_target;
    bool sideset;
} pio_program_t;

static pio_hw_t native_pios[2];
#define pio0 (&native_pios[0])
#define pio1 (&native_pios[1])
_Alignas(256) static dma_channel_hw_t native_dma[12];
static uint32_t dma_reload[12];
static bool dma_busy[12];
static uint claimed_dma, dma_cursor;
static bool dma_claimed[12], sm_claimed[2][4];
static uint32_t program_used[2];
static uint64_t dma_write_cycle[12];
static uint32_t dma_write_value[12], dma_write_count[12];
static uint dma_write_delay[12], dma_pending_delay[12];
static bool dma_pending[12];
static uint32_t dma_pending_value[12];
static uint32_t native_levels, native_dirs;
static uint32_t native_synced, native_sync[2];
static uint32_t native_pio_mask[2];
static uint64_t native_cycle;
static size_t staged_reads;
static uintptr_t staged_begin, staged_end;
static bool dma_fault;
static unsigned native_fifo_gap;
static struct native_sm {
    pio_sm_config config;
    uint pc, delay, fifo_count, fifo_read, rx_count, rx_read, osr_count;
    uint32_t fifo[8], rx[4], osr, x, y, isr, injected, forced;
    uint64_t wait_cycle, push_cycle, enable_cycle;
    bool enabled, has_injected, has_forced;
} native_sms[2][4];
static struct { uint64_t cycle; uint32_t levels, dirs; } native_edges[4096];
static size_t native_edge_count;

static inline struct native_sm *native_sm(PIO pio, uint sm) { return &native_sms[pio == pio1][sm]; }
static inline void native_output(uint32_t value, uint32_t mask, bool dirs) {
    uint32_t *target = dirs ? &native_dirs : &native_levels;
    *target = (*target & ~mask) | (value & mask);
}
static inline uint32_t native_rotate(uint32_t value, uint base) {
    return base ? (value << base) | (value >> (32u - base)) : value;
}
static inline void native_prefetch(struct native_sm *sm) {
    if (sm->config.autopull && sm->osr_count == 32u && sm->fifo_count) {
        sm->osr = sm->fifo[sm->fifo_read++ & 7u];
        --sm->fifo_count;
        sm->osr_count = 0;
    }
}
static inline void native_fifo_put(PIO pio, uint index, uint32_t word) {
    struct native_sm *sm = native_sm(pio, index);
    assert(sm->fifo_count < 8u);
    sm->fifo[(sm->fifo_read + sm->fifo_count++) & 7u] = word;
}
static inline bool native_execute(PIO pio, uint index, uint32_t instruction, bool immediate) {
    struct native_sm *sm = native_sm(pio, index);
    unsigned op = instruction >> 13, arg = instruction & 255u;
    unsigned next = immediate ? sm->pc :
                    sm->pc == sm->config.wrap ? sm->config.wrap_target : sm->pc + 1u;
    if (sm->config.sideset && (instruction & 0x1000u))
        native_output(instruction & 0x800u ? 1u << sm->config.side_base : 0,
                      1u << sm->config.side_base, true);
    if (op != 3u && !(op == 4u && (arg & 128u)) && !(op == 5u && (arg >> 5) == 7u))
        native_prefetch(sm);
    if (op == 0u) {
        unsigned condition = arg >> 5;
        bool taken = condition == 0u || (condition == 1u && !sm->x) ||
                     (condition == 2u && sm->x) || (condition == 3u && !sm->y) ||
                     (condition == 4u && sm->y) || (condition == 5u && sm->x != sm->y) ||
                     (condition == 6u && (native_synced & (1u << sm->config.jmp_pin)));
        assert(condition <= 6u);
        if (condition == 2u) --sm->x;
        if (condition == 4u) --sm->y;
        if (taken) next = arg & 31u;
    } else if (op == 1u) {
        uint pin = arg & 31u;
        if ((arg & 96u) == 32u) pin = (pin + sm->config.in_base) & 31u;
        if (!!(native_synced & (1u << pin)) != !!(arg & 128u)) return false;
        sm->wait_cycle = native_cycle;
    } else if (op == 2u) {
        unsigned count = (arg & 31u) ? arg & 31u : 32u;
        assert((arg >> 5) == 0u && count < 32u);
        sm->isr = (sm->isr << count) |
                  ((native_synced >> sm->config.in_base) & ((1u << count) - 1u));
    } else if (op == 3u) {
        unsigned count = (arg & 31u) ? arg & 31u : 32u, dest = arg >> 5;
        if (sm->config.autopull && sm->osr_count == 32u) {
            native_prefetch(sm);
            pio->fdebug |= 1u << (PIO_FDEBUG_TXSTALL_LSB + index);
            if (index == 3u) ++native_fifo_gap;
            return false;
        }
        uint32_t value = sm->osr;
        sm->osr = count == 32u ? 0 : sm->osr >> count;
        sm->osr_count += count;
        if (sm->osr_count > 32u) sm->osr_count = 32u;
        if (dest == 0u || dest == 4u) {
            uint32_t mask = sm->config.out_count == 32u ? UINT32_MAX : (1u << sm->config.out_count) - 1u;
            native_output(native_rotate(value & mask, sm->config.out_base),
                          native_rotate(mask, sm->config.out_base) & native_pio_mask[pio == pio1],
                          dest == 4u);
        } else if (dest == 1u) sm->x = value;
        else if (dest == 7u) { sm->injected = value & 65535u; sm->has_injected = true; }
        else assert(false);
        native_prefetch(sm);
    } else if (op == 4u) {
        if (arg & 128u) {
            if (sm->config.autopull && sm->osr_count < 32u) return true;
            if (!sm->fifo_count && (arg & 32u)) return false;
            sm->osr = sm->fifo_count ? sm->fifo[sm->fifo_read++ & 7u] : sm->x;
            if (sm->fifo_count) --sm->fifo_count;
            sm->osr_count = 0;
        } else {
            if (sm->rx_count == 4u) {
                pio->fdebug |= 1u << (PIO_FDEBUG_RXSTALL_LSB + index);
                if (arg & 32u) return false;
            } else {
                sm->rx[(sm->rx_read + sm->rx_count++) & 3u] = sm->isr;
                sm->push_cycle = native_cycle;
            }
            sm->isr = 0;
        }
    } else if (op == 5u) {
        unsigned src = arg & 7u, dest = arg >> 5;
        uint32_t value = src == 1u ? sm->x : src == 2u ? sm->y :
                         src == 6u ? sm->isr : src == 7u ? sm->osr : 0;
        if (dest == 1u) sm->x = value;
        else if (dest == 2u) sm->y = value;
        else if (dest == 6u) sm->isr = value;
        else if (dest == 7u) { sm->osr = value; sm->osr_count = 0; }
        else assert(false);
    } else if (op == 7u) {
        unsigned dest = arg >> 5;
        if (dest == 0u)
            native_output((arg & 1u) << sm->config.set_base, 1u << sm->config.set_base, false);
        else if (dest == 1u) sm->x = arg & 31u;
        else if (dest == 2u) sm->y = arg & 31u;
        else assert(false);
    } else assert(false);
    if (!immediate || op == 0u) sm->pc = next;
    sm->delay = (instruction >> 8) & (sm->config.sideset ? 7u : 31u);
    return true;
}
static inline void native_force(PIO pio, uint index, uint32_t instruction) {
    struct native_sm *sm = native_sm(pio, index);
    sm->has_injected = false;
    sm->forced = instruction;
    sm->has_forced = !native_execute(pio, index, instruction, true);
    sm->delay = 0; /* SMx_INSTR ignores encoded delays (datasheet 3.5.7). */
}
static inline void native_trigger(uint channel) {
    if (!(native_dma[channel].ctrl_trig & 1u)) return;
    native_dma[channel].transfer_count = dma_reload[channel];
    dma_busy[channel] = dma_reload[channel] != 0;
}
static inline void native_dma_write(uintptr_t address, uint32_t value) {
    for (uint p=0; p<2; ++p) for (uint s=0; s<4; ++s) {
        if (address == (uintptr_t)&native_pios[p].txf[s]) { native_fifo_put(&native_pios[p], s, value); return; }
        if (address == (uintptr_t)&native_pios[p].sm[s].instr) { native_force(&native_pios[p], s, value); return; }
    }
    for (uint c=0; c<12; ++c) {
        dma_channel_hw_t *reg = &native_dma[c];
        if (address == (uintptr_t)&reg->al3_ctrl) {
            reg->ctrl_trig = value & ~(DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS);
            reg->al1_ctrl = reg->ctrl_trig;
            return;
        }
        if (address == (uintptr_t)&reg->al3_write_addr) { reg->write_addr = value; return; }
        if (address == (uintptr_t)&reg->al3_transfer_count) { dma_reload[c] = reg->transfer_count = value; return; }
        if (address == (uintptr_t)&reg->al3_read_addr_trig) { reg->read_addr = value; native_trigger(c); return; }
    }
    *(uint32_t *)address = value;
}
static inline bool native_dma_ready(uint channel) {
    dma_channel_hw_t *reg = &native_dma[channel];
    if (!dma_busy[channel] || !(reg->ctrl_trig & 1u)) return false;
    uint dreq = (reg->ctrl_trig >> 15) & 63u;
    if (dreq < 16u) {
        uint p = dreq / 8u, s = dreq & 3u;
        return dreq & 4u ? native_sms[p][s].rx_count != 0u : native_sms[p][s].fifo_count < 8u;
    }
    return true;
}
static inline uint32_t native_dma_read(uintptr_t address) {
    for (uint p = 0; p < 2; ++p) for (uint s = 0; s < 4; ++s) {
        if (address == (uintptr_t) &native_pios[p].rxf[s]) {
            struct native_sm *sm = &native_sms[p][s];
            assert(sm->rx_count);
            --sm->rx_count;
            return sm->rx[sm->rx_read++ & 3u];
        }
    }
    return *(uint32_t *) address;
}
static inline void native_dma_record_write(uint c, uint32_t value) {
    native_dma_write(native_dma[c].write_addr, value);
    dma_write_cycle[c] = native_cycle;
    dma_write_value[c] = value;
    ++dma_write_count[c];
}
static inline void native_dma_tick(void) {
    if (dma_fault) return;
    for (uint c = 0; c < 12; ++c) if (dma_pending[c]) {
        if (dma_pending_delay[c]) { --dma_pending_delay[c]; continue; }
        native_dma_record_write(c, dma_pending_value[c]);
        dma_pending[c] = false;
        dma_busy[c] = false;
        native_dma[c].transfer_count = 0;
        return;
    }
    for (uint i=0; i<12; ++i) {
        uint c = (dma_cursor + i) % 12u;
        if (!native_dma_ready(c)) continue;
        dma_cursor = (c + 1u) % 12u;
        dma_channel_hw_t *reg = &native_dma[c];
        if (reg->read_addr >= staged_begin && reg->read_addr < staged_end) ++staged_reads;
        uint32_t word = native_dma_read(reg->read_addr);
        if (dma_write_delay[c]) {
            assert(reg->transfer_count == 1u);
            dma_pending[c] = true;
            dma_pending_value[c] = word;
            dma_pending_delay[c] = dma_write_delay[c];
            return;
        }
        native_dma_record_write(c, word);
        uint ring = (reg->ctrl_trig >> 6) & 15u;
        bool write_ring = reg->ctrl_trig & (1u << 10);
        uint32_t mask = ring ? (1u << ring) - 1u : UINT32_MAX;
        if (reg->ctrl_trig & (1u << 4))
            reg->read_addr = write_ring ? reg->read_addr + 4u :
                (reg->read_addr & ~mask) | ((reg->read_addr + 4u) & mask);
        if (reg->ctrl_trig & (1u << 5))
            reg->write_addr = !write_ring ? reg->write_addr + 4u :
                (reg->write_addr & ~mask) | ((reg->write_addr + 4u) & mask);
        if (!--reg->transfer_count) {
            dma_busy[c] = false;
            uint chain = (reg->ctrl_trig >> 11) & 15u;
            if (chain != c) native_trigger(chain);
        }
        return;
    }
}
static inline void native_tick(uint count) {
    while (count--) {
        uint32_t levels = native_levels, dirs = native_dirs;
        native_synced = native_sync[0];
        native_sync[0] = native_sync[1];
        native_sync[1] = native_levels;
        for (uint p=0; p<2; ++p) for (uint s=0; s<4; ++s) {
            struct native_sm *sm = &native_sms[p][s];
            if (!sm->enabled && !sm->has_forced) continue;
            if (sm->delay && !sm->has_forced) { --sm->delay; native_prefetch(sm); continue; }
            bool forced = sm->has_forced, injected = sm->has_injected;
            uint32_t instruction = forced ? sm->forced :
                                   injected ? sm->injected : native_pios[p].instr_mem[sm->pc];
            sm->has_forced = sm->has_injected = false;
            if (!native_execute(&native_pios[p], s, instruction, forced || injected)) {
                sm->has_forced = forced;
                sm->has_injected = injected;
            }
            if (forced) sm->delay = 0;
        }
        native_dma_tick();
        if ((levels != native_levels || dirs != native_dirs) && native_edge_count < 4096u) {
            native_edges[native_edge_count].cycle = native_cycle;
            native_edges[native_edge_count].levels = native_levels;
            native_edges[native_edge_count++].dirs = native_dirs;
        }
        ++native_cycle;
    }
}
static inline dma_channel_hw_t *dma_channel_hw_addr(uint c) { return &native_dma[c]; }
static inline int dma_claim_unused_channel(bool required) {
    for (uint c = 0; c < 12; ++c) if (!dma_claimed[c]) {
        dma_claimed[c] = true;
        ++claimed_dma;
        return (int) c;
    }
    assert(!required);
    return -1;
}
static inline bool dma_channel_is_claimed(uint c) { return dma_claimed[c]; }
static inline void dma_channel_unclaim(uint c) { assert(dma_claimed[c]); dma_claimed[c] = false; --claimed_dma; }
static inline bool dma_channel_is_busy(uint c) { return dma_busy[c]; }
static inline void dma_channel_abort(uint c) { dma_busy[c]=dma_pending[c]; native_dma[c].transfer_count=0; }
static inline void tight_loop_contents(void) { native_tick(1); }
static inline dma_channel_config dma_channel_get_default_config(uint c) {
    return (dma_channel_config){.ctrl=1u | (c << 11) | (63u << 15)};
}
static inline void channel_config_set_transfer_data_size(dma_channel_config *c, uint v) { c->ctrl |= v << 2; }
static inline void channel_config_set_read_increment(dma_channel_config *c, bool v) { c->ctrl=(c->ctrl & ~(1u<<4)) | ((uint)v<<4); }
static inline void channel_config_set_write_increment(dma_channel_config *c, bool v) { c->ctrl=(c->ctrl & ~(1u<<5)) | ((uint)v<<5); }
static inline void channel_config_set_dreq(dma_channel_config *c, uint v) { c->ctrl=(c->ctrl & ~(63u<<15)) | (v<<15); }
static inline void channel_config_set_high_priority(dma_channel_config *c, bool v) { (void)c; (void)v; }
static inline void channel_config_set_chain_to(dma_channel_config *c, uint v) { c->ctrl=(c->ctrl & ~(15u<<11)) | (v<<11); }
static inline void channel_config_set_ring(dma_channel_config *c, bool write, uint size) {
    c->ctrl=(c->ctrl & ~((15u<<6)|(1u<<10))) | (size<<6) | ((uint)write<<10);
}
static inline void dma_channel_configure(uint c, const dma_channel_config *config, volatile void *write,
                                         const volatile void *read, uint32_t count, bool start) {
    native_dma[c].ctrl_trig = config->ctrl & ~(DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS);
    native_dma[c].al1_ctrl = native_dma[c].ctrl_trig;
    native_dma[c].read_addr = (uintptr_t)read;
    native_dma[c].write_addr = (uintptr_t)write;
    native_dma[c].transfer_count = dma_reload[c] = count;
    dma_busy[c] = false;
    if (start) native_trigger(c);
}
static inline void dma_channel_set_read_addr(uint c, const void *read, bool start) {
    native_dma[c].read_addr=(uintptr_t)read;
    if (start) native_trigger(c);
}
static inline void hw_clear_bits(volatile uint32_t *address, uint32_t bits) {
    *address &= ~bits;
    for (uint c=0; c<12; ++c) if (address == &native_dma[c].al1_ctrl) native_dma[c].ctrl_trig &= ~bits;
}
static inline uint32_t clock_get_hz(uint clock) { (void)clock; return 125000000u; }
static inline void gpio_put(uint pin, bool v) { (void)pin; (void)v; }
static inline void gpio_set_dir(uint pin, uint v) { (void)pin; (void)v; }
static inline void gpio_set_function(uint pin, uint v) { (void)pin; (void)v; }
static inline void gpio_disable_pulls(uint pin) { (void)pin; }
static inline void gpio_set_outover(uint pin, uint v) { (void)pin; (void)v; }
static inline void gpio_set_oeover(uint pin, uint v) { (void)pin; (void)v; }
static inline bool gpio_get(uint pin) { return !!(native_levels & (1u<<pin)); }
static inline void pio_gpio_init(PIO pio, uint pin) {
    native_pio_mask[pio == pio1] |= 1u << pin;
    native_pio_mask[pio != pio1] &= ~(1u << pin);
}
static inline bool pio_sm_is_claimed(PIO pio, uint sm) { return sm_claimed[pio == pio1][sm]; }
static inline void pio_sm_claim(PIO pio, uint sm) { assert(!pio_sm_is_claimed(pio,sm)); sm_claimed[pio == pio1][sm] = true; }
static inline void pio_sm_unclaim(PIO pio, uint sm) { assert(pio_sm_is_claimed(pio,sm)); sm_claimed[pio == pio1][sm] = false; }
static inline void pio_sm_set_enabled(PIO pio, uint sm, bool enabled) {
    native_sm(pio,sm)->enabled=enabled;
    if (enabled) native_sm(pio,sm)->enable_cycle=native_cycle;
    if (enabled) pio->fdebug &= ~(1u << (PIO_FDEBUG_TXSTALL_LSB+sm));
}
static inline void pio_sm_clear_fifos(PIO pio, uint sm) {
    native_sm(pio,sm)->fifo_count=0;
    native_sm(pio,sm)->rx_count=0;
}
static inline void pio_sm_restart(PIO pio, uint index) {
    struct native_sm *sm=native_sm(pio,index);
    sm->osr_count=32; sm->has_injected=false; sm->has_forced=false; sm->delay=0;
}
static inline void pio_sm_set_pins_with_mask(PIO pio, uint sm, uint32_t value, uint32_t mask) {
    (void)pio; (void)sm; native_output(value,mask,false);
}
static inline void pio_sm_set_pindirs_with_mask(PIO pio, uint sm, uint32_t value, uint32_t mask) {
    (void)pio; (void)sm; native_output(value,mask,true);
}
static inline void sm_config_set_out_pins(pio_sm_config *c, uint base, uint count) { c->out_base=base; c->out_count=count; }
static inline void sm_config_set_set_pins(pio_sm_config *c, uint base, uint count) { (void)count; c->set_base=base; }
static inline void sm_config_set_sideset_pins(pio_sm_config *c, uint base) { c->side_base=base; }
static inline void sm_config_set_out_shift(pio_sm_config *c, bool right, bool autopull, uint threshold) {
    assert(right && threshold==32); c->autopull=autopull;
}
static inline void sm_config_set_in_shift(pio_sm_config *c, bool right, bool autopush, uint threshold) {
    (void)c; (void)right; (void)autopush; (void)threshold;
}
static inline void sm_config_set_in_pins(pio_sm_config *c, uint base) { c->in_base=base; }
static inline void sm_config_set_jmp_pin(pio_sm_config *c, uint pin) { c->jmp_pin=pin; }
static inline void sm_config_set_fifo_join(pio_sm_config *c, uint join) { (void)c; (void)join; }
static inline void pio_sm_set_out_pins(PIO pio, uint sm, uint base, uint count) { sm_config_set_out_pins(&native_sm(pio,sm)->config,base,count); }
static inline void pio_sm_init(PIO pio, uint sm, uint pc, const pio_sm_config *config) {
    memset(native_sm(pio,sm),0,sizeof(struct native_sm)); native_sm(pio,sm)->config=*config;
    native_sm(pio,sm)->pc=pc; native_sm(pio,sm)->osr_count=32;
}
static inline void pio_sm_put(PIO pio, uint sm, uint32_t word) { native_fifo_put(pio,sm,word); }
static inline uint pio_sm_get_pc(PIO pio, uint sm) { return native_sm(pio,sm)->pc; }
static inline uint pio_sm_get_tx_fifo_level(PIO pio, uint sm) { native_dma_tick(); return native_sm(pio,sm)->fifo_count; }
static inline bool pio_sm_is_tx_fifo_empty(PIO pio, uint sm) { return !native_sm(pio,sm)->fifo_count; }
static inline uint pio_get_dreq(PIO pio, uint sm, bool tx) { return (pio == pio1 ? 8u : 0u) + sm + (tx ? 0u : 4u); }
static inline void pio_sm_exec(PIO pio, uint sm, uint32_t instruction) { native_force(pio,sm,instruction); }
static inline void pio_enable_sm_mask_in_sync(PIO pio, uint32_t mask) {
    for (uint sm=0; sm<4; ++sm) if (mask & (1u<<sm)) {
        native_sm(pio,sm)->enabled=true;
        native_sm(pio,sm)->enable_cycle=native_cycle;
    }
    /* Model the W1C startup writes immediately before enable. */
    pio->fdebug=0;
}
static inline uint32_t pio_encode_jmp(uint pc) { return pc; }
static inline uint32_t pio_encode_pull(bool ifempty, bool block) { (void)ifempty; return 0x8080u | ((uint)block<<5); }
static inline uint32_t pio_encode_mov(enum pio_src_dest dest, enum pio_src_dest src) { return 0xa000u | ((uint)dest<<5) | src; }
static inline uint32_t pio_encode_wait_gpio(bool level, uint pin) { return 0x2000u | ((uint)level<<7) | pin; }
static inline uint32_t pio_encode_nop(void) { return 0xa042u; }
static inline bool pio_can_add_program_at_offset(PIO pio, const pio_program_t *program, uint offset) {
    if (offset + program->length > 32u || offset != program->origin) return false;
    uint32_t mask = ((1u << program->length) - 1u) << offset;
    return !(program_used[pio == pio1] & mask);
}
static inline void pio_add_program_at_offset(PIO pio, const pio_program_t *program, uint offset) {
    assert(pio_can_add_program_at_offset(pio, program, offset));
    program_used[pio == pio1] |= ((1u << program->length) - 1u) << offset;
    for (uint i=0; i<program->length; ++i) {
        uint32_t word=program->instructions[i];
        pio->instr_mem[offset+i]=word + ((word>>13)==0 ? offset : 0);
    }
}
static inline void pio_remove_program(PIO pio, const pio_program_t *program, uint offset) {
    uint32_t mask = ((1u << program->length) - 1u) << offset;
    assert((program_used[pio == pio1] & mask) == mask);
    program_used[pio == pio1] &= ~mask;
}
#include "timing_test_programs.h"
#endif
