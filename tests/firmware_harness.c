/* Execute production dispatch/staging/poll lifecycle with a deterministic bus
 * boundary. Neither this native model nor the CDC hardware smoke certifies
 * electrical behavior or waveforms; external authorized measurements are required. */
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#define GLITCH_SYS_CLOCK_KHZ 125000
#include "../usb_control.c"
#define main firmware_entry
#include "../main.c"
#undef main

static uint64_t now=100;
static i2c_bus_config_t configs[2];
static timing_gpio_t gpio_config;
static int sync_pin=-1;
static timing_spi_t spi_config;
static timing_pwm_t pwm_config;
static const char *pwm_failure;
static i2c_transfer_t transaction;
static timing_status_t waveform;
static const timing_block_t *active_block;
static unsigned bus_starts, stops, safe_calls;
static bool nack;
static uint8_t fixture_value;
static const char *incoming="";
static bool prepared_transfer, events_active, event_ready[2], event_released[2];
static bool event_resources_busy, event_reference_started, event_rising;
static int event_reference_pin;
static uint64_t event_origin;
static uint32_t event_offsets[TIMING_WRITE_LIMIT];
static uint64_t event_waveform_cycles;
static unsigned reserved_write_slots;
static unsigned prepared_bus, event_count, event_arms, software_references;
static volatile uint32_t enable_registers[2];
static volatile uint32_t *event_registers[2];
static const char *event_error;
static bool defer_preparation;
static unsigned preparation_us;

static char cdc_capture[RESPONSE_LIMIT];
static size_t cdc_length;
static uint32_t cdc_accept=RESPONSE_LIMIT, cdc_flush_result;
static unsigned boot_calls;

static void cdc_reset(uint32_t accepted) {
    cdc_length=0; cdc_capture[0]='\0';
    cdc_accept=accepted; cdc_flush_result=0;
}
static jmp_buf boot_complete;
static uint32_t boot_outputs=0x3fffffffu, boot_pullups=0x3fffffffu, boot_pulldowns=0x3fffffffu;
void gpio_init(unsigned pin) {
    assert(pin<NUM_BANK0_GPIOS);
    boot_outputs &= ~(1u<<pin);
}
void gpio_disable_pulls(unsigned pin) {
    assert(pin<NUM_BANK0_GPIOS);
    boot_pullups &= ~(1u<<pin);
    boot_pulldowns &= ~(1u<<pin);
}
void timing_init(void) {}
bool tusb_init(void) {
    assert(!boot_outputs && !boot_pullups && !boot_pulldowns);
    longjmp(boot_complete,1);
}
void tud_task(void) { abort(); }
void tud_cdc_read_flush(void) { incoming=""; }
uint32_t tud_cdc_write(const void *data,uint32_t length) {
    if (length>cdc_accept) length=cdc_accept;
    assert(length<sizeof(cdc_capture)-cdc_length);
    memcpy(cdc_capture+cdc_length,data,length);
    cdc_length+=length; cdc_capture[cdc_length]='\0';
    return length;
}
uint32_t tud_cdc_write_flush(void) { return cdc_flush_result; }
void reset_usb_boot(uint32_t mask,uint32_t disable) {
    assert(mask==0 && disable==0);
    assert(!sequence_running() && !waveform.active && !events_active &&
           !transaction.busy && !pwm_config.active);
    ++boot_calls;
}

uint32_t tud_cdc_available(void) { return (uint32_t)strlen(incoming); }
uint32_t tud_cdc_read(void *data,uint32_t length) {
    if (!length || !*incoming) return 0;
    *(char *)data=*incoming++; return 1;
}
uint64_t time_us_64(void) { return now; }
uint32_t clock_get_hz(int c) { (void)c; return 125000000; }
bool set_sys_clock_khz(uint32_t k,bool r) { (void)r; return k==125000; }
const i2c_bus_config_t *i2c_bus_config(unsigned bus) { return &configs[bus]; }
const char *i2c_bus_configure(unsigned b,unsigned sda,unsigned scl,uint32_t hz,bool pulls) {
    configs[b]=(i2c_bus_config_t){true,sda,scl,hz,pulls}; return NULL;
}
static const char *begin_transfer(unsigned bus,unsigned address,const uint8_t *w,size_t wl,
                                  uint8_t *r,size_t rl,uint64_t deadline,const uint16_t *commands) {
    assert(bus<2 && address>=8 && address<=0x77 && !transaction.busy);
    ++bus_starts;
    prepared_transfer=commands!=NULL; prepared_bus=bus;
    if (prepared_transfer) enable_registers[bus]=5;
    const char *error=commands ? i2c_transfer_prepare(&transaction,commands,wl,now,deadline) :
                                i2c_transfer_init(&transaction,w,wl,r,rl,now,deadline);
    if (!error) {
        now+=preparation_us;
        i2c_bus_service();
    }
    return error;
}
const char *i2c_bus_begin(unsigned bus,unsigned address,const uint8_t *w,size_t wl,uint8_t *r,size_t rl,uint64_t deadline) {
    return begin_transfer(bus,address,w,wl,r,rl,deadline,NULL);
}
const char *i2c_bus_prepare(unsigned bus,unsigned address,const uint16_t *commands,size_t length,uint64_t deadline) {
    return begin_transfer(bus,address,NULL,length,NULL,0,deadline,commands);
}
bool i2c_bus_prepared(void) {
    return prepared_transfer && transaction.busy && (enable_registers[prepared_bus]&4) &&
           transaction.writes_queued==transaction.write_length;
}
volatile uint32_t *i2c_bus_enable_register(unsigned bus) { return &enable_registers[bus]; }
void i2c_bus_service(void) {
    if (!transaction.busy) return;
    if (defer_preparation && !transaction.writes_queued) return;
    uint16_t command;
    if (prepared_transfer) {
        bool blocked=!!(enable_registers[prepared_bus]&4);
        for (unsigned n=0;n<16 && transaction.busy;++n) {
            bool queued=i2c_transfer_advance(&transaction,now,!blocked && nack?1:0,
                                             !blocked,true,!blocked,&command);
            if (!queued) break;
        }
        if (!blocked && !transaction.busy && !transaction.result.error) ++stops;
        return;
    }
    bool queued=i2c_transfer_advance(&transaction,now,nack?1:0,true,true,false,&command);
    if (queued) {
        if (command & (1u<<8)) i2c_transfer_receive(&transaction,fixture_value);
        if (command & (1u<<9)) {
            ++stops;
            i2c_transfer_advance(&transaction,now,0,true,true,true,&command);
        }
    }
}
bool i2c_bus_busy(void) { return transaction.busy; }
const i2c_bus_result_t *i2c_bus_result(void) { return &transaction.result; }
void i2c_bus_cancel(void) {
    assert(!events_active); /* DMA must not be able to reenable a reset controller. */
    i2c_transfer_cancel(&transaction);
    prepared_transfer=false;
    enable_registers[0]=enable_registers[1]=0;
}
const timing_gpio_t *timing_gpio(void) { return &gpio_config; }
int timing_sync_pin(void) { return sync_pin; }
const timing_spi_t *timing_spi(void) { return &spi_config; }
const char *timing_configure_gpio(unsigned pin,timing_drive_t drive,bool idle) {
    uint32_t bit=1u << pin;
    gpio_config.configured_mask |= bit;
    gpio_config.open_drain_mask = (gpio_config.open_drain_mask & ~bit) | (drive==TIMING_OPEN_DRAIN ? bit : 0);
    gpio_config.open_source_mask = (gpio_config.open_source_mask & ~bit) | (drive==TIMING_OPEN_SOURCE ? bit : 0);
    gpio_config.idle_high_mask = (gpio_config.idle_high_mask & ~bit) | (idle ? bit : 0);
    uint32_t levels=(gpio_config.idle_high_mask & ~(gpio_config.open_drain_mask | gpio_config.open_source_mask)) | gpio_config.open_source_mask;
    uint32_t dirs=gpio_config.configured_mask & ~(gpio_config.open_drain_mask & gpio_config.idle_high_mask) &
                  ~(gpio_config.open_source_mask & ~gpio_config.idle_high_mask);
    gpio_config.current_levels=(gpio_config.current_levels & ~bit) | (levels & bit);
    gpio_config.current_dirs=(gpio_config.current_dirs & ~bit) | (dirs & bit);
    return NULL;
}
const char *timing_configure_sync(unsigned pin) { sync_pin=(int)pin; return NULL; }
const char *timing_configure_spi(const timing_spi_t *s) { spi_config=*s; return NULL; }
const timing_pwm_t *timing_pwm(void) { return &pwm_config; }
const char *timing_configure_pwm(unsigned pin,uint32_t hz) {
    if (pwm_config.active) return "BUSY";
    uint32_t period;
    const char *e=timing_pwm_parameters(clock_get_hz(clk_sys),hz,&period);
    if (e) return e;
    pwm_config=(timing_pwm_t){.configured=true,.pin=pin,.hz=hz,.period_cycles=period,.final_mode=TIMING_PWM_LOW};
    gpio_config.out_base=(pin+1u)&31u;
    return NULL;
}
void timing_pwm_stop(void) {
    pwm_config.active=false; pwm_config.final_duty_basis_points=0;
    pwm_config.final_mode=TIMING_PWM_LOW; pwm_failure=NULL;
}
const char *timing_pwm_error(void) { return pwm_failure; }
const char *timing_prepare_run(timing_block_t *blocks,size_t count,unsigned slots) {
    (void)blocks; (void)count;
    if (event_resources_busy && slots) return "RESOURCE_BUSY";
    if (waveform.active || events_active) return "BUSY";
    reserved_write_slots=slots;
    return NULL;
}
void timing_release_run(void) {
    timing_write_stop();
    reserved_write_slots=0;
}
const char *timing_write_arm(int pin,bool rising,const uint32_t *at,
                             volatile uint32_t *const *registers,size_t count,uint64_t waveform_cycles) {
    assert(pin>=-1 && pin<30 && count>0 && count<=reserved_write_slots && at[0]>=TIMING_WRITE_MIN_CYCLES);
    event_waveform_cycles=waveform_cycles;
    assert(!events_active && !waveform.active);
    ++event_arms;
    events_active=true; event_count=count; event_error=NULL;
    event_reference_pin=pin; event_rising=rising; event_reference_started=false;
    for (unsigned i=0;i<count;++i) {
        event_offsets[i]=at[i];
        event_registers[i]=registers[i];
        event_ready[i]=event_released[i]=false;
    }
    return NULL;
}
const char *timing_write_reference(void) {
    assert(events_active && event_reference_pin==-1 && !event_reference_started);
    ++software_references;
    event_reference_started=true; event_origin=now*125;
    return NULL;
}
const char *timing_write_ready(unsigned index) {
    timing_service();
    if (event_error) return event_error;
    assert(events_active && index<event_count && !event_ready[index]);
    assert(i2c_bus_prepared());
    event_ready[index]=true; return NULL;
}
bool timing_write_released(unsigned index) { return event_released[index]; }
const char *timing_write_error(void) { return event_error; }
void timing_write_stop(void) {
    events_active=false; event_error=NULL; event_reference_started=false;
}
const char *timing_start(const timing_block_t *b,const timing_trigger_t *t,uint32_t remaining) {
    assert(remaining>0); waveform=(timing_status_t){.active=true,.waiting=b->waits};
    if (events_active && !event_reference_started) {
        assert(i2c_bus_prepared() && event_ready[0]);
        if (t->kind!=TIMING_TRIGGER_NONE) assert(event_reference_pin==sync_pin);
        if (event_waveform_cycles) {
            assert(event_reference_pin==-1 && event_waveform_cycles==b->completion_cycles);
            event_reference_started=true;
            event_origin=now*125+event_waveform_cycles;
        }
    }
    active_block=b;
    if (b->pwm_count) {
        pwm_config.active=true; pwm_config.final_duty_basis_points=b->pwm_final_duty;
        pwm_config.final_mode=b->pwm_final_mode;
    }
    return NULL;
}
void timing_service(void) {
    if (!events_active || event_error) return;
    if (!event_reference_started && event_reference_pin>=0 && waveform.active &&
        (event_reference_pin==sync_pin ? waveform.matched : waveform.done)) {
        event_reference_started=true; event_origin=now*125;
    }
    if (!event_reference_started) return;
    for (unsigned i=0;i<event_count;++i) {
        if (event_released[i]) continue;
        if (now*125 >= event_origin+event_offsets[i]-TIMING_WRITE_READY_LEAD &&
            !event_ready[i]) {
            event_error="MISSED_DEADLINE";
            return;
        }
        if (now*125 >= event_origin+event_offsets[i]) {
            assert(event_ready[i]);
            *event_registers[i]=1;
            event_released[i]=true;
        }
    }
}
timing_status_t timing_status(void) {
    if (waveform.done && waveform.active) {
        gpio_config.current_levels=active_block->final_levels;
        gpio_config.current_dirs=active_block->final_dirs;
        waveform.active=false;
        if (active_block->pwm_count && active_block->pwm_final_mode!=TIMING_PWM_CARRIER)
            pwm_config.active=false;
    }
    return waveform;
}
void timing_stop(void) {
    timing_write_stop();
    waveform.active=false; ++safe_calls;
    gpio_config.current_levels=(gpio_config.idle_high_mask & ~(gpio_config.open_drain_mask | gpio_config.open_source_mask)) |
                               gpio_config.open_source_mask;
    gpio_config.current_dirs=gpio_config.configured_mask & ~(gpio_config.open_drain_mask & gpio_config.idle_high_mask) &
                             ~(gpio_config.open_source_mask & ~gpio_config.idle_high_mask);
}

#ifndef EXECUTOR_RUNTIME_TEST
static bool executor_held;
/* Deterministic dispatch boundary; real cross-core ownership is exercised by
 * the executor's threaded runtime harness. */
void executor_init(uint8_t receive_buffer[I2C_TRANSFER_LIMIT]) {
    assert(receive_buffer==rx); executor_held=false;
}
bool executor_busy(void) { return sequence_running() || i2c_bus_busy(); }
bool executor_acquire(void) {
    assert(!executor_held);
    if (executor_busy()) return false;
    executor_held=true; return true;
}
void executor_release(void) { assert(executor_held); executor_held=false; }
void executor_status(executor_status_t *out) {
    out->sequence=*sequence_status();
    if (sequence_running()) out->sequence.run_elapsed=now-out->sequence.run_start;
    out->pwm=pwm_config;
}
void executor_abort(void) { assert(!executor_held); sequence_abort(); }
#endif

static void command(const char *line) {
    request_reset(&request); response_ready=false;
    for (const char *p=line;*p;++p) request_feed(&request,*p);
    assert(request.ready); dispatch();
}
static void ok(const char *line) {
    command(line); assert(response_ready && strstr(response.data,"\"ok\":true"));
}
static void error(const char *line,const char *code) {
    command(line); assert(response_ready && strstr(response.data,"\"ok\":false") && strstr(response.data,code));
}
static void tick(void) { ++now; timing_service(); i2c_bus_service(); sequence_service(rx); }
static void reach_event(unsigned index) {
    assert(events_active && event_reference_started && index<event_count);
    uint64_t deadline=(event_origin+event_offsets[index]+124)/125;
    if (now<deadline) now=deadline;
    tick();
}

static void gpio_checks(void) {
    unsigned unchanged=safe_calls;
    ok("200 clear\n");
    error("201 add wait_gpio 30 1 100\n","RANGE");
    error("202 add wait_gpio 21 2 100\n","RANGE");
    error("203 add wait_gpio 21 1 0\n","RANGE");
    error("204 add wait_gpio 21 1 100 extra\n","BAD_REQUEST");
    error("1204 start 1000\n","RANGE");
    assert(safe_calls==unchanged);
    const unsigned occupied[]={4,5,6,7,10,29,12,18,19,20,22};
    for (unsigned i=0;i<sizeof(occupied)/sizeof(occupied[0]);++i) {
        char line[80];
        ok("205 clear\n");
        snprintf(line,sizeof(line),"206 add wait_gpio %u 1 100\n",occupied[i]);
        ok(line);
        error("207 start 1000\n","PIN_CONFLICT");
        assert(safe_calls==unchanged);
    }
    ok("208 clear\n"); ok("209 add wait_gpio 21 0 20\n");
    ok("210 add wait_spi aa 100\n"); error("211 start 1000\n","RANGE");
    ok("212 clear\n"); ok("213 add wait_gpio 21 0 20\n"); ok("214 add pulse 29 8\n");
    ok("215 start 1000\n"); tick(); now+=21; tick();
    assert(!strcmp(sequence_status()->run_error,"WAIT_TIMEOUT") && !waveform.active);
    ok("216 start 1000\n"); tick();
    error("217 gpio_config 29 push_pull 1\n","BUSY");
    waveform.matched=true; waveform.done=true; tick();
    ok("218 status\n");
    assert(strstr(response.data,"\"source\":\"gpio\",\"pin\":21,\"rising\":false"));
    puts(response.data);
    ok("219 start 1000\n"); tick(); ok("220 abort\n");
    assert((sequence_status()->state == SEQUENCE_ABORTED) && !waveform.active);
}
static void pwm_checks(void) {
    unsigned unchanged=safe_calls;
    ok("300 clear\n"); ok("301 add pwm 6000\n");
    error("303 pwm_config 30 500000\n","RANGE");
    error("304 pwm_config 22 0\n","RANGE");
    error("305 pwm_config 4 500000\n","PIN_CONFLICT");
    ok("306 pwm_config 22 500000\n");
    error("307 bus_config 1 22 23 100000 0\n","PIN_CONFLICT");
    error("308 gpio_config 22 push_pull 0\n","PIN_CONFLICT");
    error("309 spi_config 22 23 24 0 0 1 10000000\n","PIN_CONFLICT");
    error("310 add pwm 10001\n","RANGE");
    assert(!pwm_config.active && safe_calls==unchanged);
    ok("311 add pulse 10 12500000\n"); ok("312 add delay 1250\n");
    ok("313 add pwm 1000\n"); ok("314 add delay 12500\n"); ok("315 add pwm 6000\n");
    ok("316 start 1000000\n"); tick();
    assert(pwm_config.active && pwm_config.final_duty_basis_points==6000);
    assert(sequence_status()->programmed_cycles==12513750 && active_block->pwm_segments[0].periods==50005 && active_block->pwm_segments[1].periods==50);
    waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && pwm_config.active);
    ok("317 status\n"); puts(response.data);
    error("318 pwm_config 22 250000\n","BUSY");
    ok("319 clear\n"); ok("320 add pwm 6000\n"); ok("321 add write 0 80 100 aa\n");
    ok("322 add delay 8\n"); ok("323 add pwm 1000\n");
    unchanged=safe_calls;
    error("324 start 1000000\n","RANGE");
    assert(safe_calls==unchanged && pwm_config.active && pwm_config.final_duty_basis_points==6000);
    ok("325 abort\n"); assert(!pwm_config.active);
    ok("326 clear\n"); ok("327 add wait_gpio 22 1 100\n");
    error("328 start 1000\n","PIN_CONFLICT");
    ok("329 clear\n"); ok("330 add pwm 6000\n"); ok("331 add wait_gpio 21 1 100\n");
    error("332 start 1000\n","RANGE"); assert(!pwm_config.active);
    ok("333 clear\n"); ok("334 add pwm 6000\n"); ok("335 add write 0 80 100 aa\n");
    ok("336 add wait_gpio 21 1 100\n"); ok("337 start 1000\n"); tick();
    waveform.done=true; tick();
    for (unsigned i=0;i<8 && sequence_status()->current_step<2;++i) tick();
    tick(); assert(sequence_status()->current_step==2 && waveform.waiting && pwm_config.active);
    waveform.matched=true; waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && pwm_config.active);
    pwm_failure="TIMING_ERROR"; tick();
    assert((sequence_status()->state == SEQUENCE_ERROR) && !strcmp(sequence_status()->run_error,"TIMING_ERROR") && !pwm_config.active);
    ok("338 clear\n"); ok("339 add pwm 6000\n"); ok("340 add delay 250\n");
    ok("341 start 10\n"); tick(); now+=11; tick();
    assert(!strcmp(sequence_status()->run_error,"RUN_TIMEOUT") && !pwm_config.active);
    ok("342 start 1000\n"); tick(); waveform.error="TIMING_ERROR"; tick();
    assert(!strcmp(sequence_status()->run_error,"TIMING_ERROR") && !pwm_config.active);
    ok("343 start 1000\n"); tick(); waveform.done=true; tick();
    assert(pwm_config.active);
    sequence_abort(); assert(!pwm_config.active);
    ok("344 start 1000\n"); tick(); waveform.done=true; tick();
    assert(pwm_config.active);
}
static void generic_configuration_checks(void) {
    ok("400 clear\n"); ok("401 add delay 8\n"); ok("402 start 1000\n"); tick();
    waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && !gpio_config.configured_mask && sync_pin==-1);
    ok("403 clear\n"); ok("404 add pulse 29 8\n");
    unsigned unchanged=safe_calls;
    error("405 start 1000\n","UNCONFIGURED");
    ok("406 clear\n"); ok("407 add wait_gpio 21 1 100\n");
    error("408 start 1000\n","UNCONFIGURED");
    assert(safe_calls==unchanged);
    ok("409 clear\n"); ok("410 add pwm 6000\n");
    error("411 start 1000\n","UNCONFIGURED");
    ok("412 pwm_config 22 500000\n"); ok("413 start 1000\n"); tick();
    waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && pwm_config.active && !gpio_config.configured_mask && sync_pin==-1);
    ok("414 abort\n"); assert(!pwm_config.active);
    error("415 gpio_config 30 push_pull 0\n","RANGE");
    error("416 gpio_config 0 push_pull 2\n","RANGE");
    error("417 gpio_config 0 unknown 0\n","RANGE");
    error("418 gpio_config 0 push_pull\n","BAD_REQUEST");
    error("419 sync_config 30\n","RANGE");
    assert(!gpio_config.configured_mask && sync_pin==-1);
}
static void pin_state_checks(void) {
    const uint32_t pp=1u << 29, od=1u << 10;
    ok("700 clear\n");
    error("701 add set 30 low\n","RANGE");
    error("702 add set 29 unknown\n","RANGE");
    error("703 add set 29\n","BAD_REQUEST");
    error("1703 start 1000\n","RANGE");
    ok("704 add set 29 high\n"); ok("705 add write 0 80 100 aa\n");
    ok("706 add delay 8\n"); ok("707 add set 29 floating\n");
    ok("708 start 1000 1\n"); tick(); waveform.done=true; tick();
    assert((gpio_config.current_levels & pp) && (gpio_config.current_dirs & pp));
    for (unsigned i=0;i<8 && sequence_status()->current_step<2;++i) tick();
    tick(); waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && sequence_status()->hold_final && sequence_status()->inserted_cycles==0);
    assert((gpio_config.current_levels & pp) && !(gpio_config.current_dirs & pp));
    ok("709 clear\n");
    assert((gpio_config.current_levels & pp) && !(gpio_config.current_dirs & pp));
    ok("710 add set 29 high\n"); ok("711 start 1000 0\n");
    tick(); waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && !sequence_status()->hold_final && !(gpio_config.current_levels & pp));
    assert(gpio_config.current_dirs & pp);
    ok("712 clear\n"); ok("713 add set 10 low\n"); ok("714 start 1000\n");
    tick(); waveform.done=true; tick();
    assert(sequence_status()->hold_final && (gpio_config.current_dirs & od) && !(gpio_config.current_levels & od));
    ok("715 clear\n"); ok("716 add set 10 high\n");
    ok("717 add set 29 floating\n"); ok("718 add pulse 29 8\n"); ok("719 start 1000\n");
    tick(); waveform.done=true; tick();
    assert(!(gpio_config.current_dirs & od) && !(gpio_config.current_levels & pp));
    assert((gpio_config.current_dirs & pp) && sequence_status()->inserted_cycles==16 && sequence_status()->programmed_cycles==24);
    ok("720 clear\n"); ok("721 add set 29 high\n"); ok("722 start 1000\n");
    tick(); waveform.done=true; tick();
    unsigned unchanged=safe_calls;
    ok("723 clear\n"); ok("724 add set 29 low\n"); ok("725 add write 0 80 100 aa\n");
    ok("726 add set 21 high\n"); error("727 start 1000\n","UNCONFIGURED");
    assert(safe_calls==unchanged && (gpio_config.current_levels & pp));
    ok("728 abort\n"); assert(!(gpio_config.current_levels & pp));
    ok("729 clear\n"); ok("730 add set 29 high\n"); ok("731 add write 0 80 100 aa\n");
    ok("732 start 1000\n"); tick(); nack=true; waveform.done=true; tick();
    for (unsigned i=0;i<8 && sequence_running();++i) tick();
    nack=false;
    assert((sequence_status()->state == SEQUENCE_ERROR) && !strcmp(sequence_status()->run_error,"NACK") && !(gpio_config.current_levels & pp));

    ok("733 clear\n"); ok("734 add pwm 6000\n"); ok("735 add delay 250\n");
    ok("736 add set 22 floating\n"); ok("737 add delay 250\n");
    ok("738 add pwm 1000\n"); ok("739 add delay 250\n"); ok("740 add set 22 low\n");
    ok("741 start 1000\n"); tick();
    assert(sequence_status()->programmed_cycles==750 && sequence_status()->inserted_cycles==0 && active_block->pwm_count==4);
    assert(active_block->pwm_segments[0].periods==1 && active_block->pwm_segments[0].mode==TIMING_PWM_CARRIER);
    assert(active_block->pwm_segments[1].periods==1 && active_block->pwm_segments[1].mode==TIMING_PWM_FLOATING);
    assert(active_block->pwm_segments[2].periods==1 && active_block->pwm_segments[2].duty_basis_points==1000);
    waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && !pwm_config.active && pwm_config.final_mode==TIMING_PWM_LOW);
    ok("742 clear\n"); ok("743 add set 22 floating\n"); ok("744 start 1000\n");
    tick(); waveform.done=true; tick();
    assert(!pwm_config.active && pwm_config.final_mode==TIMING_PWM_FLOATING);
    ok("745 clear\n"); ok("746 add delay 250\n"); ok("747 add pwm 6000\n");
    ok("748 start 1000\n"); tick();
    assert(active_block->pwm_segments[0].mode==TIMING_PWM_FLOATING && active_block->pwm_segments[0].periods==1);
    waveform.done=true; tick();
    assert(pwm_config.active && pwm_config.final_mode==TIMING_PWM_CARRIER);
    ok("749 clear\n"); ok("750 add set 22 high\n"); ok("751 start 1000\n");
    tick(); waveform.done=true; tick();
    assert(!pwm_config.active && pwm_config.final_mode==TIMING_PWM_HIGH);
    ok("752 clear\n"); ok("753 add set 22 floating\n"); ok("754 start 1000 0\n");
    tick(); waveform.done=true; tick();
    assert(!pwm_config.active && pwm_config.final_mode==TIMING_PWM_LOW);
    ok("755 status\n"); puts(response.data);
}

static void start_pulse_group(void) {
    ok("900 start 10000\n");
    tick();
    assert(waveform.active && i2c_bus_prepared() && event_ready[0]);
    assert(event_reference_pin==10 && event_rising && !event_reference_started);
    waveform.done=true; tick();
    assert(event_reference_started && transaction.result.written==0);
}

static void stage_background_delay(void) {
    ok("901 clear\n"); ok("902 add pulse 10 12500\n");
    ok("903 add write_at 0 80 12500 1000 aa\n");
    ok("904 add delay 1250\n");
}

static void immediate_turnaround_checks(void) {
    ok("910 abort\n"); ok("911 clear\n");
    ok("912 add write 0 80 1000 aa\n"); ok("913 add write 0 80 1000 bb\n");
    unsigned before=bus_starts, before_stops=stops;
    ok("914 start 10000\n");
    sequence_service(rx);
    assert(bus_starts==before+1 && stops==before_stops+1);
    /* The pass observing completion also starts the next ordinary write. */
    sequence_service(rx);
    assert(bus_starts==before+2 && stops==before_stops+2 && transaction.write[0]==0xbb);
    sequence_service(rx);
    assert(sequence_status()->state==SEQUENCE_DONE);

    ok("915 clear\n"); ok("916 add pulse 10 125000\n");
    ok("917 add write_at 0 80 97625 10000 23d400\n");
    ok("918 add write_at 0 80 105125 10000 230100\n");
    ok("919 start 10000\n"); tick();
    waveform.done=true; tick();
    uint64_t origin=event_origin/125;
    now=origin+781; timing_service();
    assert(event_released[0] && !event_ready[1]);
    /* Inject the measured 47us wire completion; no USB/main-loop pass follows. */
    now+=47; before_stops=stops; i2c_bus_service();
    assert(!transaction.busy && stops==before_stops+1);
    sequence_service(rx);
    assert(i2c_bus_prepared() && event_ready[1]);
    now=origin+841; timing_service(); i2c_bus_service(); sequence_service(rx);
    assert(sequence_status()->state==SEQUENCE_DONE && stops==before_stops+2);
    puts("same-pass ordinary turnaround and modeled 47us transfer / 60us scheduled spacing passed");
}

static void scheduled_checks(void) {
    ok("800 abort\n"); ok("801 clear\n");
    unsigned unchanged=bus_starts, safe_before=safe_calls;
    error("802 add write_at 0 80 7 1000 aa\n","RANGE");
    error("803 add write_at 0 80 12500 1000 000102030405060708090a0b0c0d0e0f10\n","RANGE");
    error("804 add sync_writes extra\n","BAD_REQUEST");
    ok("805 add write_at 0 80 12500 1000 aa\n");
    error("806 start 10000\n","RANGE");
    ok("807 clear\n"); ok("808 add pulse 10 12500\n");
    ok("809 add write_at 0 80 12500 1000 23d400\n");
    ok("810 add write_at 0 80 13000 1000 230100\n");
    error("811 start 10000\n","RANGE"); /* The first wire transaction cannot fit. */
    ok("812 clear\n"); ok("813 add pulse 10 12500\n");
    ok("814 add write_at 0 80 12500 1000 aa\n");
    ok("815 add delay 1250\n"); ok("816 add write_at 0 80 25000 1000 bb\n");
    error("817 start 10000\n","RANGE"); /* A different reference needs a join. */
    ok("818 clear\n"); ok("819 add pulse 10 12500\n");
    ok("820 add write_at 0 80 12500 1000 aa\n");
    ok("821 add write_at 0 80 25000 1000 bb\n");
    ok("822 add write_at 0 80 37500 1000 cc\n");
    error("823 start 10000\n","RANGE");
    ok("824 clear\n"); ok("825 add pulse 10 12500\n");
    ok("826 add write_at 0 80 12500 100 aa\n");
    error("827 start 10000\n","RANGE"); /* Preparation includes waiting for the reference. */
    ok("828 clear\n"); ok("829 add delay 1250000\n");
    ok("830 add pulse 10 12500\n"); ok("831 add write_at 0 80 12500 1000 aa\n");
    error("832 start 10000\n","RANGE"); /* Prefix work consumes the run budget too. */
    assert(bus_starts==unchanged && safe_calls==safe_before);

    /* Set up a held state, then restore it before the reference pulse. Earlier
     * same-pin pulses must finish before the reference engine is armed. */
    ok("838 clear\n"); ok("839 add set 29 high\n"); ok("840 start 1000\n");
    tick(); waveform.done=true; tick();
    ok("841 clear\n"); ok("842 add set 29 low\n"); ok("843 add pulse 10 1000\n");
    ok("844 add pulse 10 12500\n");
    ok("845 add write_at 0 80 12500 1000 23d400\n");
    ok("846 add write_at 1 80 25000 1000 230100\n");
    ok("847 add delay 1250\n"); ok("848 add sync_writes\n"); ok("849 add set 29 high\n");
    event_resources_busy=true;
    error("850 start 10000\n","RESOURCE_BUSY");
    event_resources_busy=false;
    error("851 start 100\n","RANGE");
    ok("852 start 10000\n"); tick();
    assert(waveform.active && !events_active);
    waveform.done=true; tick();
    tick(); tick();
    assert(waveform.active && event_ready[0] && event_reference_pin==10);
    waveform.done=true; tick();
    assert(sequence_status()->current_step==5 && event_reference_started);
    uint64_t origin=event_origin;
    unsigned old_stops=stops;
    tick();
    assert(waveform.active && transaction.busy && sequence_status()->current_step==5);
    reach_event(0); /* Foreground waveform continues while the next FIFO is prepared. */
    assert(stops==old_stops+1 && waveform.active && sequence_status()->current_step==5);
    tick(); assert(event_ready[1] && event_origin==origin);
    assert(event_offsets[0]==12500 && event_offsets[1]==25000);
    reach_event(1);
    assert(stops==old_stops+2 && waveform.active && !events_active && sequence_running());
    waveform.done=true; tick(); tick(); tick();
    assert(sequence_status()->current_step==7 && waveform.active);
    waveform.done=true; tick();
    assert(sequence_status()->state==SEQUENCE_DONE && (gpio_config.current_levels&(1u<<29)));

    /* An already-asserted held pin is also valid: Pulse still returns it idle. */
    ok("833 clear\n"); ok("834 add set 29 high\n"); ok("835 add pulse 29 12500\n");
    ok("836 add write_at 0 80 12500 1000 aa\n"); ok("837 start 10000\n");
    tick(); waveform.done=true; tick(); tick(); tick();
    assert(event_reference_pin==29 && !event_rising && event_ready[0]);
    waveform.done=true; tick(); reach_event(0);
    assert(sequence_status()->state==SEQUENCE_DONE && !(gpio_config.current_levels&(1u<<29)));
    ok("853 abort\n");

    /* SyncWrites is a real foreground fence, including when it is the next
     * group's reference: it joins the old group, never its own future event. */
    ok("854 clear\n"); ok("855 add pulse 10 12500\n");
    ok("856 add write_at 0 80 12500 1000 aa\n"); ok("857 add sync_writes\n");
    ok("858 add write_at 1 80 12500 1000 bb\n"); ok("859 add delay 1250\n");
    start_pulse_group(); tick();
    assert(sequence_status()->current_step==2 && !waveform.active);
    unsigned refs=software_references;
    reach_event(0);
    tick();
    assert(sequence_status()->current_step==4 && software_references==refs+1);
    tick(); assert(waveform.active);
    waveform.done=true; tick();
    assert(sequence_running() && sequence_status()->current_step==5 && events_active);
    reach_event(0); assert(sequence_status()->state==SEQUENCE_DONE);

    /* Native Write/Poll join an old group and create a CPU-observed origin only
     * after their own successful completion. Their successor FIFO is not preloaded. */
    ok("860 clear\n"); ok("861 add pulse 10 12500\n");
    ok("862 add write_at 0 80 12500 1000 aa\n"); ok("863 add delay 1250\n");
    ok("864 add write 0 80 1000 bb\n"); ok("865 add write_at 1 80 12500 1000 cc\n");
    ok("866 add poll 0 80 10 1 ff 80 0 1000 100\n");
    ok("867 add write_at 0 80 12500 1000 dd\n");
    start_pulse_group(); tick(); waveform.done=true; tick();
    unchanged=bus_starts; tick();
    assert(sequence_status()->current_step==3 && bus_starts==unchanged);
    reach_event(0);
    assert(sequence_status()->current_step==3 && bus_starts==unchanged+1 && !events_active);
    refs=software_references;
    tick();
    assert(sequence_status()->current_step==5 && software_references==refs+1);
    assert(event_reference_pin==-1 && event_reference_started);
    tick(); assert(event_ready[0]);
    fixture_value=0x80;
    reach_event(0);
    assert(sequence_status()->current_step==5 && !events_active);
    for (unsigned i=0;i<8 && sequence_status()->current_step==5;++i) tick();
    assert(sequence_status()->current_step==7 && software_references==refs+2);
    assert(event_reference_started);
    tick(); reach_event(0);
    assert(sequence_status()->state==SEQUENCE_DONE && sequence_status()->poll_attempts==1);

    const char *reference_commands[]={
        "870 add delay 1250\n", "870 add set 29 high\n",
        "870 add pwm 6000\n", "870 add sync_writes\n"
    };
    for (unsigned i=0;i<4;++i) {
        ok("868 abort\n"); ok("869 clear\n"); ok(reference_commands[i]);
        ok("871 add write_at 0 80 12500 1000 aa\n"); ok("872 start 10000\n");
        refs=software_references;
        tick();
        assert(event_reference_started && event_reference_pin==-1);
        if (i<3) {
            assert(waveform.active && event_ready[0] && event_waveform_cycles);
            uint64_t reference=event_origin;
            /* Hardware releases even if the CPU has not observed block completion. */
            now=(reference+event_offsets[0]+124)/125;
            timing_service();
            assert(event_released[0] && sequence_status()->current_step==0);
            assert(software_references==refs && event_origin==reference);
            waveform.done=true; tick();
        } else {
            assert(software_references==refs+1);
            reach_event(0);
        }
        assert(sequence_status()->state==SEQUENCE_DONE && !reserved_write_slots);
    }
    ok("873 abort\n");

    const char *wait_commands[]={
        "875 add wait_gpio 21 1 5000\n", "875 add wait_i2c 0 80 0 aa 5000\n",
        "875 add wait_spi aa 5000\n"
    };
    for (unsigned i=0;i<3;++i) {
        ok("874 clear\n"); ok(wait_commands[i]);
        ok("876 add write_at 0 80 12500 1000 aa\n"); ok("877 add delay 1250\n");
        ok("878 start 10000\n"); tick(); tick();
        assert(waveform.waiting && i2c_bus_prepared() && event_ready[0]);
        assert(event_reference_pin==sync_pin && event_rising && !event_reference_started);
        waveform.matched=true; tick(); /* Hardware latch, before CPU waveform completion. */
        assert(event_reference_started && sequence_status()->current_step==0);
        waveform.done=true; tick(); tick();
        assert(sequence_status()->current_step==2 && waveform.active && events_active);
        reach_event(0);
        assert(waveform.active && !events_active && sequence_running());
        waveform.done=true; tick(); assert(sequence_status()->state==SEQUENCE_DONE);
    }

    /* A background group can coexist with a later foreground matcher too. */
    ok("879 clear\n"); ok("880 add pulse 10 12500\n");
    ok("881 add write_at 0 80 12500 1000 aa\n"); ok("882 add wait_gpio 21 1 5000\n");
    start_pulse_group(); tick();
    assert(waveform.waiting && events_active);
    reach_event(0); assert(waveform.active && !events_active);
    waveform.matched=true; waveform.done=true; tick();
    assert(sequence_status()->state==SEQUENCE_DONE && sequence_status()->matched);

    /* Native references with offsets shorter than preparation fail, not write late. */
    ok("883 clear\n"); ok("884 add write 0 80 1000 aa\n");
    ok("885 add write_at 0 80 8 1000 bb\n"); ok("886 start 10000\n");
    tick();
    old_stops=stops;
    preparation_us=1; tick(); preparation_us=0;
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"MISSED_DEADLINE") &&
           sequence_status()->current_step==1 && stops==old_stops && !events_active);

    ok("887 clear\n"); ok("888 add pulse 10 12500\n");
    ok("889 add write_at 0 80 12500 1000 aa\n");
    ok("890 add write_at 1 80 25000 1000 bb\n"); ok("891 add delay 1250\n");
    start_pulse_group(); tick(); defer_preparation=true; reach_event(0);
    assert(waveform.active && !event_ready[1]);
    old_stops=stops; unchanged=bus_starts;
    reach_event(1); /* Controller preparation cannot finish before the cutoff. */
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"MISSED_DEADLINE") &&
           sequence_status()->current_step==2 && stops==old_stops);
    defer_preparation=false;
    assert(!waveform.active && !events_active && !transaction.busy);
    now+=1000; tick(); assert(bus_starts==unchanged && stops==old_stops);

    stage_background_delay(); start_pulse_group(); tick();
    nack=true; reach_event(0); nack=false;
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"NACK") &&
           sequence_status()->current_step==1 && !waveform.active && !events_active);
    start_pulse_group(); tick(); now+=1001; tick();
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"BUS_TIMEOUT") &&
           sequence_status()->current_step==1 && !waveform.active && !events_active);
    start_pulse_group(); tick(); waveform.error="TIMING_ERROR"; tick();
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"TIMING_ERROR") && !events_active);
    start_pulse_group(); tick(); old_stops=stops;
    ok("892 abort\n"); now+=1000; tick();
    assert(sequence_status()->state==SEQUENCE_ABORTED && stops==old_stops &&
           !transaction.busy && !events_active && !waveform.active);
    start_pulse_group(); tick();
    tud_cdc_line_state_cb(0,false,false); service_disconnect();
    now+=1000; tick();
    assert(sequence_status()->state==SEQUENCE_ABORTED && stops==old_stops && !events_active);

    ok("893 clear\n"); ok("894 add wait_gpio 21 1 5000\n");
    ok("895 add write_at 0 80 12500 1000 aa\n"); ok("896 start 10000\n");
    tick(); tick(); now+=1001; tick();
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"BUS_TIMEOUT") &&
           sequence_status()->current_step==1 && !waveform.active && !events_active);
    ok("898 clear\n"); ok("899 add wait_gpio 21 1 100\n");
    ok("905 add write_at 0 80 12500 1000 aa\n"); ok("906 start 10000\n");
    tick(); tick(); now+=101; tick();
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"WAIT_TIMEOUT") && !events_active);
    stage_background_delay(); ok("907 start 500\n");
    tick(); tick(); waveform.done=true; tick(); tick(); now+=501; tick();
    assert(sequence_status()->state==SEQUENCE_ERROR &&
           !strcmp(sequence_status()->run_error,"RUN_TIMEOUT") &&
           !waveform.active && !events_active && !transaction.busy);
    puts("command-relative writes: hardware/software references, concurrent foreground, joins, repeated groups, deadlines and cancellation passed");
}

/* Pump core 0 without advancing modeled time unless worker progress is needed. */
static void control_drain(bool worker) {
    for (unsigned i=0;i<10000;++i) {
        usb_control_task();
        if (!*incoming && !request.ready && !response_ready && !direct_pending)
            return;
        if (worker) tick();
    }
    assert(!"USB control task failed to drain");
}

static void control_reply(const char *id) {
    assert(cdc_length && cdc_capture[cdc_length-1]=='\n');
    assert(strchr(cdc_capture,'\n')==cdc_capture+cdc_length-1);
    assert(strstr(cdc_capture,id));
}

static void usb_control_checks(void) {
    /* Discard the previous helper-driven reply, but reset execution through
     * the same abort/clear commands used by a host. */
    response_ready=false;
    request_reset(&request);
    cdc_reset(RESPONSE_LIMIT); incoming="1998 abort\n"; control_drain(false);
    cdc_reset(RESPONSE_LIMIT); incoming="1999 clear\n"; control_drain(false);

    cdc_reset(7); incoming="2000 info\n"; control_drain(false);
    control_reply("\"id\":2000,");
    assert(strstr(cdc_capture,"\"ok\":true") && strstr(cdc_capture,"\"protocol\":3"));
    assert(strstr(cdc_capture,
        "\"limits\":{\"request_bytes\":4096,\"reply_bytes\":32768,\"steps\":64,"
        "\"transfer_bytes\":1024,\"routine_bytes\":256,\"payload_bytes\":8192,"
        "\"deadline_us\":60000000,\"tokens\":1024,\"pwm_words\":1024,"
        "\"scheduled_writes\":2,\"scheduled_write_bytes\":16}"));
    fputs(cdc_capture,stdout);

    cdc_reset(7); incoming="2001 invalid_command\n"; control_drain(false);
    control_reply("\"id\":2001,");
    assert(strstr(cdc_capture,"\"ok\":false") && strstr(cdc_capture,"BAD_REQUEST"));
    cdc_reset(7); incoming="2002 info\n"; control_drain(false);
    control_reply("\"id\":2002,");
    assert(strstr(cdc_capture,"\"ok\":true") && strstr(cdc_capture,"\"protocol\":3"));

    fixture_value=0x42;
    /* Hold the model's first service so even a one-byte read stays busy. */
    defer_preparation=true;
    cdc_reset(7); incoming="2003 transfer 0 80 1000 - 1\n";
    usb_control_task();
    assert(direct_pending && transaction.busy && !response_ready && !cdc_length);
    usb_control_task();
    assert(direct_pending && transaction.busy && !response_ready && !cdc_length);
    defer_preparation=false;
    control_drain(true);
    control_reply("\"id\":2003,");
    assert(strstr(cdc_capture,"\"ok\":true") &&
           strstr(cdc_capture,"\"data\":\"42\",\"written\":0,\"read\":1"));
    fputs(cdc_capture,stdout);

    cdc_reset(7); incoming="2004 clear\n"; control_drain(false);
    cdc_reset(7); incoming="2005 add pulse 29 125000\n"; control_drain(false);
    cdc_reset(0); incoming="2006 start 1000000\n"; usb_control_task(); tick();
    assert(sequence_running() && waveform.active && response_ready && !cdc_length);
    char start_reply[128];
    assert(response.length<sizeof(start_reply));
    memcpy(start_reply,response.data,response.length+1);
    incoming="2007 abort\n"; usb_control_task();
    assert(sequence_status()->state==SEQUENCE_ABORTED && !waveform.active &&
           !events_active && !transaction.busy && !pwm_config.active);
    assert(response_ready && request.ready && !cdc_length &&
           !strcmp(response.data,start_reply) && response.sent==0);
    cdc_accept=7; control_drain(false);
    size_t start_length=strlen(start_reply);
    assert(!strncmp(cdc_capture,start_reply,start_length));
    const char *abort_reply=cdc_capture+start_length;
    assert(strstr(start_reply,"\"id\":2006,") && strstr(start_reply,"\"state\":\"running\""));
    assert(strstr(abort_reply,"\"id\":2007,") && strstr(abort_reply,"\"state\":\"aborted\""));
    assert(cdc_length>start_length && cdc_capture[cdc_length-1]=='\n' &&
           strchr(abort_reply,'\n')==cdc_capture+cdc_length-1);
    fputs(cdc_capture,stdout);

    cdc_reset(0); incoming="2008 start 1000000\n"; usb_control_task(); tick();
    assert(sequence_running() && waveform.active && response_ready);
    incoming="2009 info\n";
    tud_cdc_line_state_cb(0,false,false); usb_control_task();
    assert(sequence_status()->state==SEQUENCE_ABORTED && !waveform.active &&
           !events_active && !transaction.busy && !pwm_config.active);
    assert(!response_ready && !direct_pending && !request.ready &&
           !request.length && !urgent_handled && !disconnected &&
           !response.length && !response.sent && !*incoming && !cdc_length);
    puts("USB control task checks passed: partial replies, malformed recovery, direct transfer, backpressured abort and disconnect");
}

int main(void) {
    if (!setjmp(boot_complete)) firmware_entry();
    clock_ok=true;
    ok("1 info\n"); assert(strstr(response.data,"\"protocol\":3"));
    generic_configuration_checks();
    error("2 bus_config 0 6 7 100000 0\n","RANGE");
    ok("3 bus_config 0 4 5 1500000 0\n");
    ok("4 bus_config 1 6 7 1500000 0\n");
    error("401 bus_config 0 4 5 1500001 0\n","RANGE");
    assert(configs[0].hz==1500000 && configs[1].hz==1500000);
    ok("402 bus_config 0 4 5 1000000 0\n");
    ok("403 bus_config 1 6 7 1000000 0\n");
    error("5 gpio_config 4 open_drain 1\n","PIN_CONFLICT");
    ok("6 gpio_config 10 open_drain 1\n");
    ok("601 gpio_config 29 push_pull 0\n");
    ok("602 sync_config 12\n");
    error("603 sync_config 10\n","PIN_CONFLICT");
    error("604 sync_config 22\n","PIN_CONFLICT");
    error("605 gpio_config 12 open_source 0\n","PIN_CONFLICT");
    error("606 bus_config 0 28 29 100000 0\n","PIN_CONFLICT");
    error("607 pwm_config 29 500000\n","PIN_CONFLICT");
    ok("608 gpio_config 10 open_source 0\n");
    assert((gpio_config.open_source_mask & (1u<<10)) && !(gpio_config.open_drain_mask & (1u<<10)));
    ok("609 gpio_config 10 open_drain 1\n");
    ok("610 sync_config 13\n"); assert(sync_pin==13);
    ok("611 sync_config 12\n");
    error("7 spi_config 4 14 15 0 0 1 10000000\n","PIN_CONFLICT");
    ok("101 spi_config 18 19 20 0 0 1 10000000\n");
    ok("8 clear\n");
    error("9 add pulse 29 7\n","RANGE");
    error("1009 start 1000\n","RANGE");
    error("901 add pulse 29\n","BAD_REQUEST");
    error("902 add pulse 30 8\n","RANGE");
    ok("10 add pulse 10 12500\n"); ok("11 add pulse 29 25\n");
    unsigned safe_before=safe_calls;
    error("903 start 1000000 2\n","RANGE"); assert(!sequence_running() && safe_calls==safe_before);
    ok("12 start 1000000\n"); tick(); waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && sequence_status()->inserted_cycles==8 && sequence_status()->programmed_cycles==12533);
    ok("13 clear\n");
    ok("14 add poll 0 80 10 1 80 80 10 1000 100\n");
    ok("15 start 10000\n");
    error("16 bus_config 0 8 9 100000 0\n","BUSY");
    fixture_value=0;
    for (unsigned i=0;i<15;++i) tick();
    assert(sequence_running() && sequence_status()->poll_attempts>=1 && sequence_status()->poll_length==1 && sequence_status()->poll_data[0]==0);
    fixture_value=0x80;
    for (unsigned i=0;i<20 && sequence_running();++i) tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && sequence_status()->poll_data[0]==0x80 && stops>=2);
    ok("17 status\n"); puts(response.data);
    ok("18 clear\n");
    ok("19 add poll 1 80 - 1 ff 42 10 50 10\n");
    ok("20 start 1000\n"); nack=true;
    for (unsigned i=0;i<100 && sequence_running();++i) tick();
    assert((sequence_status()->state == SEQUENCE_ERROR) && !strcmp(sequence_status()->run_error,"WAIT_TIMEOUT") && !strcmp(sequence_status()->last_poll_error,"NACK"));
    nack=false;
    ok("21 clear\n"); ok("22 add wait_i2c 1 80 0 1042 100\n"); ok("23 add pulse 29 8\n"); ok("24 start 1000\n");
    tick(); assert(waveform.waiting);
    ok("26 abort\n"); assert((sequence_status()->state == SEQUENCE_ABORTED) && !waveform.active);
    ok("27 abort\n"); assert((sequence_status()->state == SEQUENCE_ABORTED));
    ok("28 start 10\n"); now+=11; tick(); assert(!strcmp(sequence_status()->run_error,"RUN_TIMEOUT"));
    ok("29 start 1000\n"); tick(); waveform.matched=true; waveform.done=true; tick();
    assert((sequence_status()->state == SEQUENCE_DONE) && sequence_status()->matched);
    ok("30 start 1000\n"); tick(); now+=101; tick();
    assert(!strcmp(sequence_status()->run_error,"WAIT_TIMEOUT"));
    ok("31 clear\n"); ok("32 add pulse 29 8\n"); ok("33 start 1000\n"); tick();
    assert(sequence_running() && !waveform.waiting);
    waveform.done=true; tick(); assert((sequence_status()->state == SEQUENCE_DONE));
    ok("34 clear\n"); ok("35 add wait_i2c 0 80 0 - 100\n");
    ok("36 add pulse 29 8\n"); ok("37 add write 0 80 100 1042\n");
    ok("38 start 1000\n"); tick(); waveform.matched=true; waveform.done=true; tick();
    assert(sequence_running() && sequence_status()->current_step==2);
    tick(); tick(); assert(sequence_status()->state==SEQUENCE_DONE);
    ok("39 start 1000\n"); tick();
    request_reset(&request); incoming="40 abort\n";
    service_requests();
    assert((sequence_status()->state == SEQUENCE_ABORTED) && request.ready && !waveform.active);
    assert(strstr(response.data,"\"id\":39") && strstr(response.data,"running"));
    response_ready=false; service_requests();
    assert(!request.ready && strstr(response.data,"\"id\":40") && strstr(response.data,"aborted"));
    gpio_checks();
    pin_state_checks();
    scheduled_checks();
    immediate_turnaround_checks();
    pwm_checks();
    usb_control_checks();
    /* Keep a real start acknowledgment blocked while BOOTSEL is received. */
    cdc_reset(0); incoming="2010 start 1000000\n"; usb_control_task(); tick();
    assert(sequence_running() && waveform.active && response_ready);
    request_reset(&request); incoming="41 bootloader\n";
    service_requests();
    assert(reboot_deadline==now+100000 && request.ready);
    assert(!pwm_config.active);
    assert(sequence_status()->state==SEQUENCE_ABORTED && !waveform.active &&
           !events_active && !transaction.busy);
    assert(strstr(response.data,"\"id\":2010,") && !cdc_length);
    response_ready=false; service_requests();
    assert(rebooting && strstr(response.data,"\"rebooting\":true"));
    assert(strstr(response.data,"\"id\":41,"));
    now=reboot_deadline-1; usb_control_task();
    assert(!boot_calls && response_ready && !cdc_length);
    now=reboot_deadline; usb_control_task();
    assert(boot_calls==1 && response_ready && !cdc_length);
    puts("production firmware dispatch, held pin states, poll, deadlines, PWM lifecycle and abort checks passed");
    return 0;
}
