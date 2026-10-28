/* Real production drawing and driver. startup_paths.c is extracted by runner. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ui_host_module.c"
#include "screen_startup.h"
#include "screen_power.h"
#include "ui_redraw_policy.h"
u8g2_t u8g2;
device_t CH_com_buf[MAX_CH_NUM];
scan_binding g_binding_list[MAX_BINDING_NUM];
uint8_t g_binding_count, g_volt_err_count, g_name_err_count;
name_change_err_t g_name_err_list[NAME_CHK_ERR_NUM];
volatile uint8_t g_scan_mode;
void observer_set_collection_enabled(uint8_t state) { (void)state; }
uint8_t count_used_channels(void) { return 0u; }
#define START_IO_EVT 1u
#define START_DATA_EVT 2u
#define START_TIMER_EVT 8u
#define UI_IO_TICK_TICKS 16u
#define UI_HOLD_SEC 5u
#define FRAME_AGE_NEVER 0xffffffffu
#define MS1_TO_SYSTEM_TIME(ms) ((ms)*8u/5u)
#define RST_FLAG_MR 3u
#define CMD_LOG(...) ((void)0)
#define FRAME_LOG(...) ((void)0)
static screen_startup_t screen_startup;
static screen_power_t screen_power;
static ui_redraw_policy_t ui_redraw;
static uint8_t displayed_target, test_dirty, restore_pending;
static uint8_t Rx_sleep_flag=1u, usartTaskId, g_ui_reinit_req;
static uint32_t clock_now, g_frame_last_sec=FRAME_AGE_NEVER, dis_flag_cnt;
static uint32_t ui_reinit_since;
static unsigned event_count, request_pending, request_calls;
static unsigned cmd03_drop_cnt, cmd03_hold_cnt, cmd03_ignore_cnt;
static unsigned panel_off=1, on_count, off_count, transfer, data_bytes, frames;
static unsigned write_mask[48], new_frame_required=1, delay_ms;
static uint8_t ram[8064], command, args[2];
static unsigned arg_count, write_offset;
static uint32_t TMOS_GetSystemClock(void) { return clock_now; }
static uint8_t observer_cmd03_request(void)
{
    ++request_calls;
    if (request_pending) return 0u;
    request_pending=1u;
    return 1u;
}
static void TFT_IO_init(void) {}
static uint16_t usart_ProcessEvent(uint8_t id, uint16_t ev) { (void)id; return ev; }
static uint8_t TMOS_ProcessEventRegister(uint16_t (*cb)(uint8_t,uint16_t)) { (void)cb; return 1u; }
static void tmos_start_task(uint8_t id, unsigned ev, uint32_t ticks) { (void)id; (void)ev; (void)ticks; }
static void tmos_set_event(uint8_t id, unsigned ev) { (void)id; (void)ev; ++event_count; }
static uint8_t record_cad(u8x8_t *x, uint8_t msg, uint8_t count, void *ptr)
{
    unsigned i; (void)x;
    if (msg==U8X8_MSG_CAD_START_TRANSFER) { assert(!transfer); transfer=1; }
    else if (msg==U8X8_MSG_CAD_END_TRANSFER) { assert(transfer); transfer=0; }
    else if (msg==U8X8_MSG_CAD_SEND_CMD)
    {
        assert(transfer); command=count; arg_count=0;
        if (command==0x29)
        {
            if (new_frame_required) for(i=0;i<48;i++) assert(write_mask[i]);
            panel_off=0; ++on_count; new_frame_required=0;
        }
        if (command==0x28)
        {
            panel_off=1; ++off_count; new_frame_required=1;
            memset(write_mask,0,sizeof(write_mask));
        }
    }
    else if (msg==U8X8_MSG_CAD_SEND_ARG)
    {
        assert(transfer); if (arg_count<2) args[arg_count]=count; ++arg_count;
        if (command==0x2a && arg_count==2) assert(args[0]==0x17 && args[1]==0x24);
        if (command==0x2b && arg_count==2) { assert(args[1]==0xbf); write_offset=args[0]*42u; }
    }
    else if (msg==U8X8_MSG_CAD_SEND_DATA)
    {
        assert(transfer && command==0x2c && count==168);
        assert(write_offset+count<=sizeof(ram));
        memcpy(ram+write_offset,ptr,count);
        write_mask[write_offset/168]=1u; write_offset+=count; data_bytes+=count;
        if ((data_bytes%8064u)==0u) ++frames;
    }
    return 1u;
}
static uint8_t u8x8_byte_CH584M_spi(u8x8_t *x,uint8_t msg,uint8_t n,void *p)
{ (void)x; (void)msg; (void)n; (void)p; return 1u; }
static uint8_t u8x8_CH584M_gpio_and_delay(u8x8_t *x,uint8_t msg,uint8_t n,void *p)
{ (void)x; (void)p; if(msg==U8X8_MSG_DELAY_MILLI) delay_ms+=n; return 1u; }
#include "startup_paths.c"
static void draw_step(void) { clock_now+=80u; step_display(); assert(!transfer); }
static void boot_prepare(unsigned enabled,unsigned cause)
{
    memset(&screen_startup,0,sizeof(screen_startup)); memset(&screen_power,0,sizeof(screen_power));
    memset(&ui_redraw,0,sizeof(ui_redraw)); memset(&Data_list1,0,sizeof(Data_list1));
    memset(write_mask,0,sizeof(write_mask)); memset(ram,0x55,sizeof(ram));
    displayed_target=test_dirty=restore_pending=0u;
    Rx_sleep_flag=1u; g_ui_reinit_req=0u; dis_flag_cnt=0u;
    g_frame_last_sec=FRAME_AGE_NEVER;
    request_pending=request_calls=cmd03_drop_cnt=cmd03_hold_cnt=cmd03_ignore_cnt=0u;
    ui_show_msg(UI_MSG_NONE); clock_now=0u; panel_off=1u;
    data_bytes=frames=on_count=off_count=delay_ms=0u; new_frame_required=1u;
    screen_reset_display_init((uint8_t)enabled,(uint8_t)cause);
    u8g2Init(&u8g2); usart_task();
    assert(panel_off && data_bytes==0 && on_count==0 && delay_ms>=255u);
}
static void boot(unsigned enabled,unsigned cause)
{ boot_prepare(enabled,cause); draw_step(); }
static void cache_page(unsigned rank_value,unsigned page_value,unsigned timeout)
{
    Data_list1.menu_rank=(uint8_t)rank_value; Data_list1.UI_main.chu_num1=(uint16_t)page_value;
    Data_list1.Type=TPYE_MG; Data_list1.Menu_rank6.time_light=(uint16_t)timeout;
    accept_page(rank_value); dis_flag_cnt=1u;
}
static void page(unsigned rank_value,unsigned page_value,unsigned timeout)
{ cache_page(rank_value,page_value,timeout); draw_step(); }
/* Real frame guards/checksum/CMD03 and successful UART dispatch; only sensor collection is a stub.
 * Verify collection traffic cannot open/close, redraw or replace any target. */
static void sensor_read_traffic(void)
{
    uint8_t good[]={0xaa,0xee,0,1,3,3,0x0a};
    uint8_t bad_sum[]={0xaa,0xee,0,1,3,0,0x0a};
    uint8_t bad_cmd_len[]={0xaa,0xee,0,2,3,0,3,0x0a};
    uint8_t bad_frame_len[]={0xaa,0xee,0,2,3,3,0x0a};
    uint8_t bad_header[]={0xaa,0xef,0,1,3,3,0x0a};
    uint8_t bad_tail[]={0xaa,0xee,0,1,3,3,0};
    uint8_t saved_ram[8064];
    screen_startup_t saved_startup=screen_startup;
    screen_power_t saved_power=screen_power;
    unsigned old_frames=frames,old_on=on_count,old_off=off_count;
    unsigned old_bytes=data_bytes,old_pending=request_pending;
    unsigned old_calls=request_calls,old_hold=cmd03_hold_cnt;
    unsigned old_ignore=cmd03_ignore_cnt,old_drop=cmd03_drop_cnt;
    uint32_t old_age=g_frame_last_sec;
    uint8_t old_msg=g_ui_msg,old_target=displayed_target,old_test_dirty=test_dirty;
    uint8_t old_restore=restore_pending,old_rx_sleep=Rx_sleep_flag;
    unsigned i;
    memcpy(saved_ram,ram,sizeof(ram));
    assert(dispatch_cmd03_frame(NULL,7u,&Data_list1)==1u);
    assert(dispatch_cmd03_frame(good,7u,NULL)==1u);
    assert(dispatch_cmd03_frame(good,6u,&Data_list1)==1u);
    assert(dispatch_cmd03_frame(bad_header,sizeof(bad_header),&Data_list1)==1u);
    assert(dispatch_cmd03_frame(bad_tail,sizeof(bad_tail),&Data_list1)==1u);
    assert(dispatch_cmd03_frame(bad_frame_len,sizeof(bad_frame_len),&Data_list1)==2u);
    assert(dispatch_cmd03_frame(bad_sum,sizeof(bad_sum),&Data_list1)==0u);
    assert(dispatch_cmd03_frame(bad_cmd_len,sizeof(bad_cmd_len),&Data_list1)==0u);
    assert(request_calls==old_calls && cmd03_drop_cnt==old_drop+2u);
    for(i=0;i<10u;i++)
    {
        assert(dispatch_cmd03_frame(good,sizeof(good),&Data_list1)==0u);
        draw_step();
    }
    assert(request_calls==old_calls+10u && request_pending);
    assert(cmd03_hold_cnt==old_hold+(old_pending?0u:1u));
    assert(cmd03_ignore_cnt==old_ignore+(old_pending?10u:9u));
    request_pending=0u; /* Sensor backend has completed its previous round. */
    assert(dispatch_cmd03_frame(good,sizeof(good),&Data_list1)==0u);
    draw_step();
    assert(request_calls==old_calls+11u && request_pending);
    assert(cmd03_hold_cnt==old_hold+(old_pending?1u:2u));
    assert(frames==old_frames && on_count==old_on && off_count==old_off && data_bytes==old_bytes);
    assert(!memcmp(saved_ram,ram,sizeof(ram)));
    assert(!memcmp(&saved_startup,&screen_startup,sizeof(screen_startup)));
    assert(!memcmp(&saved_power,&screen_power,sizeof(screen_power)));
    assert(g_frame_last_sec==old_age && g_ui_msg==old_msg && displayed_target==old_target);
    assert(test_dirty==old_test_dirty && restore_pending==old_restore && Rx_sleep_flag==old_rx_sleep);
}
static void expire_msg(void)
{ unsigned n; for(n=0;n<10u && g_ui_msg;n++) if(ui_msg_tick_sec()) dis_flag_cnt=1u; draw_step(); assert(!g_ui_msg); }
static void assert_nonblank(void)
{ unsigned i,n=0; for(i=0;i<sizeof(ram);i++) n+=(ram[i]!=0u); assert(n>0u && n<sizeof(ram)); }
int main(void)
{
    uint8_t artwork[8064], home[8064]; unsigned enabled,cause,n,previous,old_delay;
    for(enabled=0;enabled<2;enabled++) for(cause=0;cause<8;cause++)
    {
        boot(enabled,cause);
        if(enabled && cause==RST_FLAG_MR) assert(!panel_off && frames==1u && on_count==1u);
        else assert(panel_off && !on_count && !frames);
        sensor_read_traffic();
    }
    boot(1u,1u);
    for(n=0;n<3600u;n++) {clock_now+=1600u; dis_flag_cnt=1u; step_display();}
    assert(panel_off && frames==0u && g_frame_last_sec==FRAME_AGE_NEVER);
    accept_msg(0u); dis_flag_cnt=1u; draw_step(); assert(panel_off && frames==0u);
    accept_msg(UI_MSG_PARAM_SAVE_OK); dis_flag_cnt=1u; draw_step();
    assert(!panel_off); assert_nonblank(); expire_msg(); assert(panel_off);
    boot(1u,RST_FLAG_MR); memcpy(artwork,ram,sizeof(ram)); assert_nonblank();
    /* Shutdown cuts screen power. A later single-key wake starts a fresh
     * MCU and sends msg8, with no valid page or rank5 required. */
    boot(1u,1u); page(1u,1u,1u); draw_step();
    accept_msg(UI_MSG_SHUTTING_DOWN); dis_flag_cnt=1u; draw_step();
    assert(!panel_off && displayed_target==SCREEN_TARGET_MSG);
    boot(1u,1u);
    accept_msg(UI_MSG_POWER_OFF); dis_flag_cnt=1u; draw_step();
    assert(!panel_off && displayed_target==SCREEN_TARGET_MSG);
    assert(screen_startup.resume_intro_armed && !screen_startup.valid_page);
    assert_nonblank(); memcpy(home,ram,sizeof(ram));
    for(n=0;n<12u;n++)
    {
        clock_now+=1600u;
        assert(!ui_msg_tick_sec());
        accept_msg(UI_MSG_POWER_OFF); dis_flag_cnt=1u; draw_step();
        assert(g_ui_msg==UI_MSG_POWER_OFF && !panel_off);
        assert(displayed_target==SCREEN_TARGET_MSG && !memcmp(home,ram,sizeof(ram)));
    }
    sensor_read_traffic();
    assert(!panel_off && g_ui_msg==UI_MSG_POWER_OFF);
    /* A cold MCU may receive only background sensor requests, then three
     * clear-message frames and a home on a real K1+K3 wake (no rank 5). */
    boot(1u,1u); sensor_read_traffic();
    for(n=0;n<3u;n++)
    {
        accept_msg(UI_MSG_NONE); dis_flag_cnt=1u; draw_step();
        assert(panel_off && !frames && !screen_startup.intro_pending);
    }
    cache_page(1u,1u,0u);
    assert(screen_startup.intro_pending && screen_startup.valid_page);
    draw_step();
    assert(frames==1u && !panel_off && !memcmp(artwork,ram,sizeof(ram)));
    draw_step(); assert(frames==2u && displayed_target==SCREEN_TARGET_PAGE);
    assert(memcmp(artwork,ram,sizeof(ram))); assert_nonblank();
    /* With the MCU kept powered, either soft-off prompt arms the next
     * ordinary page. Clearing the prompt alone must not draw the intro. */
    for(n=UI_MSG_POWER_OFF;n<=UI_MSG_SHUTTING_DOWN;n++)
    {
        unsigned clear;
        boot(1u,1u); page(1u,1u,1u); draw_step();
        assert(displayed_target==SCREEN_TARGET_PAGE);
        accept_msg(n); dis_flag_cnt=1u; draw_step();
        assert(g_ui_msg==n && screen_startup.resume_intro_armed);
        assert(!screen_startup.intro_pending && displayed_target==SCREEN_TARGET_MSG);
        sensor_read_traffic();
        cache_page(1u,1u,1u); draw_step();
        assert(g_ui_msg==n && screen_startup.resume_intro_armed);
        assert(!screen_startup.intro_pending && displayed_target==SCREEN_TARGET_MSG);
        for(clear=0;clear<3u;clear++)
        {
            accept_msg(UI_MSG_NONE); dis_flag_cnt=1u; draw_step();
            assert(!g_ui_msg && screen_startup.resume_intro_armed);
            assert(panel_off && !screen_startup.intro_pending);
            assert(displayed_target!=SCREEN_TARGET_TEST);
        }
        cache_page(1u,1u,1u); previous=frames;
        assert(screen_startup.intro_pending && !screen_startup.resume_intro_armed);
        draw_step();
        assert(frames==previous+1u && !memcmp(artwork,ram,sizeof(ram)));
        draw_step(); assert(displayed_target==SCREEN_TARGET_PAGE);
        assert(!screen_startup.intro_pending && !screen_startup.resume_intro_armed);
        /* Identical telemetry after timeout stays dark. Navigation that
         * really wakes the panel gets an intro before its cached page. */
        clock_now+=1600u; draw_step(); assert(panel_off);
        previous=frames; page(1u,1u,1u);
        assert(panel_off && frames==previous && !screen_startup.intro_pending);
        page(1u,2u,1u);
        assert(!panel_off && frames==previous+1u && displayed_target==SCREEN_TARGET_TEST);
        assert(!memcmp(artwork,ram,sizeof(ram)));
        draw_step(); assert(frames==previous+2u && displayed_target==SCREEN_TARGET_PAGE);
        assert(memcmp(artwork,ram,sizeof(ram)));
        previous=frames; page(1u,1u,1u);
        assert(frames==previous+1u && displayed_target==SCREEN_TARGET_PAGE);
        assert(!screen_startup.intro_pending && memcmp(artwork,ram,sizeof(ram)));
    }
    /* S2 and an ordinary page may be parsed before the first display event.
     * The complete intro must reach panel RAM once before the cached home. */
    boot_prepare(1u,RST_FLAG_MR);
    cache_page(1u,1u,0u);
    assert(panel_off && !frames && screen_startup.intro_pending);
    assert(screen_startup.valid_page && !screen_startup.test);
    draw_step();
    assert(!panel_off && frames==1u && !memcmp(artwork,ram,sizeof(ram)));
    assert(!screen_startup.intro_pending && displayed_target==SCREEN_TARGET_TEST);
    draw_step();
    assert(frames==2u && displayed_target==SCREEN_TARGET_PAGE);
    assert(memcmp(artwork,ram,sizeof(ram))); assert_nonblank();
    /* Controller startup/reinit is an explicit display command; sensor-only
     * traffic remains dark until rank 5, even with no ordinary page yet. */
    boot(1u,1u); sensor_read_traffic();
    accept_rank5();
    assert(screen_startup.intro_pending && test_dirty && g_ui_reinit_req);
    step_reinit(); assert(panel_off); draw_step();
    assert(!panel_off && frames==1u && !memcmp(artwork,ram,sizeof(ram)));
    assert(!screen_startup.intro_pending && screen_startup.test);
    sensor_read_traffic();
    page(1u,1u,0u);
    assert(displayed_target==SCREEN_TARGET_PAGE && memcmp(artwork,ram,sizeof(ram)));
    /* rank 5 followed immediately by home in the same UART batch must not
     * let that home cancel the intro before any frame is transmitted. */
    boot(1u,1u);
    accept_rank5(); cache_page(1u,2u,2u);
    assert(panel_off && !frames && screen_startup.intro_pending);
    assert(!screen_startup.test && screen_startup.valid_page);
    {
        uint8_t read[]={0xaa,0xee,0,1,3,3,0x0a};
        assert(dispatch_cmd03_frame(read,sizeof(read),&Data_list1)==0u);
        assert(screen_startup.intro_pending && !frames && panel_off);
    }
    step_reinit();
    assert(panel_off && !frames); draw_step();
    assert(frames==1u && !panel_off && !memcmp(artwork,ram,sizeof(ram)));
    draw_step();
    assert(frames==2u && displayed_target==SCREEN_TARGET_PAGE);
    assert(memcmp(artwork,ram,sizeof(ram))); assert_nonblank();
    /* Git baseline: rank5 must not clear a persistent message, even when
     * parsed with a home before drawing. Explicit msg0 releases the prompt. */
    for(n=UI_MSG_POWER_OFF;n<=UI_MSG_SHUTTING_DOWN;n++)
    {
        accept_msg(n); dis_flag_cnt=1u; draw_step();
        assert(g_ui_msg==n); memcpy(home,ram,sizeof(ram));
        accept_rank5(); cache_page(1u,1u,0u);
        assert(g_ui_msg==n && screen_startup.intro_pending);
        draw_step();
        assert(!panel_off && displayed_target==SCREEN_TARGET_MSG);
        assert(!memcmp(home,ram,sizeof(ram)));
        assert(!ui_msg_tick_sec() && g_ui_msg==n);
        accept_msg(UI_MSG_NONE); dis_flag_cnt=1u;
        draw_step(); assert(!memcmp(artwork,ram,sizeof(ram)));
        draw_step(); assert(displayed_target==SCREEN_TARGET_PAGE);
    }
    boot(1u,RST_FLAG_MR);
    sensor_read_traffic();
    for(n=0;n<3600u;n++) {clock_now+=1600u; dis_flag_cnt=1u; step_display();}
    assert(frames==1u && !memcmp(artwork,ram,sizeof(ram)) && !panel_off);
    accept_msg(0u); dis_flag_cnt=1u; draw_step(); assert(frames==1u);
    accept_msg(UI_MSG_PARAM_SAVE_OK); dis_flag_cnt=1u; draw_step();
    assert(memcmp(artwork,ram,sizeof(ram))); expire_msg();
    assert(!panel_off && !memcmp(artwork,ram,sizeof(ram)));
    accept_rank5(); step_reinit(); assert(panel_off); draw_step();
    assert(!panel_off && !memcmp(artwork,ram,sizeof(ram)));
    page(1u,1u,2u); assert(!screen_startup.test && screen_startup.valid_page);
    assert(!panel_off && memcmp(artwork,ram,sizeof(ram))); assert_nonblank(); memcpy(home,ram,sizeof(ram));
    sensor_read_traffic();
    previous=frames; Data_list1.menu_rank=5u; UI_Control(&Data_list1);
    assert(frames==previous && !memcmp(home,ram,sizeof(ram))); Data_list1.menu_rank=1u;
    accept_rank5(); cache_page(1u,2u,2u);
    previous=frames; old_delay=delay_ms; draw_step();
    assert(frames==previous+1u && !memcmp(artwork,ram,sizeof(ram)));
    assert(!g_ui_reinit_req && delay_ms>=old_delay+255u);
    old_delay=delay_ms; step_reinit(); draw_step();
    assert(delay_ms==old_delay);
    assert(!panel_off && displayed_target==SCREEN_TARGET_PAGE && memcmp(artwork,ram,sizeof(ram)));
    memcpy(home,ram,sizeof(ram));
    clock_now+=3200u; draw_step(); assert(panel_off);
    sensor_read_traffic(); assert(panel_off);
    previous=frames; page(1u,2u,2u); assert(panel_off && frames==previous);
    accept_rank5(); step_reinit(); assert(panel_off && frames==previous);
    draw_step(); assert(!panel_off && !memcmp(artwork,ram,sizeof(ram)));
    page(1u,1u,2u); assert(!panel_off && displayed_target==SCREEN_TARGET_PAGE);
    accept_rank5(); clock_now+=8001u; old_delay=delay_ms; step_reinit(); assert(!g_ui_reinit_req && delay_ms==old_delay);
    page(1u,1u,0u); draw_step();
    assert(displayed_target==SCREEN_TARGET_PAGE); memcpy(home,ram,sizeof(ram));
    accept_msg(UI_MSG_PARAM_SAVE_OK); dis_flag_cnt=1u; draw_step();
    g_frame_last_sec=100u; expire_msg(); assert(!panel_off && !memcmp(home,ram,sizeof(ram)));
    /* Page traffic during a prompt updates the cache without covering it. */
    accept_msg(UI_MSG_PARAM_SAVE_OK); dis_flag_cnt=1u; draw_step();
    previous=frames; page(1u,2u,0u); assert(g_ui_msg && frames==previous+1u);
    assert(memcmp(home,ram,sizeof(ram))); expire_msg();
    assert(!panel_off && memcmp(home,ram,sizeof(ram))); assert_nonblank();
    /* A prompt is visible after page timeout; ending it respects timeout. */
    page(1u,1u,1u); clock_now+=1600u; draw_step(); assert(panel_off);
    accept_msg(UI_MSG_PARAM_SAVE_OK); dis_flag_cnt=1u; draw_step(); assert(!panel_off);
    clock_now+=1600u; expire_msg(); assert(panel_off);
    /* A physical S2 reset restarts firmware and clears persistent prompts. */
    for(n=UI_MSG_POWER_OFF;n<=UI_MSG_SHUTTING_DOWN;n++)
    {
        accept_msg(n); dis_flag_cnt=1u; draw_step(); assert(g_ui_msg==n);
        sensor_read_traffic();
        boot(1u,RST_FLAG_MR);
        assert(!g_ui_msg && !memcmp(artwork,ram,sizeof(ram)));
    }
    boot(1u,1u);
    page(3u,1u,2u); draw_step(); clock_now+=160000u; draw_step(); assert(!panel_off);
    screen_power_note_page(&screen_power,0xfffffff0u,1,0,0,1,0,1);
    assert(!screen_power_is_off(&screen_power,0x62fu)); assert(screen_power_is_off(&screen_power,0x630u));
    puts("PASS: real UI_Control/u8g2/ST7305; all reset causes, hardware S2, no-frame hold, message restore, page takeover, timeout and reinit.");
    puts("PASS: rank5 from dark shows intro; rank5/home and S2/home before first draw transmit complete intro before latest home.");
    puts("PASS: cold first page, soft-off resume and navigation wake show intro; clear-message/same-page telemetry stay dark, lit navigation does not replay intro.");
    puts("PASS: production CMD03 parser and UART dispatch; sensor requests enqueue no redraw/navigation/event and preserve dark/test/page/prompt display.");
    puts("PASS: complete image before on; repeated artwork identical. Host replay only; no physical reset or sensor backend validation.");
    puts("PASS: shutdown then cold single-key msg8 displays a persistent nonblank prompt without any normal page/rank5; background reads preserve it.");
    return 0;
}
