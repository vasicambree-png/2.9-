"""Replay saved display state and continuous BLE collection; no hardware."""
from pathlib import Path
import argparse
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def block(source, marker):
    match = re.search(r"(?m)^[ \t]*" + re.escape(marker) + r"[^;]*?\{", source)
    assert match, marker
    start = match.start()
    cursor = source.index("{", start)
    depth, end = 1, cursor + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vcvars", type=Path, required=True)
    parser.add_argument("--out", default="verification/ble_display_only_20261028/gate_host")
    args = parser.parse_args()
    assert args.vcvars.is_file()
    out = (ROOT / args.out).resolve()
    out.relative_to(ROOT)
    out.mkdir(parents=True, exist_ok=True)
    observer = (ROOT / "APP/observer.c").read_text(encoding="utf-8")
    uart = (ROOT / "APP/Usart3_task.c").read_text(encoding="utf-8")
    # Use real startup initializers: the saved state must not disable collection.
    boot_globals = "\n".join(re.search(
        r"(?m)^(?:(?:static|volatile)\s+)*uint(?:8|32)_t\s+" + name + r"\s*=\s*[^;]+;",
        observer).group() for name in ("g_scan_enable", "g_power_state", "g_power_state_seen"))
    queue = observer[observer.index("typedef struct\n{\n    uint8_t addr[6];"):
                     observer.index("/* ---- 可见性计数")]
    save = block(uart, "if (((uint8_t)param_data[0] == UI_MSG_PARAM_SAVE_OK ||")
    # Only committed home state and a settings-save result may change the gate.
    assert uart.count("observer_set_power_state(") == 2
    assert "observer_set_power_state(pData->UI_main.state);" in uart
    prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __HIGH_CODE
#define ADV_Q_DEPTH 64
#define ADV_Q_DATA_MAX 64
#define ADV_QUIET_RESTART_SEC 10
#define DEFAULT_DISCOVERY_MODE 3
#define DEFAULT_DISCOVERY_ACTIVE_SCAN 1
#define DEFAULT_DISCOVERY_WHITE_LIST 0
#define GAP_DEVICE_INIT_DONE_EVENT 1
#define GAP_DEVICE_INFO_EVENT 2
#define GAP_DEVICE_DISCOVERY_EVENT 3
#define SUCCESS 0
#define bleAlreadyInRequestedMode 0x11
#define BLE_LOG(...) ((void)0)
static unsigned diagnostics;
static void diag_print(const char *format, ...) { (void)format; ++diagnostics; }
#define PRINT(...) diag_print(__VA_ARGS__)
#define tmos_memcpy memcpy
typedef uint8_t bStatus_t;
typedef struct {
    struct { uint8_t opcode; } gap;
    struct { uint8_t addr[6], eventType, *pEvtData, dataLen; int8_t rssi; } deviceInfo;
} gapRoleEvent_t;
static uint8_t g_adv_seen, g_adv_q_max;
static uint32_t g_adv_rx_cnt;
static uint16_t g_adv_quiet_sec, g_adv_q_drop;
static uint16_t g_bind_ok_cnt, g_bind_reject_cnt;
static uint16_t g_scan_restart_cnt, g_scan_busy_cnt, g_scan_err_cnt;
static uint16_t s_last_ok, s_last_rej, s_last_drop, s_last_rst, s_last_busy, s_last_err;
static uint8_t s_last_qmax;
static uint32_t g_round_mask, g_round_last_sec, g_round_sec;
static unsigned starts, cancels, processed;
static unsigned store_attempts, store_fail;
static uint8_t power_state_store_save(uint8_t state)
{ assert(state<=1u); ++store_attempts; return (uint8_t)store_fail; }
static bStatus_t GAPRole_ObserverStartDiscovery(unsigned a,unsigned b,unsigned c)
{ (void)a; (void)b; (void)c; ++starts; return SUCCESS; }
static void sacn_process_packet(uint8_t *a,uint8_t b,uint8_t *c,uint16_t d,int e)
{ (void)a; (void)b; (void)c; (void)d; (void)e; ++processed; }
void SACN_DATA(uint8_t *,uint8_t,uint8_t *,uint16_t,int);
'''
    production = "\n".join(block(observer, marker) for marker in (
        "void observer_set_power_state(uint8_t state)",
        "uint8_t observer_values_visible(void)",
        "static void ObserverEventCB(gapRoleEvent_t *pEvent)",
        "void observer_adv_drain(uint16_t max_items)",
        "void observer_ble_stat_tick(void)",
        "void SACN_DATA(uint8_t *addr, uint8_t EVENT, uint8_t *data, uint16_t data_len, int Rssi)"))
    ui = r'''
#define UI_MSG_PARAM_SAVE_OK 1u
#define UI_MSG_PARAM_SAVE_FAIL 2u
typedef struct { uint8_t menu_rank, rank2_addr; struct { uint8_t state; } Menu_rank6; } test_page_t;
static void save_message(test_page_t *pData, unsigned code)
{ uint16_t param_data[1]={(uint16_t)code};
'''+save+"\n}\n"
    tests = r'''
int main(void)
{
    gapRoleEvent_t event={0};
    uint8_t addr[6]={1}, data[3]={2,3,4};
    test_page_t page={3,5,{0}};
    assert(g_scan_enable && !observer_values_visible());
    event.gap.opcode=GAP_DEVICE_INIT_DONE_EVENT; ObserverEventCB(&event);
    assert(starts==1);
    SACN_DATA(addr,0,data,3,-40); observer_adv_drain(16);
    assert(processed==1 && g_adv_rx_cnt==1);
    g_round_mask=7;
    SACN_DATA(addr,0,data,3,-40); /* Queue must survive a display-state change. */
    uint8_t head=g_adv_q_head,tail=g_adv_q_tail;
    observer_set_power_state(0);
    assert(g_scan_enable && !observer_values_visible() && store_attempts==1);
    assert(g_adv_q_head==head && g_adv_q_tail==tail && g_round_mask==7);
    for(unsigned i=0;i<20;i++) observer_set_power_state(0);
    assert(store_attempts==1 && starts==1 && !cancels);
    observer_adv_drain(16); assert(processed==2);
    event.gap.opcode=GAP_DEVICE_DISCOVERY_EVENT; ObserverEventCB(&event);
    assert(starts==2); /* OFF still restarts normal scanning. */
    g_adv_seen=0;
    for(unsigned i=0;i<10;i++) observer_ble_stat_tick();
    assert(starts==3 && g_scan_restart_cnt==1);
    observer_set_power_state(1);
    assert(observer_values_visible() && g_scan_enable && starts==3);
    unsigned attempted=store_attempts;
    observer_set_power_state(1); assert(store_attempts==attempted);
    save_message(&page,3); assert(observer_values_visible());
    save_message(&page,UI_MSG_PARAM_SAVE_OK);
    assert(!observer_values_visible() && g_scan_enable);
    page.Menu_rank6.state=1; store_fail=1;
    save_message(&page,UI_MSG_PARAM_SAVE_FAIL);
    assert(observer_values_visible() && g_scan_enable);
    attempted=store_attempts;
    for(unsigned i=0;i<20;i++) observer_set_power_state(1);
    assert(store_attempts==attempted); /* Failed persistence cannot hammer Flash. */
    page.rank2_addr=4;page.Menu_rank6.state=0;
    save_message(&page,UI_MSG_PARAM_SAVE_OK); assert(observer_values_visible());
    for(unsigned i=0;i<70;i++) {
        observer_set_power_state((uint8_t)(i&1u));
        SACN_DATA(addr,0,data,3,-40); observer_adv_drain(16);
        assert(g_scan_enable && observer_values_visible()==(i&1u));
    }
    assert(processed==72 && g_adv_rx_cnt==72 && g_round_mask==7 && !cancels);
    observer_set_power_state(0);attempted=store_attempts;
    observer_set_power_state(2);
    assert(!observer_values_visible() && g_scan_enable && store_attempts==attempted);
    puts("PASS: ON/OFF only controls visibility/persistence; BLE producer, drain, restart and watchdog continue; queued data/round preserved");
    return 0;
}
'''
    (out / "test.c").write_text(prefix+boot_globals+queue+production+ui+tests, encoding="utf-8")
    command = ('@echo off\ncall "'+str(args.vcvars)+'" >nul\n'
               'if errorlevel 1 exit /b 1\ncl /nologo /utf-8 /W4 test.c /Fe:test.exe\n'
               'if errorlevel 1 exit /b 1\ntest.exe\n')
    script = out / "run.cmd"
    script.write_text(command, encoding="utf-8")
    result = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=out,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (out / "test.log").write_bytes(result.stdout)
    print(result.stdout.decode("utf-8", errors="replace"))
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
