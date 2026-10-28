"""Black-box regression tests for the production logger's field failure workload.

The executable compiles the complete current LoraTrace.c. Peripheral access is
not mocked because this module only stores diagnostics. The output gate is
extracted verbatim from LoraPro.c. Each scenario runs in a fresh process so no
test has to know or reset the production queue's internal representation.
Host success does not establish RF delivery or hardware timing.
"""
from pathlib import Path
import hashlib
import json
import re
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / "priority-trace-tests"
VCVARS = Path(r"D:\visual studio\visual studio2026\VC\Auxiliary\Build\vcvars64.bat")

HARNESS = r'''
#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <windows.h>
static char output[1048576];
static size_t used;
static int capture_printf(const char *fmt, ...) {
    int n; va_list ap; va_start(ap,fmt);
    n=vsnprintf(output+used,sizeof(output)-used,fmt,ap);va_end(ap);
    if(n<0 || (size_t)n>=sizeof(output)-used) exit(90);
    used+=(size_t)n;return n;
}
#define printf capture_printf
#include "production_trace.c"
#undef printf
static uint32_t Rt_tick;
volatile uint32_t g_cad_cnt, g_rx_hit_cnt;
static struct {uint8_t partition, Host_number;} Setup;
static struct {uint8_t Forward_flag;} Lo_data;
static struct {uint8_t lcd_open;} UI;
static uint8_t Lora_Tx_state, Lora_Rx_state, send_adrr_cnt, s_same_addr_retry;
static uint8_t s_report_pending, g_slp_keep_awake;
#define Tx_None 0
PRODUCTION_HELPERS

#define CHECK(c,msg) do{if(!(c)){fprintf(stderr,"FAIL %s line %d\n",msg,__LINE__);exit(13);}}while(0)
static void clear_output(void){memset(output,0,sizeof(output));used=0;}
static LoraTraceContext context(void){LoraTraceContext c={0};c.rt=100;c.part=2;c.host=5;c.attempt=1;return c;}
static void service(LoraTraceContext *c,uint8_t allow){
    size_t before=used,i;unsigned lines=0;
    LoraTrace_Service(c,allow);
    for(i=before;i<used;i++)if(output[i]=='\n')lines++;
    CHECK(lines<=1U,"at most one output line per Service call");
    CHECK(allow || used==before,"forbidden output remains silent");
}
static void drain(LoraTraceContext *c){unsigned i;for(i=0;i<256U;i++)service(c,1U);}
static void frame(uint8_t *p,uint8_t src,uint8_t dst,uint8_t origin){
    unsigned i;memset(p,0,95U);p[0]=2;p[1]=dst;p[2]=2;p[3]=src;p[9]=0x0A;p[12]=2;p[13]=origin;
    for(i=14U;i<94U;i++)p[i]=(uint8_t)i;
    p[14]=6;p[15]=0x14;p[16]=0xB4;
}
static void storm(void){
    LoraTraceContext c=context();RadioDiagSnapshot s={0};uint8_t a[95],ack[95];unsigned round,attempt;
    frame(a,5,2,5);frame(ack,2,122,5);
    c.rt=100;LoraTrace_Record(&c,LT_READY,17,a+14,80,0);
    for(round=0;round<10U;round++){
        for(attempt=1;attempt<=9U;attempt++){
            c.rt=110U+round*520U+(attempt-1U)*20U;c.attempt=(uint8_t)attempt;c.tx=3;c.rx=7;
            a[1]=(uint8_t)(2U+(attempt-1U)/3U);
            LoraTrace_Record(&c,LT_SELF_TX,17,a,95,2000);
            if(attempt==1U){s.rt=c.rt;s.status=0x22;s.read_flags=7;LoraTrace_Radio(&c,RF_PRE_TX,17,&s);}
            c.rt+=2U;c.rx=2;LoraTrace_Record(&c,LT_TX_EXIT,17,a,95,1);
            if(attempt==1U){s.rt=c.rt;s.status=0x2C;s.irq=1;LoraTrace_Radio(&c,RF_TX_EXIT,17,&s);}
            c.rt+=1U;c.tx=4;c.rx=3;
            if(attempt==1U){s.rt=c.rt;s.status=0x52;s.irq=0;LoraTrace_Radio(&c,RF_RX_OPEN,17,&s);}
            c.rt+=9U;LoraTrace_Record(&c,LT_SELF_TIMEOUT,17,a,95,9);
            if(attempt==1U){s.rt=c.rt;LoraTrace_Radio(&c,RF_CONFIRM_TIMEOUT,17,&s);}
            service(&c,0U);
        }
    }
    c.rt=5500;c.tx=0;c.rx=0;c.attempt=1;
    LoraTrace_Record(&c,LT_HEAL_BEGIN,17,NULL,0,1);
    s.rt=c.rt;s.busy_to=180;s.status=0x52;LoraTrace_Radio(&c,RF_HEAL_BEFORE,17,&s);
    c.rt=5502;LoraTrace_Record(&c,LT_HEAL_END,17,NULL,0,0);
    s.rt=c.rt;s.status=0x22;LoraTrace_Radio(&c,RF_HEAL_AFTER,17,&s);
    c.rt=5800;c.tx=3;LoraTrace_Record(&c,LT_SELF_TX,17,a,95,2000);
    c.rt=5803;c.tx=4;c.rx=4;LoraTrace_Record(&c,LT_RX,0,ack,95,2);
    c.tx=0;LoraTrace_Record(&c,LT_SELF_OK_A,17,ack,95,3);
    CHECK(used==0,"Record/Radio never print during ten-minute workload");
    c.rt=6000;c.tx=0;c.rx=0;drain(&c);
    puts(output);
}
static void correlation(void){
    LoraTraceContext c=context();uint8_t a[95],b[95],changed[95];
    frame(a,5,2,5);memcpy(b,a,sizeof(a));b[1]=122;b[3]=2;b[8]=2;b[10]=80;b[11]=161;b[94]=9;
    LoraTrace_Record(&c,LT_SELF_TX,7,a,95,2000);
    LoraTrace_Record(&c,LT_FORWARD_TX,0,b,95,12);
    LoraTrace_Record(&c,LT_READY,8,a+14,80,0);
    LoraTrace_Record(&c,LT_READY,9,a+14,80,0);
    memcpy(changed,b,sizeof(b));changed[20]^=1;LoraTrace_Record(&c,LT_RX,0,changed,95,2);
    drain(&c);puts(output);
}
static void bounds(void){
    LoraTraceContext c=context();SYSTEM_INFO si;uint8_t *pages,*p;unsigned n;
    GetSystemInfo(&si);pages=(uint8_t*)VirtualAlloc(NULL,si.dwPageSize*2U,MEM_RESERVE,PAGE_NOACCESS);
    CHECK(pages!=NULL,"guard allocation");
    CHECK(VirtualAlloc(pages,si.dwPageSize,MEM_COMMIT,PAGE_READWRITE)!=NULL,"guard commit");
    for(n=0;n<=14U;n++){
        p=pages+si.dwPageSize-n;if(n)memset(p,0xA5,n);
        LoraTrace_Record(&c,LT_RX,n+1U,p,(uint16_t)n,0);drain(&c);
    }
    p=pages+si.dwPageSize-95;memset(p,0,95);p[9]=0x0A;LoraTrace_Record(&c,LT_RX,20,p,95,0);drain(&c);
    LoraTrace_Record(&c,LT_READY,30,NULL,80,0);
    LoraTrace_Record(NULL,LT_RX,40,NULL,0,0);LoraTrace_Record(&c,LT_EVENT_COUNT,41,NULL,0,0);
    LoraTrace_Radio(NULL,RF_PRE_TX,0,NULL);LoraTrace_Radio(&c,RF_REASON_COUNT,0,NULL);
    CHECK(LoraTrace_DataKey(NULL,80)==0,"NULL key");
    CHECK(VirtualFree(pages,0,MEM_RELEASE)!=0,"guard free");
    drain(&c);puts(output);
}
static void snapshot(void){
    LoraTraceContext c=context();uint8_t a[95],changed[95];unsigned i;
    frame(a,5,2,5);frame(changed,35,3,35);changed[20]^=1;
    LoraTrace_Record(&c,LT_SELF_TX,9,a,95,2000);
    c.rt=109;LoraTrace_Record(&c,LT_SELF_TIMEOUT,9,changed,95,9);
    LoraTrace_Record(&c,LT_TX_EXIT,9,changed,95,1);drain(&c);
    for(i=0;i<100U;i++)LoraTrace_Record(&c,LT_CANCEL,i,NULL,0,0);
    c.attempt=8;LoraTrace_Record(&c,LT_FORWARD_TX,0,changed,95,4);
    drain(&c);
    c.rt=200;LoraTrace_Record(&c,LT_FORWARD_TIMEOUT,0,a,95,10);drain(&c);puts(output);
}
static void radio_recovery(void){
    LoraTraceContext c=context();RadioDiagSnapshot s={0};unsigned i;
    const uint8_t reasons[4]={RF_PRE_TX,RF_TX_EXIT,RF_RX_OPEN,RF_CONFIRM_TIMEOUT};
    for(i=0;i<4U;i++){
        s.rt=110U+i;c.rt=s.rt;s.status=0x52;s.read_flags=7;
        LoraTrace_Radio(&c,reasons[i],17,&s);
    }
    s.rt=300;c.rt=s.rt;s.busy_to=90;LoraTrace_Radio(&c,RF_HEAL_BEFORE,17,&s);
    s.rt=302;c.rt=s.rt;LoraTrace_Radio(&c,RF_HEAL_AFTER,17,&s);
    /* Forwarded/background traffic has no local sample seq. It must not erase
       the first CONF_TMO of the still-pending local collection seq17. */
    for(i=0;i<40U;i++){
        s.rt=400U+i;c.rt=s.rt;LoraTrace_Radio(&c,RF_PRE_TX,0,&s);service(&c,0);
    }
    s.rt=500;c.rt=s.rt;s.busy_to=180;LoraTrace_Radio(&c,RF_HEAL_BEFORE,17,&s);
    s.rt=502;c.rt=s.rt;LoraTrace_Radio(&c,RF_HEAL_AFTER,17,&s);
    CHECK(used==0,"RF overflow never bypasses output gate");
    c.rt=600;drain(&c);puts(output);
}
static void radio_postheal(void){
    LoraTraceContext c=context();RadioDiagSnapshot s={0};unsigned i;
    const uint8_t reasons[4]={RF_PRE_TX,RF_TX_EXIT,RF_RX_OPEN,RF_CONFIRM_TIMEOUT};
    for(i=0;i<4U;i++){
        s.rt=110U+i;c.rt=s.rt;s.status=0x52;s.read_flags=7;
        LoraTrace_Radio(&c,reasons[i],17,&s);
    }
    s.rt=500;c.rt=s.rt;s.busy_to=90;LoraTrace_Radio(&c,RF_HEAL_BEFORE,17,&s);
    s.rt=502;c.rt=s.rt;LoraTrace_Radio(&c,RF_HEAL_AFTER,17,&s);
    /* Recovery retains the pending sample. The first attempt after recovery
       is new evidence even though its seq and reason equal earlier records. */
    for(i=0;i<4U;i++){
        s.rt=610U+i;c.rt=s.rt;s.busy_to=92;LoraTrace_Radio(&c,reasons[i],17,&s);
    }
    CHECK(used==0,"post-heal capture does not print");c.rt=700;drain(&c);puts(output);
}
static void capture_context(void){
    LoraTraceContext c=context();uint8_t a[95];frame(a,5,2,5);
    c.rt=123;LoraTrace_Record(&c,LT_READY,25,a+14,80,0);
    c.part=3;c.host=2;c.rt=900;c.rx=1;service(&c,0);
    CHECK(used==0,"deferred context remains silent");drain(&c);puts(output);
}
static void overflow(void){
    LoraTraceContext c=context();unsigned i;
    for(i=0;i<200U;i++){c.rt=100+i;LoraTrace_Record(&c,LT_CANCEL,i,NULL,0,0);}
    c.rt=1000;service(&c,0);CHECK(used==0,"overflow never bypasses gate");drain(&c);puts(output);
}
static void gate(void){
    Rt_tick=100;Setup.partition=2;Setup.Host_number=5;
    Lora_Tx_state=0;Lora_Rx_state=0;Lo_data.Forward_flag=0;UI.lcd_open=0;g_slp_keep_awake=0;
    link_trace(LT_READY,1,NULL,0,0);link_trace_service();CHECK(used==0,"idle STOP gate");
    g_slp_keep_awake=1;Lora_Tx_state=4;link_trace_service();CHECK(used==0,"TX/ACK gate");
    Lora_Tx_state=0;Lora_Rx_state=3;link_trace_service();CHECK(used==0,"active RX gate");
    Lora_Rx_state=0;Lo_data.Forward_flag=1;link_trace_service();CHECK(used==0,"forward gate");
    Lo_data.Forward_flag=0;link_trace_service();CHECK(strstr(output,"[LS]"),"awake safe status");
    clear_output();g_slp_keep_awake=0;UI.lcd_open=1;
    {unsigned i;for(i=0;i<3U;i++)link_trace_service();}
    CHECK(strstr(output,"e=READY seq=1"),"display safe output");
    CHECK(link_trace_ticks(0U)==0U && link_trace_ticks(65535U)==65535U,"exact age");
    CHECK(link_trace_ticks(65536U)==65535U && link_trace_ticks(UINT32_MAX)==65535U,"age saturation");
    puts("PASS production gate and age saturation");
}
static void periodic(void){
    LoraTraceContext c=context();uint8_t a[95],copy[95];LoraTraceContext saved;
    frame(a,5,2,5);memcpy(copy,a,sizeof(a));saved=c;
    LoraTrace_Record(&c,LT_RX,1,a,95,0x40);
    CHECK(memcmp(copy,a,sizeof(a))==0 && memcmp(&c,&saved,sizeof(c))==0,"record is read-only");
    drain(&c);clear_output();c.rt=699;service(&c,1);CHECK(used==0,"no early status");
    c.rt=700;service(&c,0);CHECK(used==0,"status deferred");c.rt=701;service(&c,1);
    CHECK(strstr(output,"[LS]") && strstr(output,"rt=701"),"periodic status");
    clear_output();c.rt=0xFFFFFF00UL;service(&c,1);clear_output();c.rt=344;service(&c,1);
    CHECK(strstr(output,"[LS]"),"unsigned status wrap");puts("PASS read-only payload, deferred status, clock wrap");
}
int main(int argc,char **argv){
    CHECK(argc==2,"scenario argument");
    if(!strcmp(argv[1],"storm"))storm();else if(!strcmp(argv[1],"correlation"))correlation();
    else if(!strcmp(argv[1],"bounds"))bounds();else if(!strcmp(argv[1],"snapshot"))snapshot();
    else if(!strcmp(argv[1],"context"))capture_context();else if(!strcmp(argv[1],"overflow"))overflow();
    else if(!strcmp(argv[1],"radio_recovery"))radio_recovery();
    else if(!strcmp(argv[1],"radio_postheal"))radio_postheal();
    else if(!strcmp(argv[1],"gate"))gate();else if(!strcmp(argv[1],"periodic"))periodic();
    else CHECK(0,"unknown scenario");return 0;
}
'''


def parse_fields(line):
    return dict(re.findall(r"\b(\w+)=([^ ]+)", line))


def events(output):
    return [parse_fields(s) for s in output.splitlines() if s.startswith("[LK]")]


def find_event(rows, name, seq=None):
    found = [r for r in rows if r.get("e") == name and (seq is None or r["seq"] == str(seq))]
    assert found, f"missing {name} seq={seq}"
    return found[0]


def validate(name, output):
    rows = events(output)
    if name == "storm":
        first = find_event(rows, "SELF_TMO", 17)
        assert first["a"] == "1" and first["rt"] == "122", first
        assert first["dst"] == "2/2" and first["orig"] == "2/5" and first["x"] == "9", first
        assert find_event(rows, "HEAL_BEGIN", 17)["rt"] == "5500"
        assert find_event(rows, "HEAL_END", 17)["rt"] == "5502"
        assert find_event(rows, "SELF_OK_A", 17)["rt"] == "5803"
        radio = [parse_fields(s) for s in output.splitlines() if s.startswith("[RF]")]
        for reason, rt in [("HEAL_BEFORE", "5500"), ("HEAL_AFTER", "5502")]:
            kept = [r for r in radio if r["why"] == reason]
            assert kept and kept[0]["rt"] == rt and kept[0]["bt"] == "180", (reason, radio)
        status = next(parse_fields(s) for s in output.splitlines() if s.startswith("[LS]"))
        assert status["to"] == "90" and status["rx"] == "1", status
        assert len(rows) < 100, "repeated attempts did not remain bounded"
        assert int(status["compact"]) > 0 and int(status["rcompact"]) > 0, status
        summary = next(parse_fields(s) for s in output.splitlines() if s.startswith("[LC]"))
        expected = {"ready": 1, "selftx": 91, "txexit": 90, "rx": 1, "stmo": 90,
                    "sok": 1, "hbegin": 1, "hend": 1}
        assert all(summary[k] == str(v) for k, v in expected.items()), (summary, expected)
        ids = [int(r["n"]) for r in rows]
        assert len(ids) == len(set(ids)) and max(ids) > 250, ids
    elif name == "correlation":
        sent = find_event(rows, "SELF_TX", 7)
        fwd = find_event(rows, "FWD_TX", 0)
        a, b = find_event(rows, "READY", 8), find_event(rows, "READY", 9)
        assert sent["key"] == fwd["key"] == a["key"] == b["key"] != "00000000"
        assert sent["src"] == "2/5" and fwd["src"] == "2/2" and fwd["dst"] == "2/122"
        assert sent["orig"] == fwd["orig"] == "2/5" and sent["v0"] == "6:5300"
        assert find_event(rows, "RX", 0)["key"] != sent["key"]
    elif name == "bounds":
        for n in range(15):
            r = find_event(rows, "RX", n + 1)
            assert int(r["len"]) == n and r["key"] == "00000000", r
        assert find_event(rows, "RX", 20)["len"] == "95"
        assert find_event(rows, "READY", 30)["key"] == "00000000"
        assert not any(r["seq"] in {"40", "41"} for r in rows)
    elif name == "snapshot":
        sent = find_event(rows, "SELF_TX", 9)
        first = find_event(rows, "SELF_TMO", 9)
        returned = find_event(rows, "TX_EXIT", 9)
        assert first["key"] == returned["key"] == sent["key"]
        assert first["orig"] == "2/5" and first["dst"] == "2/2" and first["x"] == "9", first
        assert returned["x"] == "1", returned
        r = find_event(rows, "FWD_TMO", 0)
        assert r["orig"] == "2/35" and r["dst"] == "2/3" and r["a"] == "8" and r["x"] == "10", r
        assert r["key"] != sent["key"]
    elif name == "radio_recovery":
        radio = [parse_fields(s) for s in output.splitlines() if s.startswith("[RF]")]
        assert len(radio) <= 6, radio
        first = [r for r in radio if r["why"] == "CONF_TMO" and r["seq"] == "17"]
        assert len(first) == 1 and first[0]["rt"] == "113", radio
        for reason, rt in [("HEAL_BEFORE", "500"), ("HEAL_AFTER", "502")]:
            kept = [r for r in radio if r["why"] == reason]
            assert len(kept) == 1 and kept[0]["rt"] == rt and kept[0]["bt"] == "180", (reason, radio)
        status = next(parse_fields(s) for s in output.splitlines() if s.startswith("[LS]"))
        assert int(status["rlost"]) >= 42, status
    elif name == "radio_postheal":
        radio = [parse_fields(s) for s in output.splitlines() if s.startswith("[RF]")]
        assert len(radio) <= 6, radio
        for reason, rt in [("CONF_TMO", "113"), ("HEAL_BEFORE", "500"), ("HEAL_AFTER", "502"),
                           ("RX_OPEN", "612"), ("CONF_TMO", "613")]:
            assert any(r["why"] == reason and r["rt"] == rt and r["seq"] == "17" for r in radio), (reason, rt, radio)
        status = next(parse_fields(s) for s in output.splitlines() if s.startswith("[LS]"))
        assert int(status["rlost"]) >= 4, status
    elif name == "context":
        r = find_event(rows, "READY", 25)
        assert r["rt"] == "123" and r["p"] == "2" and r["h"] == "5", r
        status = next(parse_fields(s) for s in output.splitlines() if s.startswith("[LS]"))
        assert status["rt"] == "900" and status["p"] == "3" and status["h"] == "2"
    elif name == "overflow":
        status = next(parse_fields(s) for s in output.splitlines() if s.startswith("[LS]"))
        assert int(status["lost"]) > 0, status
        assert status["n"] == "200", status
        assert len(rows) < 200
    return {"events_printed": len(rows), "lines": len(output.splitlines())}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    result_path = OUT / "results.json"
    result = {"compile_exit": None, "test_exit": None, "hardware_verified": False, "tests": {}}
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    for source, dest in [("User/LoraTrace.c", "production_trace.c"), ("User/LoraTrace.h", "LoraTrace.h"), ("User/RadioDiag.h", "RadioDiag.h")]:
        (OUT / dest).write_bytes((ROOT / source).read_bytes())
    lora = (ROOT / "User/LoraPro.c").read_bytes().decode("gb18030")
    start = lora.index("extern volatile uint32_t g_cad_cnt, g_rx_hit_cnt;")
    end = lora.index("static void link_radio(", start)
    (OUT / "harness.c").write_text(HARNESS.replace("PRODUCTION_HELPERS", lora[start:end]), encoding="utf-8")
    compile_cmd = f'@echo off\ncall "{VCVARS}" >nul\nif errorlevel 1 exit /b %errorlevel%\ncl /nologo /W4 /WX /utf-8 /std:c11 /Od /Fe:harness.exe /Fo:harness.obj harness.c\nexit /b %errorlevel%\n'
    (OUT / "compile.cmd").write_text(compile_cmd, encoding="ascii")
    compiled = subprocess.run(["cmd.exe", "/d", "/c", "compile.cmd"], cwd=OUT, capture_output=True)
    (OUT / "compile.log").write_bytes(compiled.stdout + compiled.stderr)
    result["compile_exit"] = compiled.returncode
    result["source_sha256"] = {s: hashlib.sha256((ROOT / s).read_bytes()).hexdigest() for s in ["User/LoraTrace.c", "User/LoraTrace.h", "User/RadioDiag.h", "User/LoraPro.c"]}
    if compiled.returncode:
        result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        raise AssertionError((compiled.stdout + compiled.stderr).decode(errors="replace"))
    failures = []
    for name in ["storm", "correlation", "bounds", "snapshot", "radio_recovery", "radio_postheal", "context", "overflow", "gate", "periodic"]:
        run = subprocess.run([str(OUT / "harness.exe"), name], cwd=OUT, capture_output=True, timeout=30)
        (OUT / f"{name}.log").write_bytes(run.stdout + run.stderr)
        entry = {"exit": run.returncode}
        try:
            assert run.returncode == 0, (run.stdout + run.stderr).decode(errors="replace")
            entry.update(validate(name, run.stdout.decode("utf-8")))
            entry["passed"] = True
        except AssertionError as error:
            entry.update(passed=False, error=str(error))
            failures.append(f"{name}: {error}")
        result["tests"][name] = entry
        print(f'{name}: {"PASS" if entry["passed"] else "FAIL"}')
    result["test_exit"] = 1 if failures else 0
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    assert not failures, "\n".join(failures)


if __name__ == "__main__":
    main()
