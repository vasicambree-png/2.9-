"""Real u8g2 drawing: ON matches baseline; OFF readings/voltage become --."""
from pathlib import Path
import argparse
import struct
import subprocess
import zlib
from verify_ui_fast_pages import SOURCES

ROOT = Path(__file__).resolve().parents[1]


def png(path, frame):
    # R3 maps logical (x,y) to native (y,383-x), vertical-top-LSB buffer.
    rows = bytearray()
    for y in range(168):
        rows.append(0)
        for x in range(384):
            py = 383-x
            rows.append(0 if frame[(py//8)*168+y] & (1 << (py%8)) else 255)
    def chunk(kind, data):
        return struct.pack(">I", len(data))+kind+data+struct.pack(">I", zlib.crc32(kind+data))
    path.write_bytes(b"\x89PNG\r\n\x1a\n"+
                     chunk(b"IHDR", struct.pack(">IIBBBBB",384,168,8,0,0,0,0))+
                     chunk(b"IDAT", zlib.compress(rows))+chunk(b"IEND", b""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vcvars", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    args = parser.parse_args()
    assert args.vcvars.is_file() and args.baseline.is_file()
    out = ROOT / "verification/ble_display_only_20261028/ui_host"
    out.mkdir(parents=True, exist_ok=True)
    fixture = (ROOT / "tools/test_ui_pages.c").read_text(encoding="utf-8")
    fixture = fixture[:fixture.index("int main(")]
    fixture = fixture.replace("data_LIST before = Data_list1;", """data_LIST before = Data_list1;
    device_t channels_before[MAX_CH_NUM]; scan_binding bindings_before[MAX_BINDING_NUM];
    memcpy(channels_before,CH_com_buf,sizeof(channels_before));
    memcpy(bindings_before,g_binding_list,sizeof(bindings_before));""")
    fixture = fixture.replace("assert(memcmp(&before, &Data_list1, sizeof(before)) == 0);", """
    assert(memcmp(&before, &Data_list1, sizeof(before)) == 0);
    assert(!memcmp(channels_before,CH_com_buf,sizeof(channels_before)));
    assert(!memcmp(bindings_before,g_binding_list,sizeof(bindings_before)));""")
    fixture += r'''
static unsigned placeholders;
void ui_preview_text(uint16_t x,uint16_t y,const char *text,uint8_t size,unsigned missing)
{ (void)x; (void)y; (void)size; assert(!missing); if(!strcmp(text,"--")) ++placeholders; }
static void paint(FILE *out,unsigned page)
{
    if(page<2) {
        Data_list1.menu_rank=1; Data_list1.UI_main.chu_num1=(uint16_t)(page+1);
    } else {
        Data_list1.menu_rank=3; Data_list1.rank2_addr=3;
        Data_list1.UI_main.re_flag=(uint8_t)(page+1);
    }
    placeholders=0; record(out);
#if CHECK_DISPLAY_GATE
    if(!Data_list1.UI_main.state)
        assert(placeholders==(page==0 ? 11u : page==1 ? 10u : 20u));
#endif
}
int main(void)
{
    static u8x8_display_info_t info;
    static uint8_t buffer[8064];
    info.tile_width=21; info.tile_height=48; info.pixel_width=168; info.pixel_height=384;
    u8g2.u8x8.display_info=&info; u8g2.u8x8.display_cb=empty_callback;
    u8g2.u8x8.cad_cb=empty_callback;
    u8g2_SetupBuffer(&u8g2,buffer,48,u8g2_ll_hvline_vertical_top_lsb,U8G2_R3);
    FILE *out=fopen("pages.raw","wb"); assert(out);
    for(unsigned seed=0;seed<4;seed++) {
        prepare_data(seed<3 ? seed : 2); Data_list1.UI_main.state=1;
        if(seed==3) {
            Data_list1.UI_main.vbat=0;
            for(unsigned i=0;i<MAX_CH_NUM;i++) { CH_com_buf[i].CH_data=0;CH_com_buf[i].voltage=0; }
        }
        for(unsigned page=0;page<4;page++) paint(out,page);
    }
    for(unsigned variation=0;variation<3;variation++) {
        prepare_data(2); Data_list1.UI_main.state=0;
        Data_list1.UI_main.vbat=(uint16_t)(variation==0 ? 0 : variation==1 ? 65535 : 379);
        for(unsigned i=0;i<MAX_CH_NUM;i++) {
            CH_com_buf[i].CH_data=(uint16_t)(variation==0 ? 0 : variation==1 ? 65535 : i*71);
            CH_com_buf[i].voltage=(uint8_t)(variation==0 ? 0 : variation==1 ? 255 : i+30);
            CH_com_buf[i].rssi=(uint8_t)(variation==0 ? 0 : variation==1 ? 255 : i+180);
            CH_com_buf[i].data_re_flag=(uint8_t)(variation!=2);
        }
        for(unsigned page=0;page<4;page++) paint(out,page);
    }
    assert(!fclose(out)); puts("PASS: 28 real UI frames, display does not mutate controller/channel/binding data");
    return 0;
}
'''
    quote = lambda p: '"'+str(p)+'"'
    for variant, source in (("before",args.baseline),("after",ROOT/"APP/yuying_TFT.c")):
        folder = out / variant
        folder.mkdir(exist_ok=True)
        text = source.read_text(encoding="utf-8")
        text = text[:text.index("void u8g2Init(u8g2_t *u8g2)")]
        text = text.replace("const uint8_t bmp[]={\n};", "const uint8_t bmp[]={0};")
        (folder/"ui_host_module.c").write_text(text,encoding="utf-8")
        (folder/"test.c").write_text(fixture,encoding="utf-8")
        (folder/"CONFIG.h").write_text("#pragma once\n#include <stdint.h>\n#include <stdio.h>\n#include <string.h>\n#define __HIGH_CODE\n",encoding="utf-8")
        rsp=["/nologo","/utf-8","/W3","/O2","/Gy","/DUI_PREVIEW=1",
             "/DCHECK_DISPLAY_GATE="+str(int(variant=="after")),"/I"+quote(folder),
             "/I"+quote(ROOT/"APP/include"),"/I"+quote(ROOT/"u8g2/include"),"test.c"]
        rsp += [quote(ROOT/"u8g2"/name) for name in SOURCES]
        rsp += ["/Fe:pages.exe","/link","/OPT:REF"]
        (folder/"build.rsp").write_text(" ".join(rsp),encoding="utf-16")
        script=folder/"run.cmd"
        script.write_text('@echo off\ncall '+quote(args.vcvars)+' >nul\nif errorlevel 1 exit /b 1\ncl @build.rsp\nif errorlevel 1 exit /b 1\npages.exe\n',encoding="utf-8")
        result=subprocess.run(["cmd.exe","/d","/c",str(script)],cwd=folder,
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (folder/"test.log").write_bytes(result.stdout)
        if result.returncode:
            print(result.stdout.decode("utf-8",errors="replace")[-6000:])
            raise SystemExit(result.returncode)
    before=(out/"before/pages.raw").read_bytes()
    after=(out/"after/pages.raw").read_bytes()
    frame=8067
    assert len(before)==len(after)==28*frame
    assert before[:16*frame]==after[:16*frame],"ON drawing changed"
    for page in range(4):
        masked=after[(16+page)*frame:(17+page)*frame]
        assert masked!=after[(8+page)*frame:(9+page)*frame]
        for variation in (1,2):
            offset=(16+variation*4+page)*frame
            assert masked==after[offset:offset+frame],"OFF leaked a changed measurement"
        png(out/("off_"+str(page)+".png"),masked)
        png(out/("on_"+str(page)+".png"),after[(8+page)*frame:(9+page)*frame])
    print("PASS: 16 ON frames match baseline byte-for-byte; 12 OFF frames mask readings, both voltages and RSSI with --; PNG previews saved")


if __name__ == "__main__":
    main()
