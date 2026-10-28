"""Test production persistent-state code with a bounded fake Data-Flash device."""
from pathlib import Path
import argparse
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vcvars", required=True, type=Path)
    parser.add_argument("--out", default="verification/ble_power_persist_20261028/store_host")
    args = parser.parse_args()
    assert args.vcvars.is_file()
    out = (ROOT / args.out).resolve()
    out.relative_to(ROOT)
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "APP/Flash.c").read_text(encoding="utf-8", errors="replace")
    production = source.split("/* POWER_STATE_STORE_BEGIN", 1)[1]
    production = "/* POWER_STATE_STORE_BEGIN" + production.split("/* POWER_STATE_STORE_END */", 1)[0]
    startup = (ROOT / "APP/observer_main.c").read_text(encoding="utf-8")
    assert startup.index("binding_store_load();") < startup.index("power_state_store_load(&saved_state)")
    assert startup.index("power_state_store_load(&saved_state)") < startup.index("    Main_Circulation();")
    prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __HIGH_CODE
#define __attribute__(x) __declspec(align(4))
#define PRINT(...) ((void)0)
#define EEPROM_BLOCK_SIZE 4096u
#define EEPROM_MAX_SIZE 0x8000u
#define BLE_SNV_ADDR 0x7000u
#define BIND_STORE_BASE 0u
#define BIND_STORE_ERASE_LEN 4096u
static uint8_t flash[EEPROM_MAX_SIZE];
static unsigned reads, erases, writes, fail_read, fail_erase, fail_write, corrupt_write;
static unsigned partial_write;
static void bounds(uint32_t addr, const void *buf, uint32_t len)
{ assert(addr==0x1000u && len<=4096u && len<=sizeof(flash)-addr);
  if(buf) assert(((uintptr_t)buf&3u)==0u); }
static unsigned read_flash(uint32_t addr, void *buf, uint32_t len)
{ bounds(addr,buf,len); ++reads; if(fail_read) return 1;
  memcpy(buf,flash+addr,len); return 0; }
static unsigned erase_flash(uint32_t addr, uint32_t len)
{ bounds(addr,NULL,len); assert(len==EEPROM_BLOCK_SIZE); ++erases;
  if(fail_erase) return 1; memset(flash+addr,255,len); return 0; }
static unsigned write_flash(uint32_t addr, const void *buf, uint32_t len)
{ bounds(addr,buf,len); ++writes;
  if(fail_write) {
      if(partial_write) memcpy(flash+addr,buf,partial_write);
      return 1;
  }
  for(uint32_t i=0;i<len;i++) {
      assert((flash[addr+i]&((const uint8_t*)buf)[i])==((const uint8_t*)buf)[i]);
      flash[addr+i]=((const uint8_t*)buf)[i];
  }
  if(corrupt_write) flash[addr+5]^=1;
  return 0;
}
#define EEPROM_READ read_flash
#define EEPROM_ERASE erase_flash
#define EEPROM_WRITE write_flash
'''
    tests = r'''
int main(void)
{
    uint8_t state=99;
    memset(flash,255,sizeof(flash));
    memset(flash,0xA5,0x1000); memset(flash+0x7000,0x5A,0x1000);
    assert(power_state_store_load(&state)==1 && state==0 && !erases && !writes);
    assert(power_state_store_save(0)==0 && erases==1 && writes==1);
    unsigned r=reads,e=erases,w=writes;
    for(unsigned i=0;i<100;i++) assert(power_state_store_save(0)==0);
    assert(reads==r && erases==e && writes==w);
    assert(power_state_store_load(&state)==0 && state==0);
    r=reads; assert(power_state_store_save(state)==0);
    assert(reads==r && erases==e && writes==w); /* Restore does not write. */
    assert(power_state_store_save(1)==0 && erases==e+1 && writes==w+1);
    s_power_cached_valid=0; state=0;
    assert(power_state_store_load(&state)==0 && state==1);
    e=erases;w=writes;r=reads;
    assert(power_state_store_save(1)==0 && erases==e && writes==w && reads==r);
    assert(power_state_store_save(2)==1 && erases==e && writes==w);
    /* Every byte of the record is validated, including tag/version/complement. */
    uint8_t valid[16]; memcpy(valid,flash+0x1000,sizeof(valid));
    for(unsigned i=0;i<sizeof(valid);i++) {
        memcpy(flash+0x1000,valid,sizeof(valid)); flash[0x1000+i]^=1;
        state=99; assert(power_state_store_load(&state)==1 && state==0);
    }
    /* A torn write with any missing suffix must never restore ON. */
    for(unsigned n=0;n<16;n++) {
        memset(flash+0x1000,255,4096); memcpy(flash+0x1000,valid,n);
        state=99; assert(power_state_store_load(&state)==1 && state==0);
    }
    memcpy(flash+0x1000,valid,sizeof(valid));
    assert(power_state_store_load(&state)==0 && state==1);
    fail_erase=1; assert(power_state_store_save(0)==1); fail_erase=0;
    /* Erase failure may leave the old record; do not claim atomic off commit. */
    assert(power_state_store_load(&state)==0 && state==1);
    fail_write=1; partial_write=8;
    assert(power_state_store_save(0)==1); fail_write=0;partial_write=0;
    assert(power_state_store_load(&state)==1 && state==0);
    corrupt_write=1; assert(power_state_store_save(1)==1); corrupt_write=0;
    assert(power_state_store_load(&state)==1 && state==0);
    fail_read=1; assert(power_state_store_save(0)==1);
    assert(power_state_store_load(&state)==1 && state==0); fail_read=0;
    assert(power_state_store_save(0)==0);
    assert(power_state_store_load(&state)==0 && state==0);
    for(unsigned i=0;i<0x1000;i++) {
        assert(flash[i]==0xA5 && flash[0x7000+i]==0x5A);
    }
    puts("PASS: production Flash state restore, no repeat writes, corruption/torn writes, failure paths, binding/SNV preservation");
    return 0;
}
'''
    (out / "test.c").write_text(prefix+production+tests, encoding="utf-8")
    script = out / "run.cmd"
    script.write_text('@echo off\ncall "'+str(args.vcvars)+'" >nul\n'
                      'if errorlevel 1 exit /b 1\ncl /nologo /utf-8 /W4 test.c /Fe:test.exe\n'
                      'if errorlevel 1 exit /b 1\ntest.exe\n', encoding="utf-8")
    result = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=out,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (out / "test.log").write_bytes(result.stdout)
    print(result.stdout.decode("utf-8", errors="replace"))
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
