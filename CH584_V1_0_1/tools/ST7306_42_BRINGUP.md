# 4.2-inch ST7306 screen bring-up (2026-10-19)

## Scope / boundary

This is a **bring-up build**, not a final 400×300 UI redesign. The feature branch selects the existing 4.2-inch ST7306 controller driver from `u8g2/u8x8_d_st7305.c`. The original 2.9-inch ST7305 build can be restored by setting `SCREEN_PANEL_ST7306_42` to `0` in `APP/include/yuying_TFT.h`.

No hardware build, flashing, or screen measurement has been performed by the GitHub-side editing session. The physical 4.2-inch module's controller part number must match ST7306; this branch is **not** a drop-in driver for ST7305 4.2-inch monochrome panels.

## Display sizing (important)

- Nominal physical ST7306 panel: **300×400 in portrait**, or **400×300 in landscape**.
- In the existing driver, two horizontally adjacent virtual monochrome pixels encode one 2-bit physical pixel. The driver therefore exposes a **600×400 virtual** framebuffer before U8G2 rotation, and `U8G2_R3` exposes **400×600 virtual** coordinates to app drawing code.
- Memory is **75 tile columns × 50 tile rows × 8 bytes = 30,000 bytes**. Do **not** replace this with a 15,200-byte buffer based solely on the 300-pixel physical width.
- Existing main/menu/third-level page artwork is hardcoded for **384×168**, and remains unadapted. It may appear confined to part of the new panel; it has not been re-laid out or visually reviewed.

## Source changes

1. `APP/include/yuying_TFT.h`: selectable screen profile and dimensions (both physical and logical).
2. `APP/yuying_TFT.c`: route `u8g2Init()` to ST7306 when enabled; replace the incompatible ST7305-native boot bitmap with an ST7306 test geometry frame and diagonally opposite filled markers.
3. `APP/Usart3_task.c`: request the geometry test after an ordinary power-on reset, so a separate screen board can show something without a UART command from the baseboard.
4. `u8g2/u8x8_d_st7305.c`: correct argument-vs-command tags in ST7306 init; avoid double SPI start, pair with transfer end, bound the 600-byte conversion buffer, and use ST7306 power/flip sequences.
5. `u8g2/u8g2_d_memory.c`: clarify the correct 30 KB framebuffer size.

The original 2.9-inch ST7305 driver implementation is retained.

## Windows / MounRiver smoke test

1. Open `CH584_V1_0_1/CH584M_TFT_HB.wvproj`, use **Clean** and **Build**. Resolve compiler and linker failures before flashing. Confirm the MCU selected in the IDE matches the real silicon, because this project has older CH584/CH585 naming differences.
2. Confirm that the 4.2-inch module is **ST7306**, 300×400, and check its datasheet for voltage levels and SPI mode. The project source uses GPIO PA12=CS, PA13=SCK, PA14=MOSI, PA2=DC and defines PA3=RST; physical board routing must be checked. **GPIO reset callback is currently empty**, so confirm the panel reset wiring, pulse and timing before assuming a code defect.
3. With no baseboard UART connection, apply the correct supply and reset. Expected firmware-side behavior: draw an outer/double border, center cross, and opposite-corner filled squares, then leave the display visible.
4. If blank, capture the UART1 boot log (source uses PA9 TX / PA8 RX, 115200), and check scope/logic-analyzer waveforms for CS, CLK, MOSI, DC, and RST. Measure the panel supply. Check controller model before adjusting init voltages.
5. Then connect the baseboard UART3 on PA5 TX / PA4 RX at 115200. Verify real protocol parsing and display updates. The UI still uses 384×168 fixed-position drawings: this step identifies the remaining layout work.
6. Confirm the build's Flash and RAM use, screen-on behavior after sleep/wake, BLE scanning responsiveness during a full buffer transfer, and UART error counters.

## Known risks and next stage

- The repository's 4.2-inch ST7306 driver and source voltage tables were not hardware verified. Its 600-logical-pixel mapping is inherited from the existing code.
- Power-on test mode for 4.2-inch intentionally changes the earlier "stay black until UART/S2" policy; it is guarded by the screen profile.
- **Page layouts, text spacing, boot branding, scaling, touch/key navigation visual focus, and all 20-channel pages** need an explicit 400×300 redesign once the geometry pattern passes.
- The 2.9-inch build should also receive a build/smoke regression test with the profile set to zero.
