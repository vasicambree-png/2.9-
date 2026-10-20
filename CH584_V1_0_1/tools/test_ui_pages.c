/* Host fixture includes the production drawing prefix; hardware endpoints
 * and observer data are supplied below. No UART, BLE or GPIO is simulated. */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include "ui_host_module.c"

u8g2_t u8g2;
device_t CH_com_buf[MAX_CH_NUM];
scan_binding g_binding_list[MAX_BINDING_NUM];
uint8_t g_binding_count, g_volt_err_count, g_name_err_count;
name_change_err_t g_name_err_list[NAME_CHK_ERR_NUM];
volatile uint8_t g_scan_mode;

uint8_t count_used_channels(void) { return g_binding_count ? 20u : 0u; }

static uint8_t empty_callback(u8x8_t *s, uint8_t msg, uint8_t count, void *data)
{
    (void)s; (void)msg; (void)count; (void)data; return 1u;
}

static void prepare_data(unsigned state)
{
    unsigned i;
    memset(&Data_list1, 0, sizeof(Data_list1));
    memset(CH_com_buf, 0, sizeof(CH_com_buf));
    memset(g_binding_list, 0, sizeof(g_binding_list));
    g_binding_count = state == 0 ? 0u : (state == 1 ? 6u : 20u);
    g_volt_err_count = state == 2 ? 3u : 0u;
    g_name_err_count = state == 2 ? 3u : 0u;
    Data_list1.Type = TPYE_MG;
    Data_list1.UI_main.vbat = state == 2 ? 65535u : 373u;
    Data_list1.UI_main.host_num = 23u;
    Data_list1.UI_main.sub_num = 9u;
    Data_list1.UI_main.send_host_num = state == 2 ? 122u : 121u;
    Data_list1.UI_main.Lora_rssi = state == 2 ? 255u : 3u;
    Data_list1.Menu_rank1.set_host_num = 23;
    Data_list1.Menu_rank1.set_sub_num = 9;
    Data_list1.Menu_rank2.net_flag = (uint16_t)(state == 2 ? 4 : state);
    Data_list1.Menu_rank2.now_host_addr = 23;
    Data_list1.Menu_rank2.cur_index = (uint16_t)state;
    Data_list1.Menu_rank6.time_light = 5u;
    Data_list1.Menu_rank6.power = 3;
    Data_list1.Menu_rank6.state = 1;
    for (i = 0; i < 3; i++)
    {
        Data_list1.Menu_rank2.addr[i] = (uint16_t)(30 + i);
        Data_list1.Menu_rank2.cnt[i] = (uint16_t)(100 + i);
        Data_list1.Menu_rank2.succ[i] = (uint16_t)(98 + i);
        Data_list1.Menu_rank5.old_send_addr[i] = (uint16_t)(40 + i);
        Data_list1.Menu_rank5.new_send_addr[i] = (uint16_t)(50 + i);
    }
    for (i = 0; i < 20; i++)
    {
        device_t *d = &CH_com_buf[i];
        d->Type = (Sensor_Tpye)(1u + i % (TPYE_END - 1u));
        d->valid = (uint8_t)(state != 0);
        d->data_re_flag = (uint8_t)(state == 2 || (state == 1 && (i & 1u)));
        d->CH_data = (uint16_t)(state == 2 ? 65535u : i * 123u);
        d->rssi = (uint8_t)(180u + i); d->voltage = (uint8_t)(30u + i);
        sprintf((char *)d->name, "SW_MG_23_%u", i + 1);
        g_binding_list[i].Type = d->Type;
        g_binding_list[i].frist_ch_num = (uint8_t)(i + 1u);
        g_binding_list[i].ch_num = 1u;
        sprintf((char *)g_binding_list[i].name, "SW_MG_23_%u", i + 1);
    }
}

static void record(FILE *output)
{
    data_LIST before = Data_list1;
    unsigned char flags[3];
    memset(u8g2.tile_buf_ptr, 0xa5, 8064);
    UI_Control(&Data_list1);
    assert(memcmp(&before, &Data_list1, sizeof(before)) == 0);
    assert(fwrite(u8g2.tile_buf_ptr, 1, 8064, output) == 8064);
    flags[0] = u8g2.draw_color; flags[1] = u8g2.bitmap_transparency;
    flags[2] = u8g2.font_decode.is_transparent;
    assert(fwrite(flags, 1, 3, output) == 3);
}

int main(int argc, char **argv)
{
    static u8x8_display_info_t info;
    static uint8_t buffer[8064];
    unsigned state, page, rank_value, menu, focus, subpage, msg, cases = 0;
    FILE *output;
    clock_t started = clock();
    assert(argc == 2);
    output = fopen(argv[1], "wb"); assert(output);
    info.tile_width = 21; info.tile_height = 48;
    info.pixel_width = 168; info.pixel_height = 384;
    u8g2.u8x8.display_info = &info;
    u8g2.u8x8.display_cb = empty_callback;
    u8g2.u8x8.cad_cb = empty_callback;
    u8g2_SetupBuffer(&u8g2, buffer, 48, u8g2_ll_hvline_vertical_top_lsb, U8G2_R3);
    for (state = 0; state < 3; state++)
    {
        prepare_data(state); g_ui_msg = UI_MSG_NONE;
        for (page = 1; page <= 2; page++)
        {
            Data_list1.menu_rank = 1; Data_list1.UI_main.chu_num1 = (uint16_t)page;
            record(output); cases++;
        }
        for (rank_value = 2; rank_value <= 3; rank_value++)
            for (menu = 0; menu < 8; menu++)
                for (focus = 0; focus < 8; focus++)
                    for (subpage = 0; subpage <= 6; subpage++)
                    {
                        Data_list1.menu_rank = (uint8_t)rank_value;
                        Data_list1.rank2_addr = (uint8_t)menu;
                        Data_list1.rank3_addr = (uint8_t)focus;
                        Data_list1.UI_main.re_flag = (uint8_t)subpage;
                        Data_list1.UI_main.chu_num2 = (uint16_t)(1u + (subpage & 1u));
                        record(output); cases++;
                    }
        for (msg = 1; msg <= UI_MSG_SHUTTING_DOWN; msg++)
        { g_ui_msg = (uint8_t)msg; record(output); cases++; }
        g_ui_msg = UI_MSG_NONE;
        Data_list1.menu_rank = 4; record(output); cases++;
        Data_list1.menu_rank = 5; record(output); cases++;
    }
    assert(fclose(output) == 0);
    printf("Rendered %u production UI cases in %.3fs (host timing only)\n",
           cases, (double)(clock() - started) / CLOCKS_PER_SEC);
    return 0;
}
