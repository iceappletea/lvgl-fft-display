#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/SRAM/sram.h"
#include "./BSP/TIMER/btim.h"

/* LVGL */
#include "lvgl.h"
#include "lv_port_indev_template.h"
#include "lv_port_disp_template.h"

#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "arm_const_structs.h"

/* HAL Handles */
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_adc1;

/* FFT 參數 */
#define NPT 1024

uint16_t ADValue[NPT];       // ADC DMA 原始數據
uint16_t copyADValue[NPT];   // 複製給 Wave/FFT
float fft_inputbuf[NPT];     // FFT 輸入緩衝
float fft_outputbuf[NPT];    // FFT 輸出緩衝
arm_rfft_fast_instance_f32 rfft_instance;  // RFFT 實例

static float Samples;  // ADC 採樣率

/* FFT 動態頻率範圍(由 slider 控制) */
static float g_fft_low  = 250.0f;
static float g_fft_high = 650.0f;

/* 由 FFT_Calc() 決定的 binStart..binEnd，update_fft_chart() 會用到 */
static int s_binStart = 0;
static int s_binEnd   = 0;

/* FFT 計算結果 */
volatile float fft_max_val  = 0.0f;
volatile float fft_max_freq = 0.0f;
volatile uint8_t fft_ready  = 0;  // FFT 計算完成旗標

/* --- 與波形有關的全域變數 --- */
/* 原本 wave_chart_low & wave_chart_high 由滑桿動態調整；現在改成程式自動偵測*/
static int32_t wave_chart_low  = 600;
static int32_t wave_chart_high = 1800;

/* Wave Chart 點數、FFT Chart 點數 (由底部滑塊控制) */
static uint16_t wave_points = 250;
static uint16_t fft_points = 206;

/* LVGL 物件 */
static lv_style_t style_large_text;
static lv_obj_t * wave_chart = NULL;
static lv_obj_t * fft_chart  = NULL;
static lv_obj_t * freq_label = NULL;
static lv_obj_t * scale_container = NULL;
static lv_obj_t * scale = NULL;

static lv_obj_t * left_scale_container = NULL;
static lv_obj_t * left_scale = NULL;
static lv_obj_t * right_scale_container = NULL;
static lv_obj_t * right_scale = NULL;

static uint8_t  display_mode = 2; /* 0=只顯FFT,1=只顯Wave,2=Both */

/* Radio (FFT/Wave/Both) */
static lv_style_t style_radio;
static lv_style_t style_radio_chk;
static lv_style_t style_cb_text;
static lv_style_t style_cb_enlarge; /* 用來放大 CheckBox 可點擊區域 */
static lv_style_t style_slider_pad;
static lv_style_t style_knob_small;
static uint32_t active_index = 2;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

static void CopyDataToWaveBuff(void);
static void FFT_Calc(float samp);

void lv_mainstart_init(void);

static void init_scale_container(void);
static void remake_scale(int start, int end);
static void remake_right_scale(int start, int end);
static void create_left_scale(void);
static void create_right_scale(void);
static void radio_event_handler(lv_event_t * e);
static void slider_event_cb(lv_event_t * e);
/* ======= 已移除 wave_offset_slider_event_cb ======= */

static void fft_chart_draw_event_cb(lv_event_t * e);
static void radiobutton_create(lv_obj_t * parent, const char * txt);
static void create_mode_selector(void);
static void update_lvgl_charts(lv_timer_t * t);
static void update_fft_chart(void);

/* ----------- 程式進入點 ----------- */
int main(void)
{
    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    key_init();
    sram_init();
    btim_timx_int_init(10 - 1, 8400 - 1);

    printf("test\r\n");

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();

    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADValue, NPT);

    arm_rfft_fast_init_f32(&rfft_instance, NPT);

    lv_mainstart_init();

    while (1)
    {
        lv_task_handler();
        delay_ms(5);
    }
}

/* ---------------------------------------
   LVGL 初始化，建立各式介面元件
   --------------------------------------- */
void lv_mainstart_init(void)
{
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    
    /* 建立左邊/右邊的刻度容器 (垂直) */
    create_left_scale();
    create_right_scale();

    /*=== 建立 wave_chart (線圖) ===*/
    wave_chart = lv_chart_create(lv_scr_act());
    lv_chart_set_type(wave_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(wave_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_size(wave_chart, 0, 0, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(wave_chart, 0, LV_PART_MAIN);
    
    /* 背景、邊框透明 => 以便覆蓋在 FFT 上面但不遮擋 */
    lv_obj_set_style_bg_opa(wave_chart, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(wave_chart, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_chart_set_point_count(wave_chart, wave_points);
    lv_chart_set_range(wave_chart, LV_CHART_AXIS_PRIMARY_X, 0, wave_points - 1);
    /* 先給一個初始範圍; 之後會自動更新 */
    lv_chart_set_range(wave_chart, LV_CHART_AXIS_PRIMARY_Y, wave_chart_low, wave_chart_high);

    lv_chart_series_t * wave_ser = lv_chart_add_series(wave_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    int32_t * wave_arr = lv_chart_get_y_array(wave_chart, wave_ser);
    for (uint16_t i = 0; i < wave_points; i++)
    {
        wave_arr[i] = 0;
    }
    lv_chart_refresh(wave_chart);

    /*=== 建立 fft_chart (柱狀圖/線圖) ===*/
    fft_chart = lv_chart_create(lv_scr_act());
    lv_chart_set_type(fft_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(fft_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_size(fft_chart, 0, 0, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(fft_chart, 0, LV_PART_MAIN);
    
    lv_obj_set_style_bg_opa(fft_chart, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(fft_chart, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_chart_set_point_count(fft_chart, fft_points);
    lv_chart_set_range(fft_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 255);

    lv_chart_series_t * fft_ser = lv_chart_add_series(fft_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    int32_t * fft_arr = lv_chart_get_y_array(fft_chart, fft_ser);
    for (uint16_t i = 0; i < fft_points; i++)
    {
        fft_arr[i] = 0;
    }
    lv_chart_refresh(fft_chart);

    /*=== 根據 display_mode=0/1/2 分配大小位置 ===*/
    if (display_mode == 0)
    {
        lv_obj_add_flag(wave_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(fft_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(fft_chart, 0, 0);
        lv_obj_set_size(fft_chart, 800, 400);
    }
    else if (display_mode == 1)
    {
        lv_obj_add_flag(fft_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(wave_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(wave_chart, 0, 0);
        lv_obj_set_size(wave_chart, 800, 400);
    }
    else
    {
        lv_obj_clear_flag(wave_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(fft_chart, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_pos(fft_chart, 0, 0);
        lv_obj_set_size(fft_chart, 800, 400);

        lv_obj_set_pos(wave_chart, 0, 0);
        lv_obj_set_size(wave_chart, 800, 400);
    }

    /*=== 大字體標籤 (顯示 FFT 頻率) ===*/
    lv_style_init(&style_large_text);
    lv_style_set_text_font(&style_large_text, &lv_font_montserrat_26);
    freq_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(freq_label, 550, 10);
    lv_label_set_text(freq_label, "Freq: 0.00Hz");
    lv_obj_add_style(freq_label, &style_large_text, 0);

    /*=== 建立一個定時器 => 每 300ms 更新 Wave/FFT ===*/
    lv_timer_create(update_lvgl_charts, 300, NULL);

    /*=== 建立模式選擇 (CheckBox Radio) ===*/
    create_mode_selector();

    /*=== 建立底部 slider => 控制 Wave/FFT 點數 (0~100) ===*/
    lv_obj_t * slider = lv_slider_create(lv_scr_act());
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 100, LV_ANIM_OFF);
    lv_obj_set_width(slider, 700);
    lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -23);
    lv_obj_move_foreground(slider);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /*=== 美化一下 slider 外觀 ===*/
    lv_style_init(&style_knob_small);
    lv_style_set_bg_opa(&style_knob_small, LV_OPA_TRANSP);
    lv_style_set_border_opa(&style_knob_small, LV_OPA_TRANSP);
    lv_style_set_outline_opa(&style_knob_small, LV_OPA_TRANSP);
    lv_obj_add_style(slider, &style_knob_small, LV_PART_KNOB);

    lv_style_init(&style_slider_pad);
    lv_style_set_pad_all(&style_slider_pad, 12);
    lv_style_set_border_opa(&style_slider_pad, LV_OPA_TRANSP);
    lv_style_set_outline_opa(&style_slider_pad, LV_OPA_TRANSP);
    lv_obj_add_style(slider, &style_slider_pad, LV_PART_MAIN);

    /*=== 底部的頻率刻度 (水平) ===*/
    init_scale_container();
    remake_scale(250, 650);  // 預設顯示 250..650Hz

    /*
     * ==【注意】==
     * 右邊調整波形高低的 slider 已被移除，
     * 不再需要手動改 wave_chart_low / wave_chart_high。
     * 下面這段原本用來建立 wave_offset_slider 的程式已刪除。
     */
}

/* ---------------------------------------
   底部容器/刻度
   --------------------------------------- */
static void init_scale_container()
{
    scale_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(scale_container, 800, 80);
    lv_obj_set_pos(scale_container, 0, 395);
    lv_obj_set_style_bg_opa(scale_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(scale_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_move_background(scale_container);
    lv_obj_clear_flag(scale_container, LV_OBJ_FLAG_SCROLLABLE);
}

/* 底部頻率刻度 */
static void remake_scale(int start, int end)
{
    if(scale)
    {
        lv_obj_del(scale);
        scale = NULL;
    }

    scale = lv_scale_create(scale_container);
    lv_obj_set_size(scale, lv_pct(110), 70);
    lv_obj_center(scale);
    lv_scale_set_label_show(scale, true);
    lv_scale_set_mode(scale, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
    lv_scale_set_range(scale, start, end);
    lv_scale_set_total_tick_count(scale, 9);
    lv_scale_set_major_tick_every(scale, 1);
    lv_obj_set_style_length(scale, 0, LV_PART_ITEMS);
    lv_obj_set_style_length(scale, 8, LV_PART_INDICATOR);
}

/* ---------------------------------------
   左側/右側的刻度容器 (顯示波形或其他量測值)
   --------------------------------------- */
static void remake_right_scale(int start, int end)
{
    if(left_scale)
    {
        lv_obj_del(left_scale);
        left_scale = NULL;
    }

    left_scale = lv_scale_create(left_scale_container);

    lv_obj_set_size(left_scale, 40, 400);
    lv_obj_align(left_scale, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_scale_set_mode(left_scale, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_scale_set_range(left_scale, start, end);
    lv_scale_set_total_tick_count(left_scale, 5);
    lv_scale_set_major_tick_every(left_scale, 1);
    lv_scale_set_label_show(left_scale, true);

    lv_obj_set_style_length(left_scale, 0, LV_PART_ITEMS);
    lv_obj_set_style_length(left_scale, 8, LV_PART_INDICATOR);

    lv_obj_set_style_line_color(left_scale, lv_color_black(), LV_PART_INDICATOR);
    lv_obj_set_style_line_width(left_scale, 2, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(left_scale, lv_color_black(), LV_PART_MAIN);
}

static void create_left_scale(void)
{
    left_scale_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(left_scale_container, 70, 400);
    lv_obj_set_pos(left_scale_container, 730, 0);

    lv_obj_set_style_bg_opa(left_scale_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(left_scale_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(left_scale_container, 2, 0);
    lv_obj_set_style_border_color(left_scale_container, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_clear_flag(left_scale_container, LV_OBJ_FLAG_SCROLLABLE);

    /* 一開始先用預設 600..1800，稍後會自動調整 */
    remake_right_scale(wave_chart_low, wave_chart_high);
}

static void create_right_scale(void)
{
    right_scale_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(right_scale_container, 70, 400);
    lv_obj_set_pos(right_scale_container, 0, 0);

    lv_obj_set_style_bg_opa(right_scale_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(right_scale_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(right_scale_container, 2, 0);
    lv_obj_set_style_border_color(right_scale_container, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_clear_flag(right_scale_container, LV_OBJ_FLAG_SCROLLABLE);

    right_scale = lv_scale_create(right_scale_container);
    lv_obj_set_size(right_scale, 40, 400);
    lv_obj_align(right_scale, LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_scale_set_mode(right_scale, LV_SCALE_MODE_VERTICAL_RIGHT);
    lv_scale_set_range(right_scale, 0, 80);
    lv_scale_set_total_tick_count(right_scale, 5);
    lv_scale_set_major_tick_every(right_scale, 1);
    lv_scale_set_label_show(right_scale, true);

    lv_obj_set_style_length(right_scale, 0, LV_PART_ITEMS);
    lv_obj_set_style_length(right_scale, 8, LV_PART_INDICATOR);

    lv_obj_set_style_line_color(right_scale, lv_color_black(), LV_PART_INDICATOR);
    lv_obj_set_style_line_width(right_scale, 2, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(right_scale, lv_color_black(), LV_PART_MAIN);
}

/* ---------------------------------------
   模式選擇 (FFT / Wave / Both)
   --------------------------------------- */
static void radio_event_handler(lv_event_t * e)
{
    uint32_t *active_id = (uint32_t *)lv_event_get_user_data(e);
    lv_obj_t *cont = lv_event_get_current_target(e);
    lv_obj_t *act_cb = lv_event_get_target_obj(e);
    lv_obj_t *old_cb = lv_obj_get_child(cont, *active_id);

    if (act_cb == cont) {
        return;
    }

    lv_obj_remove_state(old_cb, LV_STATE_CHECKED);
    lv_obj_add_state(act_cb, LV_STATE_CHECKED);
    *active_id   = lv_obj_get_index(act_cb);
    display_mode = (uint8_t)(*active_id);

    if (display_mode == 0)
    {
        lv_obj_add_flag(wave_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(fft_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(fft_chart, 0, 0);
        lv_obj_set_size(fft_chart, 800, 400);
    }
    else if (display_mode == 1)
    {
        lv_obj_add_flag(fft_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(wave_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(wave_chart, 0, 0);
        lv_obj_set_size(wave_chart, 800, 400);
    }
    else
    {
        lv_obj_clear_flag(wave_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(fft_chart, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_pos(fft_chart, 0, 0);
        lv_obj_set_size(fft_chart, 800, 400);

        lv_obj_set_pos(wave_chart, 0, 0);
        lv_obj_set_size(wave_chart, 800, 400);
    }
}

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);
    int16_t val = lv_slider_get_value(slider);

    /* 改變 Wave/FFT 的點數 (範圍: wave=50~250, fft=110~206) */
    uint16_t newWavePoints = 50 + (200 * val) / 100;
    if (newWavePoints < 50)  newWavePoints = 50;
    if (newWavePoints > 250) newWavePoints = 250;
    wave_points = newWavePoints;

    uint16_t newFftPoints = 110 + (176 * val) / 100;
    if (newFftPoints < 110) newFftPoints = 110;
    if (newFftPoints > 206) newFftPoints = 206;
    fft_points = newFftPoints;

    uint16_t bin_start = 128;
    uint16_t bin_end = bin_start + fft_points - 1;
    if (bin_end > 333) {
        bin_end = 333;
    }

    float freq_per_bin = (650.0f - 250.0f) / (333 - 128);
    float freq_s = 250.0f + (bin_start - 128) * freq_per_bin;
    float freq_e = 250.0f + (bin_end - 128) * freq_per_bin;
    remake_scale((int)freq_s, (int)freq_e);

    lv_chart_set_point_count(wave_chart, wave_points);
    lv_chart_set_range(wave_chart, LV_CHART_AXIS_PRIMARY_X, 0, wave_points - 1);
    lv_chart_refresh(wave_chart);

    lv_chart_set_point_count(fft_chart, fft_points);
    lv_chart_set_range(fft_chart, LV_CHART_AXIS_PRIMARY_X, 0, fft_points - 1);
    lv_chart_refresh(fft_chart);
}

static void radiobutton_create(lv_obj_t * parent, const char * txt)
{
    lv_obj_t * obj = lv_checkbox_create(parent);
    lv_checkbox_set_text(obj, txt);
    lv_obj_add_style(obj, &style_cb_enlarge, LV_PART_MAIN);

    lv_obj_add_flag(obj, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_add_style(obj, &style_cb_text,   LV_PART_MAIN);
    lv_obj_add_style(obj, &style_radio,     LV_PART_INDICATOR);
    lv_obj_add_style(obj, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);
}

static void create_mode_selector(void)
{
    lv_style_init(&style_radio);
    lv_style_set_radius(&style_radio, LV_RADIUS_CIRCLE);
    lv_style_set_border_width(&style_radio, 3);
    lv_style_set_border_color(&style_radio, lv_color_black());
    lv_style_set_width(&style_radio, 30);
    lv_style_set_height(&style_radio, 30);

    lv_style_init(&style_radio_chk);
    lv_style_set_bg_color(&style_radio_chk, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_border_width(&style_radio_chk, 3);
    lv_style_set_border_color(&style_radio_chk, lv_palette_darken(LV_PALETTE_BLUE, 3));

    lv_style_init(&style_cb_text);
    lv_style_set_text_font(&style_cb_text, &lv_font_montserrat_26);

    lv_style_init(&style_cb_enlarge);
    lv_style_set_pad_all(&style_cb_enlarge, 10);

    lv_obj_t * cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 300, 300);
    lv_obj_set_pos(cont, 0, 0);

    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(cont, LV_OPA_TRANSP, 0);

    lv_obj_add_event_cb(cont, radio_event_handler, LV_EVENT_CLICKED, &active_index);

    radiobutton_create(cont, "FFT");
    lv_obj_set_pos(lv_obj_get_child(cont, 0), 10, 10);

    radiobutton_create(cont, "WAVE");
    lv_obj_set_pos(lv_obj_get_child(cont, 1), 10, 50);

    radiobutton_create(cont, "BOTH");
    lv_obj_set_pos(lv_obj_get_child(cont, 2), 10, 90);

    lv_obj_add_state(lv_obj_get_child(cont, 2), LV_STATE_CHECKED);
    lv_obj_move_foreground(cont);
}

/* --------------------------------------------------
   周期性 Timer 回呼 => 更新 Wave / FFT 顯示
   -------------------------------------------------- */
static void update_lvgl_charts(lv_timer_t * t)
{
    LV_UNUSED(t);

    /* --- (A) 更新波形顯示 --- */
    if (!lv_obj_has_flag(wave_chart, LV_OBJ_FLAG_HIDDEN))
    {
        lv_chart_series_t * wave_ser = lv_chart_get_series_next(wave_chart, NULL);
        int32_t * wave_arr = lv_chart_get_y_array(wave_chart, wave_ser);

        /* 填入最新 ADC 資料 */
        for (uint16_t i = 0; i < wave_points; i++)
        {
            if (i < 3 || i >= wave_points - 3)
            {
                wave_arr[i] = LV_CHART_POINT_NONE;
            }
            else
            {
                wave_arr[i] = (int32_t)copyADValue[i];
            }
        }

        /* >>>>>> 自動校準 (Auto-Scaling) : 找出波形最小 / 最大值，動態調整 Y 軸範圍 <<<<<< */
        int32_t wave_min_i32 = INT32_MAX;
        int32_t wave_max_i32 = INT32_MIN;

        for(uint16_t i = 3; i < wave_points - 3; i++)
        {
            if (wave_arr[i] != LV_CHART_POINT_NONE)
            {
                if (wave_arr[i] < wave_min_i32) wave_min_i32 = wave_arr[i];
                if (wave_arr[i] > wave_max_i32) wave_max_i32 = wave_arr[i];
            }
        }

        /* 避免全都是 NONE 或者怪異數值時失效 */
        if (wave_min_i32 >= wave_max_i32)
        {
            wave_min_i32 = 0;
            wave_max_i32 = 4095;  // 12-bit ADC 的最大值
        }
        else
        {
            /* 給一點餘裕，避免貼到邊緣 */
            wave_min_i32 -= 150;
            wave_max_i32 += 100;
            if (wave_min_i32 < 0)      wave_min_i32 = 0;
            if (wave_max_i32 > 4095)  wave_max_i32 = 4095;
        }

        wave_chart_low  = wave_min_i32;
        wave_chart_high = wave_max_i32;

        lv_chart_set_range(wave_chart, LV_CHART_AXIS_PRIMARY_Y, wave_chart_low, wave_chart_high);
        remake_right_scale(wave_chart_low, wave_chart_high);  // 順便更新左邊刻度

        lv_chart_refresh(wave_chart);
    }

    /* --- (B) 更新 FFT 顯示 --- */
    if (fft_ready && !lv_obj_has_flag(fft_chart, LV_OBJ_FLAG_HIDDEN))
    {
        update_fft_chart();
        fft_ready = 0;
    }
}

static void update_fft_chart(void)
{
    lv_chart_series_t * fft_ser = lv_chart_get_series_next(fft_chart, NULL);
    uint16_t point_count = lv_chart_get_point_count(fft_chart);

    uint16_t real_bins = s_binEnd - s_binStart + 1;
    if (real_bins < 1)
    {
        real_bins = 1;
    }

    uint16_t step = real_bins / point_count;
    if (step < 1) step = 1;

    float scale_factor = 255.0f / 50.0f;
    int32_t * arr = lv_chart_get_y_array(fft_chart, fft_ser);

    for (uint16_t i = 0; i < point_count; i++)
    {
        arr[i] = 0;
        if (i < 3 || i >= point_count - 3)
        {
            arr[i] = LV_CHART_POINT_NONE;
        }
        else
        {
            uint16_t bin_index = s_binStart + i * step;
            if (bin_index > s_binEnd)
            {
                bin_index = s_binEnd;
            }

            float val_f = fft_outputbuf[bin_index] * scale_factor;
            if (val_f > 255.0f)
            {
                val_f = 255.0f;
            }
            arr[i] = (int32_t) val_f;
        }
    }

    lv_chart_refresh(fft_chart);

    static char freq_text[32];
    snprintf(freq_text, sizeof(freq_text), "Freq: %.2fHz", fft_max_freq);
    lv_label_set_text(freq_label, freq_text);
}

/* --------------------------------------------------
   ADC DMA 完成時，複製資料 + 做一次 FFT
   -------------------------------------------------- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
    {
        CopyDataToWaveBuff();
        FFT_Calc(Samples);
    }
}

static void CopyDataToWaveBuff(void)
{
    memcpy(copyADValue, ADValue, NPT * sizeof(uint16_t));
}

static void FFT_Calc(float samp)
{
    for (int i = 0; i < NPT; i++)
    {
        float v = copyADValue[i] * 3.3f / 4095.0f;  // 12-bit ADC => 0~3.3V
        fft_inputbuf[i] = v;
    }

    arm_rfft_fast_f32(&rfft_instance, fft_inputbuf, fft_outputbuf, 0);

    fft_outputbuf[0] = fabsf(fft_outputbuf[0]);
    if ((NPT & 1) == 0)
    {
        fft_outputbuf[NPT / 2] = fabsf(fft_outputbuf[1]);
    }

    for (int i = 1; i < NPT / 2; i++)
    {
        float re = fft_outputbuf[2 * i];
        float im = fft_outputbuf[2 * i + 1];
        fft_outputbuf[i] = sqrtf(re * re + im * im);
    }

    int binStart = (int)(g_fft_low * NPT / samp + 0.5f);
    int binEnd   = (int)(g_fft_high * NPT / samp + 0.5f);
    if (binEnd > (NPT / 2)) binEnd = (NPT / 2);
    if (binStart < 0)       binStart = 0;
    if (binStart > binEnd)
    {
        binStart = 0;
        binEnd   = (NPT / 2);
    }

    s_binStart = binStart;
    s_binEnd   = binEnd;

    int subLen = binEnd - binStart + 1;
    if (subLen < 1) subLen = 1;

    float maxVal = 0.0f;
    uint32_t maxIndexLocal = 0;
    arm_max_f32(&fft_outputbuf[binStart], subLen, &maxVal, &maxIndexLocal);
    uint32_t maxIndex = binStart + maxIndexLocal;

    fft_max_val = maxVal;
    fft_max_freq = samp * (float)maxIndex / (float)NPT;
    fft_ready = 1;

    printf("%.1f..%.1fHz => bin[%d..%d], peak=%lu, freq=%.2fHz, amp=%.2f\r\n",
           g_fft_low, g_fft_high, binStart, binEnd, maxIndex, fft_max_freq, fft_max_val);
}

/* --------------------------------------------------
   DMA 中斷 (ADC1 => DMA2_Stream0) 
   -------------------------------------------------- */
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/* --------------------------------------------------
   ADC 初始化 (F4 不需手動 Calibration)
   -------------------------------------------------- */
static void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_7;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* --------------------------------------------------
   TIM2 => 觸發 ADC (Fsampling)
   -------------------------------------------------- */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler         = 419;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 99;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    float timerClock = 84000000.0f;
    uint32_t psc_val = htim2.Init.Prescaler + 1;
    uint32_t arr_val = htim2.Init.Period + 1;
    Samples = timerClock / (psc_val * arr_val);
}

/* --------------------------------------------------
   DMA 設定
   -------------------------------------------------- */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc1.Instance                 = DMA2_Stream0;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_adc1.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_adc1.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_adc1.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* --------------------------------------------------
   GPIO 初始化 (此範例用到 PA7 => ADC_IN7)
   -------------------------------------------------- */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin  = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
