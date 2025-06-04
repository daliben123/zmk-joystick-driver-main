/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

 #define DT_DRV_COMPAT zmk_analog_input

 #include <zephyr/kernel.h>
 #include <zephyr/sys/byteorder.h>
 #include <zephyr/input/input.h>
 #include <zmk/keymap.h>
 #include <stdlib.h> //for abs()
 #include <zephyr/sys/util.h> // for CLAMP
 
 #include <zephyr/logging/log.h>
 LOG_MODULE_REGISTER(ANALOG_INPUT, CONFIG_ANALOG_INPUT_LOG_LEVEL);
 
 #include <zmk/drivers/analog_input.h>
 
 static int analog_input_report_data(const struct device *dev) {
     struct analog_input_data *data = dev->data;
     const struct analog_input_config *config = dev->config;
 
     if (unlikely(!data->ready)) {
         LOG_WRN("Device is not initialized yet");
         return -EBUSY;
     }
 
 #if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
     static int64_t last_smp_time = 0;
     static int64_t last_rpt_time = 0;
     int64_t now = k_uptime_get();
 #endif
 
     struct adc_sequence* as = &data->as;
 
     for (uint8_t i = 0; i < config->io_channels_len; i++) {
         struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
         const struct device* adc = ch_cfg.adc_channel.dev;
 
         if (i == 0) {
 #ifdef CONFIG_ADC_ASYNC
             int err = adc_read_async(adc, as, &data->async_sig);
             if (err < 0) {
                 LOG_ERR("AIN%u read_async returned %d", i, err);
                 return err;
             }
             err = k_poll(&data->async_evt, 1, K_FOREVER);
             if (err < 0) {
                 LOG_ERR("AIN%u k_poll returned %d", i, err);
                 return err;
             }
             if (!data->async_evt.signal->signaled) {
                 return 0;
             }
             data->async_evt.signal->signaled = 0;
             data->async_evt.state = K_POLL_STATE_NOT_READY;
 #else
             int err = adc_read(adc, as);
             if (err < 0) {
                 LOG_ERR("AIN%u read returned %d", i, err);
                 return err;
             }
 #endif
         }
 
         int32_t raw = data->as_buff[i];
         int32_t mv = raw;
         adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, as->resolution, &mv);
 #if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_RAW)
         LOG_DBG("AIN%u raw: %d mv: %d", ch_cfg.adc_channel.channel_id, raw, mv);
 #endif
 
         int16_t v = mv - ch_cfg.mv_mid;
         int16_t dz = ch_cfg.mv_deadzone;
         if (dz) {
             if (v > 0) {
                 if (v < dz) v = 0; else v -= dz;
             }
             if (v < 0) {
                 if (v > -dz) v = 0; else v += dz;
             }
         }
         uint16_t mm = ch_cfg.mv_min_max;
         if (mm) {
             if (v > 0 && v > mm) v = mm;
             if (v < 0 && v < -mm) v = -mm;
         }
 
         if (ch_cfg.invert) v *= -1;
         v = (int16_t)((v * ch_cfg.scale_multiplier) / ch_cfg.scale_divisor);
 
         if (ch_cfg.report_on_change_only) {
             // track raw value to compare until next report interval
             data->delta[i] = v;
         }
         else {
             // accumulate delta until report in next iteration
             int32_t delta = data->delta[i];
             int32_t dv = delta + v;
             data->delta[i] = dv;
         }
     }
 
     // First read is setup as calibration
     as->calibrate = false;
 
 #if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
     // purge accumulated delta, if last sampled had not been reported on last report tick
     if (now - last_smp_time >= CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
         for (uint8_t i = 0; i < config->io_channels_len; i++) {
             data->delta[i] = 0;
             data->prev[i] = 0;
         }
     }
     last_smp_time = now;
 #endif
 
 #if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
     // strict to report inerval
     if (now - last_rpt_time < CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
         return 0;
     }
 #endif
 
     if (!data->actived) {
         return 0;
     }
 
     int8_t idx_to_sync = -1;
     for (uint8_t i = config->io_channels_len - 1; i >= 0; i--) {
         int32_t dv = data->delta[i];
         int32_t pv = data->prev[i];
         if (dv != pv) {
             idx_to_sync = i;
             break;
         }
     }
 
     for (uint8_t i = 0; i < config->io_channels_len; i++) {
         struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
         // LOG_DBG("AIN%u get delta AGAIN", i);
         int32_t dv = data->delta[i];
         int32_t pv = data->prev[i];
         if (dv != pv) {
 #if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
             last_rpt_time = now;
 #endif
             data->delta[i] = 0;
             if (ch_cfg.report_on_change_only) {
                 data->prev[i] = dv;
             }
 
 #if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)
             LOG_DBG("input_report %u rv: %d  e:%d  c:%d", i, dv, ch_cfg.evt_type, ch_cfg.input_code);
 #endif
             input_report(dev, ch_cfg.evt_type, ch_cfg.input_code, dv, i == idx_to_sync, K_NO_WAIT);
         }
     }
     return 0;
 }
 
 K_THREAD_STACK_DEFINE(analog_input_q_stack, CONFIG_ANALOG_INPUT_WORKQUEUE_STACK_SIZE);
 
 static struct k_work_q analog_input_work_q;
 
 // analog_input.c の sampling_work_handler を修正
 static void sampling_work_handler(struct k_work *work) {
     struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
     int err = analog_input_report_data(data->dev);
     
     // エラー検出とリカバリー
     if (err < 0) {
         LOG_ERR("Sampling error detected (%d), attempting recovery", err);
         
         // エラーカウントを増やす
         data->error_count++;
         
         // 連続エラーが一定回数を超えた場合
         if (data->error_count > CONFIG_ANALOG_INPUT_ERROR_THRESHOLD) {
             LOG_WRN("Multiple errors detected, resetting ADC");
             
             // ADCをリセット
             err = reset_adc_sequence(data->dev);
             if (err < 0) {
                 LOG_ERR("Failed to reset ADC (%d)", err);
                 // サンプリングを一時停止
                 k_timer_stop(&data->sampling_timer);
                 data->enabled = false;
                 return;
             }
             
             // エラーカウントをリセット
             data->error_count = 0;
         }
     } else {
         // 正常な場合はエラーカウントをリセット
         data->error_count = 0;
     }
 }
 
 // analog_input.c に追加
 static void sampling_timer_handler(struct k_timer *timer);
 static void watchdog_work_handler(struct k_work *work) {
     struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, watchdog_work);
     uint32_t now = k_uptime_get_32();
     
     // 最後の成功した読み取りからの経過時間をチェック
     if ((now - data->last_successful_read) > CONFIG_ANALOG_INPUT_WATCHDOG_TIMEOUT_MS) {
         LOG_WRN("ADC appears to be stuck, initiating reset");
         
         // ADCをリセット
         int err = reset_adc_sequence(data->dev);
         if (err < 0) {
             LOG_ERR("Watchdog reset failed (%d)", err);
         }
     }
 }
 
 static void sampling_work_handler(struct k_work *work);
 static void sampling_timer_handler(struct k_timer *timer) {
     struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);
     // LOG_DBG("sampling timer triggered");
     k_work_submit_to_queue(&analog_input_work_q, &data->sampling_work);
     k_work_submit(&data->sampling_work);
 }
 
 static int active_set_value(const struct device *dev, bool active) {
     struct analog_input_data *data = dev->data;
     if (data->actived == active) return 0;
     LOG_DBG("%d", active ? 1 : 0);
     data->actived = active;
     return 0;
 }
 
 static int sample_hz_set_value(const struct device *dev, uint32_t hz) {
     struct analog_input_data *data = dev->data;
 
     if (unlikely(!data->ready)) {
         LOG_DBG("Device is not initialized yet");
         return -EBUSY;
     }
 
     if (data->enabled) {
         LOG_DBG("Device is busy, would not update sampleing rate in enable state.");
         return -EBUSY;
     }
 
     LOG_DBG("%d", hz);
     data->sampling_hz = hz;
     return 0;
 }
 
 static int enable_set_value(const struct device *dev, bool enable) {
     struct analog_input_data *data = dev->data;
     // const struct tb6612fng_config *config = dev->config;
 
     if (unlikely(!data->ready)) {
         LOG_DBG("Device is not initialized yet");
         return -EBUSY;
     }
 
     if (data->enabled == enable) {
         return 0;
     }
     
     LOG_DBG("%d", enable ? 1 : 0);
     if (enable) {
         if (data->sampling_hz != 0) {
             uint32_t usec = 1000000UL / data->sampling_hz;
             k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
         } else {
             k_timer_start(&data->sampling_timer, K_NO_WAIT, K_NO_WAIT);
         }
         data->enabled = true;
     }
     else {
         k_timer_stop(&data->sampling_timer);
         data->enabled = false;
     }
 
     return 0;
 }
 
 static void analog_input_async_init(struct k_work *work) {
     struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
     struct analog_input_data *data = CONTAINER_OF(work_delayable, 
                                                   struct analog_input_data, init_work);
     const struct device *dev = data->dev;
     const struct analog_input_config *config = dev->config;
 
     // LOG_DBG("ANALOG_INPUT async init");
     uint32_t ch_mask = 0;
 
     for (uint8_t i = 0; i < config->io_channels_len; i++) {
         struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
         const struct device* adc = ch_cfg.adc_channel.dev;
         uint8_t channel_id = ch_cfg.adc_channel.channel_id;
         
     }
 }

// 假设reset_adc_sequence之前是非静态声明，这里去掉static
int reset_adc_sequence(const struct device *dev); // 这里需要补充函数的具体实现
