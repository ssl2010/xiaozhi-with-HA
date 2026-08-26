#include "display/lcd_display.h"
#include "esp_lcd_sh8601.h"
#include "wifi_board.h"

#include "application.h"
#include "axp2101.h"
#include "button.h"
#include "codecs/box_audio_codec.h"
#include "config.h"
#include "i2c_device.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "power_save_timer.h"

#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include "settings.h"

#include <esp_lcd_touch_cst9217.h>
#include <esp_lvgl_port.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <array>
#include <functional>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#define TAG "WaveshareEsp32c6TouchAMOLED2inch16"

class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110);  // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);   // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V
        WriteReg(0x92, (3300 - 500) / 100);
        WriteReg(0x93, (3300 - 500) / 100);
        WriteReg(0x94, (3300 - 500) / 100);
        WriteReg(0x95, (3300 - 500) / 100);

        // Enable ALDO1(MIC)
        WriteReg(0x90, 0x0F);

        WriteReg(0x64, 0x02);  // CV charger voltage setting to 4.1V

        WriteReg(0x61, 0x02);  // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x0A);  // set Main battery charger current to 400mA ( 0x08-200mA,
                               // 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01);  // set Main battery term charge current to 25mA
    }

    void Pmic_SetAldo3(bool enable) {
        uint8_t reg = ReadReg(0x90);
        if (enable) {
            reg |= 0x04;
        } else {
            reg &= ~0x04;
        }
        WriteReg(0x90, reg);
    }

    void Pmic_SetAldo2(bool enable) {
        uint8_t reg = ReadReg(0x90);
        if (enable) {
            reg |= 0x02;
        } else {
            reg &= ~0x02;
        }
        WriteReg(0x90, reg);
    }
};

static const sh8601_lcd_init_cmd_t vendor_specific_init[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x44, (uint8_t[]){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x36, (uint8_t[]){0xA0}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 10},
    {0x63, (uint8_t[]){0xFF}, 1, 10},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x2A, (uint8_t[]){0x00, 0x16, 0x01, 0xAF}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xF5}, 4, 0},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
};

// 在waveshare_amoled_2_16类之前添加新的显示类
class CustomLcdDisplay : public SpiLcdDisplay {
private:
    using ControlCallback = std::function<void(const std::string&, const std::string&,
                                               const std::string&, const std::string&)>;
    struct ActionContext {
        CustomLcdDisplay* self = nullptr;
        uint8_t room = 0;
        uint8_t device = 0;
        const char* action = nullptr;
        const char* value = nullptr;
    };

    lv_obj_t* dashboard_ = nullptr;
    lv_obj_t* room_page_ = nullptr;
    std::array<lv_obj_t*, 4> room_sensor_labels_{};
    std::array<std::array<lv_obj_t*, 4>, 4> room_device_labels_{};
    std::array<ActionContext, 64> action_contexts_{};
    size_t action_context_count_ = 0;
    std::string latest_payload_;
    ControlCallback control_callback_;
    uint8_t selected_room_ = 0;

    static constexpr const char* kRoomNames[] = {"主卧", "次卧", "客厅", "书房"};

    ActionContext* NewContext(uint8_t room, uint8_t device, const char* action,
                              const char* value = nullptr) {
        if (action_context_count_ >= action_contexts_.size()) return nullptr;
        auto& context = action_contexts_[action_context_count_++];
        context = {this, room, device, action, value};
        return &context;
    }

    static const cJSON* DeviceAt(const cJSON* room, size_t index) {
        auto devices = cJSON_GetObjectItemCaseSensitive(room, "devices");
        return cJSON_IsArray(devices) ? cJSON_GetArrayItem(devices, index) : nullptr;
    }

    static std::string DeviceIcon(const cJSON* device) {
        auto type = cJSON_GetObjectItemCaseSensitive(device, "type");
        if (!cJSON_IsString(type)) return LV_SYMBOL_POWER;
        if (strcmp(type->valuestring, "climate") == 0) return LV_SYMBOL_SETTINGS;
        if (strcmp(type->valuestring, "purifier") == 0) return LV_SYMBOL_LOOP;
        if (strcmp(type->valuestring, "light") == 0) return LV_SYMBOL_BULLET;
        return LV_SYMBOL_POWER;
    }

    static std::string CompactDevice(const cJSON* device) {
        std::string text = DeviceIcon(device);
        text += " ";
        text += StateText(device);
        auto type = cJSON_GetObjectItemCaseSensitive(device, "type");
        if (IsAvailable(device) && cJSON_IsString(type) &&
            strcmp(type->valuestring, "climate") == 0 && strcmp(StateText(device), "关") != 0) {
            auto temperature = cJSON_GetObjectItemCaseSensitive(device, "temperature");
            if (cJSON_IsNumber(temperature)) {
                char value[12];
                snprintf(value, sizeof(value), "%.0f°", temperature->valuedouble);
                text += value;
            }
        }
        return text;
    }

    static std::string RoomSensors(const cJSON* room) {
        std::string text;
        if (!cJSON_IsObject(room)) return text;
        auto temperature = cJSON_GetObjectItemCaseSensitive(room, "temperature");
        auto humidity = cJSON_GetObjectItemCaseSensitive(room, "humidity");
        if (cJSON_IsObject(temperature) && IsAvailable(temperature)) {
            text += StateText(temperature); text += "°";
        }
        if (cJSON_IsObject(humidity) && IsAvailable(humidity)) {
            if (!text.empty()) text += "  ";
            text += LV_SYMBOL_TINT; text += StateText(humidity); text += "%";
        }
        return text;
    }

    static bool IsOn(const cJSON* device) {
        auto state = cJSON_GetObjectItemCaseSensitive(device, "state");
        return cJSON_IsString(state) && strcmp(state->valuestring, "off") != 0 &&
               strcmp(state->valuestring, "unavailable") != 0 &&
               strcmp(state->valuestring, "unknown") != 0;
    }

    static lv_color_t StateColor(const cJSON* device) {
        if (!IsAvailable(device)) return lv_color_hex(0x667080);
        return IsOn(device) ? lv_color_hex(0x41B8FF) : lv_color_hex(0xAAB4C0);
    }

    static void StylePowerButton(lv_obj_t* button, const cJSON* device) {
        bool available = IsAvailable(device);
        lv_obj_set_style_bg_color(button,
            available && IsOn(device) ? lv_color_hex(0x0877B9) : lv_color_hex(0x263646), 0);
        lv_obj_set_style_text_color(button,
            available ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x77818C), 0);
        if (!available) lv_obj_remove_flag(button, LV_OBJ_FLAG_CLICKABLE);
    }

    lv_obj_t* MakeButton(lv_obj_t* parent, int x, int y, int w, int h, const char* text,
                         ActionContext* context) {
        auto button = lv_button_create(parent);
        lv_obj_set_pos(button, x, y);
        lv_obj_set_size(button, w, h);
        lv_obj_set_style_radius(button, 10, 0);
        lv_obj_remove_flag(button, LV_OBJ_FLAG_SCROLLABLE);
        auto label = lv_label_create(button);
        lv_label_set_text(label, text);
        lv_obj_center(label);
        if (context != nullptr) {
            lv_obj_add_event_cb(button, ActionEvent, LV_EVENT_CLICKED, context);
        }
        return button;
    }

    static void ActionEvent(lv_event_t* event) {
        auto context = static_cast<ActionContext*>(lv_event_get_user_data(event));
        if (context == nullptr || context->self == nullptr) return;
        context->self->HandleAction(*context);
    }

    void HandleAction(const ActionContext& context) {
        if (strcmp(context.action, "open_room") == 0) {
            ShowRoom(context.room);
            return;
        }
        if (strcmp(context.action, "overview") == 0) {
            lv_obj_add_flag(room_page_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(dashboard_, LV_OBJ_FLAG_HIDDEN);
            return;
        }
        if (strcmp(context.action, "room") == 0) {
            ShowRoom(context.room);
            return;
        }

        cJSON* root = cJSON_ParseWithLength(latest_payload_.data(), latest_payload_.size());
        auto rooms = root ? cJSON_GetObjectItemCaseSensitive(root, "rooms") : nullptr;
        auto room = cJSON_IsObject(rooms)
                        ? cJSON_GetObjectItemCaseSensitive(rooms, kRoomNames[context.room]) : nullptr;
        auto device = DeviceAt(room, context.device);
        auto id = cJSON_GetObjectItemCaseSensitive(device, "id");
        if (!cJSON_IsString(id)) {
            cJSON_Delete(root);
            return;
        }
        if (strcmp(context.action, "detail") == 0) {
            ShowDeviceDetail(context.room, context.device);
            cJSON_Delete(root);
            return;
        }
        std::string device_id = id->valuestring;
        std::string action = context.action;
        std::string value = context.value ? context.value : "";
        if (action == "toggle") {
            auto state = cJSON_GetObjectItemCaseSensitive(device, "state");
            bool on = cJSON_IsString(state) && strcmp(state->valuestring, "off") != 0 &&
                      strcmp(state->valuestring, "unavailable") != 0 &&
                      strcmp(state->valuestring, "unknown") != 0;
            action = on ? "turn_off" : "turn_on";
        } else if (action == "temperature_down" || action == "temperature_up") {
            auto temperature = cJSON_GetObjectItemCaseSensitive(device, "temperature");
            int target = cJSON_IsNumber(temperature) ? (int)temperature->valuedouble : 26;
            target += action == "temperature_up" ? 1 : -1;
            if (target < 16) target = 16;
            if (target > 30) target = 30;
            action = "set_temperature";
            value = std::to_string(target);
        }
        cJSON_Delete(root);
        if (control_callback_) control_callback_(kRoomNames[context.room], device_id, action, value);
    }

    void ShowRoom(uint8_t room_index) {
        selected_room_ = room_index;
        action_context_count_ = 4;  // Keep the four overview-card contexts stable.
        lv_obj_clean(room_page_);
        lv_obj_remove_flag(room_page_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(dashboard_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(room_page_, LV_OBJ_FLAG_HIDDEN);
        MakeButton(room_page_, 8, 6, 54, 42, LV_SYMBOL_LEFT,
                   NewContext(room_index, 0, "overview"));
        auto title = lv_label_create(room_page_);
        lv_label_set_text(title, kRoomNames[room_index]);
        lv_obj_set_pos(title, 76, 14);

        cJSON* root = cJSON_ParseWithLength(latest_payload_.data(), latest_payload_.size());
        auto rooms = root ? cJSON_GetObjectItemCaseSensitive(root, "rooms") : nullptr;
        auto room = cJSON_IsObject(rooms)
                        ? cJSON_GetObjectItemCaseSensitive(rooms, kRoomNames[room_index]) : nullptr;
        auto devices = cJSON_GetObjectItemCaseSensitive(room, "devices");
        if (cJSON_IsArray(devices)) {
            int count = cJSON_GetArraySize(devices);
            for (int i = 0; i < count && i < 4; ++i) {
                auto device = cJSON_GetArrayItem(devices, i);
                auto row = lv_obj_create(room_page_);
                lv_obj_set_pos(row, 8, 56 + i * 72);
                lv_obj_set_size(row, 428, 64);
                lv_obj_set_style_pad_all(row, 7, 0);
                lv_obj_set_style_radius(row, 12, 0);
                lv_obj_set_scrollbar_mode(row, LV_SCROLLBAR_MODE_OFF);
                lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
                auto label = lv_label_create(row);
                auto name = cJSON_GetObjectItemCaseSensitive(device, "name");
                lv_label_set_text_fmt(label, "%s %s  %s", DeviceIcon(device).c_str(),
                                      cJSON_IsString(name) ? name->valuestring : "设备", StateText(device));
                lv_obj_set_pos(label, 4, 13);
                auto power = MakeButton(row, 282, 4, 58, 44, LV_SYMBOL_POWER,
                                        NewContext(room_index, i, "toggle"));
                StylePowerButton(power, device);
                auto type = cJSON_GetObjectItemCaseSensitive(device, "type");
                if (cJSON_IsString(type) &&
                    (strcmp(type->valuestring, "climate") == 0 ||
                     strcmp(type->valuestring, "purifier") == 0)) {
                    MakeButton(row, 348, 4, 58, 44, LV_SYMBOL_SETTINGS,
                               NewContext(room_index, i, "detail"));
                }
            }
        }
        cJSON_Delete(root);
    }

    void ShowDeviceDetail(uint8_t room_index, uint8_t device_index) {
        action_context_count_ = 4;
        lv_obj_clean(room_page_);
        MakeButton(room_page_, 8, 6, 54, 42, LV_SYMBOL_LEFT,
                   NewContext(room_index, 0, "room"));
        cJSON* root = cJSON_ParseWithLength(latest_payload_.data(), latest_payload_.size());
        auto rooms = root ? cJSON_GetObjectItemCaseSensitive(root, "rooms") : nullptr;
        auto room = cJSON_IsObject(rooms)
                        ? cJSON_GetObjectItemCaseSensitive(rooms, kRoomNames[room_index]) : nullptr;
        auto device = DeviceAt(room, device_index);
        auto name = cJSON_GetObjectItemCaseSensitive(device, "name");
        auto type = cJSON_GetObjectItemCaseSensitive(device, "type");
        auto title = lv_label_create(room_page_);
        lv_label_set_text_fmt(title, "%s · %s", kRoomNames[room_index],
                              cJSON_IsString(name) ? name->valuestring : "设备");
        lv_obj_set_pos(title, 76, 14);
        auto power = MakeButton(room_page_, 350, 6, 78, 42, LV_SYMBOL_POWER,
                                NewContext(room_index, device_index, "toggle"));
        StylePowerButton(power, device);
        if (cJSON_IsString(type) && strcmp(type->valuestring, "climate") == 0) {
            auto temperature = cJSON_GetObjectItemCaseSensitive(device, "temperature");
            auto temp = lv_label_create(room_page_);
            lv_label_set_text_fmt(temp, "设定温度  %.0f°C", cJSON_IsNumber(temperature) ? temperature->valuedouble : 26);
            lv_obj_set_pos(temp, 140, 70);
            MakeButton(room_page_, 50, 60, 60, 46, LV_SYMBOL_MINUS,
                       NewContext(room_index, device_index, "temperature_down"));
            MakeButton(room_page_, 330, 60, 60, 46, LV_SYMBOL_PLUS,
                       NewContext(room_index, device_index, "temperature_up"));
            const char* modes[][2] = {{"制冷", "cool"}, {"制热", "heat"}, {"自动", "auto"}};
            for (int i = 0; i < 3; ++i) MakeButton(room_page_, 28 + i * 136, 124, 118, 44, modes[i][0],
                NewContext(room_index, device_index, "set_hvac_mode", modes[i][1]));
            const char* fans[][2] = {{"低风", "low"}, {"中风", "medium"}, {"高风", "high"}, {"自动风", "auto"}};
            for (int i = 0; i < 4; ++i) MakeButton(room_page_, 12 + i * 106, 184, 96, 44, fans[i][0],
                NewContext(room_index, device_index, "set_fan_mode", fans[i][1]));
            const char* swings[][2] = {{"关摆风", "off"}, {"上下", "vertical"}, {"左右", "horizontal"}, {"全向", "both"}};
            for (int i = 0; i < 4; ++i) MakeButton(room_page_, 12 + i * 106, 244, 96, 44, swings[i][0],
                NewContext(room_index, device_index, "set_swing", swings[i][1]));
        } else {
            const char* presets[][2] = {{"自动", "自动"}, {"睡眠", "睡眠"}, {"最爱", "最爱"}};
            for (int i = 0; i < 3; ++i) MakeButton(room_page_, 28 + i * 136, 90, 118, 50, presets[i][0],
                NewContext(room_index, device_index, "set_preset_mode", presets[i][1]));
        }
        cJSON_Delete(root);
    }

    static const char* StateText(const cJSON* item) {
        auto state = cJSON_GetObjectItemCaseSensitive(item, "state");
        if (!cJSON_IsString(state)) {
            return "--";
        }
        if (strcmp(state->valuestring, "unavailable") == 0 ||
            strcmp(state->valuestring, "unknown") == 0) {
            return "离线";
        }
        if (strcmp(state->valuestring, "on") == 0) {
            return "开";
        }
        if (strcmp(state->valuestring, "off") == 0) {
            return "关";
        }
        if (strcmp(state->valuestring, "cool") == 0) {
            return "制冷";
        }
        if (strcmp(state->valuestring, "heat") == 0) {
            return "制热";
        }
        if (strcmp(state->valuestring, "dry") == 0) {
            return "除湿";
        }
        if (strcmp(state->valuestring, "fan_only") == 0) {
            return "送风";
        }
        if (strcmp(state->valuestring, "auto") == 0) {
            return "自动";
        }
        if (strcmp(state->valuestring, "idle") == 0) {
            return "待机";
        }
        return state->valuestring;
    }

    static const char* FanText(const char* value) {
        if (strcmp(value, "low") == 0) return "低风";
        if (strcmp(value, "medium") == 0 || strcmp(value, "mid") == 0) return "中风";
        if (strcmp(value, "high") == 0) return "高风";
        if (strcmp(value, "auto") == 0) return "自动风";
        if (strcmp(value, "quiet") == 0 || strcmp(value, "silent") == 0) return "静音";
        return value;
    }

    static bool IsAvailable(const cJSON* item) {
        auto state = cJSON_GetObjectItemCaseSensitive(item, "state");
        return cJSON_IsString(state) && strcmp(state->valuestring, "unavailable") != 0 &&
               strcmp(state->valuestring, "unknown") != 0;
    }

    static void AppendRole(std::string& text, const cJSON* room, const char* role,
                           const char* title, const char* suffix = "") {
        auto item = cJSON_GetObjectItemCaseSensitive(room, role);
        if (!cJSON_IsObject(item)) {
            return;
        }
        text += "\n";
        text += title;
        text += StateText(item);
        if (IsAvailable(item)) {
            text += suffix;
        }
    }

    static void AppendLight(std::string& text, const cJSON* room) {
        auto item = cJSON_GetObjectItemCaseSensitive(room, "light");
        if (!cJSON_IsObject(item)) return;
        text += "\n灯光  ";
        text += StateText(item);
        auto brightness = cJSON_GetObjectItemCaseSensitive(item, "brightness_pct");
        if (IsAvailable(item) && cJSON_IsNumber(brightness) &&
            strcmp(StateText(item), "关") != 0) {
            char value[12];
            snprintf(value, sizeof(value), " %d%%", brightness->valueint);
            text += value;
        }
    }

    static void AppendClimate(std::string& text, const cJSON* room) {
        auto item = cJSON_GetObjectItemCaseSensitive(room, "climate");
        if (!cJSON_IsObject(item)) return;
        text += "\n空调  ";
        text += StateText(item);
        if (!IsAvailable(item) || strcmp(StateText(item), "关") == 0) return;

        auto temperature = cJSON_GetObjectItemCaseSensitive(item, "temperature");
        if (cJSON_IsNumber(temperature)) {
            char value[16];
            snprintf(value, sizeof(value), " %.0f°C", temperature->valuedouble);
            text += value;
        }
        auto fan = cJSON_GetObjectItemCaseSensitive(item, "fan_mode");
        if (cJSON_IsString(fan)) {
            text += " ";
            text += FanText(fan->valuestring);
        }
        auto swing = cJSON_GetObjectItemCaseSensitive(item, "swing_mode");
        if (cJSON_IsString(swing) && strcmp(swing->valuestring, "off") != 0) {
            text += " 摆风";
        }
    }

    void CreateDashboard() {
        auto screen = lv_screen_active();
        dashboard_ = lv_obj_create(screen);
        lv_obj_set_size(dashboard_, 444, 360);
        lv_obj_align(dashboard_, LV_ALIGN_CENTER, 0, 8);
        lv_obj_set_style_bg_color(dashboard_, lv_color_hex(0x080D16), 0);
        lv_obj_set_style_bg_opa(dashboard_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dashboard_, 0, 0);
        lv_obj_set_style_pad_all(dashboard_, 0, 0);
        lv_obj_set_scrollbar_mode(dashboard_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_remove_flag(dashboard_, LV_OBJ_FLAG_SCROLLABLE);

        for (size_t i = 0; i < room_sensor_labels_.size(); ++i) {
            auto card = lv_obj_create(dashboard_);
            lv_obj_set_size(card, 214, 170);
            lv_obj_set_pos(card, (i % 2) * 224 + 3, (i / 2) * 180 + 3);
            lv_obj_set_style_radius(card, 16, 0);
            lv_obj_set_style_bg_color(card, lv_color_hex(0x152235), 0);
            lv_obj_set_style_border_color(card, lv_color_hex(0x294461), 0);
            lv_obj_set_style_border_width(card, 1, 0);
            lv_obj_set_style_pad_all(card, 10, 0);
            lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_OFF);
            lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(card, ActionEvent, LV_EVENT_CLICKED,
                                NewContext(i, 0, "open_room"));
            auto title = lv_label_create(card);
            lv_label_set_text(title, kRoomNames[i]);
            lv_obj_set_pos(title, 2, 0);
            lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
            room_sensor_labels_[i] = lv_label_create(card);
            lv_obj_set_pos(room_sensor_labels_[i], 2, 30);
            lv_obj_set_width(room_sensor_labels_[i], 192);
            lv_label_set_long_mode(room_sensor_labels_[i], LV_LABEL_LONG_CLIP);
            lv_label_set_text(room_sensor_labels_[i], "等待网关…");
            lv_obj_set_style_text_color(room_sensor_labels_[i], lv_color_hex(0x91A8BE), 0);
            for (size_t j = 0; j < room_device_labels_[i].size(); ++j) {
                auto slot = lv_label_create(card);
                lv_obj_set_pos(slot, 2 + (j % 2) * 98, 64 + (j / 2) * 43);
                lv_obj_set_size(slot, 94, 36);
                lv_label_set_long_mode(slot, LV_LABEL_LONG_CLIP);
                lv_label_set_text(slot, "");
                lv_obj_add_flag(slot, LV_OBJ_FLAG_HIDDEN);
                room_device_labels_[i][j] = slot;
            }
        }
        room_page_ = lv_obj_create(screen);
        lv_obj_set_size(room_page_, 444, 360);
        lv_obj_align(room_page_, LV_ALIGN_CENTER, 0, 8);
        lv_obj_set_style_bg_color(room_page_, lv_color_hex(0x080D16), 0);
        lv_obj_set_style_bg_opa(room_page_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(room_page_, 0, 0);
        lv_obj_set_style_pad_all(room_page_, 0, 0);
        lv_obj_set_scrollbar_mode(room_page_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_remove_flag(room_page_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(room_page_, LV_OBJ_FLAG_HIDDEN);
        if (emoji_box_ != nullptr) {
            lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        }
    }

public:
    static void rounder_event_cb(lv_event_t* e) {
        lv_area_t* area = (lv_area_t*)lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;

        uint16_t y1 = area->y1;
        uint16_t y2 = area->y2;

        // round the start of coordinate down to the nearest 2M number
        area->x1 = (x1 >> 1) << 1;
        area->y1 = (y1 >> 1) << 1;
        // round the end of coordinate up to the nearest 2N+1 number
        area->x2 = ((x2 >> 1) << 1) + 1;
        area->y2 = ((y2 >> 1) << 1) + 1;
    }

    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t panel_handle,
                     int width, int height, int offset_x, int offset_y, bool mirror_x,
                     bool mirror_y, bool swap_xy, ControlCallback control_callback)
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x,
                        mirror_y, swap_xy), control_callback_(std::move(control_callback)) {}

    virtual void SetupUI() override {
        // Call parent SetupUI() first to create all lvgl objects
        SpiLcdDisplay::SetupUI();

        DisplayLockGuard lock(this);
        lv_obj_set_style_pad_left(top_bar_, 60, 0);
        lv_obj_set_style_pad_right(top_bar_, 60, 0);
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
        CreateDashboard();
    }

    void UpdateDashboard(const std::string& payload) {
        cJSON* root = cJSON_ParseWithLength(payload.data(), payload.size());
        if (root == nullptr) {
            ESP_LOGW(TAG, "Invalid HA display JSON");
            return;
        }
        auto rooms = cJSON_GetObjectItemCaseSensitive(root, "rooms");
        DisplayLockGuard lock(this);
        latest_payload_ = payload;
        if (cJSON_IsObject(rooms) && dashboard_ != nullptr) {
            for (size_t i = 0; i < room_sensor_labels_.size(); ++i) {
                auto room = cJSON_GetObjectItemCaseSensitive(rooms, kRoomNames[i]);
                std::string sensors = RoomSensors(room);
                lv_label_set_text(room_sensor_labels_[i], sensors.empty() ? "暂无环境数据" : sensors.c_str());
                auto devices = cJSON_GetObjectItemCaseSensitive(room, "devices");
                int count = cJSON_IsArray(devices) ? cJSON_GetArraySize(devices) : 0;
                for (size_t j = 0; j < room_device_labels_[i].size(); ++j) {
                    auto label = room_device_labels_[i][j];
                    if ((int)j >= count) {
                        lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
                        continue;
                    }
                    auto device = cJSON_GetArrayItem(devices, j);
                    std::string text = CompactDevice(device);
                    lv_label_set_text(label, text.c_str());
                    lv_obj_set_style_text_color(label, StateColor(device), 0);
                    lv_obj_clear_flag(label, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
        cJSON_Delete(root);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(esp_lcd_panel_io_handle_t panel_io) : Backlight(), panel_io_(panel_io) {}

protected:
    esp_lcd_panel_io_handle_t panel_io_;

    virtual void SetBrightnessImpl(uint8_t brightness) override {
        auto display = Board::GetInstance().GetDisplay();
        DisplayLockGuard lock(display);
        uint8_t data[1] = {((uint8_t)((255 * brightness) / 100))};
        int lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= 0x02UL << 24;
        esp_lcd_panel_io_tx_param(panel_io_, lcd_cmd, &data, sizeof(data));
    }
};

class WaveshareEsp32c6TouchAMOLED2inch16 : public WifiBoard {
private:
    struct HaControlRequest {
        WaveshareEsp32c6TouchAMOLED2inch16* self;
        std::string room;
        std::string device;
        std::string action;
        std::string value;
    };
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_ = nullptr;
    Button boot_button_;
    CustomLcdDisplay* display_;
    CustomBacklight* backlight_;
    PowerSaveTimer* power_save_timer_;

    void FetchHaDisplayOnce(const std::string& display_url) {
        auto network = GetNetwork();
        if (network == nullptr) return;
        auto http = network->CreateHttp(0);
        http->SetTimeout(5000);
        if (http->Open("GET", display_url) && http->GetStatusCode() == 200) {
            display_->UpdateDashboard(http->ReadAll());
        } else {
            ESP_LOGW(TAG, "HA display gateway unavailable");
        }
        http->Close();
    }

    void QueueHaControl(const std::string& room, const std::string& device,
                        const std::string& action, const std::string& value) {
        auto request = new HaControlRequest{this, room, device, action, value};
        xTaskCreate(
            [](void* arg) {
                std::unique_ptr<HaControlRequest> request(static_cast<HaControlRequest*>(arg));
                Settings settings("wifi", false);
                auto control_url = settings.GetString("ha_control_url", HA_CONTROL_URL);
                auto display_url = settings.GetString("ha_display_url", HA_DISPLAY_URL);
                cJSON* payload = cJSON_CreateObject();
                cJSON_AddStringToObject(payload, "room", request->room.c_str());
                cJSON_AddStringToObject(payload, "device", request->device.c_str());
                cJSON_AddStringToObject(payload, "action", request->action.c_str());
                if (!request->value.empty()) {
                    char* end = nullptr;
                    double number = strtod(request->value.c_str(), &end);
                    if (end != request->value.c_str() && *end == '\0') {
                        cJSON_AddNumberToObject(payload, "value", number);
                    } else {
                        cJSON_AddStringToObject(payload, "value", request->value.c_str());
                    }
                }
                char* json = cJSON_PrintUnformatted(payload);
                cJSON_Delete(payload);
                auto network = request->self->GetNetwork();
                if (network != nullptr && json != nullptr) {
                    auto http = network->CreateHttp(0);
                    http->SetTimeout(5000);
                    http->SetHeader("Content-Type", "application/json");
                    http->SetContent(std::string(json));
                    if (!http->Open("POST", control_url) || http->GetStatusCode() != 200) {
                        ESP_LOGW(TAG, "HA touch control failed: %d", http->GetStatusCode());
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(300));
                        request->self->FetchHaDisplayOnce(display_url);
                    }
                    http->Close();
                }
                cJSON_free(json);
                vTaskDelete(nullptr);
            },
            "ha_control", 6144, request, 3, nullptr);
    }

    void StartHaDisplayTask() {
        xTaskCreate(
            [](void* arg) {
                auto self = static_cast<WaveshareEsp32c6TouchAMOLED2inch16*>(arg);
                Settings settings("wifi", false);
                auto display_url = settings.GetString("ha_display_url", HA_DISPLAY_URL);
                ESP_LOGI(TAG, "HA display URL: %s", display_url.c_str());
                vTaskDelay(pdMS_TO_TICKS(8000));
                while (true) {
                    auto network = self->GetNetwork();
                    if (network != nullptr) {
                        self->FetchHaDisplayOnce(display_url);
                    }
                    vTaskDelay(pdMS_TO_TICKS(HA_DISPLAY_REFRESH_SECONDS * 1000));
                }
            },
            "ha_display", 6144, this, 2, nullptr);
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 120, 1200);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(20);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() { pmic_->PowerOff(); });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num = LCD_PCLK;
        buscfg.data0_io_num = LCD_D0;
        buscfg.data1_io_num = LCD_D1;
        buscfg.data2_io_num = LCD_D2;
        buscfg.data3_io_num = LCD_D3;
        buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    int DisplayPort_DispReset() {
        pmic_->Pmic_SetAldo3(1);
        vTaskDelay(pdMS_TO_TICKS(100));
        pmic_->Pmic_SetAldo3(0);
        vTaskDelay(pdMS_TO_TICKS(100));
        pmic_->Pmic_SetAldo3(1);
        vTaskDelay(pdMS_TO_TICKS(100));
        return ESP_OK;
    }

    void InitializeSH8601Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = LCD_CS;
        io_config.dc_gpio_num = GPIO_NUM_NC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 32;
        io_config.lcd_param_bits = 8;
        io_config.flags.quad_mode = true;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        const sh8601_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(sh8601_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            }};

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = LCD_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void*)&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(panel_io, &panel_config, &panel));
        ESP_ERROR_CHECK(DisplayPort_DispReset());
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        display_ = new CustomLcdDisplay(panel_io, panel, LCD_H_RES, LCD_V_RES, DISPLAY_OFFSET_X,
                                        DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                        DISPLAY_SWAP_XY,
                                        [this](const std::string& room, const std::string& device,
                                               const std::string& action, const std::string& value) {
                                            QueueHaControl(room, device, action, value);
                                        });
        backlight_ = new CustomBacklight(panel_io);
        backlight_->RestoreBrightness();
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = LCD_H_RES - 1,
            .y_max = LCD_V_RES - 1,
            .rst_gpio_num = TP_RST_GPIO,
            .int_gpio_num = TP_INT_GPIO,
            .levels =
                {
                    .reset = 0,
                    .interrupt = 0,
                },
            .flags =
                {
                    .swap_xy = 1,
                    .mirror_x = 0,
                    .mirror_y = 1,
                },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
        tp_io_config.scl_speed_hz = 400 * 1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst9217(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    // 初始化工具
    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
                           "End this conversation and enter WiFi configuration mode.\n"
                           "**CAUTION** You must ask the user to confirm this action.",
                           PropertyList(), [this](const PropertyList& properties) {
                               EnterWifiConfigMode();
                               return true;
                           });
    }

public:
    WaveshareEsp32c6TouchAMOLED2inch16() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializePowerSaveTimer();
        InitializeCodecI2c();
        InitializeAxp2101();
        InitializeSpi();
        InitializeSH8601Display();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        StartHaDisplayTask();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override { return backlight_; }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32c6TouchAMOLED2inch16);
