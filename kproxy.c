#include <furi.h>
#include <furi_hal.h>

#include <gui/gui.h>
#include <input/input.h>

/* Magic happens here -- this file is generated by fbt.
 * Just set fap_icon_assets in application.fam and #include {APPID}_icons.h */
#include "kproxy_icons.h"

#include "lib/max3421/usb.h"

#define TAG "kproxy"

typedef struct {
    uint8_t x, y;
} ImagePosition;

static ImagePosition image_position = {.x = 0, .y = 0};

// Screen is 128x64 px
static void app_draw_callback(Canvas* canvas, void* ctx) {
    UNUSED(ctx);

    canvas_clear(canvas);
    canvas_draw_icon(canvas, image_position.x % 128, image_position.y % 64, &I_dolphin_71x25);
}

static void app_input_callback(InputEvent* input_event, void* ctx) {
    furi_assert(ctx);

    FuriMessageQueue* event_queue = ctx;
    furi_message_queue_put(event_queue, input_event, FuriWaitForever);
}

static bool kproxy_event_call(InputEvent event) {
    if((event.type == InputTypePress) || (event.type == InputTypeRepeat)) {
        switch(event.key) {
        case InputKeyLeft:
            image_position.x -= 2;
            break;
        case InputKeyRight:
            image_position.x += 2;
            break;
        case InputKeyUp:
            image_position.y -= 2;
            break;
        case InputKeyDown:
            image_position.y += 2;
            break;
        default:
            return false;
            break;
        }
    }
    FURI_LOG_I(TAG, "Found: %ix%i", image_position.x % 128, image_position.y % 64);
    return true;
}

int32_t kproxy_main(void* p) {
    UNUSED(p);
    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    // Configure view port
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, app_draw_callback, view_port);
    view_port_input_callback_set(view_port, app_input_callback, event_queue);

    // Register view port in GUI
    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    view_port_update(view_port);

    InputEvent event;
    bool running = true;
    //MAX
    max3421_init();
    max3421_reset_pin();
 //   max3421_reset();

//    while(running) {
//        // FURI_LOG_I(TAG, "max3421_int: 0x%x", max3421_int());
//        uint8_t cfg = 0;
//        uint8_t rreg = 0;
//
//        rreg = max3421_read_reg(max3421_HANDLE, rUSBIRQ, &cfg, 1);
//
//        FURI_LOG_I(TAG, "cfg: 0x%x", cfg);
//        FURI_LOG_I(TAG, "rreg: 0x%x", rreg);
//
//        furi_delay_tick(200);
//    }

    max3421_poweron();

    usbTask();

    FURI_LOG_I(TAG, "TaskState: 0x%x", usbGetUsbTaskState());


    uint16_t tmp = 0;
    while(running) {

        max3421_busprobe();

        usbTask();
        FURI_LOG_I(TAG, "TaskState: 0x%x", usbGetUsbTaskState());

        furi_delay_ms(1000);
        tmp++; //timeout after 100ms
        if(tmp == 500) {
            running = false;
        }

        if(furi_message_queue_get(event_queue, &event, 100) == FuriStatusOk) {
            running = kproxy_event_call(event);
            view_port_update(view_port);
        }
    }

    max3421_deinit();
    //MAX

    view_port_enabled_set(view_port, false);
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_message_queue_free(event_queue);

    furi_record_close(RECORD_GUI);

    return 0;
}
