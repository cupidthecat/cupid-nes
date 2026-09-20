/* Desktop event, layout and device regression checks. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../ui/desktop_ui.h"
#include "../ui/device_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../capture/capture_writer.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../system/execution_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(x) do {if(!(x)){fprintf(stderr,"Desktop check %d failed: %s\n",__LINE__,#x);++failures;}}while(0)
static void key(FrontendDesktopUi *ui, SDL_Scancode sc, SDL_Keymod mod) {
    SDL_Event event={.type=SDL_KEYDOWN};event.key.keysym.scancode=sc;event.key.keysym.mod=mod;
    CHECK(frontend_desktop_handle_event(ui,&event));
}
static bool panel_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;(void)error;(void)size;
    for(unsigned i=0;i<24;++i){
        FrontendPanelControl c={i,FRONTEND_PANEL_ACTION,"Action",NULL,NULL,0,-1,true,false};
        if(!frontend_panel_add_control(model,&c))return false;
    }
    return true;
}
static bool panel_action(void *context,unsigned id,const char *value,int selected,char *error,size_t size){
    (void)value;(void)selected;(void)error;(void)size;*(unsigned *)context=id;return true;
}
static void screenshot(FrontendDesktopUi *ui,const char *path) {
    int width,height;CHECK(SDL_GetRendererOutputSize(ui->renderer,&width,&height)==0);
    uint32_t *pixels=malloc((size_t)width*height*sizeof(*pixels));CHECK(pixels);
    if(!pixels)return;
    CHECK(SDL_RenderReadPixels(ui->renderer,NULL,SDL_PIXELFORMAT_ARGB8888,pixels,width*4)==0);
    NesCaptureFrame frame={pixels,(unsigned)width,(unsigned)height,(size_t)width};
    CHECK(nes_capture_png(path,&frame)==NES_FILE_OK);free(pixels);
}
int test_desktop_accuracy(void) {
    failures=0;
    SDL_setenv("SDL_VIDEODRIVER","dummy",1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO)==0);
    SDL_Window *window=SDL_CreateWindow("Desktop checks",0,0,768,720,SDL_WINDOW_HIDDEN|SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer=window?SDL_CreateRenderer(window,-1,SDL_RENDERER_SOFTWARE):NULL;
    CHECK(window && renderer);if(!renderer){if(window)SDL_DestroyWindow(window);SDL_QuitSubSystem(SDL_INIT_VIDEO);return failures;}
    FrontendSettings settings;frontend_settings_defaults(&settings);
    settings.speed=2;settings.audio_mix.master_volume=40;
    FrontendDesktopUi ui;frontend_desktop_init(&ui,window,renderer,&settings,NULL,NULL,NULL);
    CHECK(ui.open_menu==-1 && !frontend_desktop_input_captured(&ui));
    frontend_commands_reset();frontend_panels_reset();
    CHECK(frontend_desktop_register_commands(&ui));
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS,NULL,0));
    key(&ui,SDL_SCANCODE_TAB,KMOD_CTRL);CHECK(ui.settings_category==1);
    key(&ui,SDL_SCANCODE_TAB,KMOD_NONE);CHECK(ui.settings_focus==1);
    for(int i=0;i<3;++i)key(&ui,SDL_SCANCODE_RIGHT,KMOD_NONE);
    key(&ui,SDL_SCANCODE_RETURN,KMOD_NONE);
    CHECK(ui.staged.speed==1 && ui.staged.audio_mix.master_volume==40 && settings.speed==2);
    key(&ui,SDL_SCANCODE_TAB,KMOD_NONE);CHECK(ui.settings_focus==2);
    key(&ui,SDL_SCANCODE_DOWN,KMOD_NONE);CHECK(ui.settings_category==2);
    key(&ui,SDL_SCANCODE_RETURN,KMOD_NONE);CHECK(ui.settings_focus==0);
    SDL_SetRenderDrawColor(renderer,18,20,24,255);SDL_RenderClear(renderer);
    frontend_desktop_render(&ui,256,240,"Synthetic cartridge","NTSC","Paused");
    screenshot(&ui,"build/desktop-settings.png");
    key(&ui,SDL_SCANCODE_ESCAPE,KMOD_NONE);CHECK(settings.speed==2);
    key(&ui,SDL_SCANCODE_F,KMOD_ALT);CHECK(frontend_desktop_input_captured(&ui));
    for(int i=0;i<4;++i)key(&ui,SDL_SCANCODE_RIGHT,KMOD_NONE);
    CHECK(ui.open_menu==4);key(&ui,SDL_SCANCODE_ESCAPE,KMOD_NONE);
    unsigned selected=0;
    FrontendPanelSpec panel={0x7F00,"Scrollable panel","Tools",0,panel_snapshot,panel_action,&selected};
    CHECK(frontend_panel_register(&panel));ui.panel_id=panel.id;ui.panel_open=true;
    for(int i=0;i<20;++i)key(&ui,SDL_SCANCODE_DOWN,KMOD_NONE);
    key(&ui,SDL_SCANCODE_RETURN,KMOD_NONE);CHECK(selected==20 && ui.panel_scroll>0);
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui,256,240,"Synthetic cartridge","NTSC","Paused");
    screenshot(&ui,"build/desktop-panel.png");
    ui.panel_open=false;
    SDL_SetWindowSize(window,1280,960);SDL_RenderSetLogicalSize(renderer,1280,960);ui.ui_scale=2;
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS,NULL,0));
    SDL_RenderClear(renderer);frontend_desktop_render(&ui,256,240,"Synthetic cartridge","NTSC","Paused");
    screenshot(&ui,"build/desktop-settings-2x.png");
    SDL_Rect game;frontend_desktop_game_rect(&ui,1280,960,256,240,true,&game);
    CHECK(game.y>=128 && game.y+game.h<=912);
    settings.remember_window_size=false;unsigned width=settings.window_width;
    frontend_desktop_update_window_settings(&ui);CHECK(settings.window_width==width);
    frontend_desktop_shutdown(&ui);frontend_commands_reset();frontend_panels_reset();
    SDL_DestroyRenderer(renderer);SDL_DestroyWindow(window);SDL_QuitSubSystem(SDL_INIT_VIDEO);
    NesInputConfiguration previous={joypad_adapter(),{joypad_port_device(0),joypad_port_device(1)},joypad_expansion_device()};
    CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    FrontendDeviceRuntime devices;frontend_devices_init(&devices,NULL,&settings,NULL);
    char error[256];const uint8_t tape[]={0,1,0,1};
    CHECK(nes_file_write_atomic("build/desktop-tape.bin",tape,sizeof(tape))==NES_FILE_OK);
    CHECK(frontend_devices_set_tape_paths(&devices,"build/desktop-tape.bin","build/desktop-tape-out.bin",error,sizeof(error)));
    CHECK(!frontend_devices_set_tape_paths(&devices,"build/desktop-tape.bin","build/desktop-tape.bin",error,sizeof(error)));
    CHECK(frontend_devices_tape_load(&devices,"build/desktop-tape.bin",error,sizeof(error)));
    CHECK(frontend_devices_tape_play(&devices,error,sizeof(error)));
    CHECK(family_basic_tape_mode()==FB_TAPE_PLAYING);
    CHECK(frontend_devices_tape_stop(&devices,error,sizeof(error)));
    CHECK(frontend_devices_tape_record(&devices,error,sizeof(error)));
    CHECK(devices.tape_capture_pending);
    CHECK(frontend_devices_finish(&devices,error,sizeof(error)));
    CHECK(!devices.tape_capture_pending);
    CHECK(!frontend_devices_set_barcode(&devices,"123x",error,sizeof(error)));
    CHECK(frontend_devices_set_barcode(&devices,"12345678",error,sizeof(error)));
    (void)nes_file_remove("build/desktop-tape.bin");(void)nes_file_remove("build/desktop-tape-out.bin");
    CHECK(joypad_apply_configuration(&previous));family_basic_shutdown();
    printf("Desktop and device controls: %s (%d failures)\n",failures?"FAIL":"PASS",failures);
    return failures;
}
