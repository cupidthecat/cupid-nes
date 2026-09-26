/*
 * desktop_idle.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Startup window and recoverable image selection. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_ui.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include <stdio.h>
#include <string.h>

typedef struct { FrontendImageRequest *request; bool selected; } IdleSelection;
static bool choose(void *context,char *error,size_t size) {
    IdleSelection *selection=context;char path[FRONTEND_IMAGE_PATH_MAX];
    if(!frontend_open_image_dialog(path,sizeof(path),error,size))return false;
    selection->selected=frontend_image_request_init(selection->request,path);
    return selection->selected;
}
FrontendIdleResult frontend_desktop_idle_open(FrontendSettings *settings,const FrontendSession *session,
    const char *settings_path,FrontendImageRequest *request,SDL_Window **window,SDL_Renderer **renderer,
    char *error,size_t error_size) {
    if(!settings||!request||!window||!renderer)return FRONTEND_IDLE_ERROR;
    if(!(SDL_WasInit(SDL_INIT_VIDEO)&SDL_INIT_VIDEO)&&SDL_InitSubSystem(SDL_INIT_VIDEO)!=0)goto failure;
    if(!*window)*window=SDL_CreateWindow("Cupid NES",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
        (int)settings->window_width,(int)settings->window_height,SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE|SDL_WINDOW_ALLOW_HIGHDPI);
    if(!*window)goto failure;
    if(!*renderer)*renderer=SDL_CreateRenderer(*window,-1,SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    if(!*renderer)*renderer=SDL_CreateRenderer(*window,-1,SDL_RENDERER_SOFTWARE);
    if(!*renderer)goto failure;
    FrontendDesktopUi ui;frontend_desktop_init(&ui,*window,*renderer,settings,NULL,NULL,settings_path);
    ui.native_windows=true;
    ui.idle_session=session;
    if(error&&error_size&&error[0])frontend_desktop_set_status(&ui,error);
    frontend_commands_reset();frontend_panels_reset();
    IdleSelection selection={request,false};
    FrontendCommandSpec open={FRONTEND_COMMAND_OPEN,"Open Game...","File","Ctrl+O",0,choose,&selection};
    frontend_command_register(&open);frontend_desktop_register_commands(&ui);
    if(settings->reopen_last_image&&frontend_session_recent_count(session)){
        *request=*frontend_session_recent(session,0);selection.selected=true;
    }
    bool running=true;
    while(running&&!selection.selected){
        SDL_Event event;
        while(SDL_PollEvent(&event)){
            if(event.type==SDL_QUIT){running=false;break;}
            if(frontend_desktop_handle_event(&ui,&event)){
                if(ui.idle_recent_index>=0){*request=*frontend_session_recent(session,(size_t)ui.idle_recent_index);selection.selected=true;}
                continue;
            }
            if(event.type==SDL_DROPFILE){
                if(event.drop.file){selection.selected=frontend_image_request_init(request,event.drop.file);SDL_free(event.drop.file);}
            } else if(event.type==SDL_KEYDOWN&&(event.key.keysym.scancode==SDL_SCANCODE_RETURN
                || ((event.key.keysym.mod&KMOD_CTRL)&&event.key.keysym.scancode==SDL_SCANCODE_O))){
                if(!choose(&selection,error,error_size)&&error&&error[0])frontend_desktop_set_status(&ui,error);
            }
        }
        SDL_SetRenderDrawColor(*renderer,14,18,27,255);SDL_RenderClear(*renderer);
        frontend_desktop_render(&ui,256,240,NULL,NULL,"Idle");SDL_RenderPresent(*renderer);SDL_Delay(10);
    }
    frontend_desktop_update_window_settings(&ui);frontend_desktop_shutdown(&ui);
    frontend_commands_reset();frontend_panels_reset();
    if(selection.selected){if(error&&error_size)error[0]='\0';return FRONTEND_IDLE_OPEN;}
    SDL_DestroyRenderer(*renderer);SDL_DestroyWindow(*window);*renderer=NULL;*window=NULL;
    if(error&&error_size)error[0]='\0';
    return FRONTEND_IDLE_QUIT;
failure:
    if(error&&error_size)snprintf(error,error_size,"Could not open desktop: %s",SDL_GetError());
    return FRONTEND_IDLE_ERROR;
}
