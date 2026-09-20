/* Storage location and database controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "storage_frontend.h"
#include "app_paths.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "game_database.h"
#include "../system/execution_policy.h"
#include <stdio.h>
#include <string.h>

enum { STORAGE_PANEL=0x1C00, STORAGE_SELECT, STORAGE_PATH, STORAGE_BROWSE,
       STORAGE_FOLDER, STORAGE_CORRECTIONS, STORAGE_PATCH };
static const char *const names[]={"Application data","Configuration","Cartridge save identity","Disk overlay",
    "Save-state file","Input movie","Screenshot","Audio recording","Video recording","Applied patch","HD packs","Game database"};
static void refresh(FrontendStorage *s){
    FrontendSettings *settings=s->actions->settings;
    FrontendSession *session=s->actions->session;
    const char *paths[]={frontend_paths_data_dir(),frontend_paths_config_file(),s->execution->save_identity,
        session->current.fds_overlay_path,settings->state_file_path,s->execution->movie_path,
        settings->capture_paths[0],settings->capture_paths[1],settings->capture_paths[2],session->current.patch_path,
        "",s->database_path};
    for(unsigned i=0;i<12;++i)snprintf(s->paths[i],sizeof(s->paths[i]),"%s",paths[i]?paths[i]:"");
    (void)frontend_paths_join(s->paths[10],sizeof(s->paths[10]),"hd-packs");
    if(!s->paths[11][0])(void)frontend_paths_join(s->paths[11],sizeof(s->paths[11]),FRONTEND_DATABASE_FILENAME);
}
static bool snapshot(void *context,FrontendPanelModel *model,char *error,size_t size){
    (void)error;(void)size;FrontendStorage *s=context;refresh(s);
    bool live=nes_execution_policy()==NES_EXECUTION_LIVE;
    bool editable=(s->selected>=4&&s->selected<=8)||s->selected==11;
    if(s->selected==11&&(s->actions->settings->cli_overrides&FRONTEND_OVERRIDE_DATABASE))editable=false;
    FrontendPanelControl controls[]={
        {STORAGE_SELECT,FRONTEND_PANEL_CHOICE,"Location",NULL,names,12,(int)s->selected,true,false},
        {STORAGE_PATH,FRONTEND_PANEL_TEXT,"Effective path",s->paths[s->selected],NULL,0,-1,live,!editable},
        {STORAGE_BROWSE,FRONTEND_PANEL_ACTION,"Browse...",NULL,NULL,0,-1,live&&editable,false},
        {STORAGE_FOLDER,FRONTEND_PANEL_ACTION,"Open containing folder",NULL,NULL,0,-1,true,false},
        {STORAGE_CORRECTIONS,FRONTEND_PANEL_CHECKBOX,"Apply database corrections on next load",NULL,NULL,0,
            !s->actions->settings->disable_database_corrections,live&&!(s->actions->settings->cli_overrides&FRONTEND_OVERRIDE_DATABASE_CORRECTIONS),false},
        {STORAGE_PATCH,FRONTEND_PANEL_ACTION,"Apply IPS / UPS / BPS patch...",NULL,NULL,0,-1,live,false}
    };
    for(unsigned i=0;i<sizeof(controls)/sizeof(controls[0]);++i)if(!frontend_panel_add_control(model,&controls[i]))return false;
    model->status=s->status[0]?s->status:"Paths do not move existing saves. --data-dir selects the application root at launch.";
    return true;
}
static bool set_path(FrontendStorage *s,const char *path,char *error,size_t size){
    if(!path||strlen(path)>=FRONTEND_SETTINGS_PATH_TEXT||strpbrk(path,"\r\n"))return false;
    FrontendSettings *settings=s->actions->settings;
    if(s->selected==11){
        if(settings->cli_overrides&FRONTEND_OVERRIDE_DATABASE)return false;
        FrontendDatabaseStatus status;
        if(!frontend_database_load(path[0]?path:NULL,frontend_paths_data_dir(),&status)){
            if(error&&size)snprintf(error,size,"%s",status.message);
            return false;
        }
        strcpy(settings->game_database_path,path);s->database_path=settings->game_database_path;
        snprintf(s->status,sizeof(s->status),"Database loaded. Corrections apply on the next image load.");
    }else if(s->selected==4)strcpy(settings->state_file_path,path);
    else if(s->selected==5){
        if(!frontend_execution_movie_set_path(s->execution,path,error,size))return false;
        strcpy(settings->movie_file_path,path);
    }else if(s->selected>=6&&s->selected<=8){
        if(s->capture && !nes_capture_frontend_set_path(s->capture,(FrontendSaveFileType)(s->selected-6),path,error,size))return false;
        strcpy(settings->capture_paths[s->selected-6],path);
    }
    else return false;
    return true;
}
static bool action(void *context,unsigned id,const char *value,int selected,char *error,size_t size){
    FrontendStorage *s=context;refresh(s);
    if(id==STORAGE_SELECT&&selected>=0&&selected<12){s->selected=(unsigned)selected;return true;}
    if(id==STORAGE_FOLDER){
        char path[4096];snprintf(path,sizeof(path),"%s",s->paths[s->selected]);
        if(s->selected!=0&&s->selected!=10){
            char *slash=strrchr(path,'/'),*backslash=strrchr(path,'\\');
            if(backslash&&(!slash||backslash>slash))slash=backslash;
            if(slash){if(slash==path||(slash==path+2&&path[1]==':'))slash[1]='\0';else *slash='\0';}else strcpy(path,".");
        }
        return frontend_open_folder(path,error,size);
    }
    if(nes_execution_policy()!=NES_EXECUTION_LIVE){if(error&&size)snprintf(error,size,"Stop the replay session before changing storage");return false;}
    if(id==STORAGE_PATH)return set_path(s,value,error,size);
    if(id==STORAGE_CORRECTIONS){
        if(s->actions->settings->cli_overrides&FRONTEND_OVERRIDE_DATABASE_CORRECTIONS)return false;
        s->actions->settings->disable_database_corrections=selected==0;
        rom_database_set_overrides(selected!=0);return true;
    }
    char path[FRONTEND_SETTINGS_PATH_TEXT]={0};
    if(id==STORAGE_PATCH){
        return frontend_open_file_dialog(FRONTEND_OPEN_PATCH,path,sizeof(path),error,size)
            && frontend_session_action_open_path(s->actions,path,error,size);
    }
    if(id==STORAGE_BROWSE){
        bool chosen=s->selected==11?frontend_open_file_dialog(FRONTEND_OPEN_DATABASE,path,sizeof(path),error,size)
            :frontend_save_file_dialog(s->selected==4?FRONTEND_SAVE_STATE:s->selected==5?FRONTEND_SAVE_MOVIE:
                (FrontendSaveFileType)(s->selected-6),path,sizeof(path),error,size);
        return chosen&&set_path(s,path,error,size);
    }
    return false;
}
bool frontend_storage_register(FrontendStorage *s,FrontendSessionActions *actions,
                               FrontendExecutionRuntime *execution,const char *database_path){
    if(!s||!actions||!execution)return false;
    memset(s,0,sizeof(*s));s->actions=actions;s->execution=execution;s->database_path=database_path;
    FrontendPanelSpec panel={STORAGE_PANEL,"Storage locations","Tools",FRONTEND_PANEL_NEEDS_SESSION,snapshot,action,s};
    return frontend_panel_register(&panel);
}
void frontend_storage_unregister(void){frontend_panel_unregister(STORAGE_PANEL);}
