/* Apply settings through the active feature owners. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_ui.h"
#include "../rom/fds.h"
#include <stdio.h>
#include <string.h>
bool frontend_desktop_apply_features(FrontendDesktopUi *ui,const FrontendSettings *settings,char *error,size_t size) {
    if(ui->capture){
        if(ui->capture->session.info.recording){
            if(memcmp(&ui->capture->options,&settings->capture,sizeof(settings->capture))
                || memcmp(ui->capture->paths,settings->capture_paths,sizeof(settings->capture_paths))){
                if(error&&size)snprintf(error,size,"Stop recording before changing capture settings");
                return false;
            }
        }else{
            char previous[3][CAPTURE_PATH_CAPACITY];memcpy(previous,ui->capture->paths,sizeof(previous));
            for(unsigned i=0;i<3;++i){
                if(!nes_capture_frontend_set_path(ui->capture,(FrontendSaveFileType)i,settings->capture_paths[i],error,size)){
                    memcpy(ui->capture->paths,previous,sizeof(previous));return false;
                }
            }
            ui->capture->options=settings->capture;
        }
    }
    if(ui->devices && !frontend_devices_set_tape_paths(ui->devices,settings->tape_play_path,
        settings->tape_record_path,error,size))return false;
    if(ui->music && !nsf_player_set_options(ui->music,&settings->nsf_player))return false;
    if(fds_active()){
        fds_set_write_protected(settings->fds_write_protected);
        FdsAutomationOptions options={settings->fds_auto_insert,settings->fds_loading_fast_forward};
        fds_set_automation_options(options);
    }
    return true;
}
