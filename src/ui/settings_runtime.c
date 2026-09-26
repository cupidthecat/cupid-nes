/*
 * settings_runtime.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Validation and deferred machine configuration. SPDX-License-Identifier: GPL-3.0-or-later */
#include "settings.h"
#include "../cpu/cpu.h"
#include "../system/vs_system.h"
#include <stdio.h>
#include <stdlib.h>

bool frontend_settings_validate_firmware(const FrontendSettings *settings,char *error,size_t size) {
    if(!settings)return false;
    const char *paths[]={settings->fds_bios_path,settings->studybox_bios_path,settings->epsm_adpcm_path,settings->fcns_kanji_path};
    const char *names[]={"FDS BIOS","StudyBox BIOS","EPSM percussion ROM","Network character ROM"};
    const size_t lengths[]={8192,262144,8192,262144};
    for(unsigned i=0;i<4;++i){
        if(!paths[i][0])continue;
        uint8_t *data=NULL;size_t length=0;
        NesFileResult result=nes_file_read_all(paths[i],lengths[i],&data,&length);free(data);
        if(result!=NES_FILE_OK || length!=lengths[i]){
            if(error&&size)snprintf(error,size,"%s must be a readable %zu-byte file",names[i],lengths[i]);
            return false;
        }
    }
    return true;
}
bool frontend_settings_prepare_power(const FrontendSettings *settings) {
    if(!settings)return true;
    if(!(settings->cli_overrides&FRONTEND_OVERRIDE_STARTUP)){
        if(settings->startup_phase_set){
            if(!cpu_set_startup_alignment(settings->startup_cpu_offset,settings->startup_ppu_phase))return false;
        } else if(settings->startup_seed_set)cpu_seed_startup_alignment(settings->startup_seed);
        else cpu_use_default_startup_alignment();
    }
    if(settings->power_on_seed_set && !(settings->cli_overrides&FRONTEND_OVERRIDE_POWER_SEED))
        nes_seed_power_on_random(settings->power_on_seed);
    if(vs_enabled() && !(settings->cli_overrides&FRONTEND_OVERRIDE_VS_DIPS))
        return vs_set_dip_switches(settings->vs_dips);
    return true;
}
