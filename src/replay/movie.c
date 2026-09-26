/*
 * movie.c - Deterministic input movie recording and playback
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "movie.h"
#include "../capture/movie_backup.h"
#include "input_event.h"
#include "rewind.h"
#include "tas_session.h"
#include "../apu/apu.h"
#include "../cheats/cheats.h"
#include "../cpu/cpu.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../state/state.h"
#include "../state/state_io.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../ui/machine_actions.h"
#include "../util/file_io.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

enum {
    MOVIE_VERSION = 2,
    MOVIE_MAX_EVENTS = 1000000
};

#define MOVIE_MAX_FILE_SIZE ((size_t)256u * 1024u * 1024u)

static const uint8_t movie_magic[8] = {'C','U','P','M','O','V','I','E'};

typedef struct {
    uint8_t source;
    uint8_t header[sizeof(iNESHeader)];
    uint32_t file_crc;
    uint32_t prg_crc;
    uint32_t prg_chr_crc;
    uint64_t prg_size;
    uint64_t chr_size;
    uint8_t region;
    uint8_t region_mode;
    uint8_t console_model;
    uint8_t ram_power_state;
    uint32_t power_random_state;
    bool randomize_vblank;
    uint8_t startup_alignment_mode;
    uint8_t configured_cpu_offset;
    uint8_t configured_ppu_phase;
    uint32_t startup_alignment_seed;
    bool cpu_test_mode;
    uint8_t adapter;
    uint8_t port_device[2];
    uint8_t expansion_device;
    uint8_t input_overrides;
    uint32_t zapper_radius;
    bool vs_enabled;
    uint16_t vs_dips;
    bool fds_active;
    bool fds_write_protected;
    uint8_t apu_cpu_revision;
    bool apu_disable_noise;
    bool apu_swap_duty;
    uint8_t ppu_revision;
    bool ppu_oam_row_corruption;
    bool ppu_startup_write_restriction;
    bool ppu_oam_decay;
    bool ppu_oamdata_read_disabled;
    bool ppu_palette_readback_disabled;
    bool ppu_reset_suppression;
    bool ppu_sprite_eval_wrap_bug;
    bool mmc3_revision_a;
    uint32_t cart_dips;
    uint32_t cheats_hash;
} MovieCompatibility;

typedef struct {
    uint64_t frame;
    NesInputEvent event;
} MovieEvent;

struct NesMovieSession {
    NesTasSession *tas;
    NesMovieMode mode;
    NesMovieStartKind start_kind;
    NesMovieResult last_result;
    char error[192];
    char *path;
    uint64_t frame;
    uint64_t total_frames;
    MovieEvent *events;
    size_t event_count;
    size_t event_capacity;
    NesInputEvent *pending;
    size_t pending_count;
    size_t pending_capacity;
    size_t playback_index;
    NesStateBlob start_state;
    NesStateRestore *resume_restore;
    MovieCompatibility compatibility;
    uint32_t previous_policy;
    bool frame_in_progress;
    NesMovieResult recording_error;
};

static char *movie_strdup(const char *text) {
    if (!text) return NULL;
    size_t size = strlen(text);
    if (size >= NES_FILE_PATH_LIMIT || size == SIZE_MAX) return NULL;
    char *copy = (char *)malloc(size + 1);
    if (copy) memcpy(copy, text, size + 1);
    return copy;
}

static void movie_clear_recording_data(NesMovieSession *movie) {
    if (!movie) return;
    free(movie->path);
    movie->path = NULL;
    free(movie->events);
    movie->events = NULL;
    movie->event_count = 0;
    movie->event_capacity = 0;
    free(movie->pending);
    movie->pending = NULL;
    movie->pending_count = 0;
    movie->pending_capacity = 0;
    movie->playback_index = 0;
    nes_state_blob_free(&movie->start_state);
    movie->frame = 0;
    movie->total_frames = 0;
}

static NesMovieResult movie_set_result(NesMovieSession *movie, NesMovieResult result) {
    if (movie) {
        movie->last_result = result;
        snprintf(movie->error, sizeof(movie->error), "%s",
                 result == NES_MOVIE_OK || result == NES_MOVIE_COMPLETE ? "" : nes_movie_result_string(result));
    }
    return result;
}

static NesMovieResult movie_tas_result(NesMovieSession *movie, NesMovieResult result) {
    movie_set_result(movie, result);
    if (movie && result != NES_MOVIE_OK && result != NES_MOVIE_COMPLETE && nes_tas_session_error(movie->tas)[0])
        snprintf(movie->error, sizeof(movie->error), "%s", nes_tas_session_error(movie->tas));
    return result;
}

static MovieCompatibility movie_current_compatibility(void) {
    MovieCompatibility value;
    memset(&value, 0, sizeof(value));
    value.source = (uint8_t)rom_metadata_source();
    memcpy(value.header, &ines_header, sizeof(ines_header));
    value.file_crc = rom_file_crc32();
    value.prg_crc = rom_prg_crc32();
    value.prg_chr_crc = rom_prg_chr_crc32();
    value.prg_size = prg_size;
    value.chr_size = chr_size;
    value.region = (uint8_t)nes_timing()->region;
    value.region_mode = (uint8_t)nes_region_mode();
    value.console_model = (uint8_t)nes_console_model();
    value.ram_power_state = (uint8_t)nes_ram_power_on_state();
    value.power_random_state = nes_power_on_random_state();
    value.randomize_vblank = nes_randomize_vblank_enabled();
    value.startup_alignment_mode = (uint8_t)cpu_get_startup_alignment_mode();
    CpuStartupAlignment configured = cpu_get_configured_startup_alignment();
    value.configured_cpu_offset = configured.cpu_offset;
    value.configured_ppu_phase = configured.ppu_phase;
    value.startup_alignment_seed = cpu_get_startup_alignment_seed();
    value.cpu_test_mode = cpu_test_mode_enabled();
    value.adapter = (uint8_t)joypad_adapter();
    value.port_device[0] = (uint8_t)joypad_port_device(0);
    value.port_device[1] = (uint8_t)joypad_port_device(1);
    value.expansion_device = (uint8_t)joypad_expansion_device();
    value.input_overrides = joypad_configuration_overrides();
    value.zapper_radius = joypad_zapper_radius();
    value.vs_enabled = vs_enabled();
    value.vs_dips = vs_dip_switches();
    value.fds_active = fds_active();
    value.fds_write_protected = fds_write_protected();
    value.apu_cpu_revision = (uint8_t)apu_get_cpu_revision();
    value.apu_disable_noise = apu_noise_mode_disabled();
    value.apu_swap_duty = apu_swap_duty_cycles_enabled();
    value.ppu_revision = (uint8_t)ppu_revision();
    value.ppu_oam_row_corruption = ppu_oam_row_corruption_worst_case();
    value.ppu_startup_write_restriction = ppu_startup_write_restriction_enabled();
    value.ppu_oam_decay = ppu_oam_decay_enabled();
    value.ppu_oamdata_read_disabled = ppu_oamdata_read_disabled();
    value.ppu_palette_readback_disabled = ppu_palette_readback_disabled();
    value.ppu_reset_suppression = ppu_reset_suppression_enabled();
    value.ppu_sprite_eval_wrap_bug = ppu_sprite_eval_wrap_bug_enabled();
    value.mmc3_revision_a = strcmp(cart_mmc3_revision_name(), "a") == 0;
    value.cart_dips = cart_dip_switches();
    value.cheats_hash = cheats_compatibility_hash();
    return value;
}

static bool movie_compatibility_matches(const MovieCompatibility *saved) {
    if (!saved) return false;
    MovieCompatibility active = movie_current_compatibility();
    return memcmp(saved, &active, sizeof(active)) == 0;
}

static bool movie_write_compatibility(NesStateWriter *writer,
                                      const MovieCompatibility *value) {
    return nes_state_write_u8(writer, value->source)
        && nes_state_write_bytes(writer, value->header, sizeof(value->header))
        && nes_state_write_u32(writer, value->file_crc)
        && nes_state_write_u32(writer, value->prg_crc)
        && nes_state_write_u32(writer, value->prg_chr_crc)
        && nes_state_write_u64(writer, value->prg_size)
        && nes_state_write_u64(writer, value->chr_size)
        && nes_state_write_u8(writer, value->region)
        && nes_state_write_u8(writer, value->region_mode)
        && nes_state_write_u8(writer, value->console_model)
        && nes_state_write_u8(writer, value->ram_power_state)
        && nes_state_write_u32(writer, value->power_random_state)
        && nes_state_write_bool(writer, value->randomize_vblank)
        && nes_state_write_u8(writer, value->startup_alignment_mode)
        && nes_state_write_u8(writer, value->configured_cpu_offset)
        && nes_state_write_u8(writer, value->configured_ppu_phase)
        && nes_state_write_u32(writer, value->startup_alignment_seed)
        && nes_state_write_bool(writer, value->cpu_test_mode)
        && nes_state_write_u8(writer, value->adapter)
        && nes_state_write_u8(writer, value->port_device[0])
        && nes_state_write_u8(writer, value->port_device[1])
        && nes_state_write_u8(writer, value->expansion_device)
        && nes_state_write_u8(writer, value->input_overrides)
        && nes_state_write_u32(writer, value->zapper_radius)
        && nes_state_write_bool(writer, value->vs_enabled)
        && nes_state_write_u16(writer, value->vs_dips)
        && nes_state_write_bool(writer, value->fds_active)
        && nes_state_write_bool(writer, value->fds_write_protected)
        && nes_state_write_u8(writer, value->apu_cpu_revision)
        && nes_state_write_bool(writer, value->apu_disable_noise)
        && nes_state_write_bool(writer, value->apu_swap_duty)
        && nes_state_write_u8(writer, value->ppu_revision)
        && nes_state_write_bool(writer, value->ppu_oam_row_corruption)
        && nes_state_write_bool(writer, value->ppu_startup_write_restriction)
        && nes_state_write_bool(writer, value->ppu_oam_decay)
        && nes_state_write_bool(writer, value->ppu_oamdata_read_disabled)
        && nes_state_write_bool(writer, value->ppu_palette_readback_disabled)
        && nes_state_write_bool(writer, value->ppu_reset_suppression)
        && nes_state_write_bool(writer, value->ppu_sprite_eval_wrap_bug)
        && nes_state_write_bool(writer, value->mmc3_revision_a)
        && nes_state_write_u32(writer, value->cart_dips)
        && nes_state_write_u32(writer, value->cheats_hash);
}

static bool movie_read_compatibility(NesStateReader *reader,
                                     MovieCompatibility *value) {
    memset(value, 0, sizeof(*value));
    return nes_state_read_u8(reader, &value->source)
        && nes_state_read_bytes(reader, value->header, sizeof(value->header))
        && nes_state_read_u32(reader, &value->file_crc)
        && nes_state_read_u32(reader, &value->prg_crc)
        && nes_state_read_u32(reader, &value->prg_chr_crc)
        && nes_state_read_u64(reader, &value->prg_size)
        && nes_state_read_u64(reader, &value->chr_size)
        && nes_state_read_u8(reader, &value->region)
        && nes_state_read_u8(reader, &value->region_mode)
        && nes_state_read_u8(reader, &value->console_model)
        && nes_state_read_u8(reader, &value->ram_power_state)
        && nes_state_read_u32(reader, &value->power_random_state)
        && nes_state_read_bool(reader, &value->randomize_vblank)
        && nes_state_read_u8(reader, &value->startup_alignment_mode)
        && nes_state_read_u8(reader, &value->configured_cpu_offset)
        && nes_state_read_u8(reader, &value->configured_ppu_phase)
        && nes_state_read_u32(reader, &value->startup_alignment_seed)
        && nes_state_read_bool(reader, &value->cpu_test_mode)
        && nes_state_read_u8(reader, &value->adapter)
        && nes_state_read_u8(reader, &value->port_device[0])
        && nes_state_read_u8(reader, &value->port_device[1])
        && nes_state_read_u8(reader, &value->expansion_device)
        && nes_state_read_u8(reader, &value->input_overrides)
        && nes_state_read_u32(reader, &value->zapper_radius)
        && nes_state_read_bool(reader, &value->vs_enabled)
        && nes_state_read_u16(reader, &value->vs_dips)
        && nes_state_read_bool(reader, &value->fds_active)
        && nes_state_read_bool(reader, &value->fds_write_protected)
        && nes_state_read_u8(reader, &value->apu_cpu_revision)
        && nes_state_read_bool(reader, &value->apu_disable_noise)
        && nes_state_read_bool(reader, &value->apu_swap_duty)
        && nes_state_read_u8(reader, &value->ppu_revision)
        && nes_state_read_bool(reader, &value->ppu_oam_row_corruption)
        && nes_state_read_bool(reader, &value->ppu_startup_write_restriction)
        && nes_state_read_bool(reader, &value->ppu_oam_decay)
        && nes_state_read_bool(reader, &value->ppu_oamdata_read_disabled)
        && nes_state_read_bool(reader, &value->ppu_palette_readback_disabled)
        && nes_state_read_bool(reader, &value->ppu_reset_suppression)
        && nes_state_read_bool(reader, &value->ppu_sprite_eval_wrap_bug)
        && nes_state_read_bool(reader, &value->mmc3_revision_a)
        && nes_state_read_u32(reader, &value->cart_dips)
        && nes_state_read_u32(reader, &value->cheats_hash);
}

static bool movie_bool_arg(int32_t value) {
    return value == 0 || value == 1;
}

static bool movie_digits_valid(const char *digits) {
    size_t count = digits ? strlen(digits) : 0;
    if (count != 8 && count != 13) return false;
    for (size_t i = 0; i < count; ++i)
        if (digits[i] < '0' || digits[i] > '9') return false;
    return true;
}

static bool movie_event_valid(const NesInputEvent *event,
                              const MovieCompatibility *compatibility) {
    if (!event || !compatibility
        || event->type < NES_INPUT_EVENT_PLAYER_BUTTON
        || event->type > NES_INPUT_EVENT_LAST) return false;
    if (!memchr(event->text, '\0', sizeof(event->text))) return false;
    switch (event->type) {
        case NES_INPUT_EVENT_PLAYER_BUTTON:
            return event->a >= 0 && event->a < NES_INPUT_PLAYERS
                && event->b >= BTN_A && event->b <= BTN_RIGHT && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_MICROPHONE:
            return movie_bool_arg(event->a);
        case NES_INPUT_EVENT_PADDLE:
            return event->a >= 0 && event->a < 3 && event->b >= 0x54 && event->b <= 0xF4
                && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_MAT:
            return event->a >= 0 && event->a < 3 && event->b >= 0 && event->b < 12
                && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_ZAPPER:
            return event->a >= 0 && event->a < 3 && movie_bool_arg(event->d);
        case NES_INPUT_EVENT_SUBOR_KEY:
            return event->a >= 0 && event->a < SUBOR_KEY_COUNT && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_SUBOR_MOUSE_MOTION:
            return compatibility->port_device[1] == NES_PORT_SUBOR_MOUSE;
        case NES_INPUT_EVENT_SUBOR_MOUSE_BUTTONS:
            return compatibility->port_device[1] == NES_PORT_SUBOR_MOUSE
                && movie_bool_arg(event->a) && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_SNES_BUTTON:
            return event->a >= 0 && event->a < 2
                && (compatibility->port_device[event->a] == NES_PORT_SNES_CONTROLLER
                    || compatibility->port_device[event->a] == NES_PORT_NTT_KEYPAD)
                && event->b >= 0 && event->b < SNES_BUTTON_COUNT && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_SNES_MOUSE_MOTION:
            return event->a >= 0 && event->a < 2
                && compatibility->port_device[event->a] == NES_PORT_SNES_MOUSE;
        case NES_INPUT_EVENT_SNES_MOUSE_BUTTONS:
            return event->a >= 0 && event->a < 2
                && compatibility->port_device[event->a] == NES_PORT_SNES_MOUSE
                && movie_bool_arg(event->b) && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_NTT_KEY:
            return event->a >= 0 && event->a < 2
                && compatibility->port_device[event->a] == NES_PORT_NTT_KEYPAD
                && event->b >= 0 && event->b < NTT_KEY_COUNT && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_FCNS_KEY:
            return compatibility->expansion_device == NES_EXPANSION_FCNS_CONTROLLER
                && event->a >= 0 && event->a < FCNS_KEY_COUNT && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_VIRTUAL_BOY_BUTTON:
            return event->a >= 0 && event->a < 2
                && compatibility->port_device[event->a] == NES_PORT_VIRTUAL_BOY
                && event->b >= 0 && event->b < VB_BUTTON_COUNT && movie_bool_arg(event->c);
        case NES_INPUT_EVENT_HORI_TRACK_MOTION:
            return compatibility->expansion_device == NES_EXPANSION_HORI_TRACK;
        case NES_INPUT_EVENT_PARTY_TAP:
            return compatibility->expansion_device == NES_EXPANSION_PARTY_TAP
                && event->a >= 0 && event->a < 6 && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_PACHINKO:
            return compatibility->expansion_device == NES_EXPANSION_PACHINKO
                && movie_bool_arg(event->a) && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_BOXING:
            return compatibility->expansion_device == NES_EXPANSION_EXCITING_BOXING
                && event->a >= 0 && event->a < 8 && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_JISSEN:
            return compatibility->expansion_device == NES_EXPANSION_JISSEN_MAHJONG
                && event->a >= 0 && event->a < JISSEN_KEY_COUNT && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_BARCODE_BATTLER:
            return compatibility->expansion_device == NES_EXPANSION_BARCODE_BATTLER
                && movie_digits_valid(event->text);
        case NES_INPUT_EVENT_OEKA_KIDS:
            return compatibility->expansion_device == NES_EXPANSION_OEKA_KIDS_TABLET
                && event->a >= -1 && event->a <= 255 && event->b >= -1 && event->b <= 239
                && movie_bool_arg(event->c) && movie_bool_arg(event->d);
        case NES_INPUT_EVENT_VS_COIN:
            return compatibility->vs_enabled && event->a >= 0
                && event->a < (vs_dual_system() ? 4 : 2) && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_VS_SERVICE:
            return compatibility->vs_enabled && event->a >= 0
                && event->a < (vs_dual_system() ? 2 : 1) && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_CART_BARCODE:
            return cart_barcode_supported() && movie_digits_valid(event->text);
        case NES_INPUT_EVENT_FAMILY_BASIC_KEY:
            return compatibility->expansion_device == NES_EXPANSION_FAMILY_BASIC
                && event->a >= 0 && event->a < FB_KEY_COUNT && movie_bool_arg(event->b);
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_PLAY:
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_RECORD:
        case NES_INPUT_EVENT_FAMILY_BASIC_TAPE_STOP:
            return compatibility->expansion_device == NES_EXPANSION_FAMILY_BASIC;
        case NES_INPUT_EVENT_SOFT_RESET:
        case NES_INPUT_EVENT_POWER_CYCLE:
            return true;
        case NES_INPUT_EVENT_FDS_INSERT:
            return compatibility->fds_active && event->a >= 0
                && (uint64_t)event->a < fds_side_count();
        case NES_INPUT_EVENT_FDS_EJECT:
            return compatibility->fds_active;
        case NES_INPUT_EVENT_CART_KARAOKE:
            return event->a >= 0 && event->a < CART_KARAOKE_INPUT_COUNT
                && movie_bool_arg(event->b);
        default:
            return false;
    }
}

static bool movie_reserve_events(MovieEvent **events, size_t *capacity,
                                 size_t count, size_t additional) {
    if (additional > MOVIE_MAX_EVENTS - count) return false;
    size_t needed = count + additional;
    if (needed <= *capacity) return true;
    size_t grown = *capacity ? *capacity : 64;
    while (grown < needed) {
        if (grown >= MOVIE_MAX_EVENTS / 2) {
            grown = MOVIE_MAX_EVENTS;
            break;
        }
        grown *= 2;
    }
    if (grown > SIZE_MAX / sizeof(**events)) return false;
    MovieEvent *replacement = (MovieEvent *)realloc(*events, grown * sizeof(**events));
    if (!replacement) return false;
    *events = replacement;
    *capacity = grown;
    return true;
}

static bool movie_reserve_pending(NesMovieSession *movie, size_t additional) {
    if (additional > MOVIE_MAX_EVENTS - movie->pending_count) return false;
    size_t needed = movie->pending_count + additional;
    if (needed <= movie->pending_capacity) return true;
    size_t grown = movie->pending_capacity ? movie->pending_capacity : 32;
    while (grown < needed) {
        if (grown >= MOVIE_MAX_EVENTS / 2) {
            grown = MOVIE_MAX_EVENTS;
            break;
        }
        grown *= 2;
    }
    if (grown > SIZE_MAX / sizeof(*movie->pending)) return false;
    NesInputEvent *replacement =
        (NesInputEvent *)realloc(movie->pending, grown * sizeof(*movie->pending));
    if (!replacement) return false;
    movie->pending = replacement;
    movie->pending_capacity = grown;
    return true;
}

static bool movie_input_observer(const NesInputEvent *event, void *userdata) {
    NesMovieSession *movie = (NesMovieSession *)userdata;
    if (!movie || movie->mode == NES_MOVIE_IDLE
        || !movie_event_valid(event, &movie->compatibility)) return false;
    if (movie->mode == NES_MOVIE_PLAYBACK) return false;
    if (movie->frame_in_progress || movie->recording_error != NES_MOVIE_OK) return false;
    if (movie->event_count + movie->pending_count >= MOVIE_MAX_EVENTS
        || !movie_reserve_pending(movie, 1)) {
        movie->recording_error = NES_MOVIE_LIMIT_REACHED;
        movie->last_result = movie->recording_error;
        return false;
    }
    movie->pending[movie->pending_count++] = *event;
    return true;
}

static NesMovieResult movie_prepare_resume(NesMovieSession *movie,
                                           NesStateBlob *captured) {
    NesStateResult state = nes_state_capture(captured);
    if (state == NES_STATE_ERROR_NO_IMAGE) return NES_MOVIE_NO_IMAGE;
    if (state == NES_STATE_ERROR_OUT_OF_MEMORY) return NES_MOVIE_OUT_OF_MEMORY;
    if (state != NES_STATE_OK) return NES_MOVIE_STATE_ERROR;
    state = nes_state_prepare_restore(captured->data, captured->size,
                                     &movie->resume_restore);
    if (state == NES_STATE_ERROR_OUT_OF_MEMORY) return NES_MOVIE_OUT_OF_MEMORY;
    return state == NES_STATE_OK ? NES_MOVIE_OK : NES_MOVIE_STATE_ERROR;
}

static NesMovieResult movie_restore_live(NesMovieSession *movie) {
    if (!movie || !movie->resume_restore) return NES_MOVIE_STATE_ERROR;
    NesStateResult state = nes_state_apply_prepared(movie->resume_restore);
    nes_state_restore_free(movie->resume_restore);
    movie->resume_restore = NULL;
    return state == NES_STATE_OK ? NES_MOVIE_OK : NES_MOVIE_STATE_ERROR;
}

static bool movie_write_event(NesStateWriter *writer, const MovieEvent *record,
                              const MovieCompatibility *compatibility) {
    return movie_event_valid(&record->event, compatibility)
        && nes_state_write_u64(writer, record->frame)
        && nes_state_write_u32(writer, (uint32_t)record->event.type)
        && nes_state_write_u32(writer, (uint32_t)record->event.a)
        && nes_state_write_u32(writer, (uint32_t)record->event.b)
        && nes_state_write_u32(writer, (uint32_t)record->event.c)
        && nes_state_write_u32(writer, (uint32_t)record->event.d)
        && nes_state_write_bytes(writer, record->event.text,
                                 sizeof(record->event.text));
}

static bool movie_read_event(NesStateReader *reader, MovieEvent *record,
                             const MovieCompatibility *compatibility) {
    uint32_t type, a, b, c, d;
    memset(record, 0, sizeof(*record));
    if (!nes_state_read_u64(reader, &record->frame)
        || !nes_state_read_u32(reader, &type)
        || !nes_state_read_u32(reader, &a)
        || !nes_state_read_u32(reader, &b)
        || !nes_state_read_u32(reader, &c)
        || !nes_state_read_u32(reader, &d)
        || !nes_state_read_bytes(reader, record->event.text,
                                 sizeof(record->event.text))) return false;
    record->event.type = (NesInputEventType)type;
    record->event.a = (int32_t)a;
    record->event.b = (int32_t)b;
    record->event.c = (int32_t)c;
    record->event.d = (int32_t)d;
    return movie_event_valid(&record->event, compatibility);
}

static uint32_t movie_crc32(const uint8_t *data, size_t size) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static NesMovieResult movie_write_file(NesMovieSession *movie) {
    if (!movie || !movie->path || !movie->start_state.data
        || movie->event_count > UINT32_MAX
        || movie->start_state.size > UINT32_MAX) return NES_MOVIE_STATE_ERROR;

    NesStateWriter writer;
    nes_state_writer_init(&writer, MOVIE_MAX_FILE_SIZE);
    bool ok = nes_state_write_bytes(&writer, movie_magic, sizeof(movie_magic))
        && nes_state_write_u32(&writer, MOVIE_VERSION)
        && nes_state_write_u8(&writer, (uint8_t)movie->start_kind)
        && movie_write_compatibility(&writer, &movie->compatibility)
        && nes_state_write_u64(&writer, movie->frame)
        && nes_state_write_u32(&writer, (uint32_t)movie->event_count)
        && nes_state_write_u32(&writer, (uint32_t)movie->start_state.size)
        && nes_state_write_bytes(&writer, movie->start_state.data,
                                 movie->start_state.size);
    for (size_t i = 0; ok && i < movie->event_count; ++i)
        ok = movie_write_event(&writer, &movie->events[i], &movie->compatibility);
    if (ok && !writer.failed) {
        uint32_t crc = movie_crc32(writer.data, writer.size);
        ok = nes_state_write_u32(&writer, crc);
    }
    if (!ok || writer.failed) {
        nes_state_writer_destroy(&writer);
        return NES_MOVIE_LIMIT_REACHED;
    }
    size_t size = 0;
    uint8_t *data = nes_state_writer_release(&writer, &size);
    if (!data && size) return NES_MOVIE_OUT_OF_MEMORY;
    NesFileResult file = nes_movie_save_atomic(movie->path, data, size);
    free(data);
    if (file == NES_FILE_OUT_OF_MEMORY) return NES_MOVIE_OUT_OF_MEMORY;
    if (file == NES_FILE_TOO_LARGE) return NES_MOVIE_LIMIT_REACHED;
    return file == NES_FILE_OK ? NES_MOVIE_OK : NES_MOVIE_IO_ERROR;
}

typedef struct {
    NesMovieStartKind start_kind;
    MovieCompatibility compatibility;
    uint64_t total_frames;
    MovieEvent *events;
    size_t event_count;
    NesStateRestore *start_restore;
} ParsedMovie;

static void movie_parsed_destroy(ParsedMovie *parsed) {
    if (!parsed) return;
    free(parsed->events);
    nes_state_restore_free(parsed->start_restore);
    memset(parsed, 0, sizeof(*parsed));
}

static NesMovieResult movie_parse_file(const char *path, ParsedMovie *parsed) {
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult file = nes_file_read_all(path, MOVIE_MAX_FILE_SIZE, &data, &size);
    if (file == NES_FILE_OUT_OF_MEMORY) return NES_MOVIE_OUT_OF_MEMORY;
    if (file == NES_FILE_TOO_LARGE) return NES_MOVIE_LIMIT_REACHED;
    if (file != NES_FILE_OK) return NES_MOVIE_IO_ERROR;

    if (size < sizeof(uint32_t)) {
        free(data);
        return NES_MOVIE_FORMAT_ERROR;
    }
    size_t payload_size = size - sizeof(uint32_t);
    uint32_t expected_crc = (uint32_t)data[payload_size]
        | ((uint32_t)data[payload_size + 1] << 8)
        | ((uint32_t)data[payload_size + 2] << 16)
        | ((uint32_t)data[payload_size + 3] << 24);
    if (movie_crc32(data, payload_size) != expected_crc) {
        free(data);
        return NES_MOVIE_CORRUPT;
    }

    NesStateReader reader;
    nes_state_reader_init(&reader, data, payload_size);
    uint8_t magic[sizeof(movie_magic)];
    uint32_t version, event_count, state_size;
    uint8_t start_kind;
    bool ok = nes_state_read_bytes(&reader, magic, sizeof(magic))
        && nes_state_read_u32(&reader, &version);
    if (!ok || memcmp(magic, movie_magic, sizeof(magic)) != 0) {
        free(data);
        return NES_MOVIE_FORMAT_ERROR;
    }
    if (version != MOVIE_VERSION) {
        free(data);
        return NES_MOVIE_VERSION_ERROR;
    }
    ok = nes_state_read_u8(&reader, &start_kind)
        && start_kind <= NES_MOVIE_START_POWER_ON
        && movie_read_compatibility(&reader, &parsed->compatibility)
        && nes_state_read_u64(&reader, &parsed->total_frames)
        && nes_state_read_u32(&reader, &event_count)
        && nes_state_read_u32(&reader, &state_size);
    if (!ok || event_count > MOVIE_MAX_EVENTS
        || state_size > NES_STATE_MAX_SIZE
        || state_size > nes_state_reader_remaining(&reader)) {
        free(data);
        return NES_MOVIE_FORMAT_ERROR;
    }
    parsed->start_kind = (NesMovieStartKind)start_kind;
    if (!movie_compatibility_matches(&parsed->compatibility)) {
        free(data);
        return NES_MOVIE_INCOMPATIBLE;
    }

    NesStateReader state_reader;
    if (!nes_state_reader_slice(&reader, state_size, &state_reader)) {
        free(data);
        return NES_MOVIE_FORMAT_ERROR;
    }
    NesStateResult state = nes_state_prepare_restore(state_reader.data,
                                                     state_reader.size,
                                                     &parsed->start_restore);
    if (state != NES_STATE_OK) {
        free(data);
        if (state == NES_STATE_ERROR_OUT_OF_MEMORY) return NES_MOVIE_OUT_OF_MEMORY;
        if (state == NES_STATE_ERROR_INCOMPATIBLE) return NES_MOVIE_INCOMPATIBLE;
        return NES_MOVIE_STATE_ERROR;
    }

    if (event_count) {
        if (sizeof(*parsed->events) > SIZE_MAX / (size_t)event_count) {
            free(data);
            movie_parsed_destroy(parsed);
            return NES_MOVIE_LIMIT_REACHED;
        }
        parsed->events = (MovieEvent *)calloc(event_count, sizeof(*parsed->events));
        if (!parsed->events) {
            free(data);
            movie_parsed_destroy(parsed);
            return NES_MOVIE_OUT_OF_MEMORY;
        }
    }
    parsed->event_count = event_count;
    uint64_t previous_frame = 0;
    for (size_t i = 0; i < parsed->event_count; ++i) {
        if (!movie_read_event(&reader, &parsed->events[i], &parsed->compatibility)
            || parsed->events[i].frame >= parsed->total_frames
            || (i && parsed->events[i].frame < previous_frame)) {
            free(data);
            movie_parsed_destroy(parsed);
            return NES_MOVIE_FORMAT_ERROR;
        }
        previous_frame = parsed->events[i].frame;
    }
    if (nes_state_reader_remaining(&reader) != 0) {
        free(data);
        movie_parsed_destroy(parsed);
        return NES_MOVIE_FORMAT_ERROR;
    }
    free(data);
    return NES_MOVIE_OK;
}

NesMovieSession *nes_movie_create(void) {
    NesMovieSession *movie = (NesMovieSession *)calloc(1, sizeof(*movie));
    if (movie) {
        movie->last_result = NES_MOVIE_OK;
        movie->tas = nes_tas_session_create();
        if (!movie->tas) {
            free(movie);
            return NULL;
        }
    }
    return movie;
}

void nes_movie_destroy(NesMovieSession *movie) {
    if (!movie) return;
    nes_tas_session_destroy(movie->tas);
    movie->tas = NULL;
    if (movie->mode != NES_MOVIE_IDLE) {
        nes_input_event_clear_observer();
        (void)movie_restore_live(movie);
        (void)nes_execution_set_policy(movie->previous_policy);
    }
    movie_clear_recording_data(movie);
    free(movie);
}

static bool frame_movie_extension(const char *path) {
    size_t length = path ? strlen(path) : 0;
    const char *extensions[] = {".fm2", ".fm3", ".ctas"};
    for (size_t i = 0; i < sizeof(extensions) / sizeof(extensions[0]); ++i) {
        size_t suffix = strlen(extensions[i]);
        if (length < suffix) continue;
        bool match = true;
        for (size_t j = 0; j < suffix; ++j)
            match = match && tolower((unsigned char)path[length - suffix + j]) == extensions[i][j];
        if (match) return true;
    }
    return false;
}

static bool frame_movie_file(const char *path) {
    FILE *file = nes_file_open(path, "rb");
    if (!file) return frame_movie_extension(path);
    uint8_t header[9] = {0};
    size_t size = fread(header, 1, sizeof(header), file);
    fclose(file);
    if (size >= sizeof(movie_magic) && !memcmp(header, movie_magic, sizeof(movie_magic))) return false;
    return (size == sizeof(header) && !memcmp(header, "version 3", sizeof(header))) ||
           (size >= 8 && !memcmp(header, "CUPIDTAS", 8)) || frame_movie_extension(path);
}

static NesMovieResult movie_record_start_kind(NesMovieSession *movie, const char *path,
                                              NesMovieStartKind start_kind) {
    if (!movie || !path || !*path) return movie_set_result(movie, NES_MOVIE_INVALID_ARGUMENT);
    if (start_kind != NES_MOVIE_START_STATE && start_kind != NES_MOVIE_START_POWER_ON)
        return movie_set_result(movie, NES_MOVIE_INVALID_ARGUMENT);
    if (movie->mode != NES_MOVIE_IDLE || nes_execution_policy() != NES_EXECUTION_LIVE)
        return movie_set_result(movie, NES_MOVIE_CONFLICT);
    if (!nes_replay_host_state_supported())
        return movie_set_result(movie, NES_MOVIE_UNSUPPORTED_HOST_STATE);
    if (rom_metadata_source() == ROM_METADATA_NONE)
        return movie_set_result(movie, NES_MOVIE_NO_IMAGE);

    if (frame_movie_extension(path)) {
        if (start_kind != NES_MOVIE_START_POWER_ON) {
            movie_set_result(movie, NES_MOVIE_INCOMPATIBLE);
            snprintf(movie->error, sizeof(movie->error),
                     "Choose Power-on for FM2/FM3 recording. Current-state recordings use Cupid's .cmv format.");
            return NES_MOVIE_INCOMPATIBLE;
        }
        return movie_tas_result(movie, nes_tas_session_new(movie->tas, path, true));
    }

    char *path_copy = movie_strdup(path);
    if (!path_copy) return movie_set_result(movie, NES_MOVIE_OUT_OF_MEMORY);
    NesStateBlob live = {0};
    NesMovieResult result = movie_prepare_resume(movie, &live);
    if (result != NES_MOVIE_OK) {
        free(path_copy);
        nes_state_blob_free(&live);
        return movie_set_result(movie, result);
    }

    MovieCompatibility compatibility = movie_current_compatibility();
    movie->previous_policy = nes_execution_policy();
    if (!nes_execution_set_policy(movie->previous_policy | NES_EXECUTION_MOVIE_RECORDING)) {
        free(path_copy);
        nes_state_blob_free(&live);
        nes_state_restore_free(movie->resume_restore);
        movie->resume_restore = NULL;
        return movie_set_result(movie, NES_MOVIE_CONFLICT);
    }

    NesStateBlob start = {0};
    if (start_kind == NES_MOVIE_START_POWER_ON) {
        if (!frontend_machine_power_cycle()) result = NES_MOVIE_STATE_ERROR;
        else {
            NesStateResult state = nes_state_capture(&start);
            result = state == NES_STATE_OK ? NES_MOVIE_OK
                   : state == NES_STATE_ERROR_OUT_OF_MEMORY ? NES_MOVIE_OUT_OF_MEMORY
                   : NES_MOVIE_STATE_ERROR;
        }
        nes_state_blob_free(&live);
        if (result != NES_MOVIE_OK) {
            NesMovieResult restored = movie_restore_live(movie);
            (void)nes_execution_set_policy(movie->previous_policy);
            nes_state_blob_free(&start);
            free(path_copy);
            return movie_set_result(movie, restored == NES_MOVIE_OK ? result : restored);
        }
    } else {
        start = live;
        memset(&live, 0, sizeof(live));
    }

    movie->path = path_copy;
    movie->start_state = start;
    movie->compatibility = compatibility;
    movie->start_kind = start_kind;
    movie->mode = NES_MOVIE_RECORDING;
    movie->frame = 0;
    movie->total_frames = 0;
    movie->last_result = NES_MOVIE_OK;
    movie->recording_error = NES_MOVIE_OK;
    movie->frame_in_progress = false;
    nes_input_event_set_observer(movie_input_observer, movie);
    return NES_MOVIE_OK;
}

NesMovieResult nes_movie_record_start(NesMovieSession *movie, const char *path) {
    return movie_record_start_kind(movie, path, NES_MOVIE_START_STATE);
}

NesMovieResult nes_movie_record_start_power_on(NesMovieSession *movie, const char *path) {
    return movie_record_start_kind(movie, path, NES_MOVIE_START_POWER_ON);
}

NesMovieResult nes_movie_play_start(NesMovieSession *movie, const char *path) {
    if (!movie || !path || !*path) return movie_set_result(movie, NES_MOVIE_INVALID_ARGUMENT);
    if (movie->mode != NES_MOVIE_IDLE || nes_execution_policy() != NES_EXECUTION_LIVE)
        return movie_set_result(movie, NES_MOVIE_CONFLICT);
    if (!nes_replay_host_state_supported())
        return movie_set_result(movie, NES_MOVIE_UNSUPPORTED_HOST_STATE);
    if (rom_metadata_source() == ROM_METADATA_NONE)
        return movie_set_result(movie, NES_MOVIE_NO_IMAGE);

    if (frame_movie_file(path))
        return movie_tas_result(movie, nes_tas_session_open(movie->tas, path));

    ParsedMovie parsed;
    memset(&parsed, 0, sizeof(parsed));
    NesMovieResult result = movie_parse_file(path, &parsed);
    if (result != NES_MOVIE_OK) return movie_set_result(movie, result);

    char *path_copy = movie_strdup(path);
    if (!path_copy) {
        movie_parsed_destroy(&parsed);
        return movie_set_result(movie, NES_MOVIE_OUT_OF_MEMORY);
    }
    NesStateBlob live = {0};
    result = movie_prepare_resume(movie, &live);
    nes_state_blob_free(&live);
    if (result != NES_MOVIE_OK) {
        free(path_copy);
        movie_parsed_destroy(&parsed);
        return movie_set_result(movie, result);
    }

    movie->previous_policy = nes_execution_policy();
    if (!nes_execution_set_policy(movie->previous_policy | NES_EXECUTION_MOVIE_PLAYBACK)) {
        free(path_copy);
        movie_parsed_destroy(&parsed);
        nes_state_restore_free(movie->resume_restore);
        movie->resume_restore = NULL;
        return movie_set_result(movie, NES_MOVIE_CONFLICT);
    }
    if (parsed.start_kind == NES_MOVIE_START_POWER_ON && !frontend_machine_power_cycle()) {
        (void)movie_restore_live(movie);
        (void)nes_execution_set_policy(movie->previous_policy);
        free(path_copy);
        movie_parsed_destroy(&parsed);
        return movie_set_result(movie, NES_MOVIE_STATE_ERROR);
    }
    NesStateResult state = nes_state_apply_prepared(parsed.start_restore);
    nes_state_restore_free(parsed.start_restore);
    parsed.start_restore = NULL;
    if (state != NES_STATE_OK) {
        (void)movie_restore_live(movie);
        nes_input_event_clear_observer();
        (void)nes_execution_set_policy(movie->previous_policy);
        free(path_copy);
        movie_parsed_destroy(&parsed);
        return movie_set_result(movie, NES_MOVIE_STATE_ERROR);
    }

    movie->path = path_copy;
    movie->start_kind = parsed.start_kind;
    movie->events = parsed.events;
    movie->event_count = parsed.event_count;
    movie->event_capacity = parsed.event_count;
    parsed.events = NULL;
    movie->total_frames = parsed.total_frames;
    movie->frame = 0;
    movie->playback_index = 0;
    movie->mode = NES_MOVIE_PLAYBACK;
    movie->last_result = NES_MOVIE_OK;
    movie->recording_error = NES_MOVIE_OK;
    movie->frame_in_progress = false;
    movie_parsed_destroy(&parsed);
    nes_input_event_set_observer(movie_input_observer, movie);
    return NES_MOVIE_OK;
}

NesMovieResult nes_movie_frame_boundary(NesMovieSession *movie) {
    if (!movie) return NES_MOVIE_INVALID_ARGUMENT;
    if (nes_tas_session_active(movie->tas))
        return movie_tas_result(movie, nes_tas_session_frame_boundary(movie->tas));
    if (movie->mode == NES_MOVIE_IDLE)
        return movie_set_result(movie, NES_MOVIE_CONFLICT);
    if (movie->frame_in_progress) return movie_set_result(movie, NES_MOVIE_OK);
    if (movie->mode == NES_MOVIE_RECORDING) {
        if (movie->recording_error != NES_MOVIE_OK)
            return movie_set_result(movie, movie->recording_error);
        if (movie->pending_count) {
            if (!movie_reserve_events(&movie->events, &movie->event_capacity,
                                      movie->event_count, movie->pending_count)) {
                return movie_set_result(movie, NES_MOVIE_LIMIT_REACHED);
            }
            for (size_t i = 0; i < movie->pending_count; ++i) {
                movie->events[movie->event_count + i].frame = movie->frame;
                movie->events[movie->event_count + i].event = movie->pending[i];
            }
            movie->event_count += movie->pending_count;
            movie->pending_count = 0;
        }
        movie->frame_in_progress = true;
        return movie_set_result(movie, NES_MOVIE_OK);
    }

    if (movie->frame >= movie->total_frames)
        return movie_set_result(movie, NES_MOVIE_COMPLETE);
    while (movie->playback_index < movie->event_count
           && movie->events[movie->playback_index].frame == movie->frame) {
        if (!nes_input_event_apply(&movie->events[movie->playback_index].event))
            return movie_set_result(movie, NES_MOVIE_EVENT_ERROR);
        movie->playback_index++;
    }
    movie->frame_in_progress = true;
    return movie_set_result(movie, NES_MOVIE_OK);
}

NesMovieResult nes_movie_frame_complete(NesMovieSession *movie, bool completed) {
    if (!movie) return NES_MOVIE_INVALID_ARGUMENT;
    if (nes_tas_session_active(movie->tas))
        return movie_tas_result(movie, nes_tas_session_frame_complete(movie->tas, completed));
    if (movie->mode == NES_MOVIE_IDLE) return movie_set_result(movie, NES_MOVIE_CONFLICT);
    if (!movie->frame_in_progress) return movie_set_result(movie, NES_MOVIE_OK);
    if (!completed) return movie_set_result(movie, NES_MOVIE_OK);
    movie->frame_in_progress = false;
    movie->frame++;
    if (movie->mode == NES_MOVIE_RECORDING) movie->total_frames = movie->frame;
    return movie_set_result(movie, NES_MOVIE_OK);
}

NesMovieResult nes_movie_set_path(NesMovieSession *movie, const char *path) {
    if (!movie || !path || !*path) return movie_set_result(movie, NES_MOVIE_INVALID_ARGUMENT);
    if (nes_tas_session_active(movie->tas))
        return movie_tas_result(movie, nes_tas_session_set_path(movie->tas, path));
    if (movie->mode == NES_MOVIE_PLAYBACK) return movie_set_result(movie, NES_MOVIE_CONFLICT);
    char *replacement = movie_strdup(path);
    if (!replacement) return movie_set_result(movie, NES_MOVIE_OUT_OF_MEMORY);
    free(movie->path);
    movie->path = replacement;
    return movie_set_result(movie, NES_MOVIE_OK);
}

NesMovieResult nes_movie_stop(NesMovieSession *movie) {
    if (!movie) return NES_MOVIE_INVALID_ARGUMENT;
    if (nes_tas_session_active(movie->tas))
        return movie_tas_result(movie, nes_tas_session_stop(movie->tas));
    if (movie->mode == NES_MOVIE_IDLE) return movie_set_result(movie, NES_MOVIE_CONFLICT);
    if (movie->mode == NES_MOVIE_RECORDING) {
        /* A breakpoint or an input limit may leave the last frame incomplete.
         * Save the completed prefix, while retaining every event for a retry
         * if the destination cannot be written. */
        size_t original_count = movie->event_count;
        while (movie->event_count
               && movie->events[movie->event_count - 1].frame >= movie->frame)
            --movie->event_count;
        NesMovieResult write = movie_write_file(movie);
        movie->event_count = original_count;
        if (write != NES_MOVIE_OK) return movie_set_result(movie, write);
    }

    nes_input_event_clear_observer();
    NesMovieResult restore = movie_restore_live(movie);
    bool policy_ok = nes_execution_set_policy(movie->previous_policy);
    movie->mode = NES_MOVIE_IDLE;
    movie_clear_recording_data(movie);
    if (restore != NES_MOVIE_OK) return movie_set_result(movie, restore);
    if (!policy_ok) return movie_set_result(movie, NES_MOVIE_CONFLICT);
    return movie_set_result(movie, NES_MOVIE_OK);
}

NesMovieMode nes_movie_mode(const NesMovieSession *movie) {
    return movie && nes_tas_session_active(movie->tas) ? nes_tas_session_mode(movie->tas) :
           movie ? movie->mode : NES_MOVIE_IDLE;
}

NesTasSession *nes_movie_tas(NesMovieSession *movie) { return movie ? movie->tas : NULL; }
const NesTasSession *nes_movie_tas_const(const NesMovieSession *movie) { return movie ? movie->tas : NULL; }
const char *nes_movie_error(const NesMovieSession *movie) {
    if (movie && movie->error[0]) return movie->error;
    return nes_movie_result_string(movie ? movie->last_result : NES_MOVIE_INVALID_ARGUMENT);
}

void nes_movie_progress(const NesMovieSession *movie, NesMovieProgress *progress) {
    if (!progress) return;
    memset(progress, 0, sizeof(*progress));
    if (!movie) {
        progress->last_result = NES_MOVIE_INVALID_ARGUMENT;
        return;
    }
    if (nes_tas_session_active(movie->tas)) {
        NesTasProgress tas;
        nes_tas_session_progress(movie->tas, &tas);
        progress->mode = nes_tas_session_mode(movie->tas);
        progress->start_kind = NES_MOVIE_START_POWER_ON;
        progress->frame = tas.frame;
        progress->total_frames = tas.total_frames;
        progress->path = tas.path;
        progress->last_result = movie->last_result;
        return;
    }
    progress->mode = movie->mode;
    progress->start_kind = movie->start_kind;
    progress->frame = movie->frame;
    progress->total_frames = movie->total_frames;
    progress->event_count = movie->event_count;
    progress->path = movie->path;
    progress->last_result = movie->last_result;
}

const char *nes_movie_result_string(NesMovieResult result) {
    switch (result) {
        case NES_MOVIE_OK: return "ok";
        case NES_MOVIE_COMPLETE: return "movie complete";
        case NES_MOVIE_INVALID_ARGUMENT: return "invalid argument";
        case NES_MOVIE_NO_IMAGE: return "no image is loaded";
        case NES_MOVIE_CONFLICT: return "replay mode conflict";
        case NES_MOVIE_UNSUPPORTED_HOST_STATE: return "active host state cannot be replayed safely";
        case NES_MOVIE_OUT_OF_MEMORY: return "out of memory";
        case NES_MOVIE_IO_ERROR: return "movie file I/O failed";
        case NES_MOVIE_FORMAT_ERROR: return "invalid or truncated movie";
        case NES_MOVIE_VERSION_ERROR: return "unsupported movie version";
        case NES_MOVIE_INCOMPATIBLE: return "movie is incompatible with the active machine";
        case NES_MOVIE_CORRUPT: return "movie integrity check failed";
        case NES_MOVIE_STATE_ERROR: return "movie state operation failed";
        case NES_MOVIE_EVENT_ERROR: return "recorded input event could not be applied";
        case NES_MOVIE_LIMIT_REACHED: return "movie size or event limit reached";
        default: return "unknown movie result";
    }
}
