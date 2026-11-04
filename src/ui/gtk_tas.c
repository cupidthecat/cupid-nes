/*
 * gtk_tas.c - Native tool-assisted input editor
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifdef CUPID_GTK
#include "gtk_internal.h"
#include "desktop_tas_internal.h"
#include "../system/vs_system.h"
#include <stdio.h>
#include <string.h>

/* GtkTreeView is available in GTK 4.8. A lazy, flat model avoids duplicating
 * the movie in a GtkListStore or allocating a widget/object for every frame.
 * Only the native view's visible cells query frame data. */
typedef struct {
    GObject parent;
    int count;
} TasRows;

typedef struct {
    GObjectClass parent;
} TasRowsClass;

static void tas_rows_interface(GtkTreeModelIface *iface);
G_DEFINE_TYPE_WITH_CODE(TasRows, tas_rows, G_TYPE_OBJECT,
                        G_IMPLEMENT_INTERFACE(GTK_TYPE_TREE_MODEL, tas_rows_interface))

static void tas_rows_class_init(TasRowsClass *klass) {
    (void)klass;
}

static void tas_rows_init(TasRows *rows) {
    rows->count = 0;
}

static GtkTreeModelFlags rows_flags(GtkTreeModel *model) {
    (void)model;
    return GTK_TREE_MODEL_LIST_ONLY | GTK_TREE_MODEL_ITERS_PERSIST;
}

static int rows_columns(GtkTreeModel *model) {
    (void)model;
    return 1;
}

static GType rows_type(GtkTreeModel *model, int column) {
    (void)model;
    return column == 0 ? G_TYPE_UINT64 : G_TYPE_INVALID;
}

static gboolean rows_at(GtkTreeModel *model, GtkTreeIter *iter, int row) {
    if (row < 0 || row >= ((TasRows *)model)->count) {
        iter->stamp = 0;
        return FALSE;
    }
    *iter = (GtkTreeIter){.stamp = 1, .user_data = GINT_TO_POINTER(row + 1)};
    return TRUE;
}

static int rows_index(const GtkTreeIter *iter) {
    return GPOINTER_TO_INT(iter->user_data) - 1;
}

static gboolean rows_iter(GtkTreeModel *model, GtkTreeIter *iter, GtkTreePath *path) {
    return gtk_tree_path_get_depth(path) == 1 && rows_at(model, iter, gtk_tree_path_get_indices(path)[0]);
}

static GtkTreePath *rows_path(GtkTreeModel *model, GtkTreeIter *iter) {
    int row = rows_index(iter);
    return iter->stamp == 1 && row >= 0 && row < ((TasRows *)model)->count ? gtk_tree_path_new_from_indices(row, -1)
                                                                           : NULL;
}

static void rows_value(GtkTreeModel *model, GtkTreeIter *iter, int column, GValue *value) {
    (void)model;
    (void)column;
    g_value_init(value, G_TYPE_UINT64);
    g_value_set_uint64(value, (guint64)rows_index(iter));
}

static gboolean rows_next(GtkTreeModel *model, GtkTreeIter *iter) {
    return rows_at(model, iter, rows_index(iter) + 1);
}

static gboolean rows_previous(GtkTreeModel *model, GtkTreeIter *iter) {
    return rows_at(model, iter, rows_index(iter) - 1);
}

static gboolean rows_children(GtkTreeModel *model, GtkTreeIter *iter, GtkTreeIter *parent) {
    return !parent && rows_at(model, iter, 0);
}

static gboolean rows_has_child(GtkTreeModel *model, GtkTreeIter *iter) {
    (void)model;
    (void)iter;
    return FALSE;
}

static int rows_count(GtkTreeModel *model, GtkTreeIter *iter) {
    return iter ? 0 : ((TasRows *)model)->count;
}

static gboolean rows_nth(GtkTreeModel *model, GtkTreeIter *iter, GtkTreeIter *parent, int n) {
    return !parent && rows_at(model, iter, n);
}

static gboolean rows_parent(GtkTreeModel *model, GtkTreeIter *iter, GtkTreeIter *child) {
    (void)model;
    (void)iter;
    (void)child;
    return FALSE;
}

static void tas_rows_interface(GtkTreeModelIface *iface) {
    iface->get_flags = rows_flags;
    iface->get_n_columns = rows_columns;
    iface->get_column_type = rows_type;
    iface->get_iter = rows_iter;
    iface->get_path = rows_path;
    iface->get_value = rows_value;
    iface->iter_next = rows_next;
    iface->iter_previous = rows_previous;
    iface->iter_children = rows_children;
    iface->iter_has_child = rows_has_child;
    iface->iter_n_children = rows_count;
    iface->iter_nth_child = rows_nth;
    iface->iter_parent = rows_parent;
}

typedef struct {
    GtkTreeView parent;
} CupidTasGrid;

typedef GtkTreeViewClass CupidTasGridClass;
G_DEFINE_TYPE(CupidTasGrid, cupid_tas_grid, GTK_TYPE_TREE_VIEW)

static void grid_snapshot(GtkWidget *widget, GtkSnapshot *snapshot) {
    int width = gtk_widget_get_width(widget), height = gtk_widget_get_height(widget);
    if (width <= 0 || height <= 0) {
        return;
    }
    GtkSnapshot *contents = gtk_snapshot_new();
    GTK_WIDGET_CLASS(cupid_tas_grid_parent_class)->snapshot(widget, contents);
    GskRenderNode *node = gtk_snapshot_free_to_node(contents);
    if (!node) {
        return;
    }

    /* GTK retains this snapshot until the grid changes. Rasterize its text
     * and dotted separators once, so each live preview frame can reuse one
     * texture instead of submitting every cell and repeated line to the GPU. */
    int scale = gtk_widget_get_scale_factor(widget);
    cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width * scale, height * scale);
    if (cairo_surface_status(surface) != CAIRO_STATUS_SUCCESS) {
        gtk_snapshot_append_node(snapshot, node);
        cairo_surface_destroy(surface);
        gsk_render_node_unref(node);
        return;
    }
    cairo_t *cr = cairo_create(surface);
    cairo_scale(cr, scale, scale);
    gsk_render_node_draw(node, cr);
    cairo_destroy(cr);
    gsk_render_node_unref(node);
    cairo_surface_flush(surface);
    int stride = cairo_image_surface_get_stride(surface);
    GBytes *bytes = g_bytes_new(cairo_image_surface_get_data(surface), (size_t)stride * height * scale);
    GdkTexture *texture = gdk_memory_texture_new(width * scale, height * scale, GDK_MEMORY_DEFAULT, bytes, stride);
    graphene_rect_t bounds = GRAPHENE_RECT_INIT(0, 0, width, height);
    gtk_snapshot_append_texture(snapshot, texture, &bounds);
    g_object_unref(texture);
    g_bytes_unref(bytes);
    cairo_surface_destroy(surface);
}

static void cupid_tas_grid_class_init(CupidTasGridClass *klass) {
    GTK_WIDGET_CLASS(klass)->snapshot = grid_snapshot;
}

static void cupid_tas_grid_init(CupidTasGrid *grid) {
    (void)grid;
}

enum { COLUMN_FRAME = -2, COLUMN_MARKER = -1, COLUMN_ZAPPER = 64, PAGE_ROWS = 8 };

typedef struct {
    CupidGtkTool *tool;
    GtkWidget *root, *body, *tree, *picture, *notebook, *summary, *empty;
    GtkWidget *play, *record, *readonly, *follow, *prompt, *entry, *prompt_title, *prompt_error;
    GtkWidget *branch[NES_TAS_BOOKMARK_COUNT], *history[PAGE_ROWS], *navigation[PAGE_ROWS], *markers[PAGE_ROWS];
    GtkWidget *details[7];
    GtkTreeViewColumn *pad_columns[32], *command_columns[7], *zapper_columns[2];
    GSimpleActionGroup *actions;
    const NesTasProject *project, *prompt_project;
    uint64_t generation, prompt_generation, prompt_revision;
    size_t marker_page, playback, count;
    unsigned prompt_control;
    gboolean updating, prompting, drag, disposing;
    gint64 next_metadata;
    int drag_column;
    GdkRGBA selected_bg, selected_fg;
} GtkTas;

/* GtkBox disposal releases descendants before the root's data destroy notify.
 * Stop callbacks before that release: notebook page removal can emit signals
 * while a sibling picture is already gone. */
typedef struct {
    GtkBox parent;
} CupidTasRoot;

typedef struct {
    GtkBoxClass parent;
} CupidTasRootClass;
G_DEFINE_TYPE(CupidTasRoot, cupid_tas_root, GTK_TYPE_BOX)

static void disconnect_widgets(GtkWidget *widget, GtkTas *tas) {
    g_signal_handlers_disconnect_by_data(widget, tas);
    GListModel *controllers = gtk_widget_observe_controllers(widget);
    for (guint i = 0; i < g_list_model_get_n_items(controllers); ++i) {
        GObject *controller = g_list_model_get_item(controllers, i);
        g_signal_handlers_disconnect_by_data(controller, tas);
        g_object_unref(controller);
    }
    g_object_unref(controllers);
    if (GTK_IS_TREE_VIEW(widget)) {
        GList *columns = gtk_tree_view_get_columns(GTK_TREE_VIEW(widget));
        for (GList *entry = columns; entry; entry = entry->next) {
            GtkTreeViewColumn *column = entry->data;
            g_signal_handlers_disconnect_by_data(column, tas);
            GList *cells = gtk_cell_layout_get_cells(GTK_CELL_LAYOUT(column));
            for (GList *cell = cells; cell; cell = cell->next) {
                gtk_tree_view_column_set_cell_data_func(column, cell->data, NULL, NULL, NULL);
            }
            g_list_free(cells);
        }
        g_list_free(columns);
    }
    for (GtkWidget *child = gtk_widget_get_first_child(widget); child; child = gtk_widget_get_next_sibling(child)) {
        disconnect_widgets(child, tas);
    }
}

static void cupid_tas_root_dispose(GObject *object) {
    GtkTas *tas = g_object_get_data(object, "cupid-tas");
    if (tas && !tas->disposing) {
        tas->disposing = TRUE;
        desktop_tas_finish_paint(&tas->tool->ui);
        tas->drag = FALSE;
        tas->tool->ui.edit_text_active = FALSE;
        disconnect_widgets(GTK_WIDGET(object), tas);
        GAction *action = g_action_map_lookup_action(G_ACTION_MAP(tas->actions), "run");
        if (action) {
            g_signal_handlers_disconnect_by_data(action, tas);
        }
        gtk_widget_insert_action_group(GTK_WIDGET(object), "tas", NULL);
    }
    G_OBJECT_CLASS(cupid_tas_root_parent_class)->dispose(object);
}

static void cupid_tas_root_class_init(CupidTasRootClass *klass) {
    G_OBJECT_CLASS(klass)->dispose = cupid_tas_root_dispose;
}

static void cupid_tas_root_init(CupidTasRoot *root) {
    gtk_orientable_set_orientation(GTK_ORIENTABLE(root), GTK_ORIENTATION_VERTICAL);
    gtk_box_set_spacing(GTK_BOX(root), 6);
}

typedef struct {
    const char *menu, *label;
    unsigned action;
} TasMenu;

static const TasMenu menu_items[] = {{"File", "Open project or movie…", TAS_ACTION_OPEN},
                                     {"File", "New project…", TAS_ACTION_NEW},
                                     {"File", "Save", TAS_ACTION_SAVE},
                                     {"File", "Save as…", TAS_ACTION_SAVE_AS},
                                     {"File", "Export movie…", TAS_ACTION_EXPORT},
                                     {"File", "Run Lua script…", TAS_ACTION_SCRIPT},
                                     {"File", "Stop and restore live game", TAS_ACTION_STOP},
                                     {"File", "Discard unsaved project", TAS_ACTION_DISCARD},
                                     {"Edit", "Undo", TAS_ACTION_UNDO},
                                     {"Edit", "Redo", TAS_ACTION_REDO},
                                     {"Edit", "Copy", TAS_ACTION_COPY},
                                     {"Edit", "Cut", TAS_ACTION_CUT},
                                     {"Edit", "Paste overwrite", TAS_ACTION_PASTE},
                                     {"Edit", "Paste insert", TAS_ACTION_PASTE_INSERT},
                                     {"Edit", "Insert one frame", TAS_ACTION_INSERT},
                                     {"Edit", "Insert frames…", TAS_ACTION_INSERT_MANY},
                                     {"Edit", "Delete frames", TAS_ACTION_DELETE},
                                     {"Edit", "Clone frames", TAS_ACTION_CLONE},
                                     {"Edit", "Truncate after cursor", TAS_ACTION_TRUNCATE},
                                     {"Edit", "Select all", TAS_ACTION_SELECT_ALL},
                                     {"Edit", "Clear selection", TAS_ACTION_SELECT_NONE},
                                     {"Edit", "Select between markers", TAS_ACTION_SELECT_BETWEEN_MARKERS},
                                     {"Navigate", "Go to frame…", TAS_ACTION_SEEK},
                                     {"Navigate", "Seek to cursor", TAS_ACTION_SEEK_CURSOR},
                                     {"Navigate", "Cursor to playback", TAS_ACTION_CURSOR_PLAYBACK},
                                     {"Navigate", "Follow playback", TAS_ACTION_FOLLOW},
                                     {"Navigate", "Previous marker", TAS_ACTION_MARKER_PREV},
                                     {"Navigate", "Next marker", TAS_ACTION_MARKER_NEXT},
                                     {"Navigate", "Previous checkpoint", TAS_ACTION_CACHE_PREV},
                                     {"Navigate", "Next checkpoint", TAS_ACTION_CACHE_NEXT},
                                     {"Input", "Recording insert / overwrite", TAS_ACTION_RECORD_MODE},
                                     {"Input", "Skip known lag for patterns", TAS_ACTION_SKIP_LAG},
                                     {"Input", "Apply pattern", TAS_ACTION_PATTERN},
                                     {"Input", "Next pattern", TAS_ACTION_PATTERN_NEXT},
                                     {"Input", "Toggle held button", TAS_ACTION_HOLD},
                                     {"Input", "Toggle autofire button", TAS_ACTION_AUTOFIRE},
                                     {"Input", "Edit player 1", TAS_ACTION_EDIT_PLAYER_BASE},
                                     {"Input", "Edit player 2", TAS_ACTION_EDIT_PLAYER_BASE + 1},
                                     {"Input", "Edit player 3", TAS_ACTION_EDIT_PLAYER_BASE + 2},
                                     {"Input", "Edit player 4", TAS_ACTION_EDIT_PLAYER_BASE + 3},
                                     {"Input", "Record player 1", TAS_ACTION_RECORD_PLAYER_BASE},
                                     {"Input", "Record player 2", TAS_ACTION_RECORD_PLAYER_BASE + 1},
                                     {"Input", "Record player 3", TAS_ACTION_RECORD_PLAYER_BASE + 2},
                                     {"Input", "Record player 4", TAS_ACTION_RECORD_PLAYER_BASE + 3},
                                     {"Markers", "Edit cursor marker…", TAS_ACTION_MARKER},
                                     {"Markers", "Remove cursor marker", TAS_ACTION_MARKER_REMOVE},
                                     {"Branches", "Store branch…", TAS_ACTION_BRANCH_STORE},
                                     {"Branches", "Load branch", TAS_ACTION_BRANCH_LOAD},
                                     {"Branches", "Jump to branch keyframe", TAS_ACTION_BRANCH_JUMP},
                                     {"Branches", "Clear branch slot", TAS_ACTION_BRANCH_CLEAR},
                                     {"Cache", "Budget in MiB…", TAS_ACTION_CACHE_MB},
                                     {"Cache", "Checkpoint interval…", TAS_ACTION_CACHE_INTERVAL},
                                     {"Cache", "Clear cache", TAS_ACTION_CACHE_CLEAR},
                                     {"Cache", "Clear after cursor", TAS_ACTION_CACHE_CLEAR_AFTER},
                                     {"History", "Older positions", TAS_ACTION_HISTORY_OLDER},
                                     {"History", "Newer positions", TAS_ACTION_HISTORY_NEWER},
                                     {"History", "Select current", TAS_ACTION_HISTORY_CURRENT},
                                     {"History", "Restore selected", TAS_ACTION_HISTORY_RESTORE},
                                     {"Bookmarks", "Add at cursor…", TAS_ACTION_NAVIGATION_CURSOR},
                                     {"Bookmarks", "Add at playback…", TAS_ACTION_NAVIGATION_PLAYBACK},
                                     {"Bookmarks", "Rename…", TAS_ACTION_NAVIGATION_NAME},
                                     {"Bookmarks", "Edit note…", TAS_ACTION_NAVIGATION_NOTE},
                                     {"Bookmarks", "Remove", TAS_ACTION_NAVIGATION_REMOVE},
                                     {"Bookmarks", "Previous bookmark", TAS_ACTION_NAVIGATION_PREV},
                                     {"Bookmarks", "Next bookmark", TAS_ACTION_NAVIGATION_NEXT},
                                     {"Bookmarks", "Go to selected", TAS_ACTION_NAVIGATION_GOTO},
                                     {"Bookmarks", "Older entries", TAS_ACTION_NAVIGATION_OLDER},
                                     {"Bookmarks", "Newer entries", TAS_ACTION_NAVIGATION_NEWER},
                                     {"Bookmarks", "Details", TAS_ACTION_NAVIGATION_DETAILS},
                                     {"Bookmarks", "List", TAS_ACTION_NAVIGATION_LIST},
                                     {"Bookmarks", "Previous detail page", TAS_ACTION_NAVIGATION_DETAIL_PREV},
                                     {"Bookmarks", "Next detail page", TAS_ACTION_NAVIGATION_DETAIL_NEXT},
                                     {"Splice", "Load source…", TAS_ACTION_SPLICE_LOAD},
                                     {"Splice", "Use current project", TAS_ACTION_SPLICE_CURRENT},
                                     {"Splice", "Source from selection", TAS_ACTION_SPLICE_SOURCE_SELECTION},
                                     {"Splice", "Destination from selection", TAS_ACTION_SPLICE_DESTINATION_SELECTION},
                                     {"Splice", "Source first frame…", TAS_ACTION_SPLICE_SOURCE_FIRST},
                                     {"Splice", "Source count…", TAS_ACTION_SPLICE_SOURCE_COUNT},
                                     {"Splice", "Destination first frame…", TAS_ACTION_SPLICE_DESTINATION_FIRST},
                                     {"Splice", "Replace count…", TAS_ACTION_SPLICE_DESTINATION_COUNT},
                                     {"Splice", "Copy markers", TAS_ACTION_SPLICE_MARKERS},
                                     {"Splice", "Append mode", TAS_ACTION_SPLICE_APPEND},
                                     {"Splice", "Insert mode", TAS_ACTION_SPLICE_INSERT},
                                     {"Splice", "Replace mode", TAS_ACTION_SPLICE_REPLACE},
                                     {"Splice", "Extract mode", TAS_ACTION_SPLICE_EXTRACT},
                                     {"Splice", "Apply reviewed splice", TAS_ACTION_SPLICE_APPLY}};
static const char *pad_names[] = {"A", "B", "Select", "Start", "Up", "Down", "Left", "Right"};
static const char *pad_short[] = {"A", "B", "S", "T", "U", "D", "L", "R"};
static const char *command_names[] = {"Reset", "Power", "Disk", "Side", "Coin 1", "Coin 2", "Service"};
static void sync_prompt(GtkTas *tas);

static void refresh_now(GtkTas *tas) {
    tas->next_metadata = 0;
    cupid_gtk_tas_refresh(tas->root);
}

static void scroll_cursor(GtkTas *tas) {
    DesktopTasEditor *editor = tas->tool->ui.tas_editor;
    if (!tas->count || !editor) {
        return;
    }
    GtkTreePath *path = gtk_tree_path_new_from_indices((int)MIN(editor->cursor, tas->count - 1), -1);
    gtk_tree_view_scroll_to_cell(GTK_TREE_VIEW(tas->tree), path, NULL, FALSE, 0, 0);
    gtk_tree_path_free(path);
}

static void run_action(GtkTas *tas, unsigned action) {
    if (tas->disposing || tas->updating || tas->tool->ui.edit_text_active) {
        return;
    }
    desktop_tas_finish_paint(&tas->tool->ui);
    tas->drag = FALSE;
    DesktopTasEditor *editor = tas->tool->ui.tas_editor;
    if (action >= TAS_ACTION_SPLICE_LOAD && action <= TAS_ACTION_SPLICE_EXTRACT && editor->sidebar_tab != 6) {
        desktop_tas_action(&tas->tool->ui, TAS_ACTION_SPLICE_OPEN);
        if (action == TAS_ACTION_SPLICE_APPLY) {
            desktop_tas_status(&tas->tool->ui, "Review the splice preview, then choose Apply splice.");
            refresh_now(tas);
            return;
        }
    }
    desktop_tas_action(&tas->tool->ui, action);
    refresh_now(tas);
    scroll_cursor(tas);
    sync_prompt(tas);
}

static void action_activate(GSimpleAction *action, GVariant *parameter, gpointer data) {
    (void)action;
    run_action(data, g_variant_get_uint32(parameter));
}

static void button_action(GtkButton *button, gpointer data) {
    run_action(data, GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "tas-action")));
}

static GtkWidget *button(GtkTas *tas, GtkWidget *box, const char *label, unsigned action) {
    GtkWidget *widget = gtk_button_new_with_label(label);
    g_object_set_data(G_OBJECT(widget), "tas-action", GUINT_TO_POINTER(action));
    g_signal_connect(widget, "clicked", G_CALLBACK(button_action), tas);
    gtk_box_append(GTK_BOX(box), widget);
    return widget;
}

static void label_text(GtkWidget *label, const char *text) {
    if (strcmp(gtk_label_get_text(GTK_LABEL(label)), text)) {
        gtk_label_set_text(GTK_LABEL(label), text);
    }
}

static void button_text(GtkWidget *widget, const char *text, gboolean selected) {
    if (strcmp(gtk_button_get_label(GTK_BUTTON(widget)), text)) {
        gtk_button_set_label(GTK_BUTTON(widget), text);
    }
    if (selected) {
        gtk_widget_add_css_class(widget, "suggested-action");
    } else {
        gtk_widget_remove_css_class(widget, "suggested-action");
    }
}

static GtkWidget *menus(void) {
    GMenu *bar = g_menu_new(), *section = NULL;
    const char *previous = NULL;
    for (unsigned i = 0; i < G_N_ELEMENTS(menu_items); ++i) {
        const TasMenu *entry = &menu_items[i];
        if (!previous || strcmp(previous, entry->menu)) {
            if (section) {
                g_object_unref(section);
            }
            section = g_menu_new();
            g_menu_append_submenu(bar, entry->menu, G_MENU_MODEL(section));
            previous = entry->menu;
        }
        GMenuItem *item = g_menu_item_new(entry->label, NULL);
        g_menu_item_set_action_and_target(item, "tas.run", "u", entry->action);
        g_menu_append_item(section, item);
        g_object_unref(item);
    }
    g_clear_object(&section);
    GtkWidget *menu = gtk_popover_menu_bar_new_from_model(G_MENU_MODEL(bar));
    g_object_unref(bar);
    return menu;
}

static const char *marker_at(const NesTasProject *project, size_t frame) {
    /* Markers are ordered: binary search keeps row rendering independent of movie length. */
    size_t first = 0, last = nes_tas_marker_count(project);
    while (first < last) {
        size_t mid = first + (last - first) / 2;
        NesTasMarkerView marker;
        if (!nes_tas_marker(project, mid, &marker)) {
            break;
        }
        if (marker.frame < frame) {
            first = mid + 1;
        } else if (marker.frame > frame) {
            last = mid;
        } else {
            return marker.note ? marker.note : "";
        }
    }
    return "";
}

static void cell_data(GtkTreeViewColumn *column, GtkCellRenderer *cell, GtkTreeModel *model, GtkTreeIter *iter,
                      gpointer data) {
    (void)model;
    GtkTas *tas = data;
    FrontendDesktopUi *ui = &tas->tool->ui;
    DesktopTasEditor *editor = ui->tas_editor;
    const NesTasProject *project = desktop_tas_project_const(ui);
    size_t row = (size_t)rows_index(iter);
    const NesFm2Frame *frame = nes_tas_project_frame(project, row);
    int id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(column), "tas-column"));
    char text[96] = "";
    const char *value = text;
    gboolean pressed = FALSE;
    if (frame) {
        if (id == COLUMN_FRAME) {
            snprintf(text, sizeof(text), "%s%zu", row == tas->playback ? "> " : "  ", row);
        } else if (id == COLUMN_MARKER) {
            value = marker_at(project, row);
        } else if (id == TAS_GRID_LAG) {
            NesTasLagState lag = nes_tas_lag(project, row);
            value = lag == NES_TAS_LAG_YES ? "Lag" : lag == NES_TAS_LAG_NO ? "." : "?";
        } else if (id >= COLUMN_ZAPPER) {
            unsigned player = (unsigned)(id - COLUMN_ZAPPER);
            snprintf(text, sizeof(text), "%u,%u:%u", frame->zappers[player].x, frame->zappers[player].y,
                     frame->zappers[player].button);
        } else if (id >= TAS_GRID_PAD_BASE) {
            unsigned index = (unsigned)(id - TAS_GRID_PAD_BASE), bit = index % 8;
            pressed = (frame->pads[index / 8] & (1u << bit)) != 0;
            value = pressed ? pad_short[bit] : "·";
        } else {
            unsigned bit = (unsigned)(id - TAS_GRID_COMMAND_BASE);
            pressed = (frame->commands & (1u << bit)) != 0;
            value = pressed ? command_names[bit] : "·";
        }
    }
    gboolean selected = nes_tas_selection_contains(project, row);
    gboolean cursor = editor && editor->cursor == row && (id == COLUMN_FRAME || id == editor->column);
    GdkRGBA background = tas->selected_bg;
    if (cursor && !selected) {
        background.alpha = 0.25;
    }
    g_object_set(cell, "text", value, "weight",
                 pressed || cursor || row == tas->playback ? PANGO_WEIGHT_BOLD : PANGO_WEIGHT_NORMAL,
                 "cell-background-rgba", &background, "cell-background-set", selected || cursor, "foreground-rgba",
                 &tas->selected_fg, "foreground-set", selected, NULL);
}

static void header_clicked(GtkTreeViewColumn *column, gpointer data) {
    GtkTas *tas = data;
    int id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(column), "tas-column"));
    if (id >= TAS_GRID_PAD_BASE && id < TAS_GRID_PAD_BASE + 32) {
        run_action(tas, TAS_ACTION_HEADER_BASE + (unsigned)(id - TAS_GRID_PAD_BASE));
    }
}

static GtkTreeViewColumn *add_column(GtkTas *tas, const char *title, int id, int width) {
    GtkCellRenderer *cell = gtk_cell_renderer_text_new();
    g_object_set(cell, "family", "monospace", "ypad", 3u, "xpad", 5u, "ellipsize", PANGO_ELLIPSIZE_END, NULL);
    GtkTreeViewColumn *column = gtk_tree_view_column_new();
    gtk_tree_view_column_set_title(column, title);
    if (id >= TAS_GRID_PAD_BASE && id < TAS_GRID_PAD_BASE + 32) {
        GtkWidget *heading = gtk_label_new(title);
        char tooltip[96];
        unsigned index = (unsigned)(id - TAS_GRID_PAD_BASE);
        snprintf(tooltip, sizeof(tooltip), "Player %u: %s. Click to toggle the selected range.", index / 8 + 1,
                 pad_names[index % 8]);
        gtk_widget_set_tooltip_text(heading, tooltip);
        gtk_tree_view_column_set_widget(column, heading);
    }
    gtk_tree_view_column_pack_start(column, cell, TRUE);
    gtk_tree_view_column_set_cell_data_func(column, cell, cell_data, tas, NULL);
    gtk_tree_view_column_set_sizing(column, GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(column, width);
    gtk_tree_view_column_set_resizable(column, TRUE);
    g_object_set_data(G_OBJECT(column), "tas-column", GINT_TO_POINTER(id));
    if (id >= TAS_GRID_PAD_BASE && id < TAS_GRID_PAD_BASE + 32) {
        gtk_tree_view_column_set_clickable(column, TRUE);
        g_signal_connect(column, "clicked", G_CALLBACK(header_clicked), tas);
    }
    gtk_tree_view_append_column(GTK_TREE_VIEW(tas->tree), column);
    return column;
}

static gboolean hit_cell(GtkTas *tas, double x, double y, size_t *row, int *column) {
    if (x < 0 || y < 0 || x >= gtk_widget_get_width(tas->tree) || y >= gtk_widget_get_height(tas->tree)) {
        return FALSE;
    }
    int bx, by;
    gtk_tree_view_convert_widget_to_bin_window_coords(GTK_TREE_VIEW(tas->tree), (int)x, (int)y, &bx, &by);
    GtkTreePath *path = NULL;
    GtkTreeViewColumn *col = NULL;
    if (!gtk_tree_view_get_path_at_pos(GTK_TREE_VIEW(tas->tree), bx, by, &path, &col, NULL, NULL)) {
        return FALSE;
    }
    *row = (size_t)gtk_tree_path_get_indices(path)[0];
    *column = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(col), "tas-column"));
    gtk_tree_path_free(path);
    return *row < tas->count;
}

static void grid_pressed(GtkGestureClick *gesture, int presses, double x, double y, gpointer data) {
    GtkTas *tas = data;
    size_t row;
    int column;
    if (tas->prompting || !hit_cell(tas, x, y, &row, &column)) {
        return;
    }
    gtk_widget_grab_focus(tas->tree);
    GdkModifierType modifiers = gtk_event_controller_get_current_event_state(GTK_EVENT_CONTROLLER(gesture));
    if (column < 0 || column >= COLUMN_ZAPPER || (modifiers & (GDK_SHIFT_MASK | GDK_CONTROL_MASK))) {
        desktop_tas_select_row(&tas->tool->ui, row, cupid_gtk_modifiers(modifiers));
        if (presses == 2 && column == COLUMN_FRAME) {
            desktop_tas_action(&tas->tool->ui, TAS_ACTION_SEEK_CURSOR);
        }
        if (presses == 2 && column == COLUMN_MARKER) {
            desktop_tas_action(&tas->tool->ui, TAS_ACTION_MARKER);
        }
    } else {
        desktop_tas_cell_down(&tas->tool->ui, row, column);
        tas->drag = tas->tool->ui.tas_editor->painting;
        tas->drag_column = column;
    }
    gtk_gesture_set_state(GTK_GESTURE(gesture), GTK_EVENT_SEQUENCE_CLAIMED);
    refresh_now(tas);
}

static void grid_released(GtkGestureClick *gesture, int presses, double x, double y, gpointer data) {
    (void)gesture;
    (void)presses;
    (void)x;
    (void)y;
    GtkTas *tas = data;
    desktop_tas_finish_paint(&tas->tool->ui);
    tas->drag = FALSE;
    refresh_now(tas);
}

static void grid_motion(GtkEventControllerMotion *controller, double x, double y, gpointer data) {
    (void)controller;
    GtkTas *tas = data;
    size_t row;
    int column;
    if (!tas->drag || !hit_cell(tas, x, y, &row, &column)) {
        return;
    }
    desktop_tas_cell_drag(&tas->tool->ui, row, tas->drag_column);
    gtk_widget_queue_draw(tas->tree);
}

static void grid_cancel(GtkGesture *gesture, GdkEventSequence *sequence, gpointer data) {
    (void)gesture;
    (void)sequence;
    GtkTas *tas = data;
    desktop_tas_finish_paint(&tas->tool->ui);
    tas->drag = FALSE;
}

static gboolean grid_key(GtkEventControllerKey *controller, guint key, guint code, GdkModifierType state,
                         gpointer data) {
    (void)controller;
    (void)code;
    GtkTas *tas = data;
    if (tas->prompting) {
        return FALSE;
    }
    if (key == GDK_KEY_Escape) {
        run_action(tas, TAS_ACTION_SELECT_NONE);
        return TRUE;
    }
    SDL_KeyboardEvent event = {.type = SDL_KEYDOWN};
    event.keysym.scancode = cupid_gtk_scancode(key);
    event.keysym.mod = cupid_gtk_modifiers(state);
    gboolean handled = desktop_tas_key(&tas->tool->ui, &event);
    if (handled) {
        refresh_now(tas);
        scroll_cursor(tas);
    }
    return handled;
}

static void prompt_cancel(GtkButton *button_widget, gpointer data) {
    (void)button_widget;
    GtkTas *tas = data;
    tas->tool->ui.edit_text_active = FALSE;
    tas->prompting = FALSE;
    SDL_StopTextInput();
    sync_prompt(tas);
    gtk_widget_grab_focus(tas->tree);
}

static void prompt_submit(GtkWidget *widget, gpointer data) {
    (void)widget;
    GtkTas *tas = data;
    FrontendDesktopUi *ui = &tas->tool->ui;
    if (!ui->edit_text_active || !tas->prompting) {
        return;
    }
    NesTasProgress progress = {0};
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (project != tas->prompt_project || progress.generation != tas->prompt_generation ||
        nes_tas_project_revision(project) != tas->prompt_revision || ui->edit_control != tas->prompt_control) {
        label_text(tas->prompt_error, "The project changed. Cancel and reopen this field.");
        return;
    }
    char error[256] = "";
    if (!desktop_tas_commit(ui, gtk_editable_get_text(GTK_EDITABLE(tas->entry)), error, sizeof(error))) {
        label_text(tas->prompt_error, error);
        return;
    }
    prompt_cancel(NULL, tas);
    refresh_now(tas);
    scroll_cursor(tas);
}

static void sync_prompt(GtkTas *tas) {
    FrontendDesktopUi *ui = &tas->tool->ui;
    if (ui->edit_text_active && !tas->prompting) {
        tas->prompting = TRUE;
        tas->prompt_control = ui->edit_control;
        tas->prompt_project = desktop_tas_project_const(ui);
        tas->prompt_revision = nes_tas_project_revision(tas->prompt_project);
        NesTasProgress progress = {0};
        nes_tas_session_progress(desktop_tas_session(ui), &progress);
        tas->prompt_generation = progress.generation;
        label_text(tas->prompt_title, desktop_tas_edit_title(ui->edit_control));
        gtk_editable_set_text(GTK_EDITABLE(tas->entry), ui->edit_text);
        label_text(tas->prompt_error, "");
        gtk_widget_set_visible(tas->prompt, TRUE);
        gtk_widget_grab_focus(tas->entry);
        gtk_editable_select_region(GTK_EDITABLE(tas->entry), 0, -1);
        SDL_StopTextInput();
    }
    if (!ui->edit_text_active) {
        tas->prompting = FALSE;
    }
    gtk_widget_set_visible(tas->prompt, tas->prompting);
    gtk_widget_set_sensitive(tas->body, !tas->prompting);
}

static GtkWidget *row_box(GtkWidget *parent) {
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_append(GTK_BOX(parent), box);
    return box;
}

static void marker_page(GtkButton *widget, gpointer data) {
    GtkTas *tas = data;
    int delta = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(widget), "tas-page"));
    size_t count = nes_tas_marker_count(desktop_tas_project_const(&tas->tool->ui));
    if (delta < 0 && tas->marker_page) {
        --tas->marker_page;
    }
    if (delta > 0 && (tas->marker_page + 1) * PAGE_ROWS < count) {
        ++tas->marker_page;
    }
    refresh_now(tas);
}

static void tab_changed(GtkNotebook *notebook, GtkWidget *page, guint index, gpointer data) {
    (void)notebook;
    (void)page;
    GtkTas *tas = data;
    if (tas->updating) {
        return;
    }
    static const unsigned actions[] = {
        TAS_ACTION_SIDEBAR_BRANCHES, TAS_ACTION_SIDEBAR_MARKERS,    TAS_ACTION_SIDEBAR_INPUT, TAS_ACTION_SIDEBAR_CACHE,
        TAS_ACTION_SIDEBAR_HISTORY,  TAS_ACTION_SIDEBAR_NAVIGATION, TAS_ACTION_SPLICE_OPEN};
    if (index < G_N_ELEMENTS(actions)) {
        run_action(tas, actions[index]);
    }
}

static GtkWidget *page_new(GtkTas *tas, const char *title, unsigned index) {
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    cupid_gtk_margins(box, 8);
    tas->details[index] = cupid_gtk_label("");
    gtk_label_set_wrap(GTK_LABEL(tas->details[index]), TRUE);
    gtk_label_set_selectable(GTK_LABEL(tas->details[index]), TRUE);
    gtk_box_append(GTK_BOX(box), tas->details[index]);
    GtkWidget *scroll = cupid_gtk_scroll(box);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_notebook_append_page(GTK_NOTEBOOK(tas->notebook), scroll, gtk_label_new(title));
    return box;
}

static void build_pages(GtkTas *tas) {
    GtkWidget *page = page_new(tas, "Branches", 0), *bar = row_box(page);
    button(tas, bar, "Store…", TAS_ACTION_BRANCH_STORE);
    button(tas, bar, "Load", TAS_ACTION_BRANCH_LOAD);
    button(tas, bar, "Jump", TAS_ACTION_BRANCH_JUMP);
    button(tas, bar, "Clear", TAS_ACTION_BRANCH_CLEAR);
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        tas->branch[i] = button(tas, page, "Empty slot", TAS_ACTION_BRANCH_SLOT_BASE + i);
    }
    page = page_new(tas, "Markers", 1);
    bar = row_box(page);
    button(tas, bar, "Edit…", TAS_ACTION_MARKER);
    button(tas, bar, "Remove", TAS_ACTION_MARKER_REMOVE);
    button(tas, bar, "Previous", TAS_ACTION_MARKER_PREV);
    button(tas, bar, "Next", TAS_ACTION_MARKER_NEXT);
    bar = row_box(page);
    for (int direction = -1; direction <= 1; direction += 2) {
        GtkWidget *b = gtk_button_new_with_label(direction < 0 ? "Older entries" : "Newer entries");
        g_object_set_data(G_OBJECT(b), "tas-page", GINT_TO_POINTER(direction));
        g_signal_connect(b, "clicked", G_CALLBACK(marker_page), tas);
        gtk_box_append(GTK_BOX(bar), b);
    }
    for (unsigned i = 0; i < PAGE_ROWS; ++i) {
        tas->markers[i] = button(tas, page, "", TAS_ACTION_MARKER_GOTO_BASE + i);
    }
    page = page_new(tas, "Input", 2);
    bar = row_box(page);
    for (unsigned i = 0; i < 4; ++i) {
        char label[16];
        snprintf(label, sizeof(label), "Edit P%u", i + 1);
        button(tas, bar, label, TAS_ACTION_EDIT_PLAYER_BASE + i);
    }
    bar = row_box(page);
    for (unsigned i = 0; i < 8; ++i) {
        if (i == 4) {
            bar = row_box(page);
        }
        button(tas, bar, pad_names[i], TAS_ACTION_ACTIVE_BUTTON_BASE + i);
    }
    bar = row_box(page);
    button(tas, bar, "Hold", TAS_ACTION_HOLD);
    button(tas, bar, "Autofire", TAS_ACTION_AUTOFIRE);
    button(tas, bar, "Next pattern", TAS_ACTION_PATTERN_NEXT);
    button(tas, bar, "Apply pattern", TAS_ACTION_PATTERN);
    bar = row_box(page);
    button(tas, bar, "Insert / overwrite", TAS_ACTION_RECORD_MODE);
    button(tas, bar, "Skip lag", TAS_ACTION_SKIP_LAG);
    bar = row_box(page);
    for (unsigned i = 0; i < 4; ++i) {
        char label[16];
        snprintf(label, sizeof(label), "Record P%u", i + 1);
        button(tas, bar, label, TAS_ACTION_RECORD_PLAYER_BASE + i);
    }
    page = page_new(tas, "Cache", 3);
    bar = row_box(page);
    button(tas, bar, "Budget…", TAS_ACTION_CACHE_MB);
    button(tas, bar, "Interval…", TAS_ACTION_CACHE_INTERVAL);
    bar = row_box(page);
    button(tas, bar, "Clear", TAS_ACTION_CACHE_CLEAR);
    button(tas, bar, "Clear after cursor", TAS_ACTION_CACHE_CLEAR_AFTER);
    bar = row_box(page);
    button(tas, bar, "Previous", TAS_ACTION_CACHE_PREV);
    button(tas, bar, "Next", TAS_ACTION_CACHE_NEXT);
    page = page_new(tas, "History", 4);
    bar = row_box(page);
    button(tas, bar, "Older", TAS_ACTION_HISTORY_OLDER);
    button(tas, bar, "Newer", TAS_ACTION_HISTORY_NEWER);
    button(tas, bar, "Current", TAS_ACTION_HISTORY_CURRENT);
    button(tas, bar, "Restore", TAS_ACTION_HISTORY_RESTORE);
    for (unsigned i = 0; i < PAGE_ROWS; ++i) {
        tas->history[i] = button(tas, page, "", TAS_ACTION_HISTORY_SELECT_BASE + i);
    }
    page = page_new(tas, "Bookmarks", 5);
    bar = row_box(page);
    button(tas, bar, "At cursor…", TAS_ACTION_NAVIGATION_CURSOR);
    button(tas, bar, "At playback…", TAS_ACTION_NAVIGATION_PLAYBACK);
    bar = row_box(page);
    button(tas, bar, "Rename…", TAS_ACTION_NAVIGATION_NAME);
    button(tas, bar, "Note…", TAS_ACTION_NAVIGATION_NOTE);
    button(tas, bar, "Remove", TAS_ACTION_NAVIGATION_REMOVE);
    button(tas, bar, "Go to", TAS_ACTION_NAVIGATION_GOTO);
    bar = row_box(page);
    button(tas, bar, "Previous", TAS_ACTION_NAVIGATION_PREV);
    button(tas, bar, "Next", TAS_ACTION_NAVIGATION_NEXT);
    button(tas, bar, "Older", TAS_ACTION_NAVIGATION_OLDER);
    button(tas, bar, "Newer", TAS_ACTION_NAVIGATION_NEWER);
    for (unsigned i = 0; i < PAGE_ROWS; ++i) {
        tas->navigation[i] = button(tas, page, "", TAS_ACTION_NAVIGATION_SELECT_BASE + i);
    }
    page = page_new(tas, "Splice", 6);
    bar = row_box(page);
    button(tas, bar, "Load source…", TAS_ACTION_SPLICE_LOAD);
    button(tas, bar, "Current project", TAS_ACTION_SPLICE_CURRENT);
    bar = row_box(page);
    button(tas, bar, "Source selection", TAS_ACTION_SPLICE_SOURCE_SELECTION);
    button(tas, bar, "Destination selection", TAS_ACTION_SPLICE_DESTINATION_SELECTION);
    bar = row_box(page);
    button(tas, bar, "Source first…", TAS_ACTION_SPLICE_SOURCE_FIRST);
    button(tas, bar, "Source count…", TAS_ACTION_SPLICE_SOURCE_COUNT);
    bar = row_box(page);
    button(tas, bar, "Destination first…", TAS_ACTION_SPLICE_DESTINATION_FIRST);
    button(tas, bar, "Replace count…", TAS_ACTION_SPLICE_DESTINATION_COUNT);
    bar = row_box(page);
    button(tas, bar, "Append", TAS_ACTION_SPLICE_APPEND);
    button(tas, bar, "Insert", TAS_ACTION_SPLICE_INSERT);
    button(tas, bar, "Replace", TAS_ACTION_SPLICE_REPLACE);
    button(tas, bar, "Extract", TAS_ACTION_SPLICE_EXTRACT);
    bar = row_box(page);
    button(tas, bar, "Copy markers", TAS_ACTION_SPLICE_MARKERS);
    button(tas, bar, "Apply splice", TAS_ACTION_SPLICE_APPLY);
}

static void observe_history(GtkTas *tas, const NesTasProject *project) {
    DesktopTasEditor *editor = tas->tool->ui.tas_editor;
    NesTasHistoryInfo info = {0};
    nes_tas_project_history_info(project, &info);
    uint64_t revision = nes_tas_project_revision(project);
    if (!editor->history_initialized || editor->history_project != project || editor->history_revision != revision ||
        editor->history_snapshot.operations != info.operations || editor->history_snapshot.position != info.position) {
        editor->history_selection = info.position;
        editor->history_scroll = (info.position / TAS_HISTORY_VISIBLE) * TAS_HISTORY_VISIBLE;
    }
    editor->history_initialized = true;
    editor->history_project = project;
    editor->history_revision = revision;
    editor->history_snapshot = info;
}

static void refresh_pages(GtkTas *tas, const NesTasProgress *progress) {
    FrontendDesktopUi *ui = &tas->tool->ui;
    DesktopTasEditor *editor = ui->tas_editor;
    const NesTasProject *project = desktop_tas_project_const(ui);
    char text[2048];
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        NesTasBookmarkView branch = {0};
        nes_tas_bookmark(project, i, &branch);
        if (branch.occupied) {
            snprintf(text, sizeof(text), "%u · frame %zu · %s", i + 1, branch.key_frame,
                     branch.name ? branch.name : "");
        } else {
            snprintf(text, sizeof(text), "%u · Empty slot", i + 1);
        }
        button_text(tas->branch[i], text, editor->branch_slot == i);
    }
    snprintf(text, sizeof(text), "Selected slot %u · current branch %d", editor->branch_slot + 1,
             nes_tas_current_branch(project) + 1);
    NesTasBookmarkView selected_branch = {0};
    if (nes_tas_bookmark(project, editor->branch_slot, &selected_branch) && selected_branch.occupied) {
        size_t used = strlen(text);
        snprintf(text + used, sizeof(text) - used, "\nParent slot: %d (0 = root)", selected_branch.parent_slot + 1);
    }
    label_text(tas->details[0], text);
    size_t markers = nes_tas_marker_count(project);
    if (tas->marker_page * PAGE_ROWS >= markers) {
        tas->marker_page = markers ? (markers - 1) / PAGE_ROWS : 0;
    }
    snprintf(text, sizeof(text), "%zu markers · page %zu · click a marker to move the cursor", markers,
             tas->marker_page + 1);
    label_text(tas->details[1], text);
    for (unsigned i = 0; i < PAGE_ROWS; ++i) {
        size_t index = tas->marker_page * PAGE_ROWS + i;
        NesTasMarkerView marker;
        gboolean valid = nes_tas_marker(project, index, &marker);
        gtk_widget_set_visible(tas->markers[i], valid);
        if (valid) {
            snprintf(text, sizeof(text), "%zu · %s", marker.frame, marker.note ? marker.note : "");
            button_text(tas->markers[i], text, marker.frame == editor->cursor);
            g_object_set_data(G_OBJECT(tas->markers[i]), "tas-action",
                              GUINT_TO_POINTER(TAS_ACTION_MARKER_GOTO_BASE + (unsigned)index));
        }
    }
    static const char *patterns[] = {"10", "0101", "1100", "1110"};
    snprintf(text, sizeof(text),
             "Editing player %u · button mask $%02X\nPattern %s · skip lag %s\nRecording %s · players %s%s%s%s\nHold "
             "$%02X · autofire $%02X",
             editor->edit_player + 1, editor->active_button, patterns[editor->pattern % 4],
             ui->execution && ui->execution->tas_skip_lag ? "on" : "off",
             progress->record_mode == NES_TAS_RECORD_INSERT ? "insert" : "overwrite",
             progress->record_players & 1 ? "1 " : "", progress->record_players & 2 ? "2 " : "",
             progress->record_players & 4 ? "3 " : "", progress->record_players & 8 ? "4 " : "",
             nes_tas_session_hold(desktop_tas_session(ui), editor->edit_player),
             nes_tas_session_autofire(desktop_tas_session(ui), editor->edit_player));
    label_text(tas->details[2], text);
    NesTasCacheInfo cache = {0};
    nes_tas_session_cache_info(desktop_tas_session(ui), &cache);
    snprintf(text, sizeof(text), "%zu checkpoints · %.2f / %.2f MiB\nInterval %u frames · startup %.2f MiB",
             cache.checkpoint_count, cache.checkpoint_bytes / 1048576.0, cache.byte_limit / 1048576.0, cache.interval,
             cache.initial_bytes / 1048576.0);
    label_text(tas->details[3], text);
    snprintf(text, sizeof(text), "Position %zu / %zu · %.2f MiB retained\nSelect a position, then Restore.",
             editor->history_snapshot.position, editor->history_snapshot.operations,
             editor->history_snapshot.bytes / 1048576.0);
    label_text(tas->details[4], text);
    for (unsigned i = 0; i < PAGE_ROWS; ++i) {
        size_t position = editor->history_scroll + i;
        NesTasHistoryView entry;
        gboolean valid = nes_tas_project_history_entry(project, position, &entry);
        gtk_widget_set_visible(tas->history[i], valid);
        if (valid) {
            snprintf(text, sizeof(text), "%s%zu · %s", entry.current ? "> " : "", position, entry.description);
            button_text(tas->history[i], text, editor->history_selection == position);
        }
        size_t index = editor->navigation_scroll + i;
        NesTasNavigationView bookmark;
        valid = nes_tas_navigation(project, index, &bookmark);
        gtk_widget_set_visible(tas->navigation[i], valid);
        if (valid) {
            snprintf(text, sizeof(text), "%zu · %s", bookmark.frame, bookmark.name ? bookmark.name : "");
            button_text(tas->navigation[i], text, editor->navigation_selection == index);
        }
    }
    NesTasNavigationView bookmark;
    editor->navigation_detail_pages = 1; /* Native wrapping shows the entire note without truncation. */
    if (nes_tas_navigation(project, editor->navigation_selection, &bookmark)) {
        snprintf(text, sizeof(text), "%zu bookmarks · selected frame %zu\n%s\n%s", editor->navigation_count,
                 bookmark.frame, bookmark.name ? bookmark.name : "", bookmark.note ? bookmark.note : "");
    } else {
        snprintf(text, sizeof(text), "%zu bookmarks · select an entry to view its name and note",
                 editor->navigation_count);
    }
    label_text(tas->details[5], text);
    DesktopTasSplicer *splice = &editor->splice;
    static const char *modes[] = {"Append", "Insert", "Replace", "Extract"};
    snprintf(text, sizeof(text), "%s · source: %s\nSource %zu + %zu · destination %zu, replace %zu\nCopy markers: %s\n",
             modes[(unsigned)splice->options.mode % 4], splice->source_name, splice->options.source_first,
             splice->options.source_count, splice->options.destination_first, splice->options.destination_count,
             splice->options.copy_markers ? "yes" : "no");
    size_t used = strlen(text);
    if (splice->preview_valid) {
        snprintf(text + used, sizeof(text) - used, "Preview: remove %zu, insert %zu at %zu → %zu frames, %zu markers",
                 splice->preview.removed, splice->preview.inserted, splice->preview.first,
                 splice->preview.resulting_frames, splice->preview.resulting_markers);
    } else {
        snprintf(text + used, sizeof(text) - used, "%s",
                 splice->error[0] ? splice->error : "Open Splice to preview a range.");
    }
    label_text(tas->details[6], text);
}

void cupid_gtk_tas_video(GtkWidget *root) {
    GtkTas *tas = g_object_get_data(G_OBJECT(root), "cupid-tas");
    if (!tas || tas->disposing || tas->updating) {
        return;
    }
    FrontendDesktopUi *ui = &tas->tool->ui;
    const uint32_t *pixels = NULL;
    unsigned width = 0, height = 0, stride = 0;
    if (frontend_video_runtime_pixels(ui->video, true, &pixels, &width, &height, &stride) ||
        frontend_video_runtime_pixels(ui->video, false, &pixels, &width, &height, &stride)) {
        cupid_gtk_picture(tas->picture, pixels, width, height, stride * sizeof(uint32_t));
    } else {
        gtk_picture_set_paintable(GTK_PICTURE(tas->picture), NULL);
    }
}

void cupid_gtk_tas_refresh(GtkWidget *root) {
    GtkTas *tas = g_object_get_data(G_OBJECT(root), "cupid-tas");
    if (!tas || tas->disposing || tas->updating) {
        return;
    }
    cupid_gtk_tas_video(root);
    FrontendDesktopUi *ui = &tas->tool->ui;
    gint64 now = g_get_monotonic_time();
    if (now < tas->next_metadata) {
        return;
    }
    tas->next_metadata = now + 100000;
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    if (!editor) {
        return;
    }
    tas->updating = TRUE;
    NesTasProgress progress = {0};
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    const NesTasProject *project = desktop_tas_project_const(ui);
    size_t count = MIN(nes_tas_project_frame_count(project), (size_t)G_MAXINT);
    gboolean replaced = project != tas->project || progress.generation != tas->generation;
    GtkTreeModel *model = gtk_tree_view_get_model(GTK_TREE_VIEW(tas->tree));
    if (!model || replaced || (count > tas->count ? count - tas->count : tas->count - count) > 1024) {
        TasRows *rows = g_object_new(tas_rows_get_type(), NULL);
        rows->count = (int)count;
        gtk_tree_view_set_model(GTK_TREE_VIEW(tas->tree), GTK_TREE_MODEL(rows));
        g_object_unref(rows);
        model = gtk_tree_view_get_model(GTK_TREE_VIEW(tas->tree));
        tas->count = count;
        if (replaced) {
            tas->marker_page = 0;
            tas->drag = FALSE;
        }
        if (count) {
            GtkTreePath *top = gtk_tree_path_new_from_indices((int)MIN(editor->scroll, count - 1), -1);
            gtk_tree_view_scroll_to_cell(GTK_TREE_VIEW(tas->tree), top, NULL, TRUE, 0, 0);
            gtk_tree_path_free(top);
        }
    } else {
        TasRows *rows = (TasRows *)model;
        while (tas->count > count) {
            --tas->count;
            rows->count = (int)tas->count;
            GtkTreePath *path = gtk_tree_path_new_from_indices(rows->count, -1);
            gtk_tree_model_row_deleted(model, path);
            gtk_tree_path_free(path);
        }
        while (tas->count < count) {
            int index = (int)tas->count++;
            rows->count = (int)tas->count;
            GtkTreeIter iter;
            rows_at(model, &iter, index);
            GtkTreePath *path = gtk_tree_path_new_from_indices(index, -1);
            gtk_tree_model_row_inserted(model, path, &iter);
            gtk_tree_path_free(path);
        }
    }
    tas->project = project;
    tas->generation = progress.generation;
    tas->playback = progress.frame;
    GtkTreePath *first = NULL, *last = NULL;
    if (gtk_tree_view_get_visible_range(GTK_TREE_VIEW(tas->tree), &first, &last)) {
        int a = gtk_tree_path_get_indices(first)[0], b = gtk_tree_path_get_indices(last)[0];
        editor->scroll = (size_t)a;
        editor->visible_rows = (size_t)(b - a + 1);
        for (int i = a; i <= b && i < (int)count; ++i) {
            GtkTreeIter iter;
            rows_at(model, &iter, i);
            GtkTreePath *path = gtk_tree_path_new_from_indices(i, -1);
            gtk_tree_model_row_changed(model, path, &iter);
            gtk_tree_path_free(path);
        }
        gtk_tree_path_free(first);
        gtk_tree_path_free(last);
    } else {
        editor->visible_rows = 20;
    }
    size_t previous_scroll = editor->scroll;
    desktop_tas_follow_playback(ui, &progress);
    if (count && previous_scroll != editor->scroll) {
        /* Animated scrolling never settles when playback moves the target
         * every metadata refresh. Move the viewport directly to its new row. */
        GtkTreePath *path = gtk_tree_path_new_from_indices((int)editor->scroll, -1);
        GdkRectangle cell;
        int x, y;
        gtk_tree_view_get_background_area(GTK_TREE_VIEW(tas->tree), path, NULL, &cell);
        gtk_tree_view_convert_bin_window_to_tree_coords(GTK_TREE_VIEW(tas->tree), 0, cell.y, &x, &y);
        gtk_adjustment_set_value(gtk_scrollable_get_vadjustment(GTK_SCROLLABLE(tas->tree)), y);
        gtk_tree_path_free(path);
    }
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    for (unsigned i = 0; i < 32; ++i) {
        unsigned player = i / 8;
        gboolean visible =
            movie ? movie->fourscore || (player < 2 && movie->ports[player] == NES_FM2_PORT_GAMEPAD) : player < 2;
        gtk_tree_view_column_set_visible(tas->pad_columns[i], visible);
    }
    for (unsigned i = 0; i < 7; ++i) {
        gtk_tree_view_column_set_visible(tas->command_columns[i],
                                         i < 2 || (i < 4 ? movie && movie->fds : vs_enabled()));
    }
    for (unsigned i = 0; i < 2; ++i) {
        gtk_tree_view_column_set_visible(tas->zapper_columns[i],
                                         movie && !movie->fourscore && movie->ports[i] == NES_FM2_PORT_ZAPPER);
    }
    gtk_widget_set_visible(tas->empty, !count);
    gboolean paused = !ui->execution || frontend_execution_paused(ui->execution);
    button_text(tas->play, paused ? "Play" : "Pause", !paused);
    button_text(tas->record, progress.recording ? "Recording" : "Record", progress.recording);
    button_text(tas->readonly, progress.read_only ? "Read only" : "Editable", progress.read_only);
    button_text(tas->follow, editor->follow_playback ? "Following" : "Follow", editor->follow_playback);
    char summary[512];
    snprintf(summary, sizeof(summary), "%s · frame %zu / %zu · cursor %zu · %zu selected · lag %zu · rerecords %llu%s",
             !progress.active     ? "No TAS project"
             : progress.seeking   ? "Seeking"
             : progress.recording ? "Recording"
             : paused             ? "Paused"
                                  : "Playing",
             progress.frame, progress.total_frames, editor->cursor, nes_tas_selection_count(project),
             progress.lag_count, (unsigned long long)progress.rerecord_count, progress.dirty ? " · unsaved" : "");
    label_text(tas->summary, summary);
    observe_history(tas, project);
    desktop_tas_navigation_observe(ui);
    if (editor->sidebar_tab == 6 && (!editor->splice.initialized || editor->splice.destination != project ||
                                     editor->splice.generation != progress.generation ||
                                     editor->splice.revision != nes_tas_project_revision(project))) {
        desktop_tas_splice_observe(ui);
    }
    refresh_pages(tas, &progress);
    gtk_notebook_set_current_page(GTK_NOTEBOOK(tas->notebook), (int)MIN(editor->sidebar_tab, 6u));
    if (ui->status[0]) {
        gtk_label_set_text(GTK_LABEL(tas->tool->status), ui->status);
    }
    tas->updating = FALSE;
    sync_prompt(tas);
}

static void grid_unmap(GtkWidget *widget, gpointer data) {
    (void)widget;
    GtkTas *tas = data;
    desktop_tas_finish_paint(&tas->tool->ui);
    tas->drag = FALSE;
}

static GtkWidget *horizontal_scroll(GtkWidget *child) {
    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_NEVER);
    gtk_scrolled_window_set_propagate_natural_height(GTK_SCROLLED_WINDOW(scroll), TRUE);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), child);
    return scroll;
}

static void tas_free(gpointer data) {
    GtkTas *tas = data;
    /* The shell owns tool->ui and destroys its editor after the window. */
    g_clear_object(&tas->actions);
    g_free(tas);
}

GtkWidget *cupid_gtk_tas_new(CupidGtkTool *tool) {
    GtkTas *tas = g_new0(GtkTas, 1);
    tas->tool = tool;
    tas->root = g_object_new(cupid_tas_root_get_type(), NULL);
    g_object_set_data_full(G_OBJECT(tas->root), "cupid-tas", tas, tas_free);
    cupid_gtk_margins(tas->root, 8);
    desktop_tas_editor(&tool->ui);
    tas->selected_bg = (GdkRGBA){0.16, 0.35, 0.65, 1};
    tas->selected_fg = (GdkRGBA){1, 1, 1, 1};
    GtkStyleContext *style = gtk_widget_get_style_context(tas->root);
    gtk_style_context_lookup_color(style, "theme_selected_bg_color", &tas->selected_bg);
    gtk_style_context_lookup_color(style, "theme_selected_fg_color", &tas->selected_fg);
    tas->actions = g_simple_action_group_new();
    GSimpleAction *action = g_simple_action_new("run", G_VARIANT_TYPE_UINT32);
    g_signal_connect(action, "activate", G_CALLBACK(action_activate), tas);
    g_action_map_add_action(G_ACTION_MAP(tas->actions), G_ACTION(action));
    g_object_unref(action);
    gtk_widget_insert_action_group(tas->root, "tas", G_ACTION_GROUP(tas->actions));
    tas->body = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_vexpand(tas->body, TRUE);
    gtk_box_append(GTK_BOX(tas->root), tas->body);
    gtk_box_append(GTK_BOX(tas->body), horizontal_scroll(menus()));
    GtkWidget *toolbar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_widget_add_css_class(toolbar, "toolbar");
    gtk_box_append(GTK_BOX(tas->body), horizontal_scroll(toolbar));
    button(tas, toolbar, "Open…", TAS_ACTION_OPEN);
    button(tas, toolbar, "New…", TAS_ACTION_NEW);
    button(tas, toolbar, "Save", TAS_ACTION_SAVE);
    tas->play = button(tas, toolbar, "Play", TAS_ACTION_PAUSE);
    button(tas, toolbar, "Back", TAS_ACTION_BACK);
    button(tas, toolbar, "Frame", TAS_ACTION_FRAME);
    tas->record = button(tas, toolbar, "Record", TAS_ACTION_RECORD);
    tas->readonly = button(tas, toolbar, "Read only", TAS_ACTION_READ_ONLY);
    button(tas, toolbar, "Go to…", TAS_ACTION_SEEK);
    tas->follow = button(tas, toolbar, "Follow", TAS_ACTION_FOLLOW);
    tas->summary = cupid_gtk_label("");
    gtk_label_set_wrap(GTK_LABEL(tas->summary), TRUE);
    gtk_box_append(GTK_BOX(tas->body), tas->summary);
    GtkWidget *split = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_widget_set_hexpand(split, TRUE);
    gtk_widget_set_vexpand(split, TRUE);
    gtk_box_append(GTK_BOX(tas->body), split);
    GtkWidget *left = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    tas->empty = cupid_gtk_label("Open or create a TAS project. Insert frames or record input to begin.");
    gtk_label_set_wrap(GTK_LABEL(tas->empty), TRUE);
    gtk_box_append(GTK_BOX(left), tas->empty);
    tas->tree = g_object_new(cupid_tas_grid_get_type(), NULL);
    g_signal_connect(tas->tree, "unmap", G_CALLBACK(grid_unmap), tas);
    gtk_tree_view_set_fixed_height_mode(GTK_TREE_VIEW(tas->tree), TRUE);
    gtk_tree_view_set_enable_search(GTK_TREE_VIEW(tas->tree), FALSE);
    gtk_tree_view_set_grid_lines(GTK_TREE_VIEW(tas->tree), GTK_TREE_VIEW_GRID_LINES_BOTH);
    gtk_tree_selection_set_mode(gtk_tree_view_get_selection(GTK_TREE_VIEW(tas->tree)), GTK_SELECTION_NONE);
    gtk_widget_set_focusable(tas->tree, TRUE);
    gtk_widget_set_tooltip_text(tas->tree, "Click inputs to toggle; drag to paint. Click frame numbers to select; "
                                           "Shift extends, Ctrl toggles. Space edits, Enter seeks.");
    add_column(tas, "Frame", COLUMN_FRAME, 82);
    add_column(tas, "Lag", TAS_GRID_LAG, 38);
    for (unsigned i = 0; i < 7; ++i) {
        tas->command_columns[i] = add_column(tas, command_names[i], TAS_GRID_COMMAND_BASE + (int)i, 58);
    }
    for (unsigned player = 0; player < 4; ++player) {
        for (int bit = 7; bit >= 0; --bit) {
            unsigned index = player * 8 + (unsigned)bit;
            char title[32];
            snprintf(title, sizeof(title), "%u%s", player + 1, pad_short[bit]);
            tas->pad_columns[index] = add_column(tas, title, TAS_GRID_PAD_BASE + (int)index, 32);
        }
        if (player < 2) {
            char title[32];
            snprintf(title, sizeof(title), "P%u Zapper X,Y:B", player + 1);
            tas->zapper_columns[player] = add_column(tas, title, COLUMN_ZAPPER + (int)player, 145);
        }
    }
    add_column(tas, "Marker", COLUMN_MARKER, 240);
    GtkGesture *click = gtk_gesture_click_new();
    gtk_event_controller_set_name(GTK_EVENT_CONTROLLER(click), "cupid-tas-input");
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(click), GDK_BUTTON_PRIMARY);
    gtk_event_controller_set_propagation_phase(GTK_EVENT_CONTROLLER(click), GTK_PHASE_CAPTURE);
    g_signal_connect(click, "pressed", G_CALLBACK(grid_pressed), tas);
    g_signal_connect(click, "released", G_CALLBACK(grid_released), tas);
    g_signal_connect(click, "cancel", G_CALLBACK(grid_cancel), tas);
    gtk_widget_add_controller(tas->tree, GTK_EVENT_CONTROLLER(click));
    GtkEventController *motion = gtk_event_controller_motion_new();
    g_signal_connect(motion, "motion", G_CALLBACK(grid_motion), tas);
    gtk_widget_add_controller(tas->tree, motion);
    GtkEventController *keys = gtk_event_controller_key_new();
    gtk_event_controller_set_name(keys, "cupid-tas-keys");
    gtk_event_controller_set_propagation_phase(keys, GTK_PHASE_CAPTURE);
    g_signal_connect(keys, "key-pressed", G_CALLBACK(grid_key), tas);
    gtk_widget_add_controller(tas->tree, keys);
    GtkWidget *scroll = cupid_gtk_scroll(tas->tree);
    gtk_box_append(GTK_BOX(left), scroll);
    gtk_widget_set_size_request(left, 360, -1);
    gtk_paned_set_start_child(GTK_PANED(split), left);
    GtkWidget *right = gtk_paned_new(GTK_ORIENTATION_VERTICAL);
    gtk_widget_set_size_request(right, 300, -1);
    GtkWidget *preview = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *preview_frame = gtk_frame_new("Game preview");
    gtk_frame_set_child(GTK_FRAME(preview_frame), preview);
    tas->picture = gtk_picture_new();
    gtk_picture_set_can_shrink(GTK_PICTURE(tas->picture), TRUE);
    gtk_picture_set_content_fit(GTK_PICTURE(tas->picture), GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_add_css_class(tas->picture, "game-view");
    gtk_widget_set_vexpand(tas->picture, TRUE);
    gtk_widget_set_size_request(tas->picture, 128, 120);
    gtk_box_append(GTK_BOX(preview), tas->picture);
    gtk_paned_set_start_child(GTK_PANED(right), preview_frame);
    tas->notebook = gtk_notebook_new();
    gtk_notebook_set_scrollable(GTK_NOTEBOOK(tas->notebook), TRUE);
    build_pages(tas);
    gtk_paned_set_end_child(GTK_PANED(right), tas->notebook);
    gtk_paned_set_position(GTK_PANED(right), 270);
    gtk_paned_set_end_child(GTK_PANED(split), right);
    gtk_paned_set_position(GTK_PANED(split), 720);
    gtk_paned_set_shrink_start_child(GTK_PANED(split), FALSE);
    gtk_paned_set_shrink_end_child(GTK_PANED(split), FALSE);
    g_signal_connect(tas->notebook, "switch-page", G_CALLBACK(tab_changed), tas);
    tas->prompt = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_box_append(GTK_BOX(tas->root), tas->prompt);
    tas->prompt_title = cupid_gtk_label("");
    gtk_box_append(GTK_BOX(tas->prompt), tas->prompt_title);
    GtkWidget *edit_row = row_box(tas->prompt);
    tas->entry = gtk_entry_new();
    gtk_widget_set_hexpand(tas->entry, TRUE);
    gtk_box_append(GTK_BOX(edit_row), tas->entry);
    GtkWidget *accept = gtk_button_new_with_label("Apply"), *cancel = gtk_button_new_with_label("Cancel");
    gtk_box_append(GTK_BOX(edit_row), accept);
    gtk_box_append(GTK_BOX(edit_row), cancel);
    tas->prompt_error = cupid_gtk_label("");
    gtk_label_set_wrap(GTK_LABEL(tas->prompt_error), TRUE);
    gtk_box_append(GTK_BOX(tas->prompt), tas->prompt_error);
    g_signal_connect(accept, "clicked", G_CALLBACK(prompt_submit), tas);
    g_signal_connect(tas->entry, "activate", G_CALLBACK(prompt_submit), tas);
    g_signal_connect(cancel, "clicked", G_CALLBACK(prompt_cancel), tas);
    refresh_now(tas);
    return tas->root;
}
#endif
