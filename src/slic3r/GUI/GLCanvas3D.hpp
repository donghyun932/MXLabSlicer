#ifndef slic3r_GLCanvas3D_hpp_
#define slic3r_GLCanvas3D_hpp_

#include <stddef.h>
#include <memory>

#include "3DScene.hpp"
#include "GLToolbar.hpp"
#include "Event.hpp"
#include "3DBed.hpp"
#include "Camera.hpp"
#include "Selection.hpp"
#include "Gizmos/GLGizmosManager.hpp"
#include "GUI_ObjectLayers.hpp"
#include "MeshUtils.hpp"

#include <float.h>

#include <wx/timer.h>

class wxWindow;
class wxSizeEvent;
class wxIdleEvent;
class wxKeyEvent;
class wxMouseEvent;
class wxTimerEvent;
class wxPaintEvent;
class wxGLCanvas;

// Support for Retina OpenGL on Mac OS
#define ENABLE_RETINA_GL __APPLE__

namespace Slic3r {

class GLShader;
class ExPolygon;
class BackgroundSlicingProcess;
class GCodePreviewData;
#if ENABLE_THUMBNAIL_GENERATOR
struct ThumbnailData;
#endif // ENABLE_THUMBNAIL_GENERATOR
struct SlicingParameters;
enum LayerHeightEditActionType : unsigned int;

namespace GUI {

class GLGizmoBase;

#if ENABLE_RETINA_GL
class RetinaHelper;
#endif

class Size
{
    int m_width;
    int m_height;
    float m_scale_factor;

public:
    Size();
    Size(int width, int height, float scale_factor = 1.0);

    int get_width() const;
    void set_width(int width);

    int get_height() const;
    void set_height(int height);

    int get_scale_factor() const;
    void set_scale_factor(int height);
};


wxDECLARE_EVENT(EVT_GLCANVAS_OBJECT_SELECT, SimpleEvent);

using Vec2dEvent = Event<Vec2d>;
// _bool_ value is used as a indicator of selection in the 3DScene
using RBtnEvent = Event<std::pair<Vec2d, bool>>;
template <size_t N> using Vec2dsEvent = ArrayEvent<Vec2d, N>;

using Vec3dEvent = Event<Vec3d>;
template <size_t N> using Vec3dsEvent = ArrayEvent<Vec3d, N>;

using HeightProfileSmoothEvent = Event<HeightProfileSmoothingParams>;

#if !ENABLE_VIEW_TOOLBAR_BACKGROUND_FIX
wxDECLARE_EVENT(EVT_GLCANVAS_INIT, SimpleEvent);
#endif // !ENABLE_VIEW_TOOLBAR_BACKGROUND_FIX
wxDECLARE_EVENT(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_RIGHT_CLICK, RBtnEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_REMOVE_OBJECT, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_ARRANGE, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_SELECT_ALL, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_QUESTION_MARK, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_INCREASE_INSTANCES, Event<int>); // data: +1 => increase, -1 => decrease
wxDECLARE_EVENT(EVT_GLCANVAS_INSTANCE_MOVED, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_WIPETOWER_MOVED, Vec3dEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_INSTANCE_ROTATED, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_INSTANCE_SCALED, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_WIPETOWER_ROTATED, Vec3dEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_ENABLE_ACTION_BUTTONS, Event<bool>);
wxDECLARE_EVENT(EVT_GLCANVAS_UPDATE_GEOMETRY, Vec3dsEvent<2>);
wxDECLARE_EVENT(EVT_GLCANVAS_MOUSE_DRAGGING_FINISHED, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_UPDATE_BED_SHAPE, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_TAB, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_RESETGIZMOS, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_MOVE_DOUBLE_SLIDER, wxKeyEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_EDIT_COLOR_CHANGE, wxKeyEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_UNDO, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_REDO, SimpleEvent);
#if ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
wxDECLARE_EVENT(EVT_GLCANVAS_RESET_LAYER_HEIGHT_PROFILE, SimpleEvent);
wxDECLARE_EVENT(EVT_GLCANVAS_ADAPTIVE_LAYER_HEIGHT_PROFILE, Event<float>);
wxDECLARE_EVENT(EVT_GLCANVAS_SMOOTH_LAYER_HEIGHT_PROFILE, HeightProfileSmoothEvent);
#endif // ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE

class GLCanvas3D
{
#if ENABLE_THUMBNAIL_GENERATOR
    static const double DefaultCameraZoomToBoxMarginFactor;
#endif // ENABLE_THUMBNAIL_GENERATOR

public:
    struct GCodePreviewVolumeIndex
    {
        enum EType
        {
            Extrusion,
            Travel,
            Retraction,
            Unretraction,
            Shell,
            Num_Geometry_Types
        };

        struct FirstVolume
        {
            EType type;
            unsigned int flag;
            // Index of the first volume in a GLVolumeCollection.
            unsigned int id;

            FirstVolume(EType type, unsigned int flag, unsigned int id) : type(type), flag(flag), id(id) {}
        };

        std::vector<FirstVolume> first_volumes;

        void reset() { first_volumes.clear(); }
    };

private:
    class LayersEditing
    {
    public:
        enum EState : unsigned char
        {
            Unknown,
            Editing,
            Completed,
            Num_States
        };

    private:
        static const float THICKNESS_BAR_WIDTH;
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static const float THICKNESS_RESET_BUTTON_HEIGHT;
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE

        bool                        m_enabled;
        Shader                      m_shader;
        unsigned int                m_z_texture_id;
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        mutable GLTexture           m_tooltip_texture;
        mutable GLTexture           m_reset_texture;
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        // Not owned by LayersEditing.
        const DynamicPrintConfig   *m_config;
        // ModelObject for the currently selected object (Model::objects[last_object_id]).
        const ModelObject          *m_model_object;
        // Maximum z of the currently selected object (Model::objects[last_object_id]).
        float                       m_object_max_z;
        // Owned by LayersEditing.
        SlicingParameters          *m_slicing_parameters;
        std::vector<double>         m_layer_height_profile;
        bool                        m_layer_height_profile_modified;

#if ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        mutable float               m_adaptive_quality;
        mutable HeightProfileSmoothingParams m_smooth_params;
#endif // ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE

        class LayersTexture
        {
        public:
            LayersTexture() : width(0), height(0), levels(0), cells(0), valid(false) {}

            // Texture data
            std::vector<char>   data;
            // Width of the texture, top level.
            size_t              width;
            // Height of the texture, top level.
            size_t              height;
            // For how many levels of detail is the data allocated?
            size_t              levels;
            // Number of texture cells allocated for the height texture.
            size_t              cells;
            // Does it need to be refreshed?
            bool                valid;
        };
        LayersTexture   m_layers_texture;

    public:
        EState state;
        float band_width;
        float strength;
        int last_object_id;
        float last_z;
        LayerHeightEditActionType last_action;

        LayersEditing();
        ~LayersEditing();

        bool init(const std::string& vertex_shader_filename, const std::string& fragment_shader_filename);
		void set_config(const DynamicPrintConfig* config);
        void select_object(const Model &model, int object_id);

        bool is_allowed() const;

        bool is_enabled() const;
        void set_enabled(bool enabled);

        void render_overlay(const GLCanvas3D& canvas) const;
        void render_volumes(const GLCanvas3D& canvas, const GLVolumeCollection& volumes) const;

		void adjust_layer_height_profile();
		void accept_changes(GLCanvas3D& canvas);
        void reset_layer_height_profile(GLCanvas3D& canvas);
#if ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        void adaptive_layer_height_profile(GLCanvas3D& canvas, float quality_factor);
        void smooth_layer_height_profile(GLCanvas3D& canvas, const HeightProfileSmoothingParams& smoothing_params);
#endif // ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE

        static float get_cursor_z_relative(const GLCanvas3D& canvas);
        static bool bar_rect_contains(const GLCanvas3D& canvas, float x, float y);
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static bool reset_rect_contains(const GLCanvas3D& canvas, float x, float y);
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static Rect get_bar_rect_screen(const GLCanvas3D& canvas);
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static Rect get_reset_rect_screen(const GLCanvas3D& canvas);
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static Rect get_bar_rect_viewport(const GLCanvas3D& canvas);
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static Rect get_reset_rect_viewport(const GLCanvas3D& canvas);
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE

        float object_max_z() const { return m_object_max_z; }

        std::string get_tooltip(const GLCanvas3D& canvas) const;

    private:
        bool is_initialized() const;
        void generate_layer_height_texture();
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        void _render_tooltip_texture(const GLCanvas3D& canvas, const Rect& bar_rect, const Rect& reset_rect) const;
        void _render_reset_texture(const Rect& reset_rect) const;
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        void render_active_object_annotations(const GLCanvas3D& canvas, const Rect& bar_rect) const;
        void render_profile(const Rect& bar_rect) const;
        void update_slicing_parameters();

        static float thickness_bar_width(const GLCanvas3D &canvas);
#if !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
        static float reset_button_height(const GLCanvas3D &canvas);
#endif // !ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
    };

    struct Mouse
    {
        struct Drag
        {
            static const Point Invalid_2D_Point;
            static const Vec3d Invalid_3D_Point;
            static const int MoveThresholdPx;

            Point start_position_2D;
            Vec3d start_position_3D;
            int move_volume_idx;
            bool move_requires_threshold;
            Point move_start_threshold_position_2D;

        public:
            Drag();
        };

        bool dragging;
        Vec2d position;
        Vec3d scene_position;
        Drag drag;
        bool ignore_left_up;

        Mouse();

        void set_start_position_2D_as_invalid() { drag.start_position_2D = Drag::Invalid_2D_Point; }
        void set_start_position_3D_as_invalid() { drag.start_position_3D = Drag::Invalid_3D_Point; }
        void set_move_start_threshold_position_2D_as_invalid() { drag.move_start_threshold_position_2D = Drag::Invalid_2D_Point; }

        bool is_start_position_2D_defined() const { return (drag.start_position_2D != Drag::Invalid_2D_Point); }
        bool is_start_position_3D_defined() const { return (drag.start_position_3D != Drag::Invalid_3D_Point); }
        bool is_move_start_threshold_position_2D_defined() const { return (drag.move_start_threshold_position_2D != Drag::Invalid_2D_Point); }
        bool is_move_threshold_met(const Point& mouse_pos) const {
            return (std::abs(mouse_pos(0) - drag.move_start_threshold_position_2D(0)) > Drag::MoveThresholdPx)
                || (std::abs(mouse_pos(1) - drag.move_start_threshold_position_2D(1)) > Drag::MoveThresholdPx);
        }
    };

    struct SlaCap
    {
        struct Triangles
        {
            Pointf3s object;
            Pointf3s supports;
        };
        typedef std::map<unsigned int, Triangles> ObjectIdToTrianglesMap;
        double z;
        ObjectIdToTrianglesMap triangles;

        SlaCap() { reset(); }
        void reset() { z = DBL_MAX; triangles.clear(); }
        bool matches(double z) const { return this->z == z; }
    };

    class WarningTexture : public GUI::GLTexture
    {
    public:
        WarningTexture();

        enum Warning {
            ObjectOutside,
            ToolpathOutside,
            SlaSupportsOutside,
            SomethingNotShown,
            ObjectClashed
        };

        // Sets a warning of the given type to be active/inactive. If several warnings are active simultaneously,
        // only the last one is shown (decided by the order in the enum above).
        void activate(WarningTexture::Warning warning, bool state, const GLCanvas3D& canvas);
        void render(const GLCanvas3D& canvas) const;

        // function used to get an information for rescaling of the warning
        void msw_rescale(const GLCanvas3D& canvas);

    private:
        static const unsigned char Background_Color[3];
        static const unsigned char Opacity;

        int m_original_width;
        int m_original_height;

        // information for rescaling of the warning legend
        std::string     m_msg_text = "";
        bool            m_is_colored_red{false};

        // Information about which warnings are currently active.
        std::vector<Warning> m_warnings;

        // Generates the texture with given text.
        bool generate(const std::string& msg, const GLCanvas3D& canvas, bool compress, bool red_colored = false);
    };

    class LegendTexture : public GUI::GLTexture
    {
        static const int Px_Title_Offset = 5;
        static const int Px_Text_Offset = 5;
        static const int Px_Square = 20;
        static const int Px_Square_Contour = 1;
        static const int Px_Border = Px_Square / 2;
        static const unsigned char Squares_Border_Color[3];
        static const unsigned char Default_Background_Color[3];
        static const unsigned char Error_Background_Color[3];
        static const unsigned char Opacity;

        int m_original_width;
        int m_original_height;

    public:
        LegendTexture();
        void fill_color_print_legend_items(const GLCanvas3D& canvas,
                                           const std::vector<float>& colors_in,
                                           std::vector<float>& colors,
                                           std::vector<std::string>& cp_legend_items);

        bool generate(const GCodePreviewData& preview_data, const std::vector<float>& tool_colors, const GLCanvas3D& canvas, bool compress);

        void render(const GLCanvas3D& canvas) const;
    };

#if ENABLE_RENDER_STATISTICS
    struct RenderStats
    {
        long long last_frame;

        RenderStats() : last_frame(0) {}
    };
#endif // ENABLE_RENDER_STATISTICS

public:
    enum ECursorType : unsigned char
    {
        Standard,
        Cross
    };

private:
    wxGLCanvas* m_canvas;
    wxGLContext* m_context;
#if ENABLE_RETINA_GL
    std::unique_ptr<RetinaHelper> m_retina_helper;
#endif
    bool m_in_render;
    LegendTexture m_legend_texture;
    WarningTexture m_warning_texture;
    wxTimer m_timer;
    Bed3D& m_bed;
    Camera& m_camera;
    GLToolbar& m_view_toolbar;
    LayersEditing m_layers_editing;
    Shader m_shader;
    Mouse m_mouse;
    mutable GLGizmosManager m_gizmos;
    mutable GLToolbar m_main_toolbar;
    mutable GLToolbar m_undoredo_toolbar;
    ClippingPlane m_clipping_planes[2];
    mutable ClippingPlane m_camera_clipping_plane;
    bool m_use_clipping_planes;
    mutable SlaCap m_sla_caps[2];
    std::string m_sidebar_field;
    // when true renders an extra frame by not resetting m_dirty to false
    // see request_extra_frame()
    bool m_extra_frame_requested;

    mutable GLVolumeCollection m_volumes;
    Selection m_selection;
    const DynamicPrintConfig* m_config;
    Model* m_model;
    BackgroundSlicingProcess *m_process;

    // Screen is only refreshed from the OnIdle handler if it is dirty.
    bool m_dirty;
    bool m_initialized;
    bool m_apply_zoom_to_volumes_filter;
    mutable std::vector<int> m_hover_volume_idxs;
    bool m_legend_texture_enabled;
    bool m_picking_enabled;
    bool m_moving_enabled;
    bool m_dynamic_background_enabled;
    bool m_multisample_allowed;
    bool m_moving;
    bool m_tab_down;
    ECursorType m_cursor_type;
    GLSelectionRectangle m_rectangle_selection;

    // Following variable is obsolete and it should be safe to remove it.
    // I just don't want to do it now before a release (Lukas Matena 24.3.2019)
    bool m_render_sla_auxiliaries;

    std::string m_color_by;

    bool m_reload_delayed;

    GCodePreviewVolumeIndex m_gcode_preview_volume_index;

#if ENABLE_RENDER_PICKING_PASS
    bool m_show_picking_texture;
#endif // ENABLE_RENDER_PICKING_PASS

#if ENABLE_RENDER_STATISTICS
    RenderStats m_render_stats;
#endif // ENABLE_RENDER_STATISTICS

    mutable int m_imgui_undo_redo_hovered_pos{ -1 };
    int m_selected_extruder;

public:
    GLCanvas3D(wxGLCanvas* canvas, Bed3D& bed, Camera& camera, GLToolbar& view_toolbar);
    ~GLCanvas3D();

    void set_context(wxGLContext* context) { m_context = context; }

    wxGLCanvas* get_wxglcanvas() { return m_canvas; }
	const wxGLCanvas* get_wxglcanvas() const { return m_canvas; }

    bool init();
    void post_event(wxEvent &&event);

    void set_as_dirty();

    unsigned int get_volumes_count() const;
    void reset_volumes();
    int check_volumes_outside_state() const;

    void toggle_sla_auxiliaries_visibility(bool visible, const ModelObject* mo = nullptr, int instance_idx = -1);
    void toggle_model_objects_visibility(bool visible, const ModelObject* mo = nullptr, int instance_idx = -1);
    void update_instance_printable_state_for_object(size_t obj_idx);
    void update_instance_printable_state_for_objects(std::vector<size_t>& object_idxs);

    void set_config(const DynamicPrintConfig* config);
    void set_process(BackgroundSlicingProcess* process);
    void set_model(Model* model);

    const Selection& get_selection() const { return m_selection; }
    Selection& get_selection() { return m_selection; }

    const GLGizmosManager& get_gizmos_manager() const { return m_gizmos; }
    GLGizmosManager& get_gizmos_manager() { return m_gizmos; }

    void bed_shape_changed();

    void set_clipping_plane(unsigned int id, const ClippingPlane& plane)
    {
        if (id < 2)
        {
            m_clipping_planes[id] = plane;
            m_sla_caps[id].reset();
        }
    }
    void reset_clipping_planes_cache() { m_sla_caps[0].triangles.clear(); m_sla_caps[1].triangles.clear(); }
    void set_use_clipping_planes(bool use) { m_use_clipping_planes = use; }

    void set_color_by(const std::string& value);

    const Camera& get_camera() const { return m_camera; }
    Camera& get_camera() { return m_camera; }

    BoundingBoxf3 volumes_bounding_box() const;
    BoundingBoxf3 scene_bounding_box() const;

    bool is_layers_editing_enabled() const;
    bool is_layers_editing_allowed() const;

#if ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE
    void reset_layer_height_profile();
    void adaptive_layer_height_profile(float quality_factor);
    void smooth_layer_height_profile(const HeightProfileSmoothingParams& smoothing_params);
#endif // ENABLE_ADAPTIVE_LAYER_HEIGHT_PROFILE

    bool is_reload_delayed() const;

    void enable_layers_editing(bool enable);
    void enable_legend_texture(bool enable);
    void enable_picking(bool enable);
    void enable_moving(bool enable);
    void enable_gizmos(bool enable);
    void enable_selection(bool enable);
    void enable_main_toolbar(bool enable);
    void enable_undoredo_toolbar(bool enable);
    void enable_dynamic_background(bool enable);
    void allow_multisample(bool allow);

    void zoom_to_bed();
    void zoom_to_volumes();
    void zoom_to_selection();
    void select_view(const std::string& direction);

    void update_volumes_colors_by_extruder();

    bool is_dragging() const { return m_gizmos.is_dragging() || m_moving; }

    void render();
#if ENABLE_THUMBNAIL_GENERATOR
    // printable_only == false -> render also non printable volumes as grayed
    // parts_only == false -> render also sla support and pad
    void render_thumbnail(ThumbnailData& thumbnail_data, unsigned int w, unsigned int h, bool printable_only, bool parts_only, bool show_bed, bool transparent_background) const;
#endif // ENABLE_THUMBNAIL_GENERATOR

    void select_all();
    void deselect_all();
    void delete_selected();
    void ensure_on_bed(unsigned int object_idx);

    std::vector<double> get_current_print_zs(bool active_only) const;
    void set_toolpaths_range(double low, double high);

    std::vector<int> load_object(const ModelObject& model_object, int obj_idx, std::vector<int> instance_idxs);
    std::vector<int> load_object(const Model& model, int obj_idx);

    void mirror_selection(Axis axis);

    void reload_scene(bool refresh_immediately, bool force_full_scene_refresh = false);

    void load_gcode_preview(const GCodePreviewData& preview_data, const std::vector<std::string>& str_tool_colors);
    void load_sla_preview();
    void load_preview(const std::vector<std::string>& str_tool_colors, const std::vector<Model::CustomGCode>& color_print_values);
    void bind_event_handlers();
    void unbind_event_handlers();

    void on_size(wxSizeEvent& evt);
    void on_idle(wxIdleEvent& evt);
    void on_char(wxKeyEvent& evt);
    void on_key(wxKeyEvent& evt);
    void on_mouse_wheel(wxMouseEvent& evt);
    void on_timer(wxTimerEvent& evt);
    void on_mouse(wxMouseEvent& evt);
    void on_paint(wxPaintEvent& evt);

    Size get_canvas_size() const;
    Vec2d get_local_mouse_position() const;

    void reset_legend_texture();

    void set_tooltip(const std::string& tooltip) const;

    // the following methods add a snapshot to the undo/redo stack, unless the given string is empty
    void do_move(const std::string& snapshot_type);
    void do_rotate(const std::string& snapshot_type);
    void do_scale(const std::string& snapshot_type);
    void do_flatten(const Vec3d& normal, const std::string& snapshot_type);
    void do_mirror(const std::string& snapshot_type);

    void update_gizmos_on_off_state();
    void reset_all_gizmos() { m_gizmos.reset_all_states(); }

    void handle_sidebar_focus_event(const std::string& opt_key, bool focus_on);
    void handle_layers_data_focus_event(const t_layer_height_range range, const EditorType type);

    void update_ui_from_settings();

    float get_view_toolbar_height() const { return m_view_toolbar.get_height(); }

    int get_move_volume_id() const { return m_mouse.drag.move_volume_idx; }
    int get_first_hover_volume_idx() const { return m_hover_volume_idxs.empty() ? -1 : m_hover_volume_idxs.front(); }
    void set_selected_extruder(int extruder) { m_selected_extruder = extruder;}
    
    class WipeTowerInfo {
    protected:
        Vec2d m_pos = {std::nan(""), std::nan("")};
        Vec2d m_bb_size = {0., 0.};
        double m_rotation = 0.;
        friend class GLCanvas3D;
    public:
        
        inline operator bool() const
        {
            return !std::isnan(m_pos.x()) && !std::isnan(m_pos.y());
        }
        
        inline const Vec2d& pos() const { return m_pos; }
        inline double rotation() const { return m_rotation; }
        inline const Vec2d bb_size() const { return m_bb_size; }
        
        void apply_wipe_tower() const;
    };
    
    WipeTowerInfo get_wipe_tower_info() const;

    // Returns the view ray line, in world coordinate, at the given mouse position.
    Linef3 mouse_ray(const Point& mouse_pos);

    void set_mouse_as_dragging() { m_mouse.dragging = true; }
    void refresh_camera_scene_box() { m_camera.set_scene_box(scene_bounding_box()); }
    bool is_mouse_dragging() const { return m_mouse.dragging; }

    double get_size_proportional_to_max_bed_size(double factor) const;

    void set_cursor(ECursorType type);
    void msw_rescale();

    void request_extra_frame() { m_extra_frame_requested = true; }

    int get_main_toolbar_item_id(const std::string& name) const { return m_main_toolbar.get_item_id(name); }
    void force_main_toolbar_left_action(int item_id) { m_main_toolbar.force_left_action(item_id, *this); }
    void force_main_toolbar_right_action(int item_id) { m_main_toolbar.force_right_action(item_id, *this); }

    bool has_toolpaths_to_export() const;
    void export_toolpaths_to_obj(const char* filename) const;

    void mouse_up_cleanup();

private:
    bool _is_shown_on_screen() const;

    bool _init_toolbars();
    bool _init_main_toolbar();
    bool _init_undoredo_toolbar();
#if ENABLE_VIEW_TOOLBAR_BACKGROUND_FIX
    bool _init_view_toolbar();
#endif // ENABLE_VIEW_TOOLBAR_BACKGROUND_FIX

    bool _set_current();
    void _resize(unsigned int w, unsigned int h);

    BoundingBoxf3 _max_bounding_box(bool include_gizmos, bool include_bed_model) const;

#if ENABLE_THUMBNAIL_GENERATOR
    void _zoom_to_box(const BoundingBoxf3& box, double margin_factor = DefaultCameraZoomToBoxMarginFactor);
#else
    void _zoom_to_box(const BoundingBoxf3& box);
#endif // ENABLE_THUMBNAIL_GENERATOR
    void _update_camera_zoom(double zoom);

    void _refresh_if_shown_on_screen();

    void _picking_pass() const;
    void _rectangular_selection_picking_pass() const;
    void _render_background() const;
    void _render_bed(float theta, bool show_axes) const;
    void _render_objects() const;
    void _render_selection() const;
#if ENABLE_RENDER_SELECTION_CENTER
    void _render_selection_center() const;
#endif // ENABLE_RENDER_SELECTION_CENTER
    void _render_overlays() const;
    void _render_warning_texture() const;
    void _render_legend_texture() const;
    void _render_volumes_for_picking() const;
    void _render_current_gizmo() const;
    void _render_gizmos_overlay() const;
    void _render_main_toolbar() const;
    void _render_undoredo_toolbar() const;
    void _render_view_toolbar() const;
#if ENABLE_SHOW_CAMERA_TARGET
    void _render_camera_target() const;
#endif // ENABLE_SHOW_CAMERA_TARGET
    void _render_sla_slices() const;
    void _render_selection_sidebar_hints() const;
    void _render_undo_redo_stack(const bool is_undo, float pos_x) const;
#if ENABLE_THUMBNAIL_GENERATOR
    void _render_thumbnail_internal(ThumbnailData& thumbnail_data, bool printable_only, bool parts_only, bool show_bed, bool transparent_background) const;
    // render thumbnail using an off-screen framebuffer
    void _render_thumbnail_framebuffer(ThumbnailData& thumbnail_data, unsigned int w, unsigned int h, bool printable_only, bool parts_only, bool show_bed, bool transparent_background) const;
    // render thumbnail using an off-screen framebuffer when GLEW_EXT_framebuffer_object is supported
    void _render_thumbnail_framebuffer_ext(ThumbnailData& thumbnail_data, unsigned int w, unsigned int h, bool printable_only, bool parts_only, bool show_bed, bool transparent_background) const;
    // render thumbnail using the default framebuffer
    void _render_thumbnail_legacy(ThumbnailData& thumbnail_data, unsigned int w, unsigned int h, bool printable_only, bool parts_only, bool show_bed, bool transparent_background) const;
#endif // ENABLE_THUMBNAIL_GENERATOR

    void _update_volumes_hover_state() const;

    void _perform_layer_editing_action(wxMouseEvent* evt = nullptr);

    // Convert the screen space coordinate to an object space coordinate.
    // If the Z screen space coordinate is not provided, a depth buffer value is substituted.
    Vec3d _mouse_to_3d(const Point& mouse_pos, float* z = nullptr);

    // Convert the screen space coordinate to world coordinate on the bed.
    Vec3d _mouse_to_bed_3d(const Point& mouse_pos);

    void _start_timer();
    void _stop_timer();

    // Create 3D thick extrusion lines for a skirt and brim.
    // Adds a new Slic3r::GUI::3DScene::Volume to volumes.
    void _load_print_toolpaths();
    // Create 3D thick extrusion lines for object forming extrusions.
    // Adds a new Slic3r::GUI::3DScene::Volume to $self->volumes,
    // one for perimeters, one for infill and one for supports.
    void _load_print_object_toolpaths(const PrintObject& print_object, const std::vector<std::string>& str_tool_colors,
                                      const std::vector<Model::CustomGCode>& color_print_values);
    // Create 3D thick extrusion lines for wipe tower extrusions
    void _load_wipe_tower_toolpaths(const std::vector<std::string>& str_tool_colors);

    // generates gcode extrusion paths geometry
    void _load_gcode_extrusion_paths(const GCodePreviewData& preview_data, const std::vector<float>& tool_colors);
    // generates gcode travel paths geometry
    void _load_gcode_travel_paths(const GCodePreviewData& preview_data, const std::vector<float>& tool_colors);
    // generates objects and wipe tower geometry
    void _load_fff_shells();
    // Load SLA objects and support structures for objects, for which the slaposSliceSupports step has been finished.
	void _load_sla_shells();
    // sets gcode geometry visibility according to user selection
    void _update_gcode_volumes_visibility(const GCodePreviewData& preview_data);
    void _update_toolpath_volumes_outside_state();
    void _update_sla_shells_outside_state();
    void _show_warning_texture_if_needed(WarningTexture::Warning warning);

    // generates the legend texture in dependence of the current shown view type
    void _generate_legend_texture(const GCodePreviewData& preview_data, const std::vector<float>& tool_colors);

    // generates a warning texture containing the given message
    void _set_warning_texture(WarningTexture::Warning warning, bool state);

    bool _is_any_volume_outside() const;

    // updates the selection from the content of m_hover_volume_idxs
    void _update_selection_from_hover();

    bool _deactivate_undo_redo_toolbar_items();

    static std::vector<float> _parse_colors(const std::vector<std::string>& colors);

public:
    const Print* fff_print() const;
    const SLAPrint* sla_print() const;
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLCanvas3D_hpp_
