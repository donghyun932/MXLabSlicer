#include "libslic3r/libslic3r.h"
#include "GLGizmosManager.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/3DScene.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/GUI_ObjectManipulation.hpp"
#include "slic3r/GUI/PresetBundle.hpp"
#include "slic3r/Utils/UndoRedo.hpp"

#include <GL/glew.h>
#include <wx/glcanvas.h>

namespace Slic3r {
namespace GUI {

const float GLGizmosManager::Default_Icons_Size = 64;

GLGizmosManager::GLGizmosManager(GLCanvas3D& parent)
    : m_parent(parent)
    , m_enabled(false)
    , m_icons_texture_dirty(true)
    , m_current(Undefined)
    , m_tooltip("")
    , m_serializing(false)
{
}

std::vector<size_t> GLGizmosManager::get_selectable_idxs() const
{
    std::vector<size_t> out;
    for (size_t i=0; i<m_gizmos.size(); ++i)
        if (m_gizmos[i]->is_selectable())
            out.push_back(i);
    return out;
}

std::vector<size_t> GLGizmosManager::get_activable_idxs() const
{
    std::vector<size_t> out;
    for (size_t i=0; i<m_gizmos.size(); ++i)
        if (m_gizmos[i]->is_activable())
            out.push_back(i);
    return out;
}

size_t GLGizmosManager::get_gizmo_idx_from_mouse(const Vec2d& mouse_pos) const
{
    if (! m_enabled)
        return Undefined;

    float cnv_h = (float)m_parent.get_canvas_size().get_height();
    float height = get_scaled_total_height();
    float icons_size = m_layout.scaled_icons_size();
    float border = m_layout.scaled_border();
    float stride_y = m_layout.scaled_stride_y();
    float top_y = 0.5f * (cnv_h - height) + border;

    // is mouse horizontally in the area?
    if ((border <= (float)mouse_pos(0) && ((float)mouse_pos(0) <= border + icons_size))) {
        // which icon is it on?
        size_t from_top = (size_t)((float)mouse_pos(1) - top_y) / stride_y;
        // is it really on the icon or already past the border?
        if ((float)mouse_pos(1) <= top_y + from_top * stride_y + icons_size) {
            std::vector<size_t> selectable = get_selectable_idxs();
            if (from_top < selectable.size())
                return selectable[from_top];
        }
    }
    return Undefined;
}

bool GLGizmosManager::init()
{
    m_background_texture.metadata.filename = "toolbar_background.png";
    m_background_texture.metadata.left = 16;
    m_background_texture.metadata.top = 16;
    m_background_texture.metadata.right = 16;
    m_background_texture.metadata.bottom = 16;

    if (!m_background_texture.metadata.filename.empty())
    {
        if (!m_background_texture.texture.load_from_file(resources_dir() + "/icons/" + m_background_texture.metadata.filename, false, GLTexture::SingleThreaded, false))
            return false;
    }

    m_gizmos.emplace_back(new GLGizmoMove3D(m_parent, "move.svg", 0));
    m_gizmos.emplace_back(new GLGizmoScale3D(m_parent, "scale.svg", 1));
    m_gizmos.emplace_back(new GLGizmoRotate3D(m_parent, "rotate.svg", 2));
    m_gizmos.emplace_back(new GLGizmoFlatten(m_parent, "place.svg", 3));
    m_gizmos.emplace_back(new GLGizmoCut(m_parent, "cut.svg", 4));
    m_gizmos.emplace_back(new GLGizmoSlaSupports(m_parent, "sla_supports.svg", 5));

    for (auto& gizmo : m_gizmos) {
        if (! gizmo->init()) {
            m_gizmos.clear();
            return false;
        }
    }

    m_current = Undefined;
    m_hover = Undefined;

    return true;
}

void GLGizmosManager::set_overlay_icon_size(float size)
{
    if (m_layout.icons_size != size)
    {
        m_layout.icons_size = size;
        m_icons_texture_dirty = true;
    }
}

void GLGizmosManager::set_overlay_scale(float scale)
{
    if (m_layout.scale != scale)
    {
        m_layout.scale = scale;
        m_icons_texture_dirty = true;
    }
}

void GLGizmosManager::refresh_on_off_state()
{
    if (m_serializing || m_current == Undefined || m_gizmos.empty())
        return;

    if (m_current != Undefined && ! m_gizmos[m_current]->is_activable())
        activate_gizmo(Undefined);
}

void GLGizmosManager::reset_all_states()
{
    if (! m_enabled || m_serializing)
        return;

    activate_gizmo(Undefined);
    m_hover = Undefined;
}

void GLGizmosManager::set_hover_id(int id)
{
    if (!m_enabled || m_current == Undefined)
        return;

    m_gizmos[m_current]->set_hover_id(id);
}

void GLGizmosManager::enable_grabber(EType type, unsigned int id, bool enable)
{
    if (!m_enabled || type == Undefined || m_gizmos.empty())
        return;

    if (enable)
        m_gizmos[type]->enable_grabber(id);
    else
        m_gizmos[type]->disable_grabber(id);
}

void GLGizmosManager::update(const Linef3& mouse_ray, const Point& mouse_pos)
{
    if (!m_enabled)
        return;

    GLGizmoBase* curr = get_current();
    if (curr != nullptr)
        curr->update(GLGizmoBase::UpdateData(mouse_ray, mouse_pos));
}

void GLGizmosManager::update_data()
{
    if (!m_enabled)
        return;

    const Selection& selection = m_parent.get_selection();

    bool is_wipe_tower = selection.is_wipe_tower();
    enable_grabber(Move, 2, !is_wipe_tower);
    enable_grabber(Rotate, 0, !is_wipe_tower);
    enable_grabber(Rotate, 1, !is_wipe_tower);

    bool enable_scale_xyz = selection.is_single_full_instance() || selection.is_single_volume() || selection.is_single_modifier();
    for (unsigned int i = 0; i < 6; ++i)
    {
        enable_grabber(Scale, i, enable_scale_xyz);
    }

    if (selection.is_single_full_instance())
    {
        // all volumes in the selection belongs to the same instance, any of them contains the needed data, so we take the first
        const GLVolume* volume = selection.get_volume(*selection.get_volume_idxs().begin());
        set_scale(volume->get_instance_scaling_factor());
        set_rotation(Vec3d::Zero());
        ModelObject* model_object = selection.get_model()->objects[selection.get_object_idx()];
        set_flattening_data(model_object);
        set_sla_support_data(model_object);
    }
    else if (selection.is_single_volume() || selection.is_single_modifier())
    {
        const GLVolume* volume = selection.get_volume(*selection.get_volume_idxs().begin());
        set_scale(volume->get_volume_scaling_factor());
        set_rotation(Vec3d::Zero());
        set_flattening_data(nullptr);
        set_sla_support_data(nullptr);
    }
    else if (is_wipe_tower)
    {
        DynamicPrintConfig& config = wxGetApp().preset_bundle->prints.get_edited_preset().config;
        set_scale(Vec3d::Ones());
        set_rotation(Vec3d(0., 0., (M_PI/180.) * dynamic_cast<const ConfigOptionFloat*>(config.option("wipe_tower_rotation_angle"))->value));
        set_flattening_data(nullptr);
        set_sla_support_data(nullptr);
    }
    else
    {
        set_scale(Vec3d::Ones());
        set_rotation(Vec3d::Zero());
        set_flattening_data(selection.is_from_single_object() ? selection.get_model()->objects[selection.get_object_idx()] : nullptr);
        set_sla_support_data(nullptr);
    }
}

bool GLGizmosManager::is_running() const
{
    if (!m_enabled)
        return false;

    //GLGizmoBase* curr = get_current();
    //return (curr != nullptr) ? (curr->get_state() == GLGizmoBase::On) : false;
    return m_current != Undefined;
}

bool GLGizmosManager::handle_shortcut(int key)
{
    if (!m_enabled)
        return false;

    if (m_parent.get_selection().is_empty())
        return false;

    bool handled = false;

    for (size_t idx : get_selectable_idxs()) {
        int it_key = m_gizmos[idx]->get_shortcut_key();

        if (m_gizmos[idx]->is_activable() && ((it_key == key - 64) || (it_key == key - 96))) {
                activate_gizmo(m_current == idx ? Undefined : (EType)idx);
                handled = true;
        }
    }

    return handled;
}

bool GLGizmosManager::is_dragging() const
{
    if (! m_enabled || m_current == Undefined)
        return false;

    return m_gizmos[m_current]->is_dragging();
}

void GLGizmosManager::start_dragging()
{
    if (! m_enabled || m_current == Undefined)
        return;
    m_gizmos[m_current]->start_dragging();
}

void GLGizmosManager::stop_dragging()
{
    if (! m_enabled || m_current == Undefined)
        return;

    m_gizmos[m_current]->stop_dragging();
}

Vec3d GLGizmosManager::get_displacement() const
{
    if (!m_enabled)
        return Vec3d::Zero();

    return dynamic_cast<GLGizmoMove3D*>(m_gizmos[Move].get())->get_displacement();
}

Vec3d GLGizmosManager::get_scale() const
{
    if (!m_enabled)
        return Vec3d::Ones();

    return dynamic_cast<GLGizmoScale3D*>(m_gizmos[Scale].get())->get_scale();
}

void GLGizmosManager::set_scale(const Vec3d& scale)
{
    if (!m_enabled || m_gizmos.empty())
        return;

    dynamic_cast<GLGizmoScale3D*>(m_gizmos[Scale].get())->set_scale(scale);
}

Vec3d GLGizmosManager::get_scale_offset() const
{
    if (!m_enabled || m_gizmos.empty())
        return Vec3d::Zero();

    return dynamic_cast<GLGizmoScale3D*>(m_gizmos[Scale].get())->get_offset();
}

Vec3d GLGizmosManager::get_rotation() const
{
    if (!m_enabled || m_gizmos.empty())
        return Vec3d::Zero();

    return dynamic_cast<GLGizmoRotate3D*>(m_gizmos[Rotate].get())->get_rotation();
}

void GLGizmosManager::set_rotation(const Vec3d& rotation)
{
    if (!m_enabled || m_gizmos.empty())
        return;
    dynamic_cast<GLGizmoRotate3D*>(m_gizmos[Rotate].get())->set_rotation(rotation);
}

Vec3d GLGizmosManager::get_flattening_normal() const
{
    if (!m_enabled || m_gizmos.empty())
        return Vec3d::Zero();

    return dynamic_cast<GLGizmoFlatten*>(m_gizmos[Flatten].get())->get_flattening_normal();
}

void GLGizmosManager::set_flattening_data(const ModelObject* model_object)
{
    if (!m_enabled || m_gizmos.empty())
        return;

    dynamic_cast<GLGizmoFlatten*>(m_gizmos[Flatten].get())->set_flattening_data(model_object);
}

void GLGizmosManager::set_sla_support_data(ModelObject* model_object)
{
    if (!m_enabled || m_gizmos.empty())
        return;

    dynamic_cast<GLGizmoSlaSupports*>(m_gizmos[SlaSupports].get())->set_sla_support_data(model_object, m_parent.get_selection());
}

// Returns true if the gizmo used the event to do something, false otherwise.
bool GLGizmosManager::gizmo_event(SLAGizmoEventType action, const Vec2d& mouse_position, bool shift_down, bool alt_down, bool control_down)
{
    if (!m_enabled || m_gizmos.empty())
        return false;

    return dynamic_cast<GLGizmoSlaSupports*>(m_gizmos[SlaSupports].get())->gizmo_event(action, mouse_position, shift_down, alt_down, control_down);
}

ClippingPlane GLGizmosManager::get_sla_clipping_plane() const
{
    if (!m_enabled || m_current != SlaSupports || m_gizmos.empty())
        return ClippingPlane::ClipsNothing();

    return dynamic_cast<GLGizmoSlaSupports*>(m_gizmos[SlaSupports].get())->get_sla_clipping_plane();
}

bool GLGizmosManager::wants_reslice_supports_on_undo() const
{
    return (m_current == SlaSupports
        && dynamic_cast<const GLGizmoSlaSupports*>(m_gizmos.at(SlaSupports).get())->has_backend_supports());
}

void GLGizmosManager::render_current_gizmo() const
{
    if (!m_enabled || m_current == Undefined)
        return;

    m_gizmos[m_current]->render();
}

void GLGizmosManager::render_current_gizmo_for_picking_pass() const
{
    if (! m_enabled || m_current == Undefined)
        return;

    m_gizmos[m_current]->render_for_picking();
}

void GLGizmosManager::render_overlay() const
{
    if (!m_enabled)
        return;

    if (m_icons_texture_dirty)
        generate_icons_texture();

    do_render_overlay();
}

bool GLGizmosManager::on_mouse_wheel(wxMouseEvent& evt)
{
    bool processed = false;

    if (m_current == SlaSupports) {
        float rot = (float)evt.GetWheelRotation() / (float)evt.GetWheelDelta();
        if (gizmo_event((rot > 0.f ? SLAGizmoEventType::MouseWheelUp : SLAGizmoEventType::MouseWheelDown), Vec2d::Zero(), evt.ShiftDown(), evt.AltDown(), evt.ControlDown()))
            processed = true;
    }

    return processed;
}

bool GLGizmosManager::on_mouse(wxMouseEvent& evt)
{
    // used to set a right up event as processed when needed
    static bool pending_right_up = false;

    Point pos(evt.GetX(), evt.GetY());
    Vec2d mouse_pos((double)evt.GetX(), (double)evt.GetY());

    Selection& selection = m_parent.get_selection();
    int selected_object_idx = selection.get_object_idx();
    bool processed = false;

    // mouse anywhere
    if (!evt.Dragging() && !evt.Leaving() && !evt.Entering() && (m_mouse_capture.parent != nullptr))
    {
        if (m_mouse_capture.any() && (evt.LeftUp() || evt.MiddleUp() || evt.RightUp()))
            // prevents loosing selection into the scene if mouse down was done inside the toolbar and mouse up was down outside it
            processed = true;

        m_mouse_capture.reset();
    }

    // mouse anywhere
    if (evt.Moving())
        m_tooltip = update_hover_state(mouse_pos);
    else if (evt.LeftUp())
        m_mouse_capture.left = false;
    else if (evt.MiddleUp())
        m_mouse_capture.middle = false;
    else if (evt.RightUp())
    {
        m_mouse_capture.right = false;
        if (pending_right_up)
        {
            pending_right_up = false;
            processed = true;
        }
    }
    else if (evt.Dragging() && m_mouse_capture.any())
        // if the button down was done on this toolbar, prevent from dragging into the scene
        processed = true;

    if (get_gizmo_idx_from_mouse(mouse_pos) == Undefined)
    {
        // mouse is outside the toolbar
        m_tooltip = "";

        if (evt.LeftDown())
        {
            if ((m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::LeftDown, mouse_pos, evt.ShiftDown(), evt.AltDown(), evt.ControlDown()))
                // the gizmo got the event and took some action, there is no need to do anything more
                processed = true;
            else if (!selection.is_empty() && grabber_contains_mouse()) {
                update_data();
                selection.start_dragging();
                start_dragging();

                if (m_current == Flatten) {
                    // Rotate the object so the normal points downward:
                    m_parent.do_flatten(get_flattening_normal(), L("Gizmo-Place on Face"));
                    wxGetApp().obj_manipul()->set_dirty();
                }

                m_parent.set_as_dirty();
                processed = true;
            }
        }
        else if (evt.RightDown() && (selected_object_idx != -1) && (m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::RightDown))
        {
            // we need to set the following right up as processed to avoid showing the context menu if the user release the mouse over the object
            pending_right_up = true;
            // event was taken care of by the SlaSupports gizmo
            processed = true;
        }
        else if (evt.Dragging() && (m_parent.get_move_volume_id() != -1) && (m_current == SlaSupports))
                        // don't allow dragging objects with the Sla gizmo on
            processed = true;
        else if (evt.Dragging() && (m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::Dragging, mouse_pos, evt.ShiftDown(), evt.AltDown(), evt.ControlDown()))
        {
            // the gizmo got the event and took some action, no need to do anything more here
            m_parent.set_as_dirty();
            processed = true;
        }
        else if (evt.Dragging() && is_dragging())
        {
            if (!m_parent.get_wxglcanvas()->HasCapture())
                m_parent.get_wxglcanvas()->CaptureMouse();

            m_parent.set_mouse_as_dragging();
            update(m_parent.mouse_ray(pos), pos);

            switch (m_current)
            {
            case Move:
            {
                // Apply new temporary offset
                selection.translate(get_displacement());
                wxGetApp().obj_manipul()->set_dirty();
                break;
            }
            case Scale:
            {
                // Apply new temporary scale factors
				TransformationType transformation_type(TransformationType::Local_Absolute_Joint);
				if (evt.AltDown())
					transformation_type.set_independent();
				selection.scale(get_scale(), transformation_type);
                if (evt.ControlDown())
                    selection.translate(get_scale_offset(), true);
                wxGetApp().obj_manipul()->set_dirty();
                break;
            }
            case Rotate:
            {
                // Apply new temporary rotations
                TransformationType transformation_type(TransformationType::World_Relative_Joint);
                if (evt.AltDown())
                    transformation_type.set_independent();
                selection.rotate(get_rotation(), transformation_type);
                wxGetApp().obj_manipul()->set_dirty();
                break;
            }
            default:
                break;
            }

            m_parent.set_as_dirty();
            processed = true;
        }
        else if (evt.LeftUp() && is_dragging())
        {
            switch (m_current) {
            case Move : m_parent.do_move(L("Gizmo-Move"));
                        break;
            case Scale : m_parent.do_scale(L("Gizmo-Scale"));
                         break;
            case Rotate : m_parent.do_rotate(L("Gizmo-Rotate"));
                          break;
            default : break;
            }

            stop_dragging();
            update_data();

            wxGetApp().obj_manipul()->set_dirty();
            // Let the platter know that the dragging finished, so a delayed refresh
            // of the scene with the background processing data should be performed.
            m_parent.post_event(SimpleEvent(EVT_GLCANVAS_MOUSE_DRAGGING_FINISHED));
            // updates camera target constraints
            m_parent.refresh_camera_scene_box();

            processed = true;
        }
        else if (evt.LeftUp() && (m_current == SlaSupports) && !m_parent.is_mouse_dragging())
        {
            // in case SLA gizmo is selected, we just pass the LeftUp event and stop processing - neither
            // object moving or selecting is suppressed in that case
            gizmo_event(SLAGizmoEventType::LeftUp, mouse_pos, evt.ShiftDown(), evt.AltDown(), evt.ControlDown());
            processed = true;
        }
        else if (evt.LeftUp() && (m_current == Flatten) && (m_gizmos[m_current]->get_hover_id() != -1))
        {
            // to avoid to loose the selection when user clicks an the white faces of a different object while the Flatten gizmo is active
            processed = true;
        }
    }
    else
    {
        // mouse inside toolbar
        if (evt.LeftDown() || evt.LeftDClick())
        {
            m_mouse_capture.left = true;
            m_mouse_capture.parent = &m_parent;
            processed = true;
            if (!selection.is_empty())
            {
                update_on_off_state(mouse_pos);
                update_data();
                m_parent.set_as_dirty();
            }
        }
        else if (evt.MiddleDown())
        {
            m_mouse_capture.middle = true;
            m_mouse_capture.parent = &m_parent;
        }
        else if (evt.RightDown())
        {
            m_mouse_capture.right = true;
            m_mouse_capture.parent = &m_parent;
        }
        else if (evt.LeftUp())
            processed = true;
    }

    return processed;
}

bool GLGizmosManager::on_char(wxKeyEvent& evt)
{
    // see include/wx/defs.h enum wxKeyCode
    int keyCode = evt.GetKeyCode();
    int ctrlMask = wxMOD_CONTROL;

    bool processed = false;

    if ((evt.GetModifiers() & ctrlMask) != 0)
    {
        switch (keyCode)
        {
#ifdef __APPLE__
        case 'a':
        case 'A':
#else /* __APPLE__ */
        case WXK_CONTROL_A:
#endif /* __APPLE__ */
        {
            // Sla gizmo selects all support points
            if ((m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::SelectAll))
                processed = true;

            break;
        }
        }
    }
    else if (!evt.HasModifiers())
    {
        switch (keyCode)
        {
        // key ESC
        case WXK_ESCAPE:
        {
            if (m_current != Undefined)
            {
                if ((m_current != SlaSupports) || !gizmo_event(SLAGizmoEventType::DiscardChanges))
                    reset_all_states();

                processed = true;
            }
            break;
        }
        case WXK_RETURN:
        {
            if ((m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::ApplyChanges))
                processed = true;

            break;
        }

        case 'r' :
        case 'R' :
        {
            if ((m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::ResetClippingPlane))
                processed = true;

            break;
        }

#ifdef __APPLE__
        case WXK_BACK: // the low cost Apple solutions are not equipped with a Delete key, use Backspace instead.
#else /* __APPLE__ */
        case WXK_DELETE:
#endif /* __APPLE__ */
        {
            if ((m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::Delete))
                processed = true;

            break;
        }
        case 'A':
        case 'a':
        {
            if (m_current == SlaSupports)
            {
                gizmo_event(SLAGizmoEventType::AutomaticGeneration);
                // set as processed no matter what's returned by gizmo_event() to avoid the calling canvas to process 'A' as arrange
                processed = true;
            }
            break;
        }
        case 'M':
        case 'm':
        {
            if ((m_current == SlaSupports) && gizmo_event(SLAGizmoEventType::ManualEditing))
                processed = true;
                
            break;
        }
        case 'F':
        case 'f':
        {
            if (m_current == Scale)
            {
                if (!is_dragging())
                    wxGetApp().plater()->scale_selection_to_fit_print_volume();

                processed = true;
            }

            break;
        }
        }
    }

    if (!processed && !evt.HasModifiers())
    {
        if (handle_shortcut(keyCode))
        {
            update_data();
            processed = true;
        }
    }

    if (processed)
        m_parent.set_as_dirty();

    return processed;
}

bool GLGizmosManager::on_key(wxKeyEvent& evt)
{
    const int keyCode = evt.GetKeyCode();
    bool processed = false;

    if (evt.GetEventType() == wxEVT_KEY_UP)
    {
        if (m_current == SlaSupports)
        {
            GLGizmoSlaSupports* gizmo = dynamic_cast<GLGizmoSlaSupports*>(get_current());

            if (keyCode == WXK_SHIFT)
            {
                // shift has been just released - SLA gizmo might want to close rectangular selection.
                if (gizmo_event(SLAGizmoEventType::ShiftUp) || (gizmo->is_in_editing_mode() && gizmo->is_selection_rectangle_dragging()))
                    processed = true;
            }
            else if (keyCode == WXK_ALT)
            {
                // alt has been just released - SLA gizmo might want to close rectangular selection.
                if (gizmo_event(SLAGizmoEventType::AltUp) || (gizmo->is_in_editing_mode() && gizmo->is_selection_rectangle_dragging()))
                    processed = true;
            }
        }

//        if (processed)
//            m_parent.set_cursor(GLCanvas3D::Standard);
    }
    else if (evt.GetEventType() == wxEVT_KEY_DOWN)
    {
        if ((m_current == SlaSupports) && ((keyCode == WXK_SHIFT) || (keyCode == WXK_ALT))
          && dynamic_cast<GLGizmoSlaSupports*>(get_current())->is_in_editing_mode())
        {
//            m_parent.set_cursor(GLCanvas3D::Cross);
            processed = true;
        }
    }

    if (processed)
        m_parent.set_as_dirty();

    return processed;
}

void GLGizmosManager::update_after_undo_redo(const UndoRedo::Snapshot& snapshot)
{
    update_data();
    m_serializing = false;
    if (m_current == SlaSupports
     && snapshot.snapshot_data.flags & UndoRedo::SnapshotData::RECALCULATE_SLA_SUPPORTS)
        dynamic_cast<GLGizmoSlaSupports*>(m_gizmos[SlaSupports].get())->reslice_SLA_supports(true);
}

void GLGizmosManager::render_background(float left, float top, float right, float bottom, float border) const
{
    unsigned int tex_id = m_background_texture.texture.get_id();
    float tex_width = (float)m_background_texture.texture.get_width();
    float tex_height = (float)m_background_texture.texture.get_height();
    if ((tex_id != 0) && (tex_width > 0) && (tex_height > 0))
    {
        float inv_tex_width = (tex_width != 0.0f) ? 1.0f / tex_width : 0.0f;
        float inv_tex_height = (tex_height != 0.0f) ? 1.0f / tex_height : 0.0f;

        float internal_left = left + border;
        float internal_right = right - border;
        float internal_top = top - border;
        float internal_bottom = bottom + border;

        // float left_uv = 0.0f;
        float right_uv = 1.0f;
        float top_uv = 1.0f;
        float bottom_uv = 0.0f;

        float internal_left_uv = (float)m_background_texture.metadata.left * inv_tex_width;
        float internal_right_uv = 1.0f - (float)m_background_texture.metadata.right * inv_tex_width;
        float internal_top_uv = 1.0f - (float)m_background_texture.metadata.top * inv_tex_height;
        float internal_bottom_uv = (float)m_background_texture.metadata.bottom * inv_tex_height;

        // top-left corner
        GLTexture::render_sub_texture(tex_id, left, internal_left, internal_top, top, { { internal_left_uv, internal_bottom_uv }, { internal_right_uv, internal_bottom_uv }, { internal_right_uv, internal_top_uv }, { internal_left_uv, internal_top_uv } });

        // top edge
        GLTexture::render_sub_texture(tex_id, internal_left, internal_right, internal_top, top, { { internal_left_uv, internal_top_uv }, { internal_right_uv, internal_top_uv }, { internal_right_uv, top_uv }, { internal_left_uv, top_uv } });

        // top-right corner
        GLTexture::render_sub_texture(tex_id, internal_right, right, internal_top, top, { { internal_right_uv, internal_top_uv }, { right_uv, internal_top_uv }, { right_uv, top_uv }, { internal_right_uv, top_uv } });

        // center-left edge
        GLTexture::render_sub_texture(tex_id, left, internal_left, internal_bottom, internal_top, { { internal_left_uv, internal_bottom_uv }, { internal_right_uv, internal_bottom_uv }, { internal_right_uv, internal_top_uv }, { internal_left_uv, internal_top_uv } });

        // center
        GLTexture::render_sub_texture(tex_id, internal_left, internal_right, internal_bottom, internal_top, { { internal_left_uv, internal_bottom_uv }, { internal_right_uv, internal_bottom_uv }, { internal_right_uv, internal_top_uv }, { internal_left_uv, internal_top_uv } });

        // center-right edge
        GLTexture::render_sub_texture(tex_id, internal_right, right, internal_bottom, internal_top, { { internal_right_uv, internal_bottom_uv }, { right_uv, internal_bottom_uv }, { right_uv, internal_top_uv }, { internal_right_uv, internal_top_uv } });

        // bottom-left corner
        GLTexture::render_sub_texture(tex_id, left, internal_left, bottom, internal_bottom, { { internal_left_uv, internal_bottom_uv }, { internal_right_uv, internal_bottom_uv }, { internal_right_uv, internal_top_uv }, { internal_left_uv, internal_top_uv } });

        // bottom edge
        GLTexture::render_sub_texture(tex_id, internal_left, internal_right, bottom, internal_bottom, { { internal_left_uv, bottom_uv }, { internal_right_uv, bottom_uv }, { internal_right_uv, internal_bottom_uv }, { internal_left_uv, internal_bottom_uv } });

        // bottom-right corner
        GLTexture::render_sub_texture(tex_id, internal_right, right, bottom, internal_bottom, { { internal_right_uv, bottom_uv }, { right_uv, bottom_uv }, { right_uv, internal_bottom_uv }, { internal_right_uv, internal_bottom_uv } });
    }
}

void GLGizmosManager::do_render_overlay() const
{
    if (m_gizmos.empty())
        return;

    float cnv_w = (float)m_parent.get_canvas_size().get_width();
    float cnv_h = (float)m_parent.get_canvas_size().get_height();
    float zoom = (float)m_parent.get_camera().get_zoom();
    float inv_zoom = (zoom != 0.0f) ? 1.0f / zoom : 0.0f;

    float height = get_scaled_total_height();
    float width = get_scaled_total_width();
    float zoomed_border = m_layout.scaled_border() * inv_zoom;

    float zoomed_top_x = (-0.5f * cnv_w) * inv_zoom;
    float zoomed_top_y = (0.5f * height) * inv_zoom;

    float zoomed_left = zoomed_top_x;
    float zoomed_top = zoomed_top_y;
    float zoomed_right = zoomed_left + width * inv_zoom;
    float zoomed_bottom = zoomed_top - height * inv_zoom;

    render_background(zoomed_left, zoomed_top, zoomed_right, zoomed_bottom, zoomed_border);

    zoomed_top_x += zoomed_border;
    zoomed_top_y -= zoomed_border;

    float icons_size = m_layout.scaled_icons_size();
    float zoomed_icons_size = icons_size * inv_zoom;
    float zoomed_stride_y = m_layout.scaled_stride_y() * inv_zoom;

    unsigned int icons_texture_id = m_icons_texture.get_id();
    int tex_width = m_icons_texture.get_width();
    int tex_height = m_icons_texture.get_height();
    float inv_tex_width = (tex_width != 0) ? 1.0f / tex_width : 0.0f;
    float inv_tex_height = (tex_height != 0) ? 1.0f / tex_height : 0.0f;

    if ((icons_texture_id == 0) || (tex_width <= 0) || (tex_height <= 0))
        return;

    for (size_t idx : get_selectable_idxs())
    {
        GLGizmoBase* gizmo = m_gizmos[idx].get();

        unsigned int sprite_id = gizmo->get_sprite_id();
#if ENABLE_GIZMO_ICONS_NON_ACTIVABLE_STATE
        int icon_idx = (m_current == idx) ? 2 : ((m_hover == idx) ? 1 : (gizmo->is_activable()? 0 : 3));
#else
        int icon_idx = m_current == idx ? 2 : (m_hover == idx ? 1 : 0);
#endif // ENABLE_GIZMO_ICONS_NON_ACTIVABLE_STATE

        float u_icon_size = icons_size * inv_tex_width;
        float v_icon_size = icons_size * inv_tex_height;

        float v_top = sprite_id * v_icon_size;
        float u_left = icon_idx * u_icon_size;
        float v_bottom = v_top + v_icon_size;
        float u_right = u_left + u_icon_size;

        GLTexture::render_sub_texture(icons_texture_id, zoomed_top_x, zoomed_top_x + zoomed_icons_size, zoomed_top_y - zoomed_icons_size, zoomed_top_y, { { u_left, v_bottom }, { u_right, v_bottom }, { u_right, v_top }, { u_left, v_top } });
        if (idx == m_current) {
            float toolbar_top = cnv_h - m_parent.get_view_toolbar_height();
            gizmo->render_input_window(width, 0.5f * cnv_h - zoomed_top_y * zoom, toolbar_top);
        }
        zoomed_top_y -= zoomed_stride_y;
    }
}

float GLGizmosManager::get_scaled_total_height() const
{
    return m_layout.scale * (2.0f * m_layout.border + (float)get_selectable_idxs().size() * m_layout.stride_y() - m_layout.gap_y);
}

float GLGizmosManager::get_scaled_total_width() const
{
    return 2.0f * m_layout.scaled_border() + m_layout.scaled_icons_size();
}

GLGizmoBase* GLGizmosManager::get_current() const
{
    return ((m_current == Undefined) || m_gizmos.empty()) ? nullptr : m_gizmos[m_current].get();
}

bool GLGizmosManager::generate_icons_texture() const
{
    std::string path = resources_dir() + "/icons/";
    std::vector<std::string> filenames;
    for (size_t idx=0; idx<m_gizmos.size(); ++idx)
    {
        if (m_gizmos[idx] != nullptr)
        {
            const std::string& icon_filename = m_gizmos[idx]->get_icon_filename();
            if (!icon_filename.empty())
                filenames.push_back(path + icon_filename);
        }
    }

    std::vector<std::pair<int, bool>> states;
    states.push_back(std::make_pair(1, false)); // Activable
    states.push_back(std::make_pair(0, false)); // Hovered
    states.push_back(std::make_pair(0, true));  // Selected
#if ENABLE_GIZMO_ICONS_NON_ACTIVABLE_STATE
    states.push_back(std::make_pair(2, false)); // Disabled
#endif // ENABLE_GIZMO_ICONS_NON_ACTIVABLE_STATE

    unsigned int sprite_size_px = (unsigned int)m_layout.scaled_icons_size();
//    // force even size
//    if (sprite_size_px % 2 != 0)
//        sprite_size_px += 1;

    bool res = m_icons_texture.load_from_svg_files_as_sprites_array(filenames, states, sprite_size_px, false);
    if (res)
        m_icons_texture_dirty = false;

    return res;
}

void GLGizmosManager::update_on_off_state(const Vec2d& mouse_pos)
{
    if (!m_enabled)
        return;

    size_t idx = get_gizmo_idx_from_mouse(mouse_pos);
    if (idx != Undefined && m_gizmos[idx]->is_activable() && m_hover == idx)
        activate_gizmo(m_current == idx ? Undefined : (EType)idx);
}

std::string GLGizmosManager::update_hover_state(const Vec2d& mouse_pos)
{
    std::string name = "";

    if (!m_enabled)
        return name;

    m_hover = Undefined;

    size_t idx = get_gizmo_idx_from_mouse(mouse_pos);
    if (idx != Undefined) {
        name = m_gizmos[idx]->get_name();

        if (m_gizmos[idx]->is_activable())
            m_hover = (EType)idx;
    }

    return name;
}

void GLGizmosManager::activate_gizmo(EType type)
{
    if (m_gizmos.empty() || m_current == type)
        return;

    if (m_current != Undefined) {
        m_gizmos[m_current]->set_state(GLGizmoBase::Off);
        if (m_gizmos[m_current]->get_state() != GLGizmoBase::Off)
            return; // gizmo refused to be turned off, do nothing.
    }

    if (type != Undefined)
        m_gizmos[type]->set_state(GLGizmoBase::On);

    m_current = type;
}


bool GLGizmosManager::grabber_contains_mouse() const
{
    if (!m_enabled)
        return false;

    GLGizmoBase* curr = get_current();
    return (curr != nullptr) ? (curr->get_hover_id() != -1) : false;
}

} // namespace GUI
} // namespace Slic3r
