#ifndef slic3r_GLGizmoFlatten_hpp_
#define slic3r_GLGizmoFlatten_hpp_

#include "GLGizmoBase.hpp"


namespace Slic3r {
namespace GUI {


class GLGizmoFlatten : public GLGizmoBase
{
// This gizmo does not use grabbers. The m_hover_id relates to polygon managed by the class itself.

private:
    mutable Vec3d m_normal;

    struct PlaneData {
        std::vector<Vec3d> vertices;
        Vec3d normal;
        float area;
    };

    // This holds information to decide whether recalculation is necessary:
    std::vector<Transform3d> m_volumes_matrices;
    std::vector<ModelVolumeType> m_volumes_types;
    Vec3d m_first_instance_scale;
    Vec3d m_first_instance_mirror;

    std::vector<PlaneData> m_planes;
    bool m_planes_valid = false;
    mutable Vec3d m_starting_center;
    const ModelObject* m_model_object = nullptr;
    ObjectID m_model_object_id = 0;
    std::vector<const Transform3d*> instances_matrices;

    void update_planes();
    bool is_plane_update_necessary() const;

public:
    GLGizmoFlatten(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);

    void set_flattening_data(const ModelObject* model_object);
    Vec3d get_flattening_normal() const;

protected:
    virtual bool on_init() override;
    virtual std::string on_get_name() const override;
    virtual bool on_is_activable() const override;
    virtual void on_start_dragging() override;
    virtual void on_render() const override;
    virtual void on_render_for_picking() const override;
    virtual void on_set_state() override;
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoFlatten_hpp_
