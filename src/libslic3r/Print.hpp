#ifndef slic3r_Print_hpp_
#define slic3r_Print_hpp_

#include "PrintBase.hpp"

#include "BoundingBox.hpp"
#include "Flow.hpp"
#include "Point.hpp"
#include "Layer.hpp"
#include "Model.hpp"
#include "Slicing.hpp"
#include "GCode/ToolOrdering.hpp"
#include "GCode/WipeTower.hpp"
#if ENABLE_THUMBNAIL_GENERATOR
#include "GCode/ThumbnailData.hpp"
#endif // ENABLE_THUMBNAIL_GENERATOR

namespace Slic3r {

class Print;
class PrintObject;
class ModelObject;
class GCode;
class GCodePreviewData;

// Print step IDs for keeping track of the print state.
enum PrintStep {
    psSkirt, psBrim, psWipeTower, psGCodeExport, psCount,
};
enum PrintObjectStep {
    posSlice, posPerimeters, posPrepareInfill,
    posInfill, posSupportMaterial, posCount,
};

// A PrintRegion object represents a group of volumes to print
// sharing the same config (including the same assigned extruder(s))
class PrintRegion
{
    friend class Print;

// Methods NOT modifying the PrintRegion's state:
public:
    const Print*                print() const { return m_print; }
    const PrintRegionConfig&    config() const { return m_config; }
	// 1-based extruder identifier for this region and role.
	unsigned int 				extruder(FlowRole role) const;
    Flow                        flow(FlowRole role, double layer_height, bool bridge, bool first_layer, double width, const PrintObject &object) const;
    // Average diameter of nozzles participating on extruding this region.
    coordf_t                    nozzle_dmr_avg(const PrintConfig &print_config) const;
    // Average diameter of nozzles participating on extruding this region.
    coordf_t                    bridging_height_avg(const PrintConfig &print_config) const;

    // Collect extruder indices used to print this region's object.
	void                        collect_object_printing_extruders(std::vector<unsigned int> &object_extruders) const;
	static void                 collect_object_printing_extruders(const PrintConfig &print_config, const PrintRegionConfig &region_config, std::vector<unsigned int> &object_extruders);

// Methods modifying the PrintRegion's state:
public:
    Print*                      print() { return m_print; }
    void                        set_config(const PrintRegionConfig &config) { m_config = config; }
    void                        set_config(PrintRegionConfig &&config) { m_config = std::move(config); }
    void                        config_apply_only(const ConfigBase &other, const t_config_option_keys &keys, bool ignore_nonexistent = false) 
                                        { this->m_config.apply_only(other, keys, ignore_nonexistent); }

protected:
    size_t             m_refcnt;

private:
    Print             *m_print;
    PrintRegionConfig  m_config;
    
    PrintRegion(Print* print) : m_refcnt(0), m_print(print) {}
    PrintRegion(Print* print, const PrintRegionConfig &config) : m_refcnt(0), m_print(print), m_config(config) {}
    ~PrintRegion() {}
};


typedef std::vector<Layer*> LayerPtrs;
typedef std::vector<SupportLayer*> SupportLayerPtrs;
class BoundingBoxf3;        // TODO: for temporary constructor parameter

class PrintObject : public PrintObjectBaseWithState<Print, PrintObjectStep, posCount>
{
private: // Prevents erroneous use by other classes.
    typedef PrintObjectBaseWithState<Print, PrintObjectStep, posCount> Inherited;

public:
    // vector of (layer height ranges and vectors of volume ids), indexed by region_id
    std::vector<std::vector<std::pair<t_layer_height_range, int>>> region_volumes;

    // this is set to true when LayerRegion->slices is split in top/internal/bottom
    // so that next call to make_perimeters() performs a union() before computing loops
    bool                    typed_slices;

    Vec3crd                 size;           // XYZ in scaled coordinates

    const PrintObjectConfig& config() const         { return m_config; }    
    const LayerPtrs&        layers() const          { return m_layers; }
    const SupportLayerPtrs& support_layers() const  { return m_support_layers; }
    const Transform3d&      trafo() const           { return m_trafo; }
    const Points&           copies() const          { return m_copies; }
    const Point 			copy_center(size_t idx) const { return m_copies[idx] + m_copies_shift + Point(this->size.x() / 2, this->size.y() / 2); }

    // since the object is aligned to origin, bounding box coincides with size
    BoundingBox bounding_box() const { return BoundingBox(Point(0,0), to_2d(this->size)); }

    // adds region_id, too, if necessary
    void add_region_volume(unsigned int region_id, int volume_id, const t_layer_height_range &layer_range) {
        if (region_id >= region_volumes.size())
			region_volumes.resize(region_id + 1);
        region_volumes[region_id].emplace_back(layer_range, volume_id);
    }
    // This is the *total* layer count (including support layers)
    // this value is not supposed to be compared with Layer::id
    // since they have different semantics.
    size_t total_layer_count() const { return this->layer_count() + this->support_layer_count(); }
    size_t layer_count() const { return m_layers.size(); }
    void clear_layers();
    Layer* get_layer(int idx) { return m_layers[idx]; }
    const Layer* get_layer(int idx) const { return m_layers[idx]; }

    // print_z: top of the layer; slice_z: center of the layer.
    Layer* add_layer(int id, coordf_t height, coordf_t print_z, coordf_t slice_z);

    size_t support_layer_count() const { return m_support_layers.size(); }
    void clear_support_layers();
    SupportLayer* get_support_layer(int idx) { return m_support_layers[idx]; }
    SupportLayer* add_support_layer(int id, coordf_t height, coordf_t print_z);
    SupportLayerPtrs::const_iterator insert_support_layer(SupportLayerPtrs::const_iterator pos, size_t id, coordf_t height, coordf_t print_z, coordf_t slice_z);
    void delete_support_layer(int idx);
    
    // Initialize the layer_height_profile from the model_object's layer_height_profile, from model_object's layer height table, or from slicing parameters.
    // Returns true, if the layer_height_profile was changed.
    static bool update_layer_height_profile(const ModelObject &model_object, const SlicingParameters &slicing_parameters, std::vector<coordf_t> &layer_height_profile);

    // Collect the slicing parameters, to be used by variable layer thickness algorithm,
    // by the interactive layer height editor and by the printing process itself.
    // The slicing parameters are dependent on various configuration values
    // (layer height, first layer height, raft settings, print nozzle diameter etc).
    const SlicingParameters&    slicing_parameters() const { return m_slicing_params; }
    static SlicingParameters    slicing_parameters(const DynamicPrintConfig &full_config, const ModelObject &model_object, float object_max_z);

    // returns 0-based indices of extruders used to print the object (without brim, support and other helper extrusions)
    std::vector<unsigned int>   object_extruders() const;

    // Called when slicing to SVG (see Print.pm sub export_svg), and used by perimeters.t
    void slice();

    // Helpers to slice support enforcer / blocker meshes by the support generator.
    std::vector<ExPolygons>     slice_support_volumes(const ModelVolumeType &model_volume_type) const;
    std::vector<ExPolygons>     slice_support_blockers() const { return this->slice_support_volumes(ModelVolumeType::SUPPORT_BLOCKER); }
    std::vector<ExPolygons>     slice_support_enforcers() const { return this->slice_support_volumes(ModelVolumeType::SUPPORT_ENFORCER); }

protected:
    // to be called from Print only.
    friend class Print;

	PrintObject(Print* print, ModelObject* model_object, bool add_instances = true);
	~PrintObject() {}

    void                    config_apply(const ConfigBase &other, bool ignore_nonexistent = false) { this->m_config.apply(other, ignore_nonexistent); }
    void                    config_apply_only(const ConfigBase &other, const t_config_option_keys &keys, bool ignore_nonexistent = false) { this->m_config.apply_only(other, keys, ignore_nonexistent); }
    void                    set_trafo(const Transform3d& trafo) { m_trafo = trafo; }
    PrintBase::ApplyStatus  set_copies(const Points &points);
    // Invalidates the step, and its depending steps in PrintObject and Print.
    bool                    invalidate_step(PrintObjectStep step);
    // Invalidates all PrintObject and Print steps.
    bool                    invalidate_all_steps();
    // Invalidate steps based on a set of parameters changed.
    bool                    invalidate_state_by_config_options(const std::vector<t_config_option_key> &opt_keys);
    // If ! m_slicing_params.valid, recalculate.
    void                    update_slicing_parameters();

    static PrintObjectConfig object_config_from_model_object(const PrintObjectConfig &default_object_config, const ModelObject &object, size_t num_extruders);
    static PrintRegionConfig region_config_from_model_volume(const PrintRegionConfig &default_region_config, const DynamicPrintConfig *layer_range_config, const ModelVolume &volume, size_t num_extruders);

private:
    void make_perimeters();
    void prepare_infill();
    void infill();
    void generate_support_material();

    void _slice(const std::vector<coordf_t> &layer_height_profile);
    std::string _fix_slicing_errors();
    void _simplify_slices(double distance);
    bool has_support_material() const;
    void detect_surfaces_type();
    void process_external_surfaces();
    void discover_vertical_shells();
    void bridge_over_infill();
    void clip_fill_surfaces();
    void discover_horizontal_shells();
    void combine_infill();
    void _generate_support_material();

    PrintObjectConfig                       m_config;
    // Translation in Z + Rotation + Scaling / Mirroring.
    Transform3d                             m_trafo = Transform3d::Identity();
    // Slic3r::Point objects in scaled G-code coordinates
    Points                                  m_copies;
    // scaled coordinates to add to copies (to compensate for the alignment
    // operated when creating the object but still preserving a coherent API
    // for external callers)
    Point                                   m_copies_shift;

    SlicingParameters                       m_slicing_params;
    LayerPtrs                               m_layers;
    SupportLayerPtrs                        m_support_layers;

    std::vector<ExPolygons> slice_region(size_t region_id, const std::vector<float> &z) const;
    std::vector<ExPolygons> slice_modifiers(size_t region_id, const std::vector<float> &z) const;
    std::vector<ExPolygons> slice_volumes(const std::vector<float> &z, const std::vector<const ModelVolume*> &volumes) const;
    std::vector<ExPolygons> slice_volume(const std::vector<float> &z, const ModelVolume &volume) const;
    std::vector<ExPolygons> slice_volume(const std::vector<float> &z, const std::vector<t_layer_height_range> &ranges, const ModelVolume &volume) const;
};

struct WipeTowerData
{
    // Following section will be consumed by the GCodeGenerator.
    // Tool ordering of a non-sequential print has to be known to calculate the wipe tower.
    // Cache it here, so it does not need to be recalculated during the G-code generation.
    ToolOrdering                                          tool_ordering;
    // Cache of tool changes per print layer.
    std::unique_ptr<std::vector<WipeTower::ToolChangeResult>> priming;
    std::vector<std::vector<WipeTower::ToolChangeResult>> tool_changes;
    std::unique_ptr<WipeTower::ToolChangeResult>          final_purge;
    std::vector<float>                                    used_filament;
    int                                                   number_of_toolchanges;

    // Depth of the wipe tower to pass to GLCanvas3D for exact bounding box:
    float                                                 depth;
    float                                                 brim_width;

    void clear() {
        tool_ordering.clear();
        priming.reset(nullptr);
        tool_changes.clear();
        final_purge.reset(nullptr);
        used_filament.clear();
        number_of_toolchanges = -1;
        depth = 0.f;
        brim_width = 0.f;
    }
};

struct PrintStatistics
{
    PrintStatistics() { clear(); }
    std::string                     estimated_normal_print_time;
    std::string                     estimated_silent_print_time;
    std::vector<std::string>        estimated_normal_color_print_times;
    std::vector<std::string>        estimated_silent_color_print_times;
    double                          total_used_filament;
    double                          total_extruded_volume;
    double                          total_cost;
    int                             total_toolchanges;
    double                          total_weight;
    double                          total_wipe_tower_cost;
    double                          total_wipe_tower_filament;
    std::map<size_t, float>         filament_stats;

    // Config with the filled in print statistics.
    DynamicConfig           config() const;
    // Config with the statistics keys populated with placeholder strings.
    static DynamicConfig    placeholders();
    // Replace the print statistics placeholders in the path.
    std::string             finalize_output_path(const std::string &path_in) const;

    void clear() {
        estimated_normal_print_time.clear();
        estimated_silent_print_time.clear();
        estimated_normal_color_print_times.clear();
        estimated_silent_color_print_times.clear();
        total_used_filament    = 0.;
        total_extruded_volume  = 0.;
        total_cost             = 0.;
        total_toolchanges      = 0;
        total_weight           = 0.;
        total_wipe_tower_cost  = 0.;
        total_wipe_tower_filament = 0.;
        filament_stats.clear();
    }
};

typedef std::vector<PrintObject*> PrintObjectPtrs;
typedef std::vector<PrintRegion*> PrintRegionPtrs;

// The complete print tray with possibly multiple objects.
class Print : public PrintBaseWithState<PrintStep, psCount>
{
private: // Prevents erroneous use by other classes.
    typedef PrintBaseWithState<PrintStep, psCount> Inherited;

public:
    Print() {}
	virtual ~Print() { this->clear(); }

	PrinterTechnology	technology() const noexcept { return ptFFF; }

    // Methods, which change the state of Print / PrintObject / PrintRegion.
    // The following methods are synchronized with process() and export_gcode(),
    // so that process() and export_gcode() may be called from a background thread.
    // In case the following methods need to modify data processed by process() or export_gcode(),
    // a cancellation callback is executed to stop the background processing before the operation.
    void                clear() override;
    bool                empty() const override { return m_objects.empty(); }

    ApplyStatus         apply(const Model &model, DynamicPrintConfig config) override;

    void                process() override;
    // Exports G-code into a file name based on the path_template, returns the file path of the generated G-code file.
    // If preview_data is not null, the preview_data is filled in for the G-code visualization (not used by the command line Slic3r).
#if ENABLE_THUMBNAIL_GENERATOR
    std::string         export_gcode(const std::string& path_template, GCodePreviewData* preview_data, ThumbnailsGeneratorCallback thumbnail_cb = nullptr);
#else
    std::string         export_gcode(const std::string &path_template, GCodePreviewData *preview_data);
#endif // ENABLE_THUMBNAIL_GENERATOR

    // methods for handling state
    bool                is_step_done(PrintStep step) const { return Inherited::is_step_done(step); }
    // Returns true if an object step is done on all objects and there's at least one object.    
    bool                is_step_done(PrintObjectStep step) const;
    // Returns true if the last step was finished with success.
    bool                finished() const override { return this->is_step_done(psGCodeExport); }

    bool                has_infinite_skirt() const;
    bool                has_skirt() const;

    // Returns an empty string if valid, otherwise returns an error message.
    std::string         validate() const override;
    BoundingBox         bounding_box() const;
    BoundingBox         total_bounding_box() const;
    double              skirt_first_layer_height() const;
    Flow                brim_flow() const;
    Flow                skirt_flow() const;
    
    std::vector<unsigned int> object_extruders() const;
    std::vector<unsigned int> support_material_extruders() const;
    std::vector<unsigned int> extruders() const;
    double              max_allowed_layer_height() const;
    bool                has_support_material() const;
    // Make sure the background processing has no access to this model_object during this call!
    void                auto_assign_extruders(ModelObject* model_object) const;

    const PrintConfig&          config() const { return m_config; }
    const PrintObjectConfig&    default_object_config() const { return m_default_object_config; }
    const PrintRegionConfig&    default_region_config() const { return m_default_region_config; }
    const PrintObjectPtrs&      objects() const { return m_objects; }
    PrintObject*                get_object(size_t idx) { return m_objects[idx]; }
    const PrintObject*          get_object(size_t idx) const { return m_objects[idx]; }
    const PrintRegionPtrs&      regions() const { return m_regions; }
    // How many of PrintObject::copies() over all print objects are there?
    // If zero, then the print is empty and the print shall not be executed.
    unsigned int                num_object_instances() const;

    // Returns extruder this eec should be printed with, according to PrintRegion config:
    static int                  get_extruder(const ExtrusionEntityCollection& fill, const PrintRegion &region);

    const ExtrusionEntityCollection& skirt() const { return m_skirt; }
    const ExtrusionEntityCollection& brim() const { return m_brim; }

    const PrintStatistics&      print_statistics() const { return m_print_statistics; }

    // Wipe tower support.
    bool                        has_wipe_tower() const;
    const WipeTowerData&        wipe_tower_data(size_t extruders_cnt = 0, double first_layer_height = 0., double nozzle_diameter = 0.) const;

	std::string                 output_filename(const std::string &filename_base = std::string()) const override;

    // Accessed by SupportMaterial
    const PrintRegion*  get_region(size_t idx) const  { return m_regions[idx]; }

    // force update of PrintRegions, when custom_tool_change is not empty and (Re)Slicing is started
    void set_force_update_print_regions(bool force_update_print_regions) { m_force_update_print_regions = force_update_print_regions; }

protected:
    // methods for handling regions
    PrintRegion*        get_region(size_t idx)        { return m_regions[idx]; }
    PrintRegion*        add_region();
    PrintRegion*        add_region(const PrintRegionConfig &config);

    // Invalidates the step, and its depending steps in Print.
    bool                invalidate_step(PrintStep step);

private:
	void 				config_diffs(
		const DynamicPrintConfig &new_full_config, 
		t_config_option_keys &print_diff, t_config_option_keys &object_diff, t_config_option_keys &region_diff, 
		t_config_option_keys &full_config_diff, 
		DynamicPrintConfig &placeholder_parser_overrides,
		DynamicPrintConfig &filament_overrides) const;

    bool                invalidate_state_by_config_options(const std::vector<t_config_option_key> &opt_keys);

    void                _make_skirt();
    void                _make_brim();
    void                _make_wipe_tower();

    // Declared here to have access to Model / ModelObject / ModelInstance
    static void         model_volume_list_update_supports(ModelObject &model_object_dst, const ModelObject &model_object_src);

    PrintConfig                             m_config;
    PrintObjectConfig                       m_default_object_config;
    PrintRegionConfig                       m_default_region_config;
    PrintObjectPtrs                         m_objects;
    PrintRegionPtrs                         m_regions;

    // Ordered collections of extrusion paths to build skirt loops and brim.
    ExtrusionEntityCollection               m_skirt;
    ExtrusionEntityCollection               m_brim;

    // Following section will be consumed by the GCodeGenerator.
    WipeTowerData                           m_wipe_tower_data;

    // Estimated print time, filament consumed.
    PrintStatistics                         m_print_statistics;

    // flag used
    bool                                    m_force_update_print_regions = false;

    // To allow GCode to set the Print's GCodeExport step status.
    friend class GCode;
    // Allow PrintObject to access m_mutex and m_cancel_callback.
    friend class PrintObject;
};

} /* slic3r_Print_hpp_ */

#endif
