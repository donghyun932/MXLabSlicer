#include <cassert>

#include "PresetBundle.hpp"
#include "BitmapCache.hpp"
#include "Plater.hpp"
#include "I18N.hpp"
#include "wxExtensions.hpp"

#include <algorithm>
#include <fstream>
#include <unordered_set>
#include <boost/filesystem.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <boost/nowide/cenv.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/locale.hpp>
#include <boost/log/trivial.hpp>

#include <wx/dcmemory.h>
#include <wx/image.h>
#include <wx/choice.h>
#include <wx/bmpcbox.h>
#include <wx/wupdlock.h>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Utils.hpp"
#include "GUI_App.hpp"

// Store the print/filament/printer presets into a "presets" subdirectory of the Slic3rPE config dir.
// This breaks compatibility with the upstream Slic3r if the --datadir is used to switch between the two versions.
// #define SLIC3R_PROFILE_USE_PRESETS_SUBDIR

namespace Slic3r {

static std::vector<std::string> s_project_options {
    "colorprint_heights",
    "wiping_volumes_extruders",
    "wiping_volumes_matrix"
};

const char *PresetBundle::PRUSA_BUNDLE = "PrusaResearch";

PresetBundle::PresetBundle() :
    prints(Preset::TYPE_PRINT, Preset::print_options(), static_cast<const HostConfig&>(FullPrintConfig::defaults())), 
    filaments(Preset::TYPE_FILAMENT, Preset::filament_options(), static_cast<const HostConfig&>(FullPrintConfig::defaults())), 
    sla_materials(Preset::TYPE_SLA_MATERIAL, Preset::sla_material_options(), static_cast<const SLAMaterialConfig&>(SLAFullPrintConfig::defaults())), 
    sla_prints(Preset::TYPE_SLA_PRINT, Preset::sla_print_options(), static_cast<const SLAPrintObjectConfig&>(SLAFullPrintConfig::defaults())),
    printers(Preset::TYPE_PRINTER, Preset::printer_options(), static_cast<const HostConfig&>(FullPrintConfig::defaults()), "- default FFF -"),
    m_bitmapCompatible(new wxBitmap),
    m_bitmapIncompatible(new wxBitmap),
    m_bitmapLock(new wxBitmap),
    m_bitmapLockOpen(new wxBitmap),
    m_bitmapCache(new GUI::BitmapCache)
{
    if (wxImage::FindHandler(wxBITMAP_TYPE_PNG) == nullptr)
        wxImage::AddHandler(new wxPNGHandler);

    // The following keys are handled by the UI, they do not have a counterpart in any StaticPrintConfig derived classes,
    // therefore they need to be handled differently. As they have no counterpart in StaticPrintConfig, they are not being
    // initialized based on PrintConfigDef(), but to empty values (zeros, empty vectors, empty strings).
    //
    // "compatible_printers", "compatible_printers_condition", "inherits",
    // "print_settings_id", "filament_settings_id", "printer_settings_id",
    // "printer_vendor", "printer_model", "printer_variant", "default_print_profile", "default_filament_profile"

    // Create the ID config keys, as they are not part of the Static print config classes.
    this->prints.default_preset().config.optptr("print_settings_id", true);
    this->prints.default_preset().compatible_printers_condition();
    this->prints.default_preset().inherits();

    this->filaments.default_preset().config.option<ConfigOptionStrings>("filament_settings_id", true)->values = { "" };
    this->filaments.default_preset().compatible_printers_condition();
    this->filaments.default_preset().inherits();
	// Set all the nullable values to nils.
	this->filaments.default_preset().config.null_nullables();

    this->sla_materials.default_preset().config.optptr("sla_material_settings_id", true);
    this->sla_materials.default_preset().compatible_printers_condition();
    this->sla_materials.default_preset().inherits();

    this->sla_prints.default_preset().config.optptr("sla_print_settings_id", true);
    this->sla_prints.default_preset().config.opt_string("output_filename_format", true) = "[input_filename_base].sl1";
    this->sla_prints.default_preset().compatible_printers_condition();
    this->sla_prints.default_preset().inherits();

    this->printers.add_default_preset(Preset::sla_printer_options(), static_cast<const SLAMaterialConfig&>(SLAFullPrintConfig::defaults()), "- default SLA -");
    this->printers.preset(1).printer_technology_ref() = ptSLA;
    for (size_t i = 0; i < 2; ++ i) {
		// The following ugly switch is to avoid printers.preset(0) to return the edited instance, as the 0th default is the current one.
		Preset &preset = this->printers.default_preset(i);
        preset.config.optptr("printer_settings_id", true);
        preset.config.optptr("printer_vendor", true);
        preset.config.optptr("printer_model", true);
        preset.config.optptr("printer_variant", true);
		preset.config.optptr("thumbnails", true);
        if (i == 0) {
            preset.config.optptr("default_print_profile", true);
            preset.config.option<ConfigOptionStrings>("default_filament_profile", true)->values = { "" };
        }
        else {
            preset.config.optptr("default_sla_print_profile", true);
            preset.config.optptr("default_sla_material_profile", true);
        }
        // default_sla_material_profile
        preset.inherits();
    }

	// Load the default preset bitmaps.
	// #ys_FIXME_to_delete we'll load them later, using em_unit()
//     this->prints       .load_bitmap_default("cog");
//     this->sla_prints   .load_bitmap_default("package_green.png");
//     this->filaments    .load_bitmap_default("spool.png");
//     this->sla_materials.load_bitmap_default("package_green.png");
//     this->printers     .load_bitmap_default("printer_empty.png");
//     this->printers     .load_bitmap_add("add.png");
//     this->load_compatible_bitmaps();

    // Re-activate the default presets, so their "edited" preset copies will be updated with the additional configuration values above.
    this->prints       .select_preset(0);
    this->sla_prints   .select_preset(0);
    this->filaments    .select_preset(0);
    this->sla_materials.select_preset(0);
    this->printers     .select_preset(0);

    this->project_config.apply_only(FullPrintConfig::defaults(), s_project_options);
}

PresetBundle::~PresetBundle()
{
	assert(m_bitmapCompatible != nullptr);
	assert(m_bitmapIncompatible != nullptr);
    assert(m_bitmapLock != nullptr);
    assert(m_bitmapLockOpen != nullptr);
	delete m_bitmapCompatible;
	m_bitmapCompatible = nullptr;
    delete m_bitmapIncompatible;
	m_bitmapIncompatible = nullptr;
    delete m_bitmapLock;
    m_bitmapLock = nullptr;
    delete m_bitmapLockOpen;
    m_bitmapLockOpen = nullptr;
    delete m_bitmapCache;
    m_bitmapCache = nullptr;
}

void PresetBundle::reset(bool delete_files)
{
    // Clear the existing presets, delete their respective files.
    this->vendors.clear();
    this->prints       .reset(delete_files);
    this->sla_prints   .reset(delete_files);
    this->filaments    .reset(delete_files);
    this->sla_materials.reset(delete_files);
    this->printers     .reset(delete_files);
    this->filament_presets.clear();
    this->filament_presets.emplace_back(this->filaments.get_selected_preset_name());
    this->obsolete_presets.prints.clear();
    this->obsolete_presets.sla_prints.clear();
    this->obsolete_presets.filaments.clear();
    this->obsolete_presets.sla_materials.clear();
    this->obsolete_presets.printers.clear();
}

void PresetBundle::setup_directories()
{
    boost::filesystem::path data_dir = boost::filesystem::path(Slic3r::data_dir());
    std::initializer_list<boost::filesystem::path> paths = { 
        data_dir,
		data_dir / "vendor",
        data_dir / "cache",
#ifdef SLIC3R_PROFILE_USE_PRESETS_SUBDIR
        // Store the print/filament/printer presets into a "presets" directory.
        data_dir / "presets", 
        data_dir / "presets" / "print", 
        data_dir / "presets" / "filament", 
        data_dir / "presets" / "sla_print",  
        data_dir / "presets" / "sla_material", 
        data_dir / "presets" / "printer" 
#else
        // Store the print/filament/printer presets at the same location as the upstream Slic3r.
        data_dir / "print", 
        data_dir / "filament", 
        data_dir / "sla_print", 
        data_dir / "sla_material", 
        data_dir / "printer" 
#endif
    };
    for (const boost::filesystem::path &path : paths) {
		boost::filesystem::path subdir = path;
        subdir.make_preferred();
        if (! boost::filesystem::is_directory(subdir) && 
            ! boost::filesystem::create_directory(subdir))
            throw std::runtime_error(std::string("Slic3r was unable to create its data directory at ") + subdir.string());
    }
}

void PresetBundle::load_presets(AppConfig &config, const std::string &preferred_model_id)
{
    // First load the vendor specific system presets.
    std::string errors_cummulative = this->load_system_presets();

    const std::string dir_user_presets = data_dir()
#ifdef SLIC3R_PROFILE_USE_PRESETS_SUBDIR
        // Store the print/filament/printer presets into a "presets" directory.
        + "/presets"
#else
        // Store the print/filament/printer presets at the same location as the upstream Slic3r.
#endif
        ;
    try {
        this->prints.load_presets(dir_user_presets, "print");
    } catch (const std::runtime_error &err) {
        errors_cummulative += err.what();
    }
    try {
        this->sla_prints.load_presets(dir_user_presets, "sla_print");
    } catch (const std::runtime_error &err) {
        errors_cummulative += err.what();
    }
    try {
        this->filaments.load_presets(dir_user_presets, "filament");
    } catch (const std::runtime_error &err) {
        errors_cummulative += err.what();
    }
    try {
        this->sla_materials.load_presets(dir_user_presets, "sla_material");
    } catch (const std::runtime_error &err) {
        errors_cummulative += err.what();
    }
    try {
        this->printers.load_presets(dir_user_presets, "printer");
    } catch (const std::runtime_error &err) {
        errors_cummulative += err.what();
    }
    this->update_multi_material_filament_presets();
    this->update_compatible(false);
    if (! errors_cummulative.empty())
        throw std::runtime_error(errors_cummulative);

    this->load_selections(config, preferred_model_id);
}

// Load system presets into this PresetBundle.
// For each vendor, there will be a single PresetBundle loaded.
std::string PresetBundle::load_system_presets()
{
    // Here the vendor specific read only Config Bundles are stored.
    boost::filesystem::path dir = (boost::filesystem::path(data_dir()) / "vendor").make_preferred();
    std::string errors_cummulative;
    bool        first = true;
    for (auto &dir_entry : boost::filesystem::directory_iterator(dir))
        if (Slic3r::is_ini_file(dir_entry)) {
            std::string name = dir_entry.path().filename().string();
            // Remove the .ini suffix.
            name.erase(name.size() - 4);
            try {
                // Load the config bundle, flatten it.
                if (first) {
                    // Reset this PresetBundle and load the first vendor config.
                    this->load_configbundle(dir_entry.path().string(), LOAD_CFGBNDLE_SYSTEM);
                    first = false;
                } else {
                    // Load the other vendor configs, merge them with this PresetBundle.
                    // Report duplicate profiles.
                    PresetBundle other;
                    other.load_configbundle(dir_entry.path().string(), LOAD_CFGBNDLE_SYSTEM);
                    std::vector<std::string> duplicates = this->merge_presets(std::move(other));
                    if (! duplicates.empty()) {
                        errors_cummulative += "Vendor configuration file " + name + " contains the following presets with names used by other vendors: ";
                        for (size_t i = 0; i < duplicates.size(); ++ i) {
                            if (i > 0)
                                errors_cummulative += ", ";
                            errors_cummulative += duplicates[i];
                        }
                    }
                }
            } catch (const std::runtime_error &err) {
                errors_cummulative += err.what();
                errors_cummulative += "\n";
            }
        }
    if (first) {
		// No config bundle loaded, reset.
		this->reset(false);
	}
    return errors_cummulative;
}

// Merge one vendor's presets with the other vendor's presets, report duplicates.
std::vector<std::string> PresetBundle::merge_presets(PresetBundle &&other)
{
    this->vendors.insert(other.vendors.begin(), other.vendors.end());
    std::vector<std::string> duplicate_prints        = this->prints       .merge_presets(std::move(other.prints),        this->vendors);
    std::vector<std::string> duplicate_sla_prints    = this->sla_prints   .merge_presets(std::move(other.sla_prints),    this->vendors);
    std::vector<std::string> duplicate_filaments     = this->filaments    .merge_presets(std::move(other.filaments),     this->vendors);
    std::vector<std::string> duplicate_sla_materials = this->sla_materials.merge_presets(std::move(other.sla_materials), this->vendors);
    std::vector<std::string> duplicate_printers      = this->printers     .merge_presets(std::move(other.printers),      this->vendors);
	append(this->obsolete_presets.prints,        std::move(other.obsolete_presets.prints));
	append(this->obsolete_presets.sla_prints,    std::move(other.obsolete_presets.sla_prints));
	append(this->obsolete_presets.filaments,     std::move(other.obsolete_presets.filaments));
    append(this->obsolete_presets.sla_materials, std::move(other.obsolete_presets.sla_materials));
	append(this->obsolete_presets.printers,      std::move(other.obsolete_presets.printers));
	append(duplicate_prints, std::move(duplicate_sla_prints));
	append(duplicate_prints, std::move(duplicate_filaments));
    append(duplicate_prints, std::move(duplicate_sla_materials));
    append(duplicate_prints, std::move(duplicate_printers));
    return duplicate_prints;
}

static inline std::string remove_ini_suffix(const std::string &name)
{
    std::string out = name;
    if (boost::iends_with(out, ".ini"))
        out.erase(out.end() - 4, out.end());
    return out;
}

// Set the "enabled" flag for printer vendors, printer models and printer variants
// based on the user configuration.
// If the "vendor" section is missing, enable all models and variants of the particular vendor.
void PresetBundle::load_installed_printers(const AppConfig &config)
{
    for (auto &preset : printers) {
        preset.set_visible_from_appconfig(config);
    }
}

const std::string& PresetBundle::get_preset_name_by_alias( const Preset::Type& preset_type, const std::string& alias) const
{
    // there are not aliases for Printers profiles
    if (preset_type == Preset::TYPE_PRINTER || preset_type == Preset::TYPE_INVALID)
        return alias;

    const PresetCollection& presets = preset_type == Preset::TYPE_PRINT     ? prints :
                                      preset_type == Preset::TYPE_SLA_PRINT ? sla_prints :
                                      preset_type == Preset::TYPE_FILAMENT  ? filaments :
                                      sla_materials;

    return presets.get_preset_name_by_alias(alias);
}

void PresetBundle::load_installed_filaments(AppConfig &config)
{
    if (! config.has_section(AppConfig::SECTION_FILAMENTS)) {
		// Compatibility with the PrusaSlicer 2.1.1 and older, where the filament profiles were not installable yet.
		// Find all filament profiles, which are compatible with installed printers, and act as if these filament profiles
		// were installed.
        std::unordered_set<const Preset*> compatible_filaments;
        for (const Preset &printer : printers)
            if (printer.is_visible && printer.printer_technology() == ptFFF) {
				const PresetWithVendorProfile printer_with_vendor_profile = printers.get_preset_with_vendor_profile(printer);
				for (const Preset &filament : filaments)
					if (is_compatible_with_printer(filaments.get_preset_with_vendor_profile(filament), printer_with_vendor_profile))
						compatible_filaments.insert(&filament);
			}
		// and mark these filaments as installed, therefore this code will not be executed at the next start of the application.
        for (const auto &filament: compatible_filaments)
            config.set(AppConfig::SECTION_FILAMENTS, filament->name, "1");
    }

    for (auto &preset : filaments)
        preset.set_visible_from_appconfig(config);
}

void PresetBundle::load_installed_sla_materials(AppConfig &config)
{
    if (! config.has_section(AppConfig::SECTION_MATERIALS)) {
        std::unordered_set<const Preset*> comp_sla_materials;
		// Compatibility with the PrusaSlicer 2.1.1 and older, where the SLA material profiles were not installable yet.
		// Find all SLA material profiles, which are compatible with installed printers, and act as if these SLA material profiles
		// were installed.
        for (const Preset &printer : printers)
            if (printer.is_visible && printer.printer_technology() == ptSLA) {
				const PresetWithVendorProfile printer_with_vendor_profile = printers.get_preset_with_vendor_profile(printer);
				for (const Preset &material : sla_materials)
					if (is_compatible_with_printer(sla_materials.get_preset_with_vendor_profile(material), printer_with_vendor_profile))
						comp_sla_materials.insert(&material);
			}
		// and mark these SLA materials as installed, therefore this code will not be executed at the next start of the application.
		for (const auto &material: comp_sla_materials)
            config.set(AppConfig::SECTION_MATERIALS, material->name, "1");
    }

    for (auto &preset : sla_materials)
        preset.set_visible_from_appconfig(config);
}

// Load selections (current print, current filaments, current printer) from config.ini
// This is done on application start up or after updates are applied.
void PresetBundle::load_selections(AppConfig &config, const std::string &preferred_model_id)
{
	// Update visibility of presets based on application vendor / model / variant configuration.
	this->load_installed_printers(config);

    // Update visibility of filament and sla material presets
    this->load_installed_filaments(config);
    this->load_installed_sla_materials(config);

    // Parse the initial print / filament / printer profile names.
    std::string initial_print_profile_name        = remove_ini_suffix(config.get("presets", "print"));
    std::string initial_sla_print_profile_name    = remove_ini_suffix(config.get("presets", "sla_print"));
    std::string initial_filament_profile_name     = remove_ini_suffix(config.get("presets", "filament"));
    std::string initial_sla_material_profile_name = remove_ini_suffix(config.get("presets", "sla_material"));
	std::string initial_printer_profile_name      = remove_ini_suffix(config.get("presets", "printer"));

    // Activate print / filament / printer profiles from either the config,
    // or from the preferred_model_id suggestion passed in by ConfigWizard.
    // If the printer profile enumerated by the config are not visible, select an alternate preset.
    // Do not select alternate profiles for the print / filament profiles as those presets
    // will be selected by the following call of this->update_compatible(true).

    const Preset *initial_printer = printers.find_preset(initial_printer_profile_name);
    const Preset *preferred_printer = printers.find_by_model_id(preferred_model_id);
    printers.select_preset_by_name(
        (preferred_printer != nullptr && (initial_printer == nullptr || !initial_printer->is_visible)) ? 
            preferred_printer->name : 
            initial_printer_profile_name,
        true);

    // Selects the profile, leaves it to -1 if the initial profile name is empty or if it was not found.
    prints.select_preset_by_name_strict(initial_print_profile_name);
    filaments.select_preset_by_name_strict(initial_filament_profile_name);
	sla_prints.select_preset_by_name_strict(initial_sla_print_profile_name);
    sla_materials.select_preset_by_name_strict(initial_sla_material_profile_name);

    // Load the names of the other filament profiles selected for a multi-material printer.
    // Load it even if the current printer technology is SLA.
    // The possibly excessive filament names will be later removed with this->update_multi_material_filament_presets()
    // once the FFF technology gets selected.
    this->filament_presets = { filaments.get_selected_preset_name() };
    for (unsigned int i = 1; i < 1000; ++ i) {
        char name[64];
        sprintf(name, "filament_%u", i);
        if (! config.has("presets", name))
            break;
        this->filament_presets.emplace_back(remove_ini_suffix(config.get("presets", name)));
    }

    // Update visibility of presets based on their compatibility with the active printer.
    // Always try to select a compatible print and filament preset to the current printer preset,
    // as the application may have been closed with an active "external" preset, which does not
    // exist.
    this->update_compatible(true);
    this->update_multi_material_filament_presets();
}

// Export selections (current print, current filaments, current printer) into config.ini
void PresetBundle::export_selections(AppConfig &config)
{
	assert(this->printers.get_edited_preset().printer_technology() != ptFFF || filament_presets.size() >= 1);
	assert(this->printers.get_edited_preset().printer_technology() != ptFFF || filament_presets.size() > 1 || filaments.get_selected_preset_name() == filament_presets.front());
    config.clear_section("presets");
    config.set("presets", "print",        prints.get_selected_preset_name());
    config.set("presets", "filament",     filament_presets.front());
    for (unsigned i = 1; i < filament_presets.size(); ++i) {
        char name[64];
        sprintf(name, "filament_%u", i);
        config.set("presets", name, filament_presets[i]);
    }

    config.set("presets", "sla_print",    sla_prints.get_selected_preset_name());
    config.set("presets", "sla_material", sla_materials.get_selected_preset_name());
    config.set("presets", "printer",      printers.get_selected_preset_name());
}

void PresetBundle::load_compatible_bitmaps(wxWindow *window)
{
    // We don't actually pass the window pointer here and instead generate
    // a low DPI bitmap, because the wxBitmapComboBox and wxDataViewCtrl don't support
    // high DPI bitmaps very well, they compute their dimensions wrong.
    // TODO: Update this when fixed in wxWidgets
    // See also PresetCollection::load_bitmap_default() and PresetCollection::load_bitmap_add()

    (void)window;
    *m_bitmapCompatible     = create_scaled_bitmap(nullptr, "flag_green");
    *m_bitmapIncompatible   = create_scaled_bitmap(nullptr, "flag_red");
    *m_bitmapLock           = create_scaled_bitmap(nullptr, "lock_closed");
    *m_bitmapLockOpen       = create_scaled_bitmap(nullptr, "lock_open");

    prints       .set_bitmap_compatible(m_bitmapCompatible);
    filaments    .set_bitmap_compatible(m_bitmapCompatible);
    sla_prints   .set_bitmap_compatible(m_bitmapCompatible);
    sla_materials.set_bitmap_compatible(m_bitmapCompatible);

    prints       .set_bitmap_incompatible(m_bitmapIncompatible);
    filaments    .set_bitmap_incompatible(m_bitmapIncompatible);
    sla_prints   .set_bitmap_incompatible(m_bitmapIncompatible);
    sla_materials.set_bitmap_incompatible(m_bitmapIncompatible);

    prints       .set_bitmap_lock(m_bitmapLock);
    filaments    .set_bitmap_lock(m_bitmapLock);
    sla_prints   .set_bitmap_lock(m_bitmapLock);
    sla_materials.set_bitmap_lock(m_bitmapLock);
    printers     .set_bitmap_lock(m_bitmapLock);

    prints       .set_bitmap_lock_open(m_bitmapLock);
    filaments    .set_bitmap_lock_open(m_bitmapLock);
    sla_prints   .set_bitmap_lock_open(m_bitmapLock);
    sla_materials.set_bitmap_lock_open(m_bitmapLock);
    printers     .set_bitmap_lock_open(m_bitmapLock);
}

DynamicPrintConfig PresetBundle::full_config() const
{
    return (this->printers.get_edited_preset().printer_technology() == ptFFF) ?
        this->full_fff_config() :
        this->full_sla_config();
}

DynamicPrintConfig PresetBundle::full_config_secure() const
{
    DynamicPrintConfig config = this->full_config();
    config.erase("print_host");
    config.erase("printhost_apikey");
    config.erase("printhost_cafile");
    return config;
}

DynamicPrintConfig PresetBundle::full_fff_config() const
{    
    DynamicPrintConfig out;
    out.apply(FullPrintConfig::defaults());
    out.apply(this->prints.get_edited_preset().config);
    // Add the default filament preset to have the "filament_preset_id" defined.
	out.apply(this->filaments.default_preset().config);
	out.apply(this->printers.get_edited_preset().config);
    out.apply(this->project_config);

    auto   *nozzle_diameter = dynamic_cast<const ConfigOptionFloats*>(out.option("nozzle_diameter"));
    size_t  num_extruders   = nozzle_diameter->values.size();
    // Collect the "compatible_printers_condition" and "inherits" values over all presets (print, filaments, printers) into a single vector.
    std::vector<std::string> compatible_printers_condition;
    std::vector<std::string> compatible_prints_condition;
    std::vector<std::string> inherits;
    compatible_printers_condition.emplace_back(this->prints.get_edited_preset().compatible_printers_condition());
    inherits                     .emplace_back(this->prints.get_edited_preset().inherits());

    if (num_extruders <= 1) {
        out.apply(this->filaments.get_edited_preset().config);
        compatible_printers_condition.emplace_back(this->filaments.get_edited_preset().compatible_printers_condition());
        compatible_prints_condition  .emplace_back(this->filaments.get_edited_preset().compatible_prints_condition());
        inherits                     .emplace_back(this->filaments.get_edited_preset().inherits());
    } else {
        // Retrieve filament presets and build a single config object for them.
        // First collect the filament configurations based on the user selection of this->filament_presets.
        // Here this->filaments.find_preset() and this->filaments.first_visible() return the edited copy of the preset if active.
        std::vector<const DynamicPrintConfig*> filament_configs;
        for (const std::string &filament_preset_name : this->filament_presets)
            filament_configs.emplace_back(&this->filaments.find_preset(filament_preset_name, true)->config);
		while (filament_configs.size() < num_extruders)
            filament_configs.emplace_back(&this->filaments.first_visible().config);
        for (const DynamicPrintConfig *cfg : filament_configs) {
            compatible_printers_condition.emplace_back(Preset::compatible_printers_condition(*const_cast<DynamicPrintConfig*>(cfg)));
            compatible_prints_condition  .emplace_back(Preset::compatible_prints_condition(*const_cast<DynamicPrintConfig*>(cfg)));
            inherits                     .emplace_back(Preset::inherits(*const_cast<DynamicPrintConfig*>(cfg)));
        }
        // Option values to set a ConfigOptionVector from.
        std::vector<const ConfigOption*> filament_opts(num_extruders, nullptr);
        // loop through options and apply them to the resulting config.
        for (const t_config_option_key &key : this->filaments.default_preset().config.keys()) {
			if (key == "compatible_prints" || key == "compatible_printers")
				continue;
            // Get a destination option.
            ConfigOption *opt_dst = out.option(key, false);
            if (opt_dst->is_scalar()) {
                // Get an option, do not create if it does not exist.
                const ConfigOption *opt_src = filament_configs.front()->option(key);
                if (opt_src != nullptr)
                    opt_dst->set(opt_src);
            } else {
                // Setting a vector value from all filament_configs.
                for (size_t i = 0; i < filament_opts.size(); ++ i)
                    filament_opts[i] = filament_configs[i]->option(key);
                static_cast<ConfigOptionVectorBase*>(opt_dst)->set(filament_opts);
            }
        }
    }

	// Don't store the "compatible_printers_condition" for the printer profile, there is none.
    inherits.emplace_back(this->printers.get_edited_preset().inherits());

    // These value types clash between the print and filament profiles. They should be renamed.
    out.erase("compatible_prints");
    out.erase("compatible_prints_condition");
    out.erase("compatible_printers");
    out.erase("compatible_printers_condition");
    out.erase("inherits");
    
    static const char *keys[] = { "perimeter", "infill", "solid_infill", "support_material", "support_material_interface" };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++ i) {
        std::string key = std::string(keys[i]) + "_extruder";
        auto *opt = dynamic_cast<ConfigOptionInt*>(out.option(key, false));
        assert(opt != nullptr);
        opt->value = boost::algorithm::clamp<int>(opt->value, 0, int(num_extruders));
    }

    out.option<ConfigOptionString >("print_settings_id",    true)->value  = this->prints.get_selected_preset_name();
    out.option<ConfigOptionStrings>("filament_settings_id", true)->values = this->filament_presets;
    out.option<ConfigOptionString >("printer_settings_id",  true)->value  = this->printers.get_selected_preset_name();

    // Serialize the collected "compatible_printers_condition" and "inherits" fields.
    // There will be 1 + num_exturders fields for "inherits" and 2 + num_extruders for "compatible_printers_condition" stored.
    // The vector will not be stored if all fields are empty strings.
    auto add_if_some_non_empty = [&out](std::vector<std::string> &&values, const std::string &key) {
        bool nonempty = false;
        for (const std::string &v : values)
            if (! v.empty()) {
                nonempty = true;
                break;
            }
        if (nonempty)
            out.set_key_value(key, new ConfigOptionStrings(std::move(values)));
    };
    add_if_some_non_empty(std::move(compatible_printers_condition), "compatible_printers_condition_cummulative");
    add_if_some_non_empty(std::move(compatible_prints_condition),   "compatible_prints_condition_cummulative");
    add_if_some_non_empty(std::move(inherits),                      "inherits_cummulative");

	out.option<ConfigOptionEnumGeneric>("printer_technology", true)->value = ptFFF;
    return out;
}

DynamicPrintConfig PresetBundle::full_sla_config() const
{    
    DynamicPrintConfig out;
    out.apply(SLAFullPrintConfig::defaults());
    out.apply(this->sla_prints.get_edited_preset().config);
    out.apply(this->sla_materials.get_edited_preset().config);
    out.apply(this->printers.get_edited_preset().config);
    // There are no project configuration values as of now, the project_config is reserved for FFF printers.
//    out.apply(this->project_config);

    // Collect the "compatible_printers_condition" and "inherits" values over all presets (sla_prints, sla_materials, printers) into a single vector.
    std::vector<std::string> compatible_printers_condition;
	std::vector<std::string> compatible_prints_condition;
    std::vector<std::string> inherits;
    compatible_printers_condition.emplace_back(this->sla_prints.get_edited_preset().compatible_printers_condition());
	inherits					 .emplace_back(this->sla_prints.get_edited_preset().inherits());
    compatible_printers_condition.emplace_back(this->sla_materials.get_edited_preset().compatible_printers_condition());
	compatible_prints_condition  .emplace_back(this->sla_materials.get_edited_preset().compatible_prints_condition());
    inherits                     .emplace_back(this->sla_materials.get_edited_preset().inherits());
    inherits                     .emplace_back(this->printers.get_edited_preset().inherits());

    // These two value types clash between the print and filament profiles. They should be renamed.
    out.erase("compatible_printers");
    out.erase("compatible_printers_condition");
    out.erase("inherits");
    
    out.option<ConfigOptionString >("sla_print_settings_id",    true)->value  = this->sla_prints.get_selected_preset_name();
    out.option<ConfigOptionString >("sla_material_settings_id", true)->value  = this->sla_materials.get_selected_preset_name();
    out.option<ConfigOptionString >("printer_settings_id",      true)->value  = this->printers.get_selected_preset_name();

    // Serialize the collected "compatible_printers_condition" and "inherits" fields.
    // There will be 1 + num_exturders fields for "inherits" and 2 + num_extruders for "compatible_printers_condition" stored.
    // The vector will not be stored if all fields are empty strings.
    auto add_if_some_non_empty = [&out](std::vector<std::string> &&values, const std::string &key) {
        bool nonempty = false;
        for (const std::string &v : values)
            if (! v.empty()) {
                nonempty = true;
                break;
            }
        if (nonempty)
            out.set_key_value(key, new ConfigOptionStrings(std::move(values)));
    };
    add_if_some_non_empty(std::move(compatible_printers_condition), "compatible_printers_condition_cummulative");
    add_if_some_non_empty(std::move(compatible_prints_condition),   "compatible_prints_condition_cummulative");
    add_if_some_non_empty(std::move(inherits),                      "inherits_cummulative");

	out.option<ConfigOptionEnumGeneric>("printer_technology", true)->value = ptSLA;
	return out;
}

// Load an external config file containing the print, filament and printer presets.
// Instead of a config file, a G-code may be loaded containing the full set of parameters.
// In the future the configuration will likely be read from an AMF file as well.
// If the file is loaded successfully, its print / filament / printer profiles will be activated.
void PresetBundle::load_config_file(const std::string &path)
{
	if (boost::iends_with(path, ".gcode") || boost::iends_with(path, ".g")) {
		DynamicPrintConfig config;
		config.apply(FullPrintConfig::defaults());
        config.load_from_gcode_file(path);
        Preset::normalize(config);
		load_config_file_config(path, true, std::move(config));
		return;
	}

    // 1) Try to load the config file into a boost property tree.
    boost::property_tree::ptree tree;
    try {
        boost::nowide::ifstream ifs(path);
        boost::property_tree::read_ini(ifs, tree);
    } catch (const std::ifstream::failure &err) {
        throw std::runtime_error(std::string("The config file cannot be loaded: ") + path + "\n\tReason: " + err.what());
    } catch (const std::runtime_error &err) {
        throw std::runtime_error(std::string("Failed loading the preset file: ") + path + "\n\tReason: " + err.what());
    }

    // 2) Continue based on the type of the configuration file.
    ConfigFileType config_file_type = guess_config_file_type(tree);
    switch (config_file_type) {
    case CONFIG_FILE_TYPE_UNKNOWN:
        throw std::runtime_error(std::string("Unknown configuration file type: ") + path);   
    case CONFIG_FILE_TYPE_APP_CONFIG:
        throw std::runtime_error(std::string("Invalid configuration file: ") + path + ". This is an application config file.");
	case CONFIG_FILE_TYPE_CONFIG:
	{
		// Initialize a config from full defaults.
		DynamicPrintConfig config;
		config.apply(FullPrintConfig::defaults());
		config.load(tree);
		Preset::normalize(config);
		load_config_file_config(path, true, std::move(config));
		break;
	}
    case CONFIG_FILE_TYPE_CONFIG_BUNDLE:
		load_config_file_config_bundle(path, tree);
        break;
    }
}

// Load a config file from a boost property_tree. This is a private method called from load_config_file.
void PresetBundle::load_config_file_config(const std::string &name_or_path, bool is_external, DynamicPrintConfig &&config)
{
    PrinterTechnology printer_technology = Preset::printer_technology(config);

    // The "compatible_printers" field should not have been exported into a config.ini or a G-code anyway, 
    // but some of the alpha versions of Slic3r did.
    {
        ConfigOption *opt_compatible = config.optptr("compatible_printers");
        if (opt_compatible != nullptr) {
            assert(opt_compatible->type() == coStrings);
            if (opt_compatible->type() == coStrings)
                static_cast<ConfigOptionStrings*>(opt_compatible)->values.clear();
        }
    }

    size_t num_extruders = (printer_technology == ptFFF) ?
        std::min(config.option<ConfigOptionFloats>("nozzle_diameter"  )->values.size(), 
                 config.option<ConfigOptionFloats>("filament_diameter")->values.size()) :
		// 1 SLA material
        1;
    // Make a copy of the "compatible_printers_condition_cummulative" and "inherits_cummulative" vectors, which 
    // accumulate values over all presets (print, filaments, printers).
    // These values will be distributed into their particular presets when loading.
    std::vector<std::string> compatible_printers_condition_values   = std::move(config.option<ConfigOptionStrings>("compatible_printers_condition_cummulative", true)->values);
    std::vector<std::string> compatible_prints_condition_values     = std::move(config.option<ConfigOptionStrings>("compatible_prints_condition_cummulative",   true)->values);
    std::vector<std::string> inherits_values                        = std::move(config.option<ConfigOptionStrings>("inherits_cummulative", true)->values);
    std::string &compatible_printers_condition  = Preset::compatible_printers_condition(config);
    std::string &compatible_prints_condition    = Preset::compatible_prints_condition(config);
    std::string &inherits                       = Preset::inherits(config);
    compatible_printers_condition_values.resize(num_extruders + 2, std::string());
    compatible_prints_condition_values.resize(num_extruders, std::string());
    inherits_values.resize(num_extruders + 2, std::string());
    // The "default_filament_profile" will be later extracted into the printer profile.
	switch (printer_technology) {
	case ptFFF:
		config.option<ConfigOptionString>("default_print_profile", true);
		config.option<ConfigOptionStrings>("default_filament_profile", true)->values.resize(num_extruders, std::string());
		break;
	case ptSLA:
		config.option<ConfigOptionString>("default_sla_print_profile", true);
		config.option<ConfigOptionString>("default_sla_material_profile", true);
		break;
    default: break;
	}

    // 1) Create a name from the file name.
    // Keep the suffix (.ini, .gcode, .amf, .3mf etc) to differentiate it from the normal profiles.
    std::string name = is_external ? boost::filesystem::path(name_or_path).filename().string() : name_or_path;

    // 2) If the loading succeeded, split and load the config into print / filament / printer settings.
    // First load the print and printer presets.

	auto load_preset = 
		[&config, &inherits, &inherits_values, 
         &compatible_printers_condition, &compatible_printers_condition_values, 
         &compatible_prints_condition, &compatible_prints_condition_values, 
         is_external, &name, &name_or_path]
		(PresetCollection &presets, size_t idx, const std::string &key) {
		// Split the "compatible_printers_condition" and "inherits" values one by one from a single vector to the print & printer profiles.
		inherits = inherits_values[idx];
		compatible_printers_condition = compatible_printers_condition_values[idx];
        if (idx > 0 && idx - 1 < compatible_prints_condition_values.size())
            compatible_prints_condition = compatible_prints_condition_values[idx - 1];
		if (is_external)
			presets.load_external_preset(name_or_path, name, config.opt_string(key, true), config);
		else
			presets.load_preset(presets.path_from_name(name), name, config).save();
	};

    switch (Preset::printer_technology(config)) {
    case ptFFF:
    {
        load_preset(this->prints, 0, "print_settings_id");
        load_preset(this->printers, num_extruders + 1, "printer_settings_id");

        // 3) Now load the filaments. If there are multiple filament presets, split them and load them.
        auto old_filament_profile_names = config.option<ConfigOptionStrings>("filament_settings_id", true);
    	old_filament_profile_names->values.resize(num_extruders, std::string());

        if (num_extruders <= 1) {
            // Split the "compatible_printers_condition" and "inherits" from the cummulative vectors to separate filament presets.
            inherits                      = inherits_values[1];
            compatible_printers_condition = compatible_printers_condition_values[1];
			compatible_prints_condition   = compatible_prints_condition_values.front();
			Preset                *loaded = nullptr;
			if (is_external) {
				loaded = &this->filaments.load_external_preset(name_or_path, name, old_filament_profile_names->values.front(), config);
			} else {
				loaded = &this->filaments.load_preset(this->filaments.path_from_name(name), name, config);
				loaded->save();
			}
            this->filament_presets.clear();
			this->filament_presets.emplace_back(loaded->name);
        } else {
            // Split the filament presets, load each of them separately.
            std::vector<DynamicPrintConfig> configs(num_extruders, this->filaments.default_preset().config);
            // loop through options and scatter them into configs.
            for (const t_config_option_key &key : this->filaments.default_preset().config.keys()) {
                const ConfigOption *other_opt = config.option(key);
                if (other_opt == nullptr)
                    continue;
                if (other_opt->is_scalar()) {
                    for (size_t i = 0; i < configs.size(); ++ i)
                        configs[i].option(key, false)->set(other_opt);
                } else if (key != "compatible_printers" && key != "compatible_prints") {
                    for (size_t i = 0; i < configs.size(); ++ i)
                        static_cast<ConfigOptionVectorBase*>(configs[i].option(key, false))->set_at(other_opt, 0, i);
                }
            }
            // Load the configs into this->filaments and make them active.
            this->filament_presets = std::vector<std::string>(configs.size());
            // To avoid incorrect selection of the first filament preset (means a value of Preset->m_idx_selected) 
            // in a case when next added preset take a place of previosly selected preset,
            // we should add presets from last to first
            for (int i = (int)configs.size()-1; i >= 0; i--) {
                DynamicPrintConfig &cfg = configs[i];
                // Split the "compatible_printers_condition" and "inherits" from the cummulative vectors to separate filament presets.
                cfg.opt_string("compatible_printers_condition", true) = compatible_printers_condition_values[i + 1];
                cfg.opt_string("compatible_prints_condition",   true) = compatible_prints_condition_values[i];
                cfg.opt_string("inherits", true)                      = inherits_values[i + 1];
                // Load all filament presets, but only select the first one in the preset dialog.
                Preset *loaded = nullptr;
                if (is_external)
                    loaded = &this->filaments.load_external_preset(name_or_path, name,
                        (i < int(old_filament_profile_names->values.size())) ? old_filament_profile_names->values[i] : "",
                        std::move(cfg), i == 0);
                else {
                    // Used by the config wizard when creating a custom setup.
                    // Therefore this block should only be called for a single extruder.
                    char suffix[64];
                    if (i == 0)
                        suffix[0] = 0;
                    else
                        sprintf(suffix, "%d", (int)i);
                    std::string new_name = name + suffix;
                    loaded = &this->filaments.load_preset(this->filaments.path_from_name(new_name),
                        new_name, std::move(cfg), i == 0);
                    loaded->save();
                }
                this->filament_presets[i] = loaded->name;
            }
        }
        // 4) Load the project config values (the per extruder wipe matrix etc).
        this->project_config.apply_only(config, s_project_options);

        update_custom_gcode_per_print_z_from_config(GUI::wxGetApp().plater()->model().custom_gcode_per_print_z, &this->project_config);

        break;
    }
    case ptSLA:
        load_preset(this->sla_prints,    0, "sla_print_settings_id");
        load_preset(this->sla_materials, 1, "sla_material_settings_id");
        load_preset(this->printers,      2, "printer_settings_id");
        break;
    default: break;
    }

	this->update_compatible(false);
}

// Load the active configuration of a config bundle from a boost property_tree. This is a private method called from load_config_file.
void PresetBundle::load_config_file_config_bundle(const std::string &path, const boost::property_tree::ptree &tree)
{
    // 1) Load the config bundle into a temp data.
    PresetBundle tmp_bundle;
    // Load the config bundle, don't save the loaded presets to user profile directory.
    tmp_bundle.load_configbundle(path, 0);
    std::string bundle_name = std::string(" - ") + boost::filesystem::path(path).filename().string();

    // 2) Extract active configs from the config bundle, copy them and activate them in this bundle.
    auto load_one = [this, &path, &bundle_name](PresetCollection &collection_dst, PresetCollection &collection_src, const std::string &preset_name_src, bool activate) -> std::string {
        Preset *preset_src = collection_src.find_preset(preset_name_src, false);
        Preset *preset_dst = collection_dst.find_preset(preset_name_src, false);
        assert(preset_src != nullptr);
        std::string preset_name_dst;
        if (preset_dst != nullptr && preset_dst->is_default) {
            // No need to copy a default preset, it always exists in collection_dst.
            if (activate)
                collection_dst.select_preset(0);
            return preset_name_src;
        } else if (preset_dst != nullptr && preset_src->config == preset_dst->config) {
            // Don't save as the config exists in the current bundle and its content is the same.
            return preset_name_src;
        } else {
            // Generate a new unique name.
            preset_name_dst = preset_name_src + bundle_name;
            Preset *preset_dup = nullptr;
            for (size_t i = 1; (preset_dup = collection_dst.find_preset(preset_name_dst, false)) != nullptr; ++ i) {
                if (preset_src->config == preset_dup->config)
                    // The preset has been already copied into collection_dst.
                    return preset_name_dst;
                // Try to generate another name.
                char buf[64];
                sprintf(buf, " (%d)", (int)i);
                preset_name_dst = preset_name_src + buf + bundle_name;
            }
        }
        assert(! preset_name_dst.empty());
        // Save preset_src->config into collection_dst under preset_name_dst.
        // The "compatible_printers" field should not have been exported into a config.ini or a G-code anyway, 
        // but some of the alpha versions of Slic3r did.
        ConfigOption *opt_compatible = preset_src->config.optptr("compatible_printers");
        if (opt_compatible != nullptr) {
            assert(opt_compatible->type() == coStrings);
            if (opt_compatible->type() == coStrings)
                static_cast<ConfigOptionStrings*>(opt_compatible)->values.clear();
        }
        collection_dst.load_preset(path, preset_name_dst, std::move(preset_src->config), activate).is_external = true;
        return preset_name_dst;
    };
    load_one(this->prints,        tmp_bundle.prints,        tmp_bundle.prints       .get_selected_preset_name(), true);
    load_one(this->sla_prints,    tmp_bundle.sla_prints,    tmp_bundle.sla_prints   .get_selected_preset_name(), true);
    load_one(this->filaments,     tmp_bundle.filaments,     tmp_bundle.filaments    .get_selected_preset_name(), true);
    load_one(this->sla_materials, tmp_bundle.sla_materials, tmp_bundle.sla_materials.get_selected_preset_name(), true);
    load_one(this->printers,      tmp_bundle.printers,      tmp_bundle.printers     .get_selected_preset_name(), true);
    this->update_multi_material_filament_presets();
    for (size_t i = 1; i < std::min(tmp_bundle.filament_presets.size(), this->filament_presets.size()); ++ i)
        this->filament_presets[i] = load_one(this->filaments, tmp_bundle.filaments, tmp_bundle.filament_presets[i], false);

    this->update_compatible(false);
}

// Process the Config Bundle loaded as a Boost property tree.
// For each print, filament and printer preset (group defined by group_name), apply the inherited presets.
// The presets starting with '*' are considered non-terminal and they are
// removed through the flattening process by this function.
// This function will never fail, but it will produce error messages through boost::log.
// system_profiles will not be flattened, and they will be kept inside the "inherits" field
static void flatten_configbundle_hierarchy(boost::property_tree::ptree &tree, const std::string &group_name, const std::vector<std::string> &system_profiles)
{
    namespace pt = boost::property_tree;

    typedef std::pair<pt::ptree::key_type, pt::ptree> ptree_child_type;

    // 1) For the group given by group_name, initialize the presets.
    struct Prst {
        Prst(const std::string &name, pt::ptree *node) : name(name), node(node) {}
        // Name of this preset. If the name starts with '*', it is an intermediate preset,
        // which will not make it into the result.
        const std::string           name;
        // Link to the source boost property tree node, owned by tree.
        pt::ptree                  *node;
        // Link to the presets, from which this preset inherits.
        std::vector<Prst*>          inherits;
        // Link to the presets, for which this preset is a direct parent.
        std::vector<Prst*>          parent_of;
        // When running the Kahn's Topological sorting algorithm, this counter is decreased from inherits.size() to zero.
        // A cycle is indicated, if the number does not drop to zero after the Kahn's algorithm finishes.
        size_t                      num_incoming_edges_left = 0;
        // Sorting by the name, to be used when inserted into std::set.
        bool operator==(const Prst &rhs) const { return this->name == rhs.name; }
        bool operator< (const Prst &rhs) const { return this->name < rhs.name; }
    };
    // Find the presets, store them into a std::map, addressed by their names.
    std::set<Prst> presets;
    std::string group_name_preset = group_name + ":";
    for (auto &section : tree)
        if (boost::starts_with(section.first, group_name_preset) && section.first.size() > group_name_preset.size())
            presets.emplace(section.first.substr(group_name_preset.size()), &section.second);
    // Fill in the "inherits" and "parent_of" members, report invalid inheritance fields.
    for (const Prst &prst : presets) {
        // Parse the list of comma separated values, possibly enclosed in quotes.
        std::vector<std::string> inherits_names;
        std::vector<std::string> inherits_system;
        if (Slic3r::unescape_strings_cstyle(prst.node->get<std::string>("inherits", ""), inherits_names)) {
            // Resolve the inheritance by name.
            std::vector<Prst*> &inherits_nodes = const_cast<Prst&>(prst).inherits;
            for (const std::string &node_name : inherits_names) {
                auto it_system = std::lower_bound(system_profiles.begin(), system_profiles.end(), node_name);
                if (it_system != system_profiles.end() && *it_system == node_name) {
                    // Loading a user config budnle, this preset is derived from a system profile.
                    inherits_system.emplace_back(node_name);
                } else {
                    auto it = presets.find(Prst(node_name, nullptr));
                    if (it == presets.end())
                        BOOST_LOG_TRIVIAL(error) << "flatten_configbundle_hierarchy: The preset " << prst.name << " inherits an unknown preset \"" << node_name << "\"";
                    else {
                        inherits_nodes.emplace_back(const_cast<Prst*>(&(*it)));
                        inherits_nodes.back()->parent_of.emplace_back(const_cast<Prst*>(&prst));
                    }
                }
            }
        } else {
            BOOST_LOG_TRIVIAL(error) << "flatten_configbundle_hierarchy: The preset " << prst.name << " has an invalid \"inherits\" field";
        }
        // Remove the "inherits" key, it has no meaning outside of the config bundle.
        const_cast<pt::ptree*>(prst.node)->erase("inherits");
        if (! inherits_system.empty()) {
            // Loaded a user config bundle, where a profile inherits a system profile.
			// User profile should be derived from a single system profile only.
			assert(inherits_system.size() == 1);
			if (inherits_system.size() > 1)
				BOOST_LOG_TRIVIAL(error) << "flatten_configbundle_hierarchy: The preset " << prst.name << " inherits from more than single system preset";
			prst.node->put("inherits", Slic3r::escape_string_cstyle(inherits_system.front()));
        }
    }

    // 2) Create a linear ordering for the directed acyclic graph of preset inheritance.
    // https://en.wikipedia.org/wiki/Topological_sorting
    // Kahn's algorithm.
    std::vector<Prst*> sorted;
    {
        // Initialize S with the set of all nodes with no incoming edge.
        std::deque<Prst*> S;
        for (const Prst &prst : presets)
            if (prst.inherits.empty())
                S.emplace_back(const_cast<Prst*>(&prst));
            else
                const_cast<Prst*>(&prst)->num_incoming_edges_left = prst.inherits.size();
        while (! S.empty()) {
            Prst *n = S.front();
            S.pop_front();
            sorted.emplace_back(n);
            for (Prst *m : n->parent_of) {
                assert(m->num_incoming_edges_left > 0);
                if (-- m->num_incoming_edges_left == 0) {
                    // We have visited all parents of m.
                    S.emplace_back(m);
                }
            }
        }
        if (sorted.size() < presets.size()) {
            for (const Prst &prst : presets)
                if (prst.num_incoming_edges_left)
                    BOOST_LOG_TRIVIAL(error) << "flatten_configbundle_hierarchy: The preset " << prst.name << " has cyclic dependencies";
        }
    }

    // Apply the dependencies in their topological ordering.
    for (Prst *prst : sorted) {
        // Merge the preset nodes in their order of application.
        // Iterate in a reverse order, so the last change will be placed first in merged.
        for (auto it_inherits = prst->inherits.rbegin(); it_inherits != prst->inherits.rend(); ++ it_inherits)
            for (auto it = (*it_inherits)->node->begin(); it != (*it_inherits)->node->end(); ++ it)
                if (prst->node->find(it->first) == prst->node->not_found())
                    prst->node->add_child(it->first, it->second);
    }

    // Remove the "internal" presets from the ptree. These presets are marked with '*'.
    group_name_preset += '*';
    for (auto it_section = tree.begin(); it_section != tree.end(); ) {
        if (boost::starts_with(it_section->first, group_name_preset) && it_section->first.size() > group_name_preset.size())
            // Remove the "internal" preset from the ptree.
            it_section = tree.erase(it_section);
        else
            // Keep the preset.
            ++ it_section;
    }
}

// preset_bundle is set when loading user config bundles, which must not overwrite the system profiles.
static void flatten_configbundle_hierarchy(boost::property_tree::ptree &tree, const PresetBundle *preset_bundle)
{
    flatten_configbundle_hierarchy(tree, "print",           preset_bundle ? preset_bundle->prints.system_preset_names()        : std::vector<std::string>());
    flatten_configbundle_hierarchy(tree, "filament",        preset_bundle ? preset_bundle->filaments.system_preset_names()     : std::vector<std::string>());
    flatten_configbundle_hierarchy(tree, "sla_print",       preset_bundle ? preset_bundle->sla_prints.system_preset_names()    : std::vector<std::string>());
    flatten_configbundle_hierarchy(tree, "sla_material",    preset_bundle ? preset_bundle->sla_materials.system_preset_names() : std::vector<std::string>());
    flatten_configbundle_hierarchy(tree, "printer",         preset_bundle ? preset_bundle->printers.system_preset_names()      : std::vector<std::string>());
}

// Load a config bundle file, into presets and store the loaded presets into separate files
// of the local configuration directory.
size_t PresetBundle::load_configbundle(const std::string &path, unsigned int flags)
{
    if (flags & (LOAD_CFGBNDLE_RESET_USER_PROFILE | LOAD_CFGBNDLE_SYSTEM))
        // Reset this bundle, delete user profile files if LOAD_CFGBNDLE_SAVE.
        this->reset(flags & LOAD_CFGBNDLE_SAVE);

    // 1) Read the complete config file into a boost::property_tree.
    namespace pt = boost::property_tree;
    pt::ptree tree;
    boost::nowide::ifstream ifs(path);
    pt::read_ini(ifs, tree);

    const VendorProfile *vendor_profile = nullptr;
    if (flags & (LOAD_CFGBNDLE_SYSTEM | LOAD_CFGBUNDLE_VENDOR_ONLY)) {
        auto vp = VendorProfile::from_ini(tree, path);
        if (vp.num_variants() == 0)
            return 0;
        vendor_profile = &this->vendors.insert({vp.id, vp}).first->second;
    }

    if (flags & LOAD_CFGBUNDLE_VENDOR_ONLY) {
        return 0;
    }

    // 1.5) Flatten the config bundle by applying the inheritance rules. Internal profiles (with names starting with '*') are removed.
    // If loading a user config bundle, do not flatten with the system profiles, but keep the "inherits" flag intact.
    flatten_configbundle_hierarchy(tree, ((flags & LOAD_CFGBNDLE_SYSTEM) == 0) ? this : nullptr);

    // 2) Parse the property_tree, extract the active preset names and the profiles, save them into local config files.
    // Parse the obsolete preset names, to be deleted when upgrading from the old configuration structure.
    std::vector<std::string> loaded_prints;
    std::vector<std::string> loaded_filaments;
    std::vector<std::string> loaded_sla_prints;
    std::vector<std::string> loaded_sla_materials;
    std::vector<std::string> loaded_printers;
    std::string              active_print;
    std::vector<std::string> active_filaments;
    std::string              active_sla_print;
    std::string              active_sla_material;
    std::string              active_printer;
    size_t                   presets_loaded = 0;
    for (const auto &section : tree) {
        PresetCollection         *presets = nullptr;
        std::vector<std::string> *loaded  = nullptr;
        std::string               preset_name;
        std::string               alias_name;
        if (boost::starts_with(section.first, "print:")) {
            presets = &this->prints;
            loaded  = &loaded_prints;
            preset_name = section.first.substr(6);
        } else if (boost::starts_with(section.first, "filament:")) {
            presets = &this->filaments;
            loaded  = &loaded_filaments;
            preset_name = section.first.substr(9);

            for (const auto& item : section.second)
                if (boost::starts_with(item.first, "alias")) {
                    alias_name = item.second.data();
                    break;
                }
        } else if (boost::starts_with(section.first, "sla_print:")) {
            presets = &this->sla_prints;
            loaded  = &loaded_sla_prints;
            preset_name = section.first.substr(10);
        } else if (boost::starts_with(section.first, "sla_material:")) {
            presets = &this->sla_materials;
            loaded  = &loaded_sla_materials;
            preset_name = section.first.substr(13);

            int end_pos = preset_name.find_first_of("0.");
            alias_name = preset_name.substr(0, end_pos-1);
        } else if (boost::starts_with(section.first, "printer:")) {
            presets = &this->printers;
            loaded  = &loaded_printers;
            preset_name = section.first.substr(8);
        } else if (section.first == "presets") {
            // Load the names of the active presets.
            for (auto &kvp : section.second) {
                if (kvp.first == "print") {
                    active_print = kvp.second.data();
                } else if (boost::starts_with(kvp.first, "filament")) {
                    int idx = 0;
                    if (kvp.first == "filament" || sscanf(kvp.first.c_str(), "filament_%d", &idx) == 1) {
                        if (int(active_filaments.size()) <= idx)
                            active_filaments.resize(idx + 1, std::string());
                        active_filaments[idx] = kvp.second.data();
                    }
                } else if (kvp.first == "sla_print") {
                    active_sla_print = kvp.second.data();
                } else if (kvp.first == "sla_material") {
                    active_sla_material = kvp.second.data();
                } else if (kvp.first == "printer") {
                    active_printer = kvp.second.data();
                }
            }
        } else if (section.first == "obsolete_presets") {
            // Parse the names of obsolete presets. These presets will be deleted from user's
            // profile directory on installation of this vendor preset.
            for (auto &kvp : section.second) {
                std::vector<std::string> *dst = nullptr;
                if (kvp.first == "print")
                    dst = &this->obsolete_presets.prints;
                else if (kvp.first == "filament")
                    dst = &this->obsolete_presets.filaments;
                else if (kvp.first == "sla_print")
                    dst = &this->obsolete_presets.sla_prints;
                else if (kvp.first == "sla_material")
                    dst = &this->obsolete_presets.sla_materials;
                else if (kvp.first == "printer")
                    dst = &this->obsolete_presets.printers;
                if (dst)
                    unescape_strings_cstyle(kvp.second.data(), *dst);
            }
        } else if (section.first == "settings") {
            // Load the settings.
            for (auto &kvp : section.second) {
                if (kvp.first == "autocenter") {
                }
            }
        } else
            // Ignore an unknown section.
            continue;
        if (presets != nullptr) {
            // Load the print, filament or printer preset.
            const DynamicPrintConfig *default_config = nullptr;
            DynamicPrintConfig        config;
            if (presets == &this->printers) {
                // Select the default config based on the printer_technology field extracted from kvp.
                DynamicPrintConfig config_src;
                for (auto &kvp : section.second)
                    config_src.set_deserialize(kvp.first, kvp.second.data());
                default_config = &presets->default_preset_for(config_src).config;
                config = *default_config;
                config.apply(config_src);
            } else {
                default_config = &presets->default_preset().config;
                config = *default_config;
                for (auto &kvp : section.second)
                    config.set_deserialize(kvp.first, kvp.second.data());
            }
            Preset::normalize(config);
            // Report configuration fields, which are misplaced into a wrong group.
            std::string incorrect_keys = Preset::remove_invalid_keys(config, *default_config);
            if (! incorrect_keys.empty())
                BOOST_LOG_TRIVIAL(error) << "Error in a Vendor Config Bundle \"" << path << "\": The printer preset \"" << 
                    section.first << "\" contains the following incorrect keys: " << incorrect_keys << ", which were removed";
            if ((flags & LOAD_CFGBNDLE_SYSTEM) && presets == &printers) {
                // Filter out printer presets, which are not mentioned in the vendor profile.
                // These presets are considered not installed.
                auto printer_model   = config.opt_string("printer_model");
                if (printer_model.empty()) {
                    BOOST_LOG_TRIVIAL(error) << "Error in a Vendor Config Bundle \"" << path << "\": The printer preset \"" << 
                        section.first << "\" defines no printer model, it will be ignored.";
                    continue;
                }
                auto printer_variant = config.opt_string("printer_variant");
                if (printer_variant.empty()) {
                    BOOST_LOG_TRIVIAL(error) << "Error in a Vendor Config Bundle \"" << path << "\": The printer preset \"" << 
                        section.first << "\" defines no printer variant, it will be ignored.";
                    continue;
                }
                auto it_model = std::find_if(vendor_profile->models.cbegin(), vendor_profile->models.cend(),
                    [&](const VendorProfile::PrinterModel &m) { return m.id == printer_model; }
                );
                if (it_model == vendor_profile->models.end()) {
                    BOOST_LOG_TRIVIAL(error) << "Error in a Vendor Config Bundle \"" << path << "\": The printer preset \"" << 
                        section.first << "\" defines invalid printer model \"" << printer_model << "\", it will be ignored.";
                    continue;
                }
                auto it_variant = it_model->variant(printer_variant);
                if (it_variant == nullptr) {
                    BOOST_LOG_TRIVIAL(error) << "Error in a Vendor Config Bundle \"" << path << "\": The printer preset \"" << 
                        section.first << "\" defines invalid printer variant \"" << printer_variant << "\", it will be ignored.";
                    continue;
                }
                const Preset *preset_existing = presets->find_preset(section.first, false);
                if (preset_existing != nullptr) {
                    BOOST_LOG_TRIVIAL(error) << "Error in a Vendor Config Bundle \"" << path << "\": The printer preset \"" << 
                        section.first << "\" has already been loaded from another Confing Bundle.";
                    continue;
                }
            } else if ((flags & LOAD_CFGBNDLE_SYSTEM) == 0) {
                // This is a user config bundle.
                const Preset *existing = presets->find_preset(preset_name, false);
                if (existing != nullptr) {
                    if (existing->is_system) {
    					assert(existing->vendor != nullptr);
                        BOOST_LOG_TRIVIAL(error) << "Error in a user provided Config Bundle \"" << path << "\": The " << presets->name() << " preset \"" << 
    						existing->name << "\" is a system preset of vendor " << existing->vendor->name << " and it will be ignored.";
                        continue;
                    } else {
                        assert(existing->vendor == nullptr);
                        BOOST_LOG_TRIVIAL(trace) << "A " << presets->name() << " preset \"" << existing->name << "\" was overwritten with a preset from user Config Bundle \"" << path << "\"";
                    }
                } else {
					BOOST_LOG_TRIVIAL(trace) << "A new " << presets->name() << " preset \"" << preset_name << "\" was imported from user Config Bundle \"" << path << "\"";
                }
            }
            // Decide a full path to this .ini file.
            auto file_name = boost::algorithm::iends_with(preset_name, ".ini") ? preset_name : preset_name + ".ini";
            auto file_path = (boost::filesystem::path(data_dir()) 
#ifdef SLIC3R_PROFILE_USE_PRESETS_SUBDIR
                // Store the print/filament/printer presets into a "presets" directory.
                / "presets" 
#else
                // Store the print/filament/printer presets at the same location as the upstream Slic3r.
#endif
                / presets->section_name() / file_name).make_preferred();
            // Load the preset into the list of presets, save it to disk.
            Preset &loaded = presets->load_preset(file_path.string(), preset_name, std::move(config), false);
            if (flags & LOAD_CFGBNDLE_SAVE)
                loaded.save();
            if (flags & LOAD_CFGBNDLE_SYSTEM) {
                loaded.is_system = true;
                loaded.vendor = vendor_profile;
            }

            // next step of an preset name aliasing
            int end_pos = preset_name.find_first_of("@");
            if (end_pos != std::string::npos)
                alias_name = preset_name.substr(0, end_pos - 1);

            loaded.alias = alias_name.empty() ? preset_name : alias_name;

            ++ presets_loaded;
        }
    }

    // 3) Activate the presets.
    if ((flags & LOAD_CFGBNDLE_SYSTEM) == 0) {
        if (! active_print.empty()) 
            prints.select_preset_by_name(active_print, true);
        if (! active_sla_print.empty()) 
            sla_materials.select_preset_by_name(active_sla_print, true);
        if (! active_sla_material.empty()) 
            sla_materials.select_preset_by_name(active_sla_material, true);
        if (! active_printer.empty())
            printers.select_preset_by_name(active_printer, true);
        // Activate the first filament preset.
        if (! active_filaments.empty() && ! active_filaments.front().empty())
            filaments.select_preset_by_name(active_filaments.front(), true);
        this->update_multi_material_filament_presets();
        for (size_t i = 0; i < std::min(this->filament_presets.size(), active_filaments.size()); ++ i)
            this->filament_presets[i] = filaments.find_preset(active_filaments[i], true)->name;
        this->update_compatible(false);
    }

    return presets_loaded;
}

void PresetBundle::update_multi_material_filament_presets()
{
    if (printers.get_edited_preset().printer_technology() != ptFFF)
        return;

    // Verify and select the filament presets.
    auto   *nozzle_diameter = static_cast<const ConfigOptionFloats*>(printers.get_edited_preset().config.option("nozzle_diameter"));
    size_t  num_extruders   = nozzle_diameter->values.size();
    // Verify validity of the current filament presets.
    for (size_t i = 0; i < std::min(this->filament_presets.size(), num_extruders); ++ i)
        this->filament_presets[i] = this->filaments.find_preset(this->filament_presets[i], true)->name;
    // Append the rest of filament presets.
    this->filament_presets.resize(num_extruders, this->filament_presets.empty() ? this->filaments.first_visible().name : this->filament_presets.back());

    // Now verify if wiping_volumes_matrix has proper size (it is used to deduce number of extruders in wipe tower generator):
    std::vector<double> old_matrix = this->project_config.option<ConfigOptionFloats>("wiping_volumes_matrix")->values;
    size_t old_number_of_extruders = size_t(sqrt(old_matrix.size())+EPSILON);
    if (num_extruders != old_number_of_extruders) {
            // First verify if purging volumes presets for each extruder matches number of extruders
            std::vector<double>& extruders = this->project_config.option<ConfigOptionFloats>("wiping_volumes_extruders")->values;
            while (extruders.size() < 2*num_extruders) {
                extruders.push_back(extruders.size()>1 ? extruders[0] : 50.);  // copy the values from the first extruder
                extruders.push_back(extruders.size()>1 ? extruders[1] : 50.);
            }
            while (extruders.size() > 2*num_extruders) {
                extruders.pop_back();
                extruders.pop_back();
            }

        std::vector<double> new_matrix;
        for (unsigned int i=0;i<num_extruders;++i)
            for (unsigned int j=0;j<num_extruders;++j) {
                // append the value for this pair from the old matrix (if it's there):
                if (i<old_number_of_extruders && j<old_number_of_extruders)
                    new_matrix.push_back(old_matrix[i*old_number_of_extruders + j]);
                else
                    new_matrix.push_back( i==j ? 0. : extruders[2*i]+extruders[2*j+1]); // so it matches new extruder volumes
            }
		this->project_config.option<ConfigOptionFloats>("wiping_volumes_matrix")->values = new_matrix;
    }
}

void PresetBundle::update_compatible(bool select_other_if_incompatible)
{
    const Preset					&printer_preset					    = this->printers.get_edited_preset();
	const PresetWithVendorProfile    printer_preset_with_vendor_profile = this->printers.get_preset_with_vendor_profile(printer_preset);

	switch (printer_preset.printer_technology()) {
    case ptFFF:
    {
		assert(printer_preset.config.has("default_print_profile"));
		assert(printer_preset.config.has("default_filament_profile"));
		const std::string              &prefered_print_profile = printer_preset.config.opt_string("default_print_profile");
        const std::vector<std::string> &prefered_filament_profiles = printer_preset.config.option<ConfigOptionStrings>("default_filament_profile")->values;
        prefered_print_profile.empty() ?
            this->prints.update_compatible(printer_preset_with_vendor_profile, nullptr, select_other_if_incompatible) :
            this->prints.update_compatible(printer_preset_with_vendor_profile, nullptr, select_other_if_incompatible,
                [&prefered_print_profile](const std::string& profile_name) { return profile_name == prefered_print_profile; });
        const PresetWithVendorProfile   print_preset_with_vendor_profile = this->prints.get_edited_preset_with_vendor_profile();
        prefered_filament_profiles.empty() ?
            this->filaments.update_compatible(printer_preset_with_vendor_profile, &print_preset_with_vendor_profile, select_other_if_incompatible) :
            this->filaments.update_compatible(printer_preset_with_vendor_profile, &print_preset_with_vendor_profile, select_other_if_incompatible,
                [&prefered_filament_profiles](const std::string& profile_name)
                    { return std::find(prefered_filament_profiles.begin(), prefered_filament_profiles.end(), profile_name) != prefered_filament_profiles.end(); });
        if (select_other_if_incompatible) {
            // Verify validity of the current filament presets.
            if (this->filament_presets.size() == 1)
                this->filament_presets.front() = this->filaments.get_edited_preset().name;
            else
            {
                for (size_t idx = 0; idx < this->filament_presets.size(); ++idx) {
                    std::string &filament_name = this->filament_presets[idx];
                    Preset      *preset = this->filaments.find_preset(filament_name, false);
                    if (preset == nullptr || !preset->is_compatible) {
                        // Pick a compatible profile. If there are prefered_filament_profiles, use them.
                        if (prefered_filament_profiles.empty())
                            filament_name = this->filaments.first_compatible().name;
                        else {
                            const std::string &preferred = (idx < prefered_filament_profiles.size()) ?
                                prefered_filament_profiles[idx] : prefered_filament_profiles.front();
                            filament_name = this->filaments.first_compatible(
                                [&preferred](const std::string& profile_name) { return profile_name == preferred; }).name;
                        }
                    }
                }
            }
        }
		break;
    }
    case ptSLA:
    {
		assert(printer_preset.config.has("default_sla_print_profile"));
		assert(printer_preset.config.has("default_sla_material_profile"));
		const PresetWithVendorProfile    sla_print_preset_with_vendor_profile = this->sla_prints.get_edited_preset_with_vendor_profile();
		const std::string				&prefered_sla_print_profile = printer_preset.config.opt_string("default_sla_print_profile");
		(prefered_sla_print_profile.empty()) ?
			this->sla_prints.update_compatible(printer_preset_with_vendor_profile, nullptr, select_other_if_incompatible) :
			this->sla_prints.update_compatible(printer_preset_with_vendor_profile, nullptr, select_other_if_incompatible,
			[&prefered_sla_print_profile](const std::string& profile_name){ return profile_name == prefered_sla_print_profile; });
		const std::string &prefered_sla_material_profile = printer_preset.config.opt_string("default_sla_material_profile");
        prefered_sla_material_profile.empty() ?
            this->sla_materials.update_compatible(printer_preset_with_vendor_profile, &sla_print_preset_with_vendor_profile, select_other_if_incompatible) :
			this->sla_materials.update_compatible(printer_preset_with_vendor_profile, &sla_print_preset_with_vendor_profile, select_other_if_incompatible,
                [&prefered_sla_material_profile](const std::string& profile_name){ return profile_name == prefered_sla_material_profile; });
		break;
	}
    default: break;
    }
}

void PresetBundle::export_configbundle(const std::string &path, bool export_system_settings)
{
    boost::nowide::ofstream c;
    c.open(path, std::ios::out | std::ios::trunc);

    // Put a comment at the first line including the time stamp and Slic3r version.
    c << "# " << Slic3r::header_slic3r_generated() << std::endl;

    // Export the print, filament and printer profiles.

	for (const PresetCollection *presets : { 
		(const PresetCollection*)&this->prints, (const PresetCollection*)&this->filaments, 
		(const PresetCollection*)&this->sla_prints, (const PresetCollection*)&this->sla_materials, 
		(const PresetCollection*)&this->printers }) {
        for (const Preset &preset : (*presets)()) {
            if (preset.is_default || preset.is_external || (preset.is_system && ! export_system_settings))
                // Only export the common presets, not external files or the default preset.
                continue;
            c << std::endl << "[" << presets->section_name() << ":" << preset.name << "]" << std::endl;
            for (const std::string &opt_key : preset.config.keys())
                c << opt_key << " = " << preset.config.opt_serialize(opt_key) << std::endl;
        }
    }

    // Export the names of the active presets.
    c << std::endl << "[presets]" << std::endl;
    c << "print = " << this->prints.get_selected_preset_name() << std::endl;
    c << "sla_print = " << this->sla_prints.get_selected_preset_name() << std::endl;
    c << "sla_material = " << this->sla_materials.get_selected_preset_name() << std::endl;
    c << "printer = " << this->printers.get_selected_preset_name() << std::endl;
    for (size_t i = 0; i < this->filament_presets.size(); ++ i) {
        char suffix[64];
        if (i > 0)
            sprintf(suffix, "_%d", (int)i);
        else
            suffix[0] = 0;
        c << "filament" << suffix << " = " << this->filament_presets[i] << std::endl;
    }

#if 0
    // Export the following setting values from the provided setting repository.
    static const char *settings_keys[] = { "autocenter" };
    c << "[settings]" << std::endl;
    for (size_t i = 0; i < sizeof(settings_keys) / sizeof(settings_keys[0]); ++ i)
        c << settings_keys[i] << " = " << settings.serialize(settings_keys[i]) << std::endl;
#endif

    c.close();
}

// Set the filament preset name. As the name could come from the UI selection box, 
// an optional "(modified)" suffix will be removed from the filament name.
void PresetBundle::set_filament_preset(size_t idx, const std::string &name)
{
	if (name.find_first_of(PresetCollection::separator_head()) == 0)
		return;

    if (idx >= filament_presets.size())
        filament_presets.resize(idx + 1, filaments.default_preset().name);
    filament_presets[idx] = Preset::remove_suffix_modified(name);
}

static inline int hex_digit_to_int(const char c)
{
    return 
        (c >= '0' && c <= '9') ? int(c - '0') : 
        (c >= 'A' && c <= 'F') ? int(c - 'A') + 10 :
        (c >= 'a' && c <= 'f') ? int(c - 'a') + 10 : -1;
}

bool PresetBundle::parse_color(const std::string &scolor, unsigned char *rgb_out)
{
    rgb_out[0] = rgb_out[1] = rgb_out[2] = 0;
    if (scolor.size() != 7 || scolor.front() != '#')
        return false;
    const char *c = scolor.data() + 1;
    for (size_t i = 0; i < 3; ++ i) {
        int digit1 = hex_digit_to_int(*c ++);
        int digit2 = hex_digit_to_int(*c ++);
        if (digit1 == -1 || digit2 == -1)
            return false;
        rgb_out[i] = (unsigned char)(digit1 * 16 + digit2);
    }
    return true;
}

void PresetBundle::load_default_preset_bitmaps(wxWindow *window)
{
    // Clear bitmap cache, before load new scaled default preset bitmaps 
    m_bitmapCache->clear();
    this->prints.clear_bitmap_cache();
    this->sla_prints.clear_bitmap_cache();
    this->filaments.clear_bitmap_cache();
    this->sla_materials.clear_bitmap_cache();
    this->printers.clear_bitmap_cache();

    this->prints.load_bitmap_default(window, "cog");
    this->sla_prints.load_bitmap_default(window, "cog");
    this->filaments.load_bitmap_default(window, "spool.png");
    this->sla_materials.load_bitmap_default(window, "resin");
    this->printers.load_bitmap_default(window, "printer");
    this->printers.load_bitmap_add(window, "add.png");
    this->load_compatible_bitmaps(window);
}

void PresetBundle::update_platter_filament_ui(unsigned int idx_extruder, GUI::PresetComboBox *ui)
{
    if (ui == nullptr || this->printers.get_edited_preset().printer_technology() == ptSLA ||
        this->filament_presets.size() <= idx_extruder )
        return;

    unsigned char rgb[3];
    std::string extruder_color = this->printers.get_edited_preset().config.opt_string("extruder_colour", idx_extruder);
    if (! parse_color(extruder_color, rgb))
        // Extruder color is not defined.
        extruder_color.clear();

    // Fill in the list from scratch.
    ui->Freeze();
    ui->Clear();
	size_t selected_preset_item = INT_MAX; // some value meaning that no one item is selected 

    const Preset *selected_preset = this->filaments.find_preset(this->filament_presets[idx_extruder]);
    // Show wide icons if the currently selected preset is not compatible with the current printer,
    // and draw a red flag in front of the selected preset.
    bool          wide_icons      = selected_preset != nullptr && ! selected_preset->is_compatible && m_bitmapIncompatible != nullptr;
    assert(selected_preset != nullptr);
	std::map<wxString, wxBitmap*> nonsys_presets;
	wxString selected_str = "";
	if (!this->filaments().front().is_visible)
        ui->set_label_marker(ui->Append(PresetCollection::separator(L("System presets")), wxNullBitmap));

    /* It's supposed that standard size of an icon is 16px*16px for 100% scaled display.
     * So set sizes for solid_colored icons used for filament preset 
     * and scale them in respect to em_unit value
     */
    const float scale_f = ui->em_unit() * 0.1f;

    // To avoid the errors of number rounding for different combination of monitor configuration,
    // let use scaled 8px, as a smallest icon unit 
    const int icon_unit         = 8 * scale_f + 0.5f;
    const int normal_icon_width = 2 * icon_unit;    //16 * scale_f + 0.5f;
    const int thin_icon_width   = icon_unit;        //8 * scale_f + 0.5f;
    const int wide_icon_width   = 3 * icon_unit;    //24 * scale_f + 0.5f;

    const int space_icon_width  = 2  * scale_f + 0.5f;

    // To avoid asserts, each added bitmap to wxBitmapCombobox should be the same size, so
    // set a bitmap height to m_bitmapLock->GetHeight()
    const int icon_height       = m_bitmapLock->GetHeight();//2 * icon_unit;    //16 * scale_f + 0.5f;

    wxString tooltip = "";

	for (int i = this->filaments().front().is_visible ? 0 : 1; i < int(this->filaments().size()); ++i) {
        const Preset &preset    = this->filaments.preset(i);
        bool          selected  = this->filament_presets[idx_extruder] == preset.name;
		if (! preset.is_visible || (! preset.is_compatible && ! selected))
			continue;
		// Assign an extruder color to the selected item if the extruder color is defined.
		std::string   filament_rgb = preset.config.opt_string("filament_colour", 0);
		std::string   extruder_rgb = (selected && !extruder_color.empty()) ? extruder_color : filament_rgb;
        bool          single_bar   = filament_rgb == extruder_rgb;
        std::string   bitmap_key   = single_bar ? filament_rgb : filament_rgb + extruder_rgb;
        // If the filament preset is not compatible and there is a "red flag" icon loaded, show it left
        // to the filament color image.
        if (wide_icons)
            bitmap_key += preset.is_compatible ? ",cmpt" : ",ncmpt";
        bitmap_key += (preset.is_system || preset.is_default) ? ",syst" : ",nsyst";
        if (preset.is_dirty)
            bitmap_key += ",drty";
        wxBitmap     *bitmap       = m_bitmapCache->find(bitmap_key);
        if (bitmap == nullptr) {
            // Create the bitmap with color bars.
            std::vector<wxBitmap> bmps;
            if (wide_icons)
                // Paint a red flag for incompatible presets.
                bmps.emplace_back(preset.is_compatible ? m_bitmapCache->mkclear(normal_icon_width, icon_height) : *m_bitmapIncompatible);
            // Paint the color bars.
            parse_color(filament_rgb, rgb);
            bmps.emplace_back(m_bitmapCache->mksolid(single_bar ? wide_icon_width : normal_icon_width, icon_height, rgb));
            if (! single_bar) {
                parse_color(extruder_rgb, rgb);
                bmps.emplace_back(m_bitmapCache->mksolid(thin_icon_width, icon_height, rgb));
            }
            // Paint a lock at the system presets.
            bmps.emplace_back(m_bitmapCache->mkclear(space_icon_width, icon_height));
            // To avoid asserts, each added bitmap to wxBitmapCombobox should be the same size, so
            // for nonsystem presets set a width of empty bitmap to m_bitmapLock->GetWidth()
            bmps.emplace_back((preset.is_system || preset.is_default) ? *m_bitmapLock : m_bitmapCache->mkclear(m_bitmapLock->GetWidth(), icon_height));
//                 (preset.is_dirty ? *m_bitmapLockOpen : *m_bitmapLock) : m_bitmapCache->mkclear(16, 16));
            bitmap = m_bitmapCache->insert(bitmap_key, bmps);
		}

        const std::string name = preset.alias.empty() ? preset.name : preset.alias;
        if (preset.is_default || preset.is_system) {
			ui->Append(wxString::FromUTF8((/*preset.*/name + (preset.is_dirty ? Preset::suffix_modified() : "")).c_str()), 
				(bitmap == 0) ? wxNullBitmap : *bitmap);
			if (selected ||
                // just in case: mark selected_preset_item as a first added element
                selected_preset_item == INT_MAX ) {
				selected_preset_item = ui->GetCount() - 1;
                tooltip = wxString::FromUTF8(preset.name.c_str());
            }
		}
		else
		{
			nonsys_presets.emplace(wxString::FromUTF8((/*preset.*/name + (preset.is_dirty ? Preset::suffix_modified() : "")).c_str()), 
				(bitmap == 0) ? &wxNullBitmap : bitmap);
			if (selected) {
				selected_str = wxString::FromUTF8((/*preset.*/name + (preset.is_dirty ? Preset::suffix_modified() : "")).c_str());
                tooltip = wxString::FromUTF8(preset.name.c_str());
            }
		}
		if (preset.is_default)
            ui->set_label_marker(ui->Append(PresetCollection::separator(L("System presets")), wxNullBitmap));
    }

	if (!nonsys_presets.empty())
	{
        ui->set_label_marker(ui->Append(PresetCollection::separator(L("User presets")), wxNullBitmap));
		for (std::map<wxString, wxBitmap*>::iterator it = nonsys_presets.begin(); it != nonsys_presets.end(); ++it) {
			ui->Append(it->first, *it->second);
			if (it->first == selected_str ||
                // just in case: mark selected_preset_item as a first added element
                selected_preset_item == INT_MAX) {
				selected_preset_item = ui->GetCount() - 1;
			}
		}
	}

    ui->set_label_marker(ui->Append(PresetCollection::separator(L("Add/Remove filaments")), wxNullBitmap), GUI::PresetComboBox::LABEL_ITEM_WIZARD_FILAMENTS);

    /* But, if selected_preset_item is still equal to INT_MAX, it means that
     * there is no presets added to the list.
     * So, select last combobox item ("Add/Remove filaments")
     */
    if (selected_preset_item == INT_MAX)
        selected_preset_item = ui->GetCount() - 1;

	ui->SetSelection(selected_preset_item);
	ui->SetToolTip(tooltip.IsEmpty() ? ui->GetString(selected_preset_item) : tooltip);
    ui->check_selection();
    ui->Thaw();

    // Update control min size after rescale (changed Display DPI under MSW)
    if (ui->GetMinWidth() != 20 * ui->em_unit())
        ui->SetMinSize(wxSize(20 * ui->em_unit(), ui->GetSize().GetHeight()));
}

void PresetBundle::set_default_suppressed(bool default_suppressed)
{
    prints.set_default_suppressed(default_suppressed);
    filaments.set_default_suppressed(default_suppressed);
    sla_prints.set_default_suppressed(default_suppressed);
    sla_materials.set_default_suppressed(default_suppressed);
    printers.set_default_suppressed(default_suppressed);
}

} // namespace Slic3r
