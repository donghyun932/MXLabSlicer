#include "libslic3r/libslic3r.h"
#include "libslic3r/Utils.hpp"
#include "AppConfig.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <utility>
#include <assert.h>
#include <vector>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <boost/nowide/cenv.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/format.hpp>

#include <wx/string.h>
#include "I18N.hpp"

namespace Slic3r {

static const std::string VENDOR_PREFIX = "vendor:";
static const std::string MODEL_PREFIX = "model:";
static const std::string VERSION_CHECK_URL = "https://files.prusa3d.com/wp-content/uploads/repository/PrusaSlicer-settings-master/live/PrusaSlicer.version";

const std::string AppConfig::SECTION_FILAMENTS = "filaments";
const std::string AppConfig::SECTION_MATERIALS = "sla_materials";

void AppConfig::reset()
{
    m_storage.clear();
    set_defaults();
};

// Override missing or keys with their defaults.
void AppConfig::set_defaults()
{
    // Reset the empty fields to defaults.
    if (get("autocenter").empty())
        set("autocenter", "0");
    // Disable background processing by default as it is not stable.
    if (get("background_processing").empty())
        set("background_processing", "0");
    // If set, the "Controller" tab for the control of the printer over serial line and the serial port settings are hidden.
    // By default, Prusa has the controller hidden.
    if (get("no_controller").empty())
        set("no_controller", "1");
    // If set, the "- default -" selections of print/filament/printer are suppressed, if there is a valid preset available.
    if (get("no_defaults").empty())
        set("no_defaults", "1");
    if (get("show_incompatible_presets").empty())
        set("show_incompatible_presets", "0");

    if (get("version_check").empty())
        set("version_check", "1");
    if (get("preset_update").empty())
        set("preset_update", "1");

    // remove old 'use_legacy_opengl' parameter from this config, if present
    if (!get("use_legacy_opengl").empty())
        erase("", "use_legacy_opengl");

#ifdef __APPLE__
    if (get("use_retina_opengl").empty())
        set("use_retina_opengl", "1");
#endif

    if (get("remember_output_path").empty())
        set("remember_output_path", "1");

    if (get("use_custom_toolbar_size").empty())
        set("use_custom_toolbar_size", "0");

    if (get("custom_toolbar_size").empty())
        set("custom_toolbar_size", "100");

    if (get("use_perspective_camera").empty())
        set("use_perspective_camera", "1");

    // Remove legacy window positions/sizes
    erase("", "main_frame_maximized");
    erase("", "main_frame_pos");
    erase("", "main_frame_size");
    erase("", "object_settings_maximized");
    erase("", "object_settings_pos");
    erase("", "object_settings_size");
}

void AppConfig::load()
{
    // 1) Read the complete config file into a boost::property_tree.
    namespace pt = boost::property_tree;
    pt::ptree tree;
    boost::nowide::ifstream ifs(AppConfig::config_path());
    try {
        pt::read_ini(ifs, tree);
    } catch (pt::ptree_error& ex) {
        // Error while parsing config file. We'll customize the error message and rethrow to be displayed.
        throw std::runtime_error(
        	_utf8(L("Error parsing PrusaSlicer config file, it is probably corrupted. "
                    "Try to manually delete the file to recover from the error. Your user profiles will not be affected.")) + 
        	"\n\n" + AppConfig::config_path() + "\n\n" + ex.what());
    }

    // 2) Parse the property_tree, extract the sections and key / value pairs.
    for (const auto &section : tree) {
    	if (section.second.empty()) {
    		// This may be a top level (no section) entry, or an empty section.
    		std::string data = section.second.data();
    		if (! data.empty())
    			// If there is a non-empty data, then it must be a top-level (without a section) config entry.
    			m_storage[""][section.first] = data;
    	} else if (boost::starts_with(section.first, VENDOR_PREFIX)) {
            // This is a vendor section listing enabled model / variants
            const auto vendor_name = section.first.substr(VENDOR_PREFIX.size());
            auto &vendor = m_vendors[vendor_name];
            for (const auto &kvp : section.second) {
                if (! boost::starts_with(kvp.first, MODEL_PREFIX)) { continue; }
                const auto model_name = kvp.first.substr(MODEL_PREFIX.size());
                std::vector<std::string> variants;
                if (! unescape_strings_cstyle(kvp.second.data(), variants)) { continue; }
                for (const auto &variant : variants) {
                    vendor[model_name].insert(variant);
                }
            }
    	} else {
    		// This must be a section name. Read the entries of a section.
    		std::map<std::string, std::string> &storage = m_storage[section.first];
            for (auto &kvp : section.second)
            	storage[kvp.first] = kvp.second.data();
        }
    }

    // Figure out if datadir has legacy presets
    auto ini_ver = Semver::parse(get("version"));
    m_legacy_datadir = false;
    if (ini_ver) {
        m_orig_version = *ini_ver;
        // Make 1.40.0 alphas compare well
        ini_ver->set_metadata(boost::none);
        ini_ver->set_prerelease(boost::none);
        m_legacy_datadir = ini_ver < Semver(1, 40, 0);
    }

    // Override missing or keys with their defaults.
    this->set_defaults();
    m_dirty = false;
}

void AppConfig::save()
{
    // The config is first written to a file with a PID suffix and then moved
    // to avoid race conditions with multiple instances of Slic3r
    const auto path = config_path();
    std::string path_pid = (boost::format("%1%.%2%") % path % get_current_pid()).str();

    boost::nowide::ofstream c;
    c.open(path_pid, std::ios::out | std::ios::trunc);
    c << "# " << Slic3r::header_slic3r_generated() << std::endl;
    // Make sure the "no" category is written first.
    for (const std::pair<std::string, std::string> &kvp : m_storage[""])
        c << kvp.first << " = " << kvp.second << std::endl;
    // Write the other categories.
    for (const auto category : m_storage) {
    	if (category.first.empty())
    		continue;
    	c << std::endl << "[" << category.first << "]" << std::endl;
    	for (const std::pair<std::string, std::string> &kvp : category.second)
	        c << kvp.first << " = " << kvp.second << std::endl;
	}
    // Write vendor sections
    for (const auto &vendor : m_vendors) {
        size_t size_sum = 0;
        for (const auto &model : vendor.second) { size_sum += model.second.size(); }
        if (size_sum == 0) { continue; }

        c << std::endl << "[" << VENDOR_PREFIX << vendor.first << "]" << std::endl;

        for (const auto &model : vendor.second) {
            if (model.second.size() == 0) { continue; }
            const std::vector<std::string> variants(model.second.begin(), model.second.end());
            const auto escaped = escape_strings_cstyle(variants);
            c << MODEL_PREFIX << model.first << " = " << escaped << std::endl;
        }
    }
    c.close();

    rename_file(path_pid, path);
    m_dirty = false;
}

bool AppConfig::get_variant(const std::string &vendor, const std::string &model, const std::string &variant) const
{
    const auto it_v = m_vendors.find(vendor);
    if (it_v == m_vendors.end()) { return false; }
    const auto it_m = it_v->second.find(model);
    return it_m == it_v->second.end() ? false : it_m->second.find(variant) != it_m->second.end();
}

void AppConfig::set_variant(const std::string &vendor, const std::string &model, const std::string &variant, bool enable)
{
    if (enable) {
        if (get_variant(vendor, model, variant)) { return; }
        m_vendors[vendor][model].insert(variant);
    } else {
        auto it_v = m_vendors.find(vendor);
        if (it_v == m_vendors.end()) { return; }
        auto it_m = it_v->second.find(model);
        if (it_m == it_v->second.end()) { return; }
        auto it_var = it_m->second.find(variant);
        if (it_var == it_m->second.end()) { return; }
        it_m->second.erase(it_var);
    }
    // If we got here, there was an update
    m_dirty = true;
}

void AppConfig::set_vendors(const AppConfig &from)
{
    m_vendors = from.m_vendors;
    m_dirty = true;
}

std::string AppConfig::get_last_dir() const
{
    const auto it = m_storage.find("recent");
    if (it != m_storage.end()) {
        {
            const auto it2 = it->second.find("skein_directory");
            if (it2 != it->second.end() && ! it2->second.empty())
                return it2->second;
        }
        {
            const auto it2 = it->second.find("config_directory");
            if (it2 != it->second.end() && ! it2->second.empty())
                return it2->second;
        }
    }
    return std::string();
}

std::vector<std::string> AppConfig::get_recent_projects() const
{
    std::vector<std::string> ret;
    const auto it = m_storage.find("recent_projects");
    if (it != m_storage.end())
    {
        for (const std::map<std::string, std::string>::value_type& item : it->second)
        {
            ret.push_back(item.second);
        }
    }
    return ret;
}

void AppConfig::set_recent_projects(const std::vector<std::string>& recent_projects)
{
    auto it = m_storage.find("recent_projects");
    if (it == m_storage.end())
        it = m_storage.insert(std::map<std::string, std::map<std::string, std::string>>::value_type("recent_projects", std::map<std::string, std::string>())).first;

    it->second.clear();
    for (unsigned int i = 0; i < (unsigned int)recent_projects.size(); ++i)
    {
        it->second[std::to_string(i + 1)] = recent_projects[i];
    }
}

void AppConfig::set_mouse_device(const std::string& name, double translation_speed, double translation_deadzone, float rotation_speed, float rotation_deadzone)
{
    std::string key = std::string("mouse_device:") + name;
    auto it = m_storage.find(key);
    if (it == m_storage.end())
        it = m_storage.insert(std::map<std::string, std::map<std::string, std::string>>::value_type(key, std::map<std::string, std::string>())).first;

    it->second.clear();
    it->second["translation_speed"] = std::to_string(translation_speed);
    it->second["translation_deadzone"] = std::to_string(translation_deadzone);
    it->second["rotation_speed"] = std::to_string(rotation_speed);
    it->second["rotation_deadzone"] = std::to_string(rotation_deadzone);
}

bool AppConfig::get_mouse_device_translation_speed(const std::string& name, double& speed)
{
    std::string key = std::string("mouse_device:") + name;
    auto it = m_storage.find(key);
    if (it == m_storage.end())
        return false;

    auto it_val = it->second.find("translation_speed");
    if (it_val == it->second.end())
        return false;

    speed = ::atof(it_val->second.c_str());
    return true;
}

bool AppConfig::get_mouse_device_translation_deadzone(const std::string& name, double& deadzone)
{
    std::string key = std::string("mouse_device:") + name;
    auto it = m_storage.find(key);
    if (it == m_storage.end())
        return false;

    auto it_val = it->second.find("translation_deadzone");
    if (it_val == it->second.end())
        return false;

    deadzone = ::atof(it_val->second.c_str());
    return true;
}

bool AppConfig::get_mouse_device_rotation_speed(const std::string& name, float& speed)
{
    std::string key = std::string("mouse_device:") + name;
    auto it = m_storage.find(key);
    if (it == m_storage.end())
        return false;

    auto it_val = it->second.find("rotation_speed");
    if (it_val == it->second.end())
        return false;

    speed = (float)::atof(it_val->second.c_str());
    return true;
}

bool AppConfig::get_mouse_device_rotation_deadzone(const std::string& name, float& deadzone)
{
    std::string key = std::string("mouse_device:") + name;
    auto it = m_storage.find(key);
    if (it == m_storage.end())
        return false;

    auto it_val = it->second.find("rotation_deadzone");
    if (it_val == it->second.end())
        return false;

    deadzone = (float)::atof(it_val->second.c_str());
    return true;
}

void AppConfig::update_config_dir(const std::string &dir)
{
    this->set("recent", "config_directory", dir);
}

void AppConfig::update_skein_dir(const std::string &dir)
{
    this->set("recent", "skein_directory", dir);
}

std::string AppConfig::get_last_output_dir(const std::string &alt) const
{
	
    const auto it = m_storage.find("");
    if (it != m_storage.end()) {
        const auto it2 = it->second.find("last_output_path");
        const auto it3 = it->second.find("remember_output_path");
        if (it2 != it->second.end() && it3 != it->second.end() && ! it2->second.empty() && it3->second == "1")
            return it2->second;
    }
    return alt;
}

void AppConfig::update_last_output_dir(const std::string &dir)
{
    this->set("", "last_output_path", dir);
}

void AppConfig::reset_selections()
{
    auto it = m_storage.find("presets");
    if (it != m_storage.end()) {
        it->second.erase("print");
        it->second.erase("filament");
        it->second.erase("sla_print");
        it->second.erase("sla_material");
        it->second.erase("printer");
        m_dirty = true;
    }
}

std::string AppConfig::config_path()
{
	return (boost::filesystem::path(Slic3r::data_dir()) / (SLIC3R_APP_KEY ".ini")).make_preferred().string();
}

std::string AppConfig::version_check_url() const
{
    auto from_settings = get("version_check_url");
    return from_settings.empty() ? VERSION_CHECK_URL : from_settings;
}

bool AppConfig::exists()
{
    return boost::filesystem::exists(AppConfig::config_path());
}

}; // namespace Slic3r
