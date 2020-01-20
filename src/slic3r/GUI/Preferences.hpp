#ifndef slic3r_Preferences_hpp_
#define slic3r_Preferences_hpp_

#include "GUI.hpp"
#include "GUI_Utils.hpp"

#include <wx/dialog.h>
#include <map>

namespace Slic3r {
namespace GUI {

class ConfigOptionsGroup;

class PreferencesDialog : public DPIDialog
{
	std::map<std::string, std::string>	m_values;
	std::shared_ptr<ConfigOptionsGroup>	m_optgroup;
    wxSizer*                            m_icon_size_sizer;
    bool                                isOSX {false};
public:
	PreferencesDialog(wxWindow* parent);
	~PreferencesDialog() {}

	void	build();
	void	accept();

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;
    void layout();
    void create_icon_size_slider();
};

} // GUI
} // Slic3r


#endif /* slic3r_Preferences_hpp_ */
