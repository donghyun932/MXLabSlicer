#ifndef slic3r_OptionsGroup_hpp_
#define slic3r_OptionsGroup_hpp_

#include <wx/stattext.h>
#include <wx/settings.h>

#include <map>
#include <functional>

#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "Field.hpp"
#include "GUI_App.hpp"

// Translate the ifdef 
#ifdef __WXOSX__
    #define wxOSX true
#else
    #define wxOSX false
#endif

#define BORDER(a, b) ((wxOSX ? a : b))

namespace Slic3r { namespace GUI {

/// Widget type describes a function object that returns a wxWindow (our widget) and accepts a wxWidget (parent window).
using widget_t = std::function<wxSizer*(wxWindow*)>;//!std::function<wxWindow*(wxWindow*)>;

//auto default_label_clr = wxSystemSettings::GetColour(wxSYS_COLOUR_3DLIGHT); //GetSystemColour
//auto modified_label_clr = *new wxColour(254, 189, 101);

/// Wraps a ConfigOptionDef and adds function object for creating a side_widget.
struct Option {
	ConfigOptionDef			opt { ConfigOptionDef() };
	t_config_option_key		opt_id;//! {""};
    widget_t				side_widget {nullptr};
    bool					readonly {false};

	Option(const ConfigOptionDef& _opt, t_config_option_key id) :
		opt(_opt), opt_id(id) {}
};
using t_option = std::unique_ptr<Option>;	//!

/// Represents option lines
class Line {
public:
    wxString	label {wxString("")};
    wxString	label_tooltip {wxString("")};
    size_t		full_width {0}; 
    wxSizer*	sizer {nullptr};
    widget_t	widget {nullptr};
    std::function<wxWindow*(wxWindow*)>	near_label_widget{ nullptr };

    void append_option(const Option& option) {
        m_options.push_back(option);
    }
	void append_widget(const widget_t widget) {
		m_extra_widgets.push_back(widget);
    }
	Line(wxString label, wxString tooltip) :
		label(label), label_tooltip(tooltip) {}

    const std::vector<widget_t>&	get_extra_widgets() const {return m_extra_widgets;}
    const std::vector<Option>&		get_options() const { return m_options; }

private:
	std::vector<Option>		m_options;//! {std::vector<Option>()};
    std::vector<widget_t>	m_extra_widgets;//! {std::vector<widget_t>()};
};

using column_t = std::function<wxWindow*(wxWindow* parent, const Line&)>;

using t_optionfield_map = std::map<t_config_option_key, t_field>;
using t_opt_map = std::map< std::string, std::pair<std::string, int> >;

class OptionsGroup {
	wxStaticBox*	stb;
public:
    const bool		staticbox {true};
    const wxString	title {wxString("")};
    size_t			label_width = 20 ;// {200};
    wxSizer*		sizer {nullptr};
    column_t		extra_column {nullptr};
    t_change		m_on_change { nullptr };
	// To be called when the field loses focus, to assign a new initial value to the field.
	// Used by the relative position / rotation / scale manipulation fields of the Object Manipulation UI.
    t_kill_focus    m_fill_empty_value { nullptr };
    t_kill_focus    m_set_focus { nullptr };
	std::function<DynamicPrintConfig()>	m_get_initial_config{ nullptr };
	std::function<DynamicPrintConfig()>	m_get_sys_config{ nullptr };
	std::function<bool()>	have_sys_config{ nullptr };

    std::function<void(wxWindow* win)> rescale_extra_column_item { nullptr };
    std::function<void(wxWindow* win)> rescale_near_label_widget { nullptr };
    
    wxFont			sidetext_font {wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT) };
    wxFont			label_font {wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT) };
	int				sidetext_width{ -1 };

    /// Returns a copy of the pointer of the parent wxWindow.
    /// Accessor function is because users are not allowed to change the parent
    /// but defining it as const means a lot of const_casts to deal with wx functions.
    inline wxWindow* parent() const { 
#if 0//#ifdef __WXGTK__
		return m_panel;
#else
		return m_parent;
#endif /* __WXGTK__ */
    }
#if 0//#ifdef __WXGTK__
    wxWindow* get_parent() const {
        return m_parent;
    }
#endif /* __WXGTK__ */

    wxWindow* ctrl_parent() const {
    	return this->stb ? (wxWindow*)this->stb : this->parent();
    }

	void		append_line(const Line& line, wxStaticText** full_Label = nullptr);
    Line		create_single_option_line(const Option& option) const;
    void		append_single_option_line(const Option& option) { append_line(create_single_option_line(option)); }

    // return a non-owning pointer reference 
    inline Field*	get_field(const t_config_option_key& id) const{
							if (m_fields.find(id) == m_fields.end()) return nullptr;
							return m_fields.at(id).get();
    }
	bool			set_value(const t_config_option_key& id, const boost::any& value, bool change_event = false) {
							if (m_fields.find(id) == m_fields.end()) return false;
							m_fields.at(id)->set_value(value, change_event);
							return true;
    }
	boost::any		get_value(const t_config_option_key& id) {
							boost::any out; 
    						if (m_fields.find(id) == m_fields.end()) ;
							else 
								out = m_fields.at(id)->get_value();
							return out;
    }

	bool			set_side_text(const t_config_option_key& opt_key, const wxString& side_text) {
							if (m_fields.find(opt_key) == m_fields.end()) return false;
							auto st = m_fields.at(opt_key)->m_side_text;
							if (!st) return false;
							st->SetLabel(side_text);
							return true;
    }

	void			set_name(const wxString& new_name) {
							stb->SetLabel(new_name);
    }

	inline void		enable() { for (auto& field : m_fields) field.second->enable(); }
    inline void		disable() { for (auto& field : m_fields) field.second->disable(); }
	void			set_grid_vgap(int gap) { m_grid_sizer->SetVGap(gap); }

	void            set_show_modified_btns_val(bool show) {
		                m_show_modified_btns = show;
    }

    void            clear_fields_except_of(const std::vector<std::string> left_fields);

    void            hide_labels() {
        label_width = 0;
        m_grid_sizer->SetCols(m_grid_sizer->GetEffectiveColsCount()-1);
        static_cast<wxFlexGridSizer*>(m_grid_sizer)->AddGrowableCol(!extra_column ? 0 : 1);
    }

	OptionsGroup(	wxWindow* _parent, const wxString& title, bool is_tab_opt = false, 
					column_t extra_clmn = nullptr) :
					m_parent(_parent), title(title), 
                    m_show_modified_btns(is_tab_opt),
					staticbox(title!=""), extra_column(extra_clmn) {
        if (staticbox) {
            stb = new wxStaticBox(_parent, wxID_ANY, title);
            if (!wxOSX) stb->SetBackgroundStyle(wxBG_STYLE_PAINT);
            stb->SetFont(wxGetApp().bold_font());
        } else
        	stb = nullptr;
        sizer = (staticbox ? new wxStaticBoxSizer(stb, wxVERTICAL) : new wxBoxSizer(wxVERTICAL));
        auto num_columns = 1U;
        if (label_width != 0) num_columns++;
        if (extra_column != nullptr) num_columns++;
        m_grid_sizer = new wxFlexGridSizer(0, num_columns, 1,0);
        static_cast<wxFlexGridSizer*>(m_grid_sizer)->SetFlexibleDirection(wxBOTH/*wxHORIZONTAL*/);
        static_cast<wxFlexGridSizer*>(m_grid_sizer)->AddGrowableCol(label_width == 0 ? 0 : !extra_column ? 1 : 2 );
#if 0//#ifdef __WXGTK__
        m_panel = new wxPanel( _parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
        sizer->Fit(m_panel);
        sizer->Add(m_panel, 0, wxEXPAND | wxALL, wxOSX||!staticbox ? 0: 5);
#else
        sizer->Add(m_grid_sizer, 0, wxEXPAND | wxALL, wxOSX||!staticbox ? 0: 5);
#endif /* __WXGTK__ */
    }

    wxGridSizer*        get_grid_sizer() { return m_grid_sizer; }

protected:
	std::map<t_config_option_key, Option>	m_options;
    wxWindow*				m_parent {nullptr};
    std::vector<ConfigOptionMode>           m_options_mode;
    std::vector<wxWindow*>                  m_extra_column_item_ptrs;
    std::vector<wxWindow*>                  m_near_label_widget_ptrs;

    /// Field list, contains unique_ptrs of the derived type.
    /// using types that need to know what it is beyond the public interface 
    /// need to cast based on the related ConfigOptionDef.
    t_optionfield_map		m_fields;
    bool					m_disabled {false};
    wxGridSizer*			m_grid_sizer {nullptr};
	// "true" if option is created in preset tabs
	bool					m_show_modified_btns{ false };

	// This panel is needed for correct showing of the ToolTips for Button, StaticText and CheckBox
	// Tooltips on GTK doesn't work inside wxStaticBoxSizer unless you insert a panel 
	// inside it before you insert the other controls.
#if 0//#ifdef__WXGTK__
	wxPanel*				m_panel {nullptr};
#endif /* __WXGTK__ */

    /// Generate a wxSizer or wxWindow from a configuration option
    /// Precondition: opt resolves to a known ConfigOption
    /// Postcondition: fields contains a wx gui object.
	const t_field&		build_field(const t_config_option_key& id, const ConfigOptionDef& opt, wxStaticText* label = nullptr);
	const t_field&		build_field(const t_config_option_key& id, wxStaticText* label = nullptr);
	const t_field&		build_field(const Option& opt, wxStaticText* label = nullptr);
	void				add_undo_buttuns_to_sizer(wxSizer* sizer, const t_field& field);

    virtual void		on_kill_focus(const std::string& opt_key) {};
	virtual void		on_set_focus(const std::string& opt_key);
	virtual void		on_change_OG(const t_config_option_key& opt_id, const boost::any& value);
	virtual void		back_to_initial_value(const std::string& opt_key) {}
	virtual void		back_to_sys_value(const std::string& opt_key) {}
};

class ConfigOptionsGroup: public OptionsGroup {
public:
	ConfigOptionsGroup(	wxWindow* parent, const wxString& title, DynamicPrintConfig* _config = nullptr, 
						bool is_tab_opt = false, column_t extra_clmn = nullptr) :
		OptionsGroup(parent, title, is_tab_opt, extra_clmn), m_config(_config) {}

    /// reference to libslic3r config, non-owning pointer (?).
    DynamicPrintConfig*		m_config {nullptr};
    bool					m_full_labels {0};
	t_opt_map				m_opt_map;

    void        set_config(DynamicPrintConfig* config) { m_config = config; }
	Option		get_option(const std::string& opt_key, int opt_index = -1);
	Line		create_single_option_line(const std::string& title, int idx = -1) /*const*/{
		Option option = get_option(title, idx);
		return OptionsGroup::create_single_option_line(option);
	}
	Line		create_single_option_line(const Option& option) const {
		return OptionsGroup::create_single_option_line(option);
	}
	void		append_single_option_line(const Option& option)	{
		OptionsGroup::append_single_option_line(option);
	}
	void		append_single_option_line(const std::string title, int idx = -1)
	{
		Option option = get_option(title, idx);
		append_single_option_line(option);		
	}

	void		on_change_OG(const t_config_option_key& opt_id, const boost::any& value) override;
	void		back_to_initial_value(const std::string& opt_key) override;
	void		back_to_sys_value(const std::string& opt_key) override;
	void		back_to_config_value(const DynamicPrintConfig& config, const std::string& opt_key);
    void		on_kill_focus(const std::string& opt_key) override;
	void		reload_config();
    // return value shows visibility : false => all options are hidden
    void        Hide();
    void        Show(const bool show);
    bool        update_visibility(ConfigOptionMode mode);
    void        msw_rescale();
	boost::any	config_value(const std::string& opt_key, int opt_index, bool deserialize);
	// return option value from config 
	boost::any	get_config_value(const DynamicPrintConfig& config, const std::string& opt_key, int opt_index = -1);
	Field*		get_fieldc(const t_config_option_key& opt_key, int opt_index);
};

//  Static text shown among the options.
class ogStaticText :public wxStaticText{
public:
	ogStaticText() {}
	ogStaticText(wxWindow* parent, const char *text) : wxStaticText(parent, wxID_ANY, text, wxDefaultPosition, wxDefaultSize) {}
	~ogStaticText() {}

	void		SetText(const wxString& value, bool wrap = true);
};

}}

#endif /* slic3r_OptionsGroup_hpp_ */
