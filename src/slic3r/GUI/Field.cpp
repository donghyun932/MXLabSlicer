#include "GUI.hpp"
#include "GUI_App.hpp"
#include "I18N.hpp"
#include "Field.hpp"
#include "wxExtensions.hpp"

#include "libslic3r/PrintConfig.hpp"

#include <regex>
#include <wx/numformatter.h>
#include <wx/tooltip.h>
#include <boost/algorithm/string/predicate.hpp>

#ifdef __WXOSX__
#define wxOSX true
#else
#define wxOSX false
#endif

namespace Slic3r { namespace GUI {

wxString double_to_string(double const value, const int max_precision /*= 4*/)
{
// Style_NoTrailingZeroes does not work on OSX. It also does not work correctly with some locales on Windows.
//	return wxNumberFormatter::ToString(value, max_precision, wxNumberFormatter::Style_NoTrailingZeroes);

	wxString s = wxNumberFormatter::ToString(value, max_precision, wxNumberFormatter::Style_None);

	// The following code comes from wxNumberFormatter::RemoveTrailingZeroes(wxString& s)
	// with the exception that here one sets the decimal separator explicitely to dot.
    // If number is in scientific format, trailing zeroes belong to the exponent and cannot be removed.
    if (s.find_first_of("eE") == wxString::npos) {
	    const size_t posDecSep = s.find(".");
	    // No decimal point => removing trailing zeroes irrelevant for integer number.
	    if (posDecSep != wxString::npos) {
		    // Find the last character to keep.
		    size_t posLastNonZero = s.find_last_not_of("0");
		    // If it's the decimal separator itself, don't keep it neither.
		    if (posLastNonZero == posDecSep)
		        -- posLastNonZero;
		    s.erase(posLastNonZero + 1);
		    // Remove sign from orphaned zero.
		    if (s.compare("-0") == 0)
		        s = "0";
		}
	}

    return s;
}

void Field::PostInitialize()
{
	auto color = wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW);
	m_Undo_btn			= new RevertButton(m_parent, "bullet_white.png");
	m_Undo_to_sys_btn	= new RevertButton(m_parent, "bullet_white.png");

    m_Undo_btn->Bind(wxEVT_BUTTON, ([this](wxCommandEvent) { on_back_to_initial_value(); }));
	m_Undo_to_sys_btn->Bind(wxEVT_BUTTON, ([this](wxCommandEvent) { on_back_to_sys_value(); }));

	switch (m_opt.type)
	{
	case coPercents:
	case coFloats:
	case coStrings:	
	case coBools:		
	case coInts: {
		auto tag_pos = m_opt_id.find("#");
		if (tag_pos != std::string::npos)
			m_opt_idx = stoi(m_opt_id.substr(tag_pos + 1, m_opt_id.size()));
		break;
	}
	default:
		break;
	}

    // initialize m_unit_value
    m_em_unit = em_unit(m_parent);

	BUILD();
}

void Field::on_kill_focus()
{
	// call the registered function if it is available
    if (m_on_kill_focus!=nullptr) 
        m_on_kill_focus(m_opt_id);
}

void Field::on_set_focus(wxEvent& event)
{
    // to allow the default behavior
	event.Skip();
	// call the registered function if it is available
    if (m_on_set_focus!=nullptr) 
        m_on_set_focus(m_opt_id);
}

void Field::on_change_field()
{
//       std::cerr << "calling Field::_on_change \n";
    if (m_on_change != nullptr  && !m_disable_change_event)
        m_on_change(m_opt_id, get_value());
}

void Field::on_back_to_initial_value()
{
	if (m_back_to_initial_value != nullptr && m_is_modified_value)
		m_back_to_initial_value(m_opt_id);
}

void Field::on_back_to_sys_value()
{
	if (m_back_to_sys_value != nullptr && m_is_nonsys_value)
		m_back_to_sys_value(m_opt_id);
}

wxString Field::get_tooltip_text(const wxString& default_string)
{
	wxString tooltip_text("");
	wxString tooltip = _(m_opt.tooltip);
    edit_tooltip(tooltip);

    std::string opt_id = m_opt_id;
    auto hash_pos = opt_id.find("#");
    if (hash_pos != std::string::npos) {
        opt_id.replace(hash_pos, 1,"[");
        opt_id += "]";
    }

	if (tooltip.length() > 0)
        tooltip_text = tooltip + "\n" + _(L("default value")) + "\t: " +
        (boost::iends_with(opt_id, "_gcode") ? "\n" : "") + default_string +
        (boost::iends_with(opt_id, "_gcode") ? "" : "\n") + 
        _(L("parameter name")) + "\t: " + opt_id;

	return tooltip_text;
}

bool Field::is_matched(const std::string& string, const std::string& pattern)
{
	std::regex regex_pattern(pattern, std::regex_constants::icase); // use ::icase to make the matching case insensitive like /i in perl
	return std::regex_match(string, regex_pattern);
}

static wxString na_value() { return _(L("N/A")); }

void Field::get_value_by_opt_type(wxString& str, const bool check_value/* = true*/)
{
	switch (m_opt.type) {
	case coInt:
		m_value = wxAtoi(str);
		break;
	case coPercent:
	case coPercents:
	case coFloats:
	case coFloat:{
		if (m_opt.type == coPercent && !str.IsEmpty() &&  str.Last() == '%') 
			str.RemoveLast();
		else if (!str.IsEmpty() && str.Last() == '%')
        {
            if (!check_value) {
                m_value.clear();
                break;
            }

			wxString label = m_Label->GetLabel();
			if		(label.Last() == '\n')	label.RemoveLast();
			while	(label.Last() == ' ')	label.RemoveLast();
			if		(label.Last() == ':')	label.RemoveLast();
			show_error(m_parent, wxString::Format(_(L("%s doesn't support percentage")), label));
			set_value(double_to_string(m_opt.min), true);
			m_value = double(m_opt.min);
			break;
		}
		double val;
		// Replace the first occurence of comma in decimal number.
		str.Replace(",", ".", false);
        if (str == ".")
            val = 0.0;
        else
        {
            if (m_opt.nullable && str == na_value())
                val = ConfigOptionFloatsNullable::nil_value();
            else if (!str.ToCDouble(&val))
            {
                if (!check_value) {
                    m_value.clear();
                    break;
                }
                show_error(m_parent, _(L("Invalid numeric input.")));
                set_value(double_to_string(val), true);
            }
            if (m_opt.min > val || val > m_opt.max)
            {
                if (!check_value) {
                    m_value.clear();
                    break;
                }
                show_error(m_parent, _(L("Input value is out of range")));
                if (m_opt.min > val) val = m_opt.min;
                if (val > m_opt.max) val = m_opt.max;
                set_value(double_to_string(val), true);
            }
        }
        m_value = val;
		break; }
	case coString:
	case coStrings:
    case coFloatOrPercent: {
        if (m_opt.type == coFloatOrPercent && !str.IsEmpty() &&  str.Last() != '%')
        {
            double val = 0.;
			// Replace the first occurence of comma in decimal number.
			str.Replace(",", ".", false);
            if (!str.ToCDouble(&val))
            {
                if (!check_value) {
                    m_value.clear();
                    break;
                }
                show_error(m_parent, _(L("Invalid numeric input.")));
                set_value(double_to_string(val), true);
            }
            else if (((m_opt.sidetext.rfind("mm/s") != std::string::npos && val > m_opt.max) ||
                     (m_opt.sidetext.rfind("mm ") != std::string::npos && val > 1)) &&
                     (m_value.empty() || std::string(str.ToUTF8().data()) != boost::any_cast<std::string>(m_value)))
            {
                if (!check_value) {
                    m_value.clear();
                    break;
                }

                const std::string sidetext = m_opt.sidetext.rfind("mm/s") != std::string::npos ? "mm/s" : "mm";
                const wxString stVal = double_to_string(val, 2);
                const wxString msg_text = wxString::Format(_(L("Do you mean %s%% instead of %s %s?\n"
                    "Select YES if you want to change this value to %s%%, \n"
                    "or NO if you are sure that %s %s is a correct value.")), stVal, stVal, sidetext, stVal, stVal, sidetext);
                wxMessageDialog dialog(m_parent, msg_text, _(L("Parameter validation")) + ": " + m_opt_id , wxICON_WARNING | wxYES | wxNO);
                if (dialog.ShowModal() == wxID_YES) {
                    set_value(wxString::Format("%s%%", stVal), false/*true*/);
                    str += "%%";
                }
            }
        }
    
        m_value = std::string(str.ToUTF8().data());
		break; }
	default:
		break;
	}
}

template<class T>
bool is_defined_input_value(wxWindow* win, const ConfigOptionType& type)
{
    if (static_cast<T*>(win)->GetValue().empty() && type != coString && type != coStrings)
        return false;
    return true;
}

void TextCtrl::BUILD() {
    auto size = wxSize(wxDefaultSize);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

	wxString text_value = wxString(""); 

	switch (m_opt.type) {
	case coFloatOrPercent:
	{
		text_value = double_to_string(m_opt.default_value->getFloat());
		if (m_opt.get_default_value<ConfigOptionFloatOrPercent>()->percent)
			text_value += "%";
		break;
	}
	case coPercent:
	{
		text_value = wxString::Format(_T("%i"), int(m_opt.default_value->getFloat()));
		text_value += "%";
		break;
	}	
	case coPercents:
	case coFloats:
	case coFloat:
	{
		double val = m_opt.type == coFloats ?
			m_opt.get_default_value<ConfigOptionFloats>()->get_at(m_opt_idx) :
			m_opt.type == coFloat ? 
				m_opt.default_value->getFloat() :
				m_opt.get_default_value<ConfigOptionPercents>()->get_at(m_opt_idx);
		text_value = double_to_string(val);
        m_last_meaningful_value = text_value;
		break;
	}
	case coString:			
		text_value = m_opt.get_default_value<ConfigOptionString>()->value;
		break;
	case coStrings:
	{
		const ConfigOptionStrings *vec = m_opt.get_default_value<ConfigOptionStrings>();
		if (vec == nullptr || vec->empty()) break; //for the case of empty default value
		text_value = vec->get_at(m_opt_idx);
		break;
	}
	default:
		break; 
	}

    const long style = m_opt.multiline ? wxTE_MULTILINE : wxTE_PROCESS_ENTER/*0*/;
	auto temp = new wxTextCtrl(m_parent, wxID_ANY, text_value, wxDefaultPosition, size, style);
	temp->SetFont(Slic3r::GUI::wxGetApp().normal_font());

    if (! m_opt.multiline && !wxOSX)
		// Only disable background refresh for single line input fields, as they are completely painted over by the edit control.
		// This does not apply to the multi-line edit field, where the last line and a narrow frame around the text is not cleared.
		temp->SetBackgroundStyle(wxBG_STYLE_PAINT);
#ifdef __WXOSX__
    temp->OSXDisableAllSmartSubstitutions();
#endif // __WXOSX__

	temp->SetToolTip(get_tooltip_text(text_value));

    if (style == wxTE_PROCESS_ENTER) {
        temp->Bind(wxEVT_TEXT_ENTER, ([this, temp](wxEvent& e)
        {
#if !defined(__WXGTK__)
            e.Skip();
            temp->GetToolTip()->Enable(true);
#endif // __WXGTK__
            bEnterPressed = true;
            propagate_value();
        }), temp->GetId());
    }

    temp->Bind(wxEVT_SET_FOCUS, ([this](wxEvent& e) { on_set_focus(e); }), temp->GetId());
    
	temp->Bind(wxEVT_LEFT_DOWN, ([temp](wxEvent& event)
	{
		//! to allow the default handling
		event.Skip();
		//! eliminating the g-code pop up text description
		bool flag = false;
#ifdef __WXGTK__
		// I have no idea why, but on GTK flag works in other way
		flag = true;
#endif // __WXGTK__
		temp->GetToolTip()->Enable(flag);
	}), temp->GetId());

	temp->Bind(wxEVT_KILL_FOCUS, ([this, temp](wxEvent& e)
	{
		e.Skip();
#if !defined(__WXGTK__)
		temp->GetToolTip()->Enable(true);
#endif // __WXGTK__
        if (bEnterPressed) {
            bEnterPressed = false;
            return;
        }
        propagate_value();
	}), temp->GetId());

	// select all text using Ctrl+A
	temp->Bind(wxEVT_CHAR, ([temp](wxKeyEvent& event)
	{
		if (wxGetKeyState(wxKeyCode('A')) && wxGetKeyState(WXK_CONTROL))
			temp->SetSelection(-1, -1); //select all
		event.Skip();
	}));

    // recast as a wxWindow to fit the calling convention
    window = dynamic_cast<wxWindow*>(temp);
}	

bool TextCtrl::value_was_changed()
{
    if (m_value.empty())
        return true;

    boost::any val = m_value;
    wxString ret_str = static_cast<wxTextCtrl*>(window)->GetValue();
    // update m_value!
    // ret_str might be changed inside get_value_by_opt_type
    get_value_by_opt_type(ret_str);

    switch (m_opt.type) {
    case coInt:
        return boost::any_cast<int>(m_value) != boost::any_cast<int>(val);
    case coPercent:
    case coPercents:
    case coFloats:
    case coFloat: {
        if (m_opt.nullable && std::isnan(boost::any_cast<double>(m_value)) && 
                              std::isnan(boost::any_cast<double>(val)))
            return false;
        return boost::any_cast<double>(m_value) != boost::any_cast<double>(val);
    }
    case coString:
    case coStrings:
    case coFloatOrPercent:
        return boost::any_cast<std::string>(m_value) != boost::any_cast<std::string>(val);
    default:
        return true;
    }
}

void TextCtrl::propagate_value()
{
    if (is_defined_input_value<wxTextCtrl>(window, m_opt.type) && value_was_changed())
        on_change_field();
    else
        on_kill_focus();
}

void TextCtrl::set_value(const boost::any& value, bool change_event/* = false*/) {
    m_disable_change_event = !change_event;
    if (m_opt.nullable) {
        const bool m_is_na_val = boost::any_cast<wxString>(value) == na_value();
        if (!m_is_na_val)
            m_last_meaningful_value = value;
        dynamic_cast<wxTextCtrl*>(window)->SetValue(m_is_na_val ? na_value() : boost::any_cast<wxString>(value));
    }
    else
        dynamic_cast<wxTextCtrl*>(window)->SetValue(boost::any_cast<wxString>(value));
    m_disable_change_event = false;

    if (!change_event) {
        wxString ret_str = static_cast<wxTextCtrl*>(window)->GetValue();
        /* Update m_value to correct work of next value_was_changed(). 
         * But after checking of entered value, don't fix the "incorrect" value and don't show a warning message, 
         * just clear m_value in this case. 
         */
        get_value_by_opt_type(ret_str, false);
    }
}

void TextCtrl::set_last_meaningful_value()
{
    dynamic_cast<wxTextCtrl*>(window)->SetValue(boost::any_cast<wxString>(m_last_meaningful_value));
    propagate_value();
}

void TextCtrl::set_na_value()
{
    dynamic_cast<wxTextCtrl*>(window)->SetValue(na_value());
    propagate_value();
}

boost::any& TextCtrl::get_value()
{
	wxString ret_str = static_cast<wxTextCtrl*>(window)->GetValue();
	// update m_value
	get_value_by_opt_type(ret_str);

	return m_value;
}

void TextCtrl::msw_rescale()
{
    Field::msw_rescale();
    auto size = wxSize(wxDefaultSize);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

    if (size != wxDefaultSize)
    {
        wxTextCtrl* field = dynamic_cast<wxTextCtrl*>(window);
        field->SetMinSize(size);
    }

}

void TextCtrl::enable() { dynamic_cast<wxTextCtrl*>(window)->Enable(); dynamic_cast<wxTextCtrl*>(window)->SetEditable(true); }
void TextCtrl::disable() { dynamic_cast<wxTextCtrl*>(window)->Disable(); dynamic_cast<wxTextCtrl*>(window)->SetEditable(false); }

#ifdef __WXGTK__
void TextCtrl::change_field_value(wxEvent& event)
{
	if (bChangedValueEvent = (event.GetEventType()==wxEVT_KEY_UP))
		on_change_field();
    event.Skip();
};
#endif //__WXGTK__

void CheckBox::BUILD() {
	auto size = wxSize(wxDefaultSize);
	if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
	if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

	bool check_value =	m_opt.type == coBool ? 
						m_opt.default_value->getBool() : m_opt.type == coBools ? 
							m_opt.get_default_value<ConfigOptionBools>()->get_at(m_opt_idx) : 
    						false;

    m_last_meaningful_value = static_cast<unsigned char>(check_value);

	// Set Label as a string of at least one space simbol to correct system scaling of a CheckBox 
	auto temp = new wxCheckBox(m_parent, wxID_ANY, wxString(" "), wxDefaultPosition, size); 
	temp->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	if (!wxOSX) temp->SetBackgroundStyle(wxBG_STYLE_PAINT);
	temp->SetValue(check_value);
	if (m_opt.readonly) temp->Disable();

	temp->Bind(wxEVT_CHECKBOX, ([this](wxCommandEvent e) {
        m_is_na_val = false;
	    on_change_field();
	}), temp->GetId());

	temp->SetToolTip(get_tooltip_text(check_value ? "true" : "false")); 

	// recast as a wxWindow to fit the calling convention
	window = dynamic_cast<wxWindow*>(temp);
}

void CheckBox::set_value(const boost::any& value, bool change_event)
{
    m_disable_change_event = !change_event;
    if (m_opt.nullable) {
        m_is_na_val = boost::any_cast<unsigned char>(value) == ConfigOptionBoolsNullable::nil_value();
        if (!m_is_na_val)
            m_last_meaningful_value = value;
        dynamic_cast<wxCheckBox*>(window)->SetValue(m_is_na_val ? false : boost::any_cast<unsigned char>(value) != 0);
    }
    else
        dynamic_cast<wxCheckBox*>(window)->SetValue(boost::any_cast<bool>(value));
    m_disable_change_event = false;
}

void CheckBox::set_last_meaningful_value()
{
    if (m_opt.nullable) {
        m_is_na_val = false;
        dynamic_cast<wxCheckBox*>(window)->SetValue(boost::any_cast<unsigned char>(m_last_meaningful_value) != 0);
        on_change_field();
    }
}

void CheckBox::set_na_value()
{
    if (m_opt.nullable) {
        m_is_na_val = true;
        dynamic_cast<wxCheckBox*>(window)->SetValue(false);
        on_change_field();
    }
}

boost::any& CheckBox::get_value()
{
// 	boost::any m_value;
	bool value = dynamic_cast<wxCheckBox*>(window)->GetValue();
	if (m_opt.type == coBool)
		m_value = static_cast<bool>(value);
	else
		m_value = m_is_na_val ? ConfigOptionBoolsNullable::nil_value() : static_cast<unsigned char>(value);
 	return m_value;
}

void CheckBox::msw_rescale()
{
    Field::msw_rescale();

    wxCheckBox* field = dynamic_cast<wxCheckBox*>(window);
    field->SetMinSize(wxSize(-1, int(1.5f*field->GetFont().GetPixelSize().y +0.5f)));
}


void SpinCtrl::BUILD() {
	auto size = wxSize(wxDefaultSize);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

	wxString	text_value = wxString("");
	int			default_value = 0;

	switch (m_opt.type) {
	case coInt:
		default_value = m_opt.default_value->getInt();
		text_value = wxString::Format(_T("%i"), default_value);
		break;
	case coInts:
	{
		const ConfigOptionInts *vec = m_opt.get_default_value<ConfigOptionInts>();
		if (vec == nullptr || vec->empty()) break;
		for (size_t id = 0; id < vec->size(); ++id)
		{
			default_value = vec->get_at(id);
			text_value += wxString::Format(_T("%i"), default_value);
		}
		break;
	}
	default:
		break;
	}

    const int min_val = m_opt.min == INT_MIN 
#ifdef __WXOSX__
    // We will forcibly set the input value for SpinControl, since the value 
    // inserted from the keyboard is not updated under OSX.
    // So, we can't set min control value bigger then 0.
    // Otherwise, it couldn't be possible to input from keyboard value 
    // less then min_val.
    || m_opt.min > 0 
#endif
    ? 0 : m_opt.min;
	const int max_val = m_opt.max < 2147483647 ? m_opt.max : 2147483647;

	auto temp = new wxSpinCtrl(m_parent, wxID_ANY, text_value, wxDefaultPosition, size,
		0|wxTE_PROCESS_ENTER, min_val, max_val, default_value);
	temp->SetFont(Slic3r::GUI::wxGetApp().normal_font());
    if (!wxOSX) temp->SetBackgroundStyle(wxBG_STYLE_PAINT);

// XXX: On OS X the wxSpinCtrl widget is made up of two subwidgets, unfortunatelly
// the kill focus event is not propagated to the encompassing widget,
// so we need to bind it on the inner text widget instead. (Ugh.)
#ifdef __WXOSX__
	temp->GetText()->Bind(wxEVT_KILL_FOCUS, ([this](wxEvent& e)
#else
	temp->Bind(wxEVT_KILL_FOCUS, ([this](wxEvent& e)
#endif
	{
        e.Skip();
        if (bEnterPressed) {
            bEnterPressed = false;
            return;
        }

        propagate_value();
	}));

    temp->Bind(wxEVT_SPINCTRL, ([this](wxCommandEvent e) {  propagate_value();  }), temp->GetId()); 
    
    temp->Bind(wxEVT_TEXT_ENTER, ([this](wxCommandEvent e)
    {
        e.Skip();
        propagate_value();
        bEnterPressed = true;
    }), temp->GetId());

	temp->Bind(wxEVT_TEXT, ([this](wxCommandEvent e)
	{
// 		# On OSX / Cocoa, wxSpinCtrl::GetValue() doesn't return the new value
// 		# when it was changed from the text control, so the on_change callback
// 		# gets the old one, and on_kill_focus resets the control to the old value.
// 		# As a workaround, we get the new value from $event->GetString and store
// 		# here temporarily so that we can return it from get_value()

		long value;
		const bool parsed = e.GetString().ToLong(&value);
		tmp_value = parsed && value >= INT_MIN && value <= INT_MAX ? (int)value : UNDEF_VALUE;

#ifdef __WXOSX__
        // Forcibly set the input value for SpinControl, since the value 
	    // inserted from the keyboard or clipboard is not updated under OSX
        if (tmp_value != UNDEF_VALUE) {
            wxSpinCtrl* spin = static_cast<wxSpinCtrl*>(window);
            spin->SetValue(tmp_value);

            // But in SetValue() is executed m_text_ctrl->SelectAll(), so
            // discard this selection and set insertion point to the end of string
            spin->GetText()->SetInsertionPointEnd();
        }
#endif
	}), temp->GetId());
	
	temp->SetToolTip(get_tooltip_text(text_value));

	// recast as a wxWindow to fit the calling convention
	window = dynamic_cast<wxWindow*>(temp);
}

void SpinCtrl::propagate_value()
{
    if (suppress_propagation)
        return;

    suppress_propagation = true;
    if (tmp_value == UNDEF_VALUE) {
        on_kill_focus();
	} else {
#ifdef __WXOSX__
        // check input value for minimum
        if (m_opt.min > 0 && tmp_value < m_opt.min) {
            wxSpinCtrl* spin = static_cast<wxSpinCtrl*>(window);
            spin->SetValue(m_opt.min);
            spin->GetText()->SetInsertionPointEnd();
        }
#endif
        on_change_field();
    }
    suppress_propagation = false;
}

void SpinCtrl::msw_rescale()
{
    Field::msw_rescale();

    wxSpinCtrl* field = dynamic_cast<wxSpinCtrl*>(window);
    field->SetMinSize(wxSize(-1, int(1.9f*field->GetFont().GetPixelSize().y)));
}

void Choice::BUILD() {
    wxSize size(m_width * m_em_unit, -1);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

	wxBitmapComboBox* temp;	
    if (!m_opt.gui_type.empty() && m_opt.gui_type.compare("select_open") != 0) {
        m_is_editable = true;
        temp = new wxBitmapComboBox(m_parent, wxID_ANY, wxString(""), wxDefaultPosition, size);
    }
    else {
#ifdef __WXOSX__
        /* wxBitmapComboBox with wxCB_READONLY style return NULL for GetTextCtrl(),
         * so ToolTip doesn't shown.
         * Next workaround helps to solve this problem
         */
        temp = new wxBitmapComboBox();
        temp->SetTextCtrlStyle(wxTE_READONLY);
        temp->Create(m_parent, wxID_ANY, wxString(""), wxDefaultPosition, size, 0, nullptr);
#else
        temp = new wxBitmapComboBox(m_parent, wxID_ANY, wxString(""), wxDefaultPosition, size, 0, nullptr, wxCB_READONLY);
#endif //__WXOSX__
    }

	temp->SetFont(Slic3r::GUI::wxGetApp().normal_font());
    if (!wxOSX) temp->SetBackgroundStyle(wxBG_STYLE_PAINT);

	// recast as a wxWindow to fit the calling convention
	window = dynamic_cast<wxWindow*>(temp);

	if (! m_opt.enum_labels.empty() || ! m_opt.enum_values.empty()) {
		if (m_opt.enum_labels.empty()) {
			// Append non-localized enum_values
			for (auto el : m_opt.enum_values)
				temp->Append(el);
		} else {
			// Append localized enum_labels
			for (auto el : m_opt.enum_labels)
				temp->Append(_(el));
		}
		set_selection();
	}

#ifndef __WXGTK__
    /* Workaround for a correct rendering of the control without Bitmap (under MSW and OSX):
     * 
     * 1. We should create small Bitmap to fill Bitmaps RefData,
     *    ! in this case wxBitmap.IsOK() return true.
     * 2. But then set width to 0 value for no using of bitmap left and right spacing 
     * 3. Set this empty bitmap to the at list one item and BitmapCombobox will be recreated correct
     * 
     * Note: Set bitmap height to the Font size because of OSX rendering.
     */
    wxBitmap empty_bmp(1, temp->GetFont().GetPixelSize().y + 2);
    empty_bmp.SetWidth(0);
    temp->SetItemBitmap(0, empty_bmp);
#endif

// 	temp->Bind(wxEVT_TEXT, ([this](wxCommandEvent e) { on_change_field(); }), temp->GetId());
 	temp->Bind(wxEVT_COMBOBOX, ([this](wxCommandEvent e) { on_change_field(); }), temp->GetId());

    if (m_is_editable) {
        temp->Bind(wxEVT_KILL_FOCUS, ([this](wxEvent& e) {
            e.Skip();
            if (m_opt.type == coStrings) {
                on_change_field();
                return;
            }

            double old_val = !m_value.empty() ? boost::any_cast<double>(m_value) : -99999;
            if (is_defined_input_value<wxBitmapComboBox>(window, m_opt.type)) {
                if (fabs(old_val - boost::any_cast<double>(get_value())) <= 0.0001)
                    return;
                else
                    on_change_field();
            }
            else
                on_kill_focus();
        }), temp->GetId());
    }

	temp->SetToolTip(get_tooltip_text(temp->GetValue()));
}

void Choice::set_selection()
{
    /* To prevent earlier control updating under OSX set m_disable_change_event to true
     * (under OSX wxBitmapComboBox send wxEVT_COMBOBOX even after SetSelection())
     */
    m_disable_change_event = true;

	wxString text_value = wxString("");

    wxBitmapComboBox* field = dynamic_cast<wxBitmapComboBox*>(window);
	switch (m_opt.type) {
	case coFloat:
	case coPercent:	{
		double val = m_opt.default_value->getFloat();
		text_value = val - int(val) == 0 ? wxString::Format(_T("%i"), int(val)) : wxNumberFormatter::ToString(val, 1);
		size_t idx = 0;
		for (auto el : m_opt.enum_values)
		{
			if (el == text_value)
				break;
			++idx;
		}
//		if (m_opt.type == coPercent) text_value += "%";
		idx == m_opt.enum_values.size() ?
			field->SetValue(text_value) :
			field->SetSelection(idx);
		break;
	}
	case coEnum:{
		int id_value = m_opt.get_default_value<ConfigOptionEnum<SeamPosition>>()->value; //!!
        field->SetSelection(id_value);
		break;
	}
	case coInt:{
		int val = m_opt.default_value->getInt(); //!!
		text_value = wxString::Format(_T("%i"), int(val));
		size_t idx = 0;
		for (auto el : m_opt.enum_values)
		{
			if (el == text_value)
				break;
			++idx;
		}
		idx == m_opt.enum_values.size() ?
			field->SetValue(text_value) :
			field->SetSelection(idx);
		break;
	}
	case coStrings:{
		text_value = m_opt.get_default_value<ConfigOptionStrings>()->get_at(m_opt_idx);

		size_t idx = 0;
		for (auto el : m_opt.enum_values)
		{
			if (el == text_value)
				break;
			++idx;
		}
		idx == m_opt.enum_values.size() ?
			field->SetValue(text_value) :
			field->SetSelection(idx);
		break;
	}
    default: break;
	}
}

void Choice::set_value(const std::string& value, bool change_event)  //! Redundant?
{
	m_disable_change_event = !change_event;

	size_t idx=0;
	for (auto el : m_opt.enum_values)
	{
		if (el == value)
			break;
		++idx;
	}

    wxBitmapComboBox* field = dynamic_cast<wxBitmapComboBox*>(window);
	idx == m_opt.enum_values.size() ? 
		field->SetValue(value) :
		field->SetSelection(idx);
	
	m_disable_change_event = false;
}

void Choice::set_value(const boost::any& value, bool change_event)
{
	m_disable_change_event = !change_event;

    wxBitmapComboBox* field = dynamic_cast<wxBitmapComboBox*>(window);

	switch (m_opt.type) {
	case coInt:
	case coFloat:
	case coPercent:
	case coString:
	case coStrings: {
		wxString text_value;
		if (m_opt.type == coInt) 
			text_value = wxString::Format(_T("%i"), int(boost::any_cast<int>(value)));
		else
			text_value = boost::any_cast<wxString>(value);
        size_t idx = 0;
		for (auto el : m_opt.enum_values)
		{
			if (el == text_value)
				break;
			++idx;
		}
        if (idx == m_opt.enum_values.size()) {
            // For editable Combobox under OSX is needed to set selection to -1 explicitly,
            // otherwise selection doesn't be changed
            field->SetSelection(-1);
            field->SetValue(text_value);
        }
        else
			field->SetSelection(idx);
		break;
	}
	case coEnum: {
		int val = boost::any_cast<int>(value);
		if (m_opt_id == "top_fill_pattern" || m_opt_id == "bottom_fill_pattern")
		{
			if (!m_opt.enum_values.empty()) {
				std::string key;
				t_config_enum_values map_names = ConfigOptionEnum<InfillPattern>::get_enum_values();				
				for (auto it : map_names) {
					if (val == it.second) {
						key = it.first;
						break;
					}
				}

				size_t idx = 0;
				for (auto el : m_opt.enum_values)
				{
					if (el == key)
						break;
					++idx;
				}

				val = idx == m_opt.enum_values.size() ? 0 : idx;
			}
			else
				val = 0;
		}
		field->SetSelection(val);
		break;
	}
	default:
		break;
	}

	m_disable_change_event = false;
}

//! it's needed for _update_serial_ports()
void Choice::set_values(const std::vector<std::string>& values)
{
	if (values.empty())
		return;
	m_disable_change_event = true;

// 	# it looks that Clear() also clears the text field in recent wxWidgets versions,
// 	# but we want to preserve it
	auto ww = dynamic_cast<wxBitmapComboBox*>(window);
	auto value = ww->GetValue();
	ww->Clear();
	ww->Append("");
	for (auto el : values)
		ww->Append(wxString(el));
	ww->SetValue(value);

	m_disable_change_event = false;
}

boost::any& Choice::get_value()
{
    wxBitmapComboBox* field = dynamic_cast<wxBitmapComboBox*>(window);

	wxString ret_str = field->GetValue();	

	// options from right panel
	std::vector <std::string> right_panel_options{ "support", "pad", "scale_unit" };
	for (auto rp_option: right_panel_options)
		if (m_opt_id == rp_option)
			return m_value = boost::any(ret_str);

	if (m_opt.type == coEnum)
	{
		int ret_enum = field->GetSelection(); 
		if (m_opt_id == "top_fill_pattern" || m_opt_id == "bottom_fill_pattern")
		{
			if (!m_opt.enum_values.empty()) {
				std::string key = m_opt.enum_values[ret_enum];
				t_config_enum_values map_names = ConfigOptionEnum<InfillPattern>::get_enum_values();
				int value = map_names.at(key);

				m_value = static_cast<InfillPattern>(value);
			}
			else
				m_value = static_cast<InfillPattern>(0);
		}
		if (m_opt_id.compare("fill_pattern") == 0)
			m_value = static_cast<InfillPattern>(ret_enum);
		else if (m_opt_id.compare("gcode_flavor") == 0)
			m_value = static_cast<GCodeFlavor>(ret_enum);
		else if (m_opt_id.compare("support_material_pattern") == 0)
			m_value = static_cast<SupportMaterialPattern>(ret_enum);
		else if (m_opt_id.compare("seam_position") == 0)
			m_value = static_cast<SeamPosition>(ret_enum);
		else if (m_opt_id.compare("host_type") == 0)
			m_value = static_cast<PrintHostType>(ret_enum);
		else if (m_opt_id.compare("display_orientation") == 0)
			m_value = static_cast<SLADisplayOrientation>(ret_enum);
        else if (m_opt_id.compare("support_pillar_connection_mode") == 0)
            m_value = static_cast<SLAPillarConnectionMode>(ret_enum);
	}
    else if (m_opt.gui_type == "f_enum_open") {
        const int ret_enum = field->GetSelection();
        if (ret_enum < 0 || m_opt.enum_values.empty() || m_opt.type == coStrings ||
            (ret_str != m_opt.enum_values[ret_enum] && ret_str != _(m_opt.enum_labels[ret_enum])))
			// modifies ret_string!
            get_value_by_opt_type(ret_str);
        else 
            m_value = atof(m_opt.enum_values[ret_enum].c_str());
    }
	else	
		// modifies ret_string!
        get_value_by_opt_type(ret_str);

	return m_value;
}

void Choice::msw_rescale()
{
    Field::msw_rescale();

    wxBitmapComboBox* field = dynamic_cast<wxBitmapComboBox*>(window);

    const wxString selection = field->GetString(field->GetSelection());

	/* To correct scaling (set new controll size) of a wxBitmapCombobox 
	 * we need to refill control with new bitmaps. So, in our case : 
	 * 1. clear conrol
	 * 2. add content
	 * 3. add scaled "empty" bitmap to the at least one item
	 */
    field->Clear();
    wxSize size(wxDefaultSize);
    size.SetWidth((m_opt.width > 0 ? m_opt.width : m_width) * m_em_unit);
 
    // Set rescaled min height to correct layout
    field->SetMinSize(wxSize(-1, int(1.5f*field->GetFont().GetPixelSize().y + 0.5f)));
    // Set rescaled size
    field->SetSize(size);

    size_t idx = 0;
    if (! m_opt.enum_labels.empty() || ! m_opt.enum_values.empty()) {
    	size_t counter = 0;
    	bool   labels = ! m_opt.enum_labels.empty();
        for (const std::string &el : labels ? m_opt.enum_labels : m_opt.enum_values) {
        	wxString text = labels ? _(el) : wxString::FromUTF8(el.c_str());
            field->Append(text);
            if (text == selection)
                idx = counter;
            ++ counter;
        }
    }

    wxBitmap empty_bmp(1, field->GetFont().GetPixelSize().y + 2);
    empty_bmp.SetWidth(0);
    field->SetItemBitmap(0, empty_bmp);

    idx == m_opt.enum_values.size() ?
        field->SetValue(selection) :
        field->SetSelection(idx);
}

void ColourPicker::BUILD()
{
	auto size = wxSize(wxDefaultSize);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

	// Validate the color
	wxString clr_str(m_opt.get_default_value<ConfigOptionStrings>()->get_at(m_opt_idx));
	wxColour clr(clr_str);
	if (clr_str.IsEmpty() || !clr.IsOk()) {
		clr = wxTransparentColour;
	}

	auto temp = new wxColourPickerCtrl(m_parent, wxID_ANY, clr, wxDefaultPosition, size);
    temp->SetFont(Slic3r::GUI::wxGetApp().normal_font());
    if (!wxOSX) temp->SetBackgroundStyle(wxBG_STYLE_PAINT);

	// 	// recast as a wxWindow to fit the calling convention
	window = dynamic_cast<wxWindow*>(temp);

	temp->Bind(wxEVT_COLOURPICKER_CHANGED, ([this](wxCommandEvent e) { on_change_field(); }), temp->GetId());

	temp->SetToolTip(get_tooltip_text(clr_str));
}

void ColourPicker::set_undef_value(wxColourPickerCtrl* field)
{
    field->SetColour(wxTransparentColour);

    wxButton* btn = dynamic_cast<wxButton*>(field->GetPickerCtrl());
    wxBitmap bmp = btn->GetBitmap();
    wxMemoryDC dc(bmp);
    if (!dc.IsOk()) return;
    dc.SetTextForeground(*wxWHITE);
    dc.SetFont(wxGetApp().normal_font());

    const wxRect rect = wxRect(0, 0, bmp.GetWidth(), bmp.GetHeight());
    dc.DrawLabel("undef", rect, wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL);

    dc.SelectObject(wxNullBitmap);
    btn->SetBitmapLabel(bmp);
}

void ColourPicker::set_value(const boost::any& value, bool change_event)
{
    m_disable_change_event = !change_event;
    const wxString clr_str(boost::any_cast<wxString>(value));
    auto field = dynamic_cast<wxColourPickerCtrl*>(window);

    wxColour clr(clr_str);
    if (clr_str.IsEmpty() || !clr.IsOk())
        set_undef_value(field);
    else
        field->SetColour(clr);

    m_disable_change_event = false;
}

boost::any& ColourPicker::get_value()
{
	auto colour = static_cast<wxColourPickerCtrl*>(window)->GetColour();
    if (colour == wxTransparentColour)
        m_value = std::string("");
    else {
		auto clr_str = wxString::Format(wxT("#%02X%02X%02X"), colour.Red(), colour.Green(), colour.Blue());
		m_value = clr_str.ToStdString();
    }
	return m_value;
}

void ColourPicker::msw_rescale()
{
    Field::msw_rescale();

    wxColourPickerCtrl* field = dynamic_cast<wxColourPickerCtrl*>(window);
    if (field->GetColour() == wxTransparentColour)
        set_undef_value(field);
}

void PointCtrl::BUILD()
{
	auto temp = new wxBoxSizer(wxHORIZONTAL);

    const wxSize field_size(4 * m_em_unit, -1);

	auto default_pt = m_opt.get_default_value<ConfigOptionPoints>()->values.at(0);
	double val = default_pt(0);
	wxString X = val - int(val) == 0 ? wxString::Format(_T("%i"), int(val)) : wxNumberFormatter::ToString(val, 2, wxNumberFormatter::Style_None);
	val = default_pt(1);
	wxString Y = val - int(val) == 0 ? wxString::Format(_T("%i"), int(val)) : wxNumberFormatter::ToString(val, 2, wxNumberFormatter::Style_None);

	x_textctrl = new wxTextCtrl(m_parent, wxID_ANY, X, wxDefaultPosition, field_size, wxTE_PROCESS_ENTER);
	y_textctrl = new wxTextCtrl(m_parent, wxID_ANY, Y, wxDefaultPosition, field_size, wxTE_PROCESS_ENTER);
	x_textctrl->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	x_textctrl->SetBackgroundStyle(wxBG_STYLE_PAINT);
	y_textctrl->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	y_textctrl->SetBackgroundStyle(wxBG_STYLE_PAINT);

	auto static_text_x = new wxStaticText(m_parent, wxID_ANY, "x : ");
	auto static_text_y = new wxStaticText(m_parent, wxID_ANY, "   y : ");
	static_text_x->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	static_text_x->SetBackgroundStyle(wxBG_STYLE_PAINT);
	static_text_y->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	static_text_y->SetBackgroundStyle(wxBG_STYLE_PAINT);

	temp->Add(static_text_x, 0, wxALIGN_CENTER_VERTICAL, 0);
	temp->Add(x_textctrl);
	temp->Add(static_text_y, 0, wxALIGN_CENTER_VERTICAL, 0);
	temp->Add(y_textctrl);

// 	x_textctrl->Bind(wxEVT_TEXT, ([this](wxCommandEvent e) { on_change_field(); }), x_textctrl->GetId());
// 	y_textctrl->Bind(wxEVT_TEXT, ([this](wxCommandEvent e) { on_change_field(); }), y_textctrl->GetId());

    x_textctrl->Bind(wxEVT_TEXT_ENTER, ([this](wxCommandEvent e) { propagate_value(x_textctrl); }), x_textctrl->GetId());
	y_textctrl->Bind(wxEVT_TEXT_ENTER, ([this](wxCommandEvent e) { propagate_value(y_textctrl); }), y_textctrl->GetId());

    x_textctrl->Bind(wxEVT_KILL_FOCUS, ([this](wxEvent& e) { e.Skip(); propagate_value(x_textctrl); }), x_textctrl->GetId());
    y_textctrl->Bind(wxEVT_KILL_FOCUS, ([this](wxEvent& e) { e.Skip(); propagate_value(y_textctrl); }), y_textctrl->GetId());

	// 	// recast as a wxWindow to fit the calling convention
	sizer = dynamic_cast<wxSizer*>(temp);

	x_textctrl->SetToolTip(get_tooltip_text(X+", "+Y));
	y_textctrl->SetToolTip(get_tooltip_text(X+", "+Y));
}

void PointCtrl::msw_rescale()
{
    Field::msw_rescale();

    const wxSize field_size(4 * m_em_unit, -1);

    x_textctrl->SetMinSize(field_size);
    y_textctrl->SetMinSize(field_size);
}

void PointCtrl::propagate_value(wxTextCtrl* win)
{
    if (!win->GetValue().empty()) 
        on_change_field();
    else
        on_kill_focus();
}

void PointCtrl::set_value(const Vec2d& value, bool change_event)
{
	m_disable_change_event = !change_event;

	double val = value(0);
	x_textctrl->SetValue(val - int(val) == 0 ? wxString::Format(_T("%i"), int(val)) : wxNumberFormatter::ToString(val, 2, wxNumberFormatter::Style_None));
	val = value(1);
	y_textctrl->SetValue(val - int(val) == 0 ? wxString::Format(_T("%i"), int(val)) : wxNumberFormatter::ToString(val, 2, wxNumberFormatter::Style_None));

	m_disable_change_event = false;
}

void PointCtrl::set_value(const boost::any& value, bool change_event)
{
	Vec2d pt(Vec2d::Zero());
	const Vec2d *ptf = boost::any_cast<Vec2d>(&value);
	if (!ptf)
	{
		ConfigOptionPoints* pts = boost::any_cast<ConfigOptionPoints*>(value);
		pt = pts->values.at(0);
	}
	else
		pt = *ptf;
	set_value(pt, change_event);
}

boost::any& PointCtrl::get_value()
{
	double x, y;
	x_textctrl->GetValue().ToDouble(&x);
	y_textctrl->GetValue().ToDouble(&y);
	return m_value = Vec2d(x, y);
}

void StaticText::BUILD()
{
	auto size = wxSize(wxDefaultSize);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

    const wxString legend = wxString::FromUTF8(m_opt.get_default_value<ConfigOptionString>()->value.c_str());
    auto temp = new wxStaticText(m_parent, wxID_ANY, legend, wxDefaultPosition, size, wxST_ELLIPSIZE_MIDDLE);
	temp->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	temp->SetBackgroundStyle(wxBG_STYLE_PAINT);
    temp->SetFont(wxGetApp().bold_font());

	// 	// recast as a wxWindow to fit the calling convention
	window = dynamic_cast<wxWindow*>(temp);

	temp->SetToolTip(get_tooltip_text(legend));
}

void StaticText::msw_rescale()
{
    Field::msw_rescale();

    auto size = wxSize(wxDefaultSize);
    if (m_opt.height >= 0) size.SetHeight(m_opt.height*m_em_unit);
    if (m_opt.width >= 0) size.SetWidth(m_opt.width*m_em_unit);

    if (size != wxDefaultSize)
    {
        wxStaticText* field = dynamic_cast<wxStaticText*>(window);
        field->SetSize(size);
        field->SetMinSize(size);
    }
}

void SliderCtrl::BUILD()
{
	auto size = wxSize(wxDefaultSize);
	if (m_opt.height >= 0) size.SetHeight(m_opt.height);
	if (m_opt.width >= 0) size.SetWidth(m_opt.width);

	auto temp = new wxBoxSizer(wxHORIZONTAL);

	auto def_val = m_opt.get_default_value<ConfigOptionInt>()->value;
	auto min = m_opt.min == INT_MIN ? 0 : m_opt.min;
	auto max = m_opt.max == INT_MAX ? 100 : m_opt.max;

	m_slider = new wxSlider(m_parent, wxID_ANY, def_val * m_scale,
							min * m_scale, max * m_scale,
							wxDefaultPosition, size);
	m_slider->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	m_slider->SetBackgroundStyle(wxBG_STYLE_PAINT);
 	wxSize field_size(40, -1);

	m_textctrl = new wxTextCtrl(m_parent, wxID_ANY, wxString::Format("%d", m_slider->GetValue()/m_scale), 
								wxDefaultPosition, field_size);
	m_textctrl->SetFont(Slic3r::GUI::wxGetApp().normal_font());
	m_textctrl->SetBackgroundStyle(wxBG_STYLE_PAINT);

	temp->Add(m_slider, 1, wxEXPAND | wxALIGN_CENTER_VERTICAL, 0);
	temp->Add(m_textctrl, 0, wxALIGN_CENTER_VERTICAL, 0);

	m_slider->Bind(wxEVT_SLIDER, ([this](wxCommandEvent e) {
		if (!m_disable_change_event) {
			int val = boost::any_cast<int>(get_value());
			m_textctrl->SetLabel(wxString::Format("%d", val));
			on_change_field();
		}
	}), m_slider->GetId());

	m_textctrl->Bind(wxEVT_TEXT, ([this](wxCommandEvent e) {
		std::string value = e.GetString().utf8_str().data();
		if (is_matched(value, "^-?\\d+(\\.\\d*)?$")) {
			m_disable_change_event = true;
			m_slider->SetValue(stoi(value)*m_scale);
			m_disable_change_event = false;
			on_change_field();
		}
	}), m_textctrl->GetId());

	m_sizer = dynamic_cast<wxSizer*>(temp);
}

void SliderCtrl::set_value(const boost::any& value, bool change_event)
{
	m_disable_change_event = !change_event;

	m_slider->SetValue(boost::any_cast<int>(value)*m_scale);
	int val = boost::any_cast<int>(get_value());
	m_textctrl->SetLabel(wxString::Format("%d", val));

	m_disable_change_event = false;
}

boost::any& SliderCtrl::get_value()
{
// 	int ret_val;
// 	x_textctrl->GetValue().ToDouble(&val);
	return m_value = int(m_slider->GetValue()/m_scale);
}


} // GUI
} // Slic3r


