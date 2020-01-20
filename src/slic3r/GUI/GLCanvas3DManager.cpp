#include "GLCanvas3DManager.hpp"
#include "../../slic3r/GUI/GUI.hpp"
#include "../../slic3r/GUI/AppConfig.hpp"
#include "../../slic3r/GUI/GLCanvas3D.hpp"

#include <GL/glew.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <wx/glcanvas.h>
#include <wx/timer.h>

#include <vector>
#include <string>
#include <iostream>

#ifdef __APPLE__
#include "../Utils/MacDarkMode.hpp"
#endif // __APPLE__

namespace Slic3r {
namespace GUI {

GLCanvas3DManager::GLInfo::GLInfo()
    : m_detected(false)
    , m_version("")
    , m_glsl_version("")
    , m_vendor("")
    , m_renderer("")
    , m_max_tex_size(0)
    , m_max_anisotropy(0.0f)
{
}

const std::string& GLCanvas3DManager::GLInfo::get_version() const
{
    if (!m_detected)
        detect();

    return m_version;
}

const std::string& GLCanvas3DManager::GLInfo::get_glsl_version() const
{
    if (!m_detected)
        detect();

    return m_glsl_version;
}

const std::string& GLCanvas3DManager::GLInfo::get_vendor() const
{
    if (!m_detected)
        detect();

    return m_vendor;
}

const std::string& GLCanvas3DManager::GLInfo::get_renderer() const
{
    if (!m_detected)
        detect();

    return m_renderer;
}

int GLCanvas3DManager::GLInfo::get_max_tex_size() const
{
    if (!m_detected)
        detect();

    // clamp to avoid the texture generation become too slow and use too much GPU memory
#ifdef __APPLE__
    // and use smaller texture for non retina systems
    return (Slic3r::GUI::mac_max_scaling_factor() > 1.0) ? std::min(m_max_tex_size, 8192) : std::min(m_max_tex_size / 2, 4096);
#else
    // and use smaller texture for older OpenGL versions
    return is_version_greater_or_equal_to(3, 0) ? std::min(m_max_tex_size, 8192) : std::min(m_max_tex_size / 2, 4096);
#endif // __APPLE__
}

float GLCanvas3DManager::GLInfo::get_max_anisotropy() const
{
    if (!m_detected)
        detect();

    return m_max_anisotropy;
}

void GLCanvas3DManager::GLInfo::detect() const
{
    const char* data = (const char*)::glGetString(GL_VERSION);
    if (data != nullptr)
        m_version = data;

    data = (const char*)::glGetString(GL_SHADING_LANGUAGE_VERSION);
    if (data != nullptr)
        m_glsl_version = data;

    data = (const char*)::glGetString(GL_VENDOR);
    if (data != nullptr)
        m_vendor = data;

    data = (const char*)::glGetString(GL_RENDERER);
    if (data != nullptr)
        m_renderer = data;

    glsafe(::glGetIntegerv(GL_MAX_TEXTURE_SIZE, &m_max_tex_size));

    m_max_tex_size /= 2;

    if (GLEW_EXT_texture_filter_anisotropic)
        glsafe(::glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &m_max_anisotropy));

    m_detected = true;
}

bool GLCanvas3DManager::GLInfo::is_version_greater_or_equal_to(unsigned int major, unsigned int minor) const
{
    if (!m_detected)
        detect();

    std::vector<std::string> tokens;
    boost::split(tokens, m_version, boost::is_any_of(" "), boost::token_compress_on);

    if (tokens.empty())
        return false;

    std::vector<std::string> numbers;
    boost::split(numbers, tokens[0], boost::is_any_of("."), boost::token_compress_on);

    unsigned int gl_major = 0;
    unsigned int gl_minor = 0;

    if (numbers.size() > 0)
        gl_major = ::atoi(numbers[0].c_str());

    if (numbers.size() > 1)
        gl_minor = ::atoi(numbers[1].c_str());

    if (gl_major < major)
        return false;
    else if (gl_major > major)
        return true;
    else
        return gl_minor >= minor;
}

std::string GLCanvas3DManager::GLInfo::to_string(bool format_as_html, bool extensions) const
{
    if (!m_detected)
        detect();

    std::stringstream out;

    std::string h2_start = format_as_html ? "<b>" : "";
    std::string h2_end = format_as_html ? "</b>" : "";
    std::string b_start = format_as_html ? "<b>" : "";
    std::string b_end = format_as_html ? "</b>" : "";
    std::string line_end = format_as_html ? "<br>" : "\n";

    out << h2_start << "OpenGL installation" << h2_end << line_end;
    out << b_start << "GL version:   " << b_end << (m_version.empty() ? "N/A" : m_version) << line_end;
    out << b_start << "Vendor:       " << b_end << (m_vendor.empty() ? "N/A" : m_vendor) << line_end;
    out << b_start << "Renderer:     " << b_end << (m_renderer.empty() ? "N/A" : m_renderer) << line_end;
    out << b_start << "GLSL version: " << b_end << (m_glsl_version.empty() ? "N/A" : m_glsl_version) << line_end;

    if (extensions)
    {
        std::vector<std::string> extensions_list;
        std::string extensions_str = (const char*)::glGetString(GL_EXTENSIONS);
        boost::split(extensions_list, extensions_str, boost::is_any_of(" "), boost::token_compress_off);

        if (!extensions_list.empty())
        {
            out << h2_start << "Installed extensions:" << h2_end << line_end;

            std::sort(extensions_list.begin(), extensions_list.end());
            for (const std::string& ext : extensions_list)
            {
                out << ext << line_end;
            }
        }
    }

    return out.str();
}

GLCanvas3DManager::EMultisampleState GLCanvas3DManager::s_multisample = GLCanvas3DManager::MS_Unknown;
bool GLCanvas3DManager::s_compressed_textures_supported = false;
GLCanvas3DManager::EFramebufferType GLCanvas3DManager::s_framebuffers_type = GLCanvas3DManager::FB_None;
GLCanvas3DManager::GLInfo GLCanvas3DManager::s_gl_info;

GLCanvas3DManager::GLCanvas3DManager()
    : m_context(nullptr)
    , m_gl_initialized(false)
{
}

GLCanvas3DManager::~GLCanvas3DManager()
{
	this->destroy();
}

bool GLCanvas3DManager::add(wxGLCanvas* canvas, Bed3D& bed, Camera& camera, GLToolbar& view_toolbar)
{
    if (canvas == nullptr)
        return false;

    if (do_get_canvas(canvas) != m_canvases.end())
        return false;

    GLCanvas3D* canvas3D = new GLCanvas3D(canvas, bed, camera, view_toolbar);
    if (canvas3D == nullptr)
        return false;

    canvas3D->bind_event_handlers();

    if (m_context == nullptr)
    {
        m_context = new wxGLContext(canvas);
        if (m_context == nullptr)
            return false;
    }

    canvas3D->set_context(m_context);

    m_canvases.insert(CanvasesMap::value_type(canvas, canvas3D));

    return true;
}

bool GLCanvas3DManager::remove(wxGLCanvas* canvas)
{
    CanvasesMap::iterator it = do_get_canvas(canvas);
    if (it == m_canvases.end())
        return false;

    it->second->unbind_event_handlers();
    delete it->second;
    m_canvases.erase(it);

    return true;
}

void GLCanvas3DManager::remove_all()
{
    for (CanvasesMap::value_type& item : m_canvases)
    {
        item.second->unbind_event_handlers();
        delete item.second;
    }
    m_canvases.clear();
}

unsigned int GLCanvas3DManager::count() const
{
    return (unsigned int)m_canvases.size();
}

void GLCanvas3DManager::init_gl()
{
    if (!m_gl_initialized)
    {
        glewInit();
        m_gl_initialized = true;
        if (GLEW_EXT_texture_compression_s3tc)
            s_compressed_textures_supported = true;
        else
            s_compressed_textures_supported = false;

        if (GLEW_ARB_framebuffer_object)
            s_framebuffers_type = FB_Arb;
        else if (GLEW_EXT_framebuffer_object)
            s_framebuffers_type = FB_Ext;
        else
            s_framebuffers_type = FB_None;

        if (! s_gl_info.is_version_greater_or_equal_to(2, 0)) {
        	// Complain about the OpenGL version.
        	wxString message = wxString::Format(
        		_(L("PrusaSlicer requires OpenGL 2.0 capable graphics driver to run correctly, \n"
        			"while OpenGL version %s, render %s, vendor %s was detected.")), wxString(s_gl_info.get_version()), wxString(s_gl_info.get_renderer()), wxString(s_gl_info.get_vendor()));
        	message += "\n";
        	message += _(L("You may need to update your graphics card driver."));
#ifdef _WIN32
        	message += "\n";
        	message += _(L("As a workaround, you may run PrusaSlicer with a software rendered 3D graphics by running prusa-slicer.exe with the --sw_renderer parameter."));
#endif
        	wxMessageBox(message, wxString("PrusaSlicer - ") + _(L("Unsupported OpenGL version")), wxOK | wxICON_ERROR);
        }
    }
}

bool GLCanvas3DManager::init(wxGLCanvas* canvas)
{
    CanvasesMap::const_iterator it = do_get_canvas(canvas);
    if (it != m_canvases.end())
        return (it->second != nullptr) ? init(*it->second) : false;
    else
        return false;
}

void GLCanvas3DManager::destroy()
{
    if (m_context != nullptr)
    {
        delete m_context;
        m_context = nullptr;
    }
}

GLCanvas3D* GLCanvas3DManager::get_canvas(wxGLCanvas* canvas)
{
    CanvasesMap::const_iterator it = do_get_canvas(canvas);
    return (it != m_canvases.end()) ? it->second : nullptr;
}

wxGLCanvas* GLCanvas3DManager::create_wxglcanvas(wxWindow *parent)
{
    int attribList[] = { 
    	WX_GL_RGBA,
    	WX_GL_DOUBLEBUFFER,
    	// RGB channels each should be allocated with 8 bit depth. One should almost certainly get these bit depths by default.
      	WX_GL_MIN_RED, 			8,
      	WX_GL_MIN_GREEN, 		8,
      	WX_GL_MIN_BLUE, 		8,
      	// Requesting an 8 bit alpha channel. Interestingly, the NVIDIA drivers would most likely work with some alpha plane, but glReadPixels would not return
      	// the alpha channel on NVIDIA if not requested when the GL context is created.
      	WX_GL_MIN_ALPHA, 		8,
    	WX_GL_DEPTH_SIZE, 		24,
    	WX_GL_SAMPLE_BUFFERS, 	GL_TRUE,
    	WX_GL_SAMPLES, 			4,
    	0
    };

    if (s_multisample == MS_Unknown)
    {
        detect_multisample(attribList);
//        // debug output
//        std::cout << "Multisample " << (can_multisample() ? "enabled" : "disabled") << std::endl;
    }

    if (! can_multisample())
        attribList[12] = 0;

    return new wxGLCanvas(parent, wxID_ANY, attribList, wxDefaultPosition, wxDefaultSize, wxWANTS_CHARS);
}

GLCanvas3DManager::CanvasesMap::iterator GLCanvas3DManager::do_get_canvas(wxGLCanvas* canvas)
{
    return (canvas == nullptr) ? m_canvases.end() : m_canvases.find(canvas);
}

GLCanvas3DManager::CanvasesMap::const_iterator GLCanvas3DManager::do_get_canvas(wxGLCanvas* canvas) const
{
    return (canvas == nullptr) ? m_canvases.end() : m_canvases.find(canvas);
}

bool GLCanvas3DManager::init(GLCanvas3D& canvas)
{
    if (!m_gl_initialized)
        init_gl();

    return canvas.init();
}

void GLCanvas3DManager::detect_multisample(int* attribList)
{
    int wxVersion = wxMAJOR_VERSION * 10000 + wxMINOR_VERSION * 100 + wxRELEASE_NUMBER;
    bool enable_multisample = wxVersion >= 30003;
    s_multisample = (enable_multisample && wxGLCanvas::IsDisplaySupported(attribList)) ? MS_Enabled : MS_Disabled;
    // Alternative method: it was working on previous version of wxWidgets but not with the latest, at least on Windows
    // s_multisample = enable_multisample && wxGLCanvas::IsExtensionSupported("WGL_ARB_multisample");
}

} // namespace GUI
} // namespace Slic3r
