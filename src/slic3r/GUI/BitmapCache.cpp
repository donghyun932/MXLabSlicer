#include "BitmapCache.hpp"

#include "libslic3r/Utils.hpp"

#if ! defined(WIN32) && ! defined(__APPLE__)
#define BROKEN_ALPHA
#endif

#ifdef BROKEN_ALPHA
    #include <wx/mstream.h>
    #include <wx/rawbmp.h>
#endif /* BROKEN_ALPHA */

#define NANOSVG_IMPLEMENTATION
#include "nanosvg/nanosvg.h"
#define NANOSVGRAST_IMPLEMENTATION
#include "nanosvg/nanosvgrast.h"
#include "GUI_App.hpp"

namespace Slic3r { namespace GUI {

void BitmapCache::clear()
{
    for (std::pair<const std::string, wxBitmap*> &bitmap : m_map)
        delete bitmap.second;

    m_map.clear();
}

static wxBitmap wxImage_to_wxBitmap_with_alpha(wxImage &&image, float scale = 1.0f)
{
#ifdef BROKEN_ALPHA
    wxMemoryOutputStream stream;
    image.SaveFile(stream, wxBITMAP_TYPE_PNG);
    wxStreamBuffer *buf = stream.GetOutputStreamBuffer();
    return wxBitmap::NewFromPNGData(buf->GetBufferStart(), buf->GetBufferSize());
#else
#ifdef __APPLE__
    // This is a c-tor native to Mac OS. We need to let the Mac OS wxBitmap implementation
    // know that the image may already be scaled appropriately for Retina,
    // and thereby that it's not supposed to upscale it.
    // Contrary to intuition, the `scale` argument isn't "please scale this to such and such"
    // but rather "the wxImage is sized for backing scale such and such".
    return wxBitmap(std::move(image), -1, scale);
#else
    return wxBitmap(std::move(image));
#endif
#endif
}

wxBitmap* BitmapCache::insert(const std::string &bitmap_key, size_t width, size_t height)
{
    wxBitmap *bitmap = nullptr;
    auto      it     = m_map.find(bitmap_key);
    if (it == m_map.end()) {
        bitmap = new wxBitmap(width, height);
        m_map[bitmap_key] = bitmap;
    } else {
        bitmap = it->second;
        if (size_t(bitmap->GetWidth()) != width || size_t(bitmap->GetHeight()) != height)
            bitmap->Create(width, height);
    }
#ifndef BROKEN_ALPHA
    bitmap->UseAlpha();
#endif
    return bitmap;
}

wxBitmap* BitmapCache::insert(const std::string &bitmap_key, const wxBitmap &bmp)
{
    wxBitmap *bitmap = nullptr;
    auto      it     = m_map.find(bitmap_key);
    if (it == m_map.end()) {
        bitmap = new wxBitmap(bmp);
        m_map[bitmap_key] = bitmap;
    } else {
        bitmap = it->second;
        *bitmap = bmp;
    }
    return bitmap;
}

wxBitmap* BitmapCache::insert(const std::string &bitmap_key, const wxBitmap &bmp, const wxBitmap &bmp2)
{
    // Copying the wxBitmaps is cheap as the bitmap's content is reference counted.
    const wxBitmap bmps[2] = { bmp, bmp2 };
    return this->insert(bitmap_key, bmps, bmps + 2);
}

wxBitmap* BitmapCache::insert(const std::string &bitmap_key, const wxBitmap &bmp, const wxBitmap &bmp2, const wxBitmap &bmp3)
{
    // Copying the wxBitmaps is cheap as the bitmap's content is reference counted.
    const wxBitmap bmps[3] = { bmp, bmp2, bmp3 };
    return this->insert(bitmap_key, bmps, bmps + 3);
}

wxBitmap* BitmapCache::insert(const std::string &bitmap_key, const wxBitmap *begin, const wxBitmap *end)
{
    size_t width  = 0;
    size_t height = 0;
    for (const wxBitmap *bmp = begin; bmp != end; ++ bmp) {
        width += bmp->GetWidth();
        height = std::max<size_t>(height, bmp->GetHeight());
    }

#ifdef BROKEN_ALPHA

    wxImage image(width, height);
    image.InitAlpha();
    // Fill in with a white color.
    memset(image.GetData(), 0x0ff, width * height * 3);
    // Fill in with full transparency.
    memset(image.GetAlpha(),    0, width * height);
    size_t x = 0;
    for (const wxBitmap *bmp = begin; bmp != end; ++ bmp) {
        if (bmp->GetWidth() > 0) {
            if (bmp->GetDepth() == 32) {
                wxAlphaPixelData data(*const_cast<wxBitmap*>(bmp));
                //FIXME The following method is missing from wxWidgets 3.1.1.
                // It looks like the wxWidgets 3.0.3 called the wrapped bitmap's UseAlpha().
                //data.UseAlpha();
                if (data) {
                    for (int r = 0; r < bmp->GetHeight(); ++ r) {
                        wxAlphaPixelData::Iterator src(data);
                        src.Offset(data, 0, r);
                        unsigned char *dst_pixels = image.GetData()  + (x + r * width) * 3;
                        unsigned char *dst_alpha  = image.GetAlpha() +  x + r * width;
                        for (int c = 0; c < bmp->GetWidth(); ++ c, ++ src) {
                            *dst_pixels ++ = src.Red();
                            *dst_pixels ++ = src.Green();
                            *dst_pixels ++ = src.Blue();
                            *dst_alpha  ++ = src.Alpha();
                        }
                    }
                }
            } else if (bmp->GetDepth() == 24) {
                wxNativePixelData data(*const_cast<wxBitmap*>(bmp));
                if (data) {
                    for (int r = 0; r < bmp->GetHeight(); ++ r) {
                        wxNativePixelData::Iterator src(data);
                        src.Offset(data, 0, r);
                        unsigned char *dst_pixels = image.GetData()  + (x + r * width) * 3;
                        unsigned char *dst_alpha  = image.GetAlpha() +  x + r * width;
                        for (int c = 0; c < bmp->GetWidth(); ++ c, ++ src) {
                            *dst_pixels ++ = src.Red();
                            *dst_pixels ++ = src.Green();
                            *dst_pixels ++ = src.Blue();
                            *dst_alpha  ++ = wxALPHA_OPAQUE;
                        }
                    }
                }
            }
        }
        x += bmp->GetWidth();
    }
    return this->insert(bitmap_key, wxImage_to_wxBitmap_with_alpha(std::move(image)));

#else

    wxBitmap *bitmap = this->insert(bitmap_key, width, height);
    wxMemoryDC memDC;
    memDC.SelectObject(*bitmap);
    memDC.SetBackground(*wxTRANSPARENT_BRUSH);
    memDC.Clear();
    size_t x = 0;
    for (const wxBitmap *bmp = begin; bmp != end; ++ bmp) {
        if (bmp->GetWidth() > 0)
            memDC.DrawBitmap(*bmp, x, 0, true);
        x += bmp->GetWidth();
    }
    memDC.SelectObject(wxNullBitmap);
    return bitmap;

#endif
}

wxBitmap* BitmapCache::insert_raw_rgba(const std::string &bitmap_key, unsigned width, unsigned height, const unsigned char *raw_data, float scale /* = 1.0f */, const bool grayscale/* = false*/)
{
    wxImage image(width, height);
    image.InitAlpha();
    unsigned char *rgb   = image.GetData();
    unsigned char *alpha = image.GetAlpha();
    unsigned int pixels = width * height;
    for (unsigned int i = 0; i < pixels; ++ i) {
        *rgb   ++ = *raw_data ++;
        *rgb   ++ = *raw_data ++;
        *rgb   ++ = *raw_data ++;
        *alpha ++ = *raw_data ++;
    }

    if (grayscale)
        image = image.ConvertToGreyscale(m_gs, m_gs, m_gs);

    return this->insert(bitmap_key, wxImage_to_wxBitmap_with_alpha(std::move(image), scale));
}

wxBitmap* BitmapCache::load_png(const std::string &bitmap_name, unsigned width, unsigned height,
    const bool grayscale/* = false*/)
{
    std::string bitmap_key = bitmap_name + ( height !=0 ? 
                                           "-h" + std::to_string(height) : 
                                           "-w" + std::to_string(width))
                                         + (grayscale ? "-gs" : "");

    auto it = m_map.find(bitmap_key);
    if (it != m_map.end())
        return it->second;

    wxImage image;
    if (! image.LoadFile(Slic3r::GUI::from_u8(Slic3r::var(bitmap_name + ".png")), wxBITMAP_TYPE_PNG) ||
        image.GetWidth() == 0 || image.GetHeight() == 0)
        return nullptr;

    if (height != 0 && unsigned(image.GetHeight()) != height)
        width   = unsigned(0.5f + float(image.GetWidth()) * height / image.GetHeight());
    else if (width != 0 && unsigned(image.GetWidth()) != width)
        height  = unsigned(0.5f + float(image.GetHeight()) * width / image.GetWidth());

    if (height != 0 && width != 0)
        image.Rescale(width, height, wxIMAGE_QUALITY_BILINEAR);

    if (grayscale)
        image = image.ConvertToGreyscale(m_gs, m_gs, m_gs);

    return this->insert(bitmap_key, wxImage_to_wxBitmap_with_alpha(std::move(image)));
}

wxBitmap* BitmapCache::load_svg(const std::string &bitmap_name, unsigned target_width, unsigned target_height, 
    float scale /* = 1.0f */, const bool grayscale/* = false*/)
{
    std::string bitmap_key = bitmap_name + ( target_height !=0 ? 
                                           "-h" + std::to_string(target_height) : 
                                           "-w" + std::to_string(target_width))
                                         + (scale != 1.0f ? "-s" + std::to_string(scale) : "")
                                         + (grayscale ? "-gs" : "");

    target_height != 0 ? target_height *= scale : target_width *= scale;

    auto it = m_map.find(bitmap_key);
    if (it != m_map.end())
        return it->second;

    NSVGimage *image = ::nsvgParseFromFile(Slic3r::var(bitmap_name + ".svg").c_str(), "px", 96.0f);
    if (image == nullptr)
        return nullptr;

    float svg_scale = target_height != 0 ? 
                  (float)target_height / image->height  : target_width != 0 ?
                  (float)target_width / image->width    : 1;

    int   width    = (int)(svg_scale * image->width + 0.5f);
    int   height   = (int)(svg_scale * image->height + 0.5f);
    int   n_pixels = width * height;
    if (n_pixels <= 0) {
        ::nsvgDelete(image);
        return nullptr;
    }

    NSVGrasterizer *rast = ::nsvgCreateRasterizer();
    if (rast == nullptr) {
        ::nsvgDelete(image);
        return nullptr;
    }

    std::vector<unsigned char> data(n_pixels * 4, 0);
    ::nsvgRasterize(rast, image, 0, 0, svg_scale, data.data(), width, height, width * 4);
    ::nsvgDeleteRasterizer(rast);
    ::nsvgDelete(image);

    return this->insert_raw_rgba(bitmap_key, width, height, data.data(), scale, grayscale);
}

wxBitmap BitmapCache::mksolid(size_t width, size_t height, unsigned char r, unsigned char g, unsigned char b, unsigned char transparency)
{
    wxImage image(width, height);
    image.InitAlpha();
    unsigned char* imgdata = image.GetData();
    unsigned char* imgalpha = image.GetAlpha();
    for (size_t i = 0; i < width * height; ++ i) {
        *imgdata ++ = r;
        *imgdata ++ = g;
        *imgdata ++ = b;
        *imgalpha ++ = transparency;
    }
    return wxImage_to_wxBitmap_with_alpha(std::move(image));
}

} // namespace GUI
} // namespace Slic3r
