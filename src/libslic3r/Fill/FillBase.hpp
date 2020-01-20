#ifndef slic3r_FillBase_hpp_
#define slic3r_FillBase_hpp_

#include <assert.h>
#include <memory.h>
#include <float.h>
#include <stdint.h>

#include <type_traits>

#include "../libslic3r.h"
#include "../BoundingBox.hpp"
#include "../PrintConfig.hpp"
#include "../Utils.hpp"

namespace Slic3r {

class ExPolygon;
class Surface;

struct FillParams
{
    FillParams() { 
        memset(this, 0, sizeof(FillParams));
        // Adjustment does not work.
        dont_adjust = true;
    }

    bool        full_infill() const { return density > 0.9999f; }

    // Fill density, fraction in <0, 1>
    float       density;

    // Don't connect the fill lines around the inner perimeter.
    bool        dont_connect;

    // Don't adjust spacing to fill the space evenly.
    bool        dont_adjust;

    // For Honeycomb.
    // we were requested to complete each loop;
    // in this case we don't try to make more continuous paths
    bool        complete;
};
static_assert(IsTriviallyCopyable<FillParams>::value, "FillParams class is not POD (and it should be - see constructor).");

class Fill
{
public:
    // Index of the layer.
    size_t      layer_id;
    // Z coordinate of the top print surface, in unscaled coordinates
    coordf_t    z;
    // in unscaled coordinates
    coordf_t    spacing;
    // infill / perimeter overlap, in unscaled coordinates
    coordf_t    overlap;
    // in radians, ccw, 0 = East
    float       angle;
    // In scaled coordinates. Maximum lenght of a perimeter segment connecting two infill lines.
    // Used by the FillRectilinear2, FillGrid2, FillTriangles, FillStars and FillCubic.
    // If left to zero, the links will not be limited.
    coord_t     link_max_length;
    // In scaled coordinates. Used by the concentric infill pattern to clip the loops to create extrusion paths.
    coord_t     loop_clipping;
    // In scaled coordinates. Bounding box of the 2D projection of the object.
    BoundingBox bounding_box;

public:
    virtual ~Fill() {}

    static Fill* new_from_type(const InfillPattern type);
    static Fill* new_from_type(const std::string &type);
    static bool  use_bridge_flow(const InfillPattern type);

    void         set_bounding_box(const Slic3r::BoundingBox &bbox) { bounding_box = bbox; }

    // Use bridge flow for the fill?
    virtual bool use_bridge_flow() const { return false; }

    // Do not sort the fill lines to optimize the print head path?
    virtual bool no_sort() const { return false; }

    // Perform the fill.
    virtual Polylines fill_surface(const Surface *surface, const FillParams &params);

protected:
    Fill() :
        layer_id(size_t(-1)),
        z(0.),
        spacing(0.),
        // Infill / perimeter overlap.
        overlap(0.),
        // Initial angle is undefined.
        angle(FLT_MAX),
        link_max_length(0),
        loop_clipping(0),
        // The initial bounding box is empty, therefore undefined.
        bounding_box(Point(0, 0), Point(-1, -1))
        {}

    // The expolygon may be modified by the method to avoid a copy.
    virtual void    _fill_surface_single(
        const FillParams                & /* params */, 
        unsigned int                      /* thickness_layers */,
        const std::pair<float, Point>   & /* direction */, 
        ExPolygon                       & /* expolygon */, 
        Polylines                       & /* polylines_out */) {};

    virtual float _layer_angle(size_t idx) const { return (idx & 1) ? float(M_PI/2.) : 0; }

    virtual std::pair<float, Point> _infill_direction(const Surface *surface) const;

public:
    static void connect_infill(Polylines &&infill_ordered, const ExPolygon &boundary, Polylines &polylines_out, double spacing, const FillParams &params);

    static coord_t  _adjust_solid_spacing(const coord_t width, const coord_t distance);

    // Align a coordinate to a grid. The coordinate may be negative,
    // the aligned value will never be bigger than the original one.
    static coord_t _align_to_grid(const coord_t coord, const coord_t spacing) {
        // Current C++ standard defines the result of integer division to be rounded to zero,
        // for both positive and negative numbers. Here we want to round down for negative
        // numbers as well.
        coord_t aligned = (coord < 0) ?
        		((coord - spacing + 1) / spacing) * spacing :
        		(coord / spacing) * spacing;
        assert(aligned <= coord);
        return aligned;
    }
    static Point   _align_to_grid(Point   coord, Point   spacing) 
        { return Point(_align_to_grid(coord(0), spacing(0)), _align_to_grid(coord(1), spacing(1))); }
    static coord_t _align_to_grid(coord_t coord, coord_t spacing, coord_t base) 
        { return base + _align_to_grid(coord - base, spacing); }
    static Point   _align_to_grid(Point   coord, Point   spacing, Point   base)
        { return Point(_align_to_grid(coord(0), spacing(0), base(0)), _align_to_grid(coord(1), spacing(1), base(1))); }
};

} // namespace Slic3r

#endif // slic3r_FillBase_hpp_
