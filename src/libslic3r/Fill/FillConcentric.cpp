#include "../ClipperUtils.hpp"
#include "../ExPolygon.hpp"
#include "../Surface.hpp"

#include "FillConcentric.hpp"

namespace Slic3r {

void FillConcentric::_fill_surface_single(
    const FillParams                &params, 
    unsigned int                     thickness_layers,
    const std::pair<float, Point>   &direction, 
    ExPolygon                       &expolygon, 
    Polylines                       &polylines_out)
{
    // no rotation is supported for this infill pattern
    BoundingBox bounding_box = expolygon.contour.bounding_box();
    
    coord_t min_spacing = scale_(this->spacing);
    coord_t distance = coord_t(min_spacing / params.density);
    
    if (params.density > 0.9999f && !params.dont_adjust) {
        distance = this->_adjust_solid_spacing(bounding_box.size()(0), distance);
        this->spacing = unscale<double>(distance);
    }

    Polygons loops = (Polygons)expolygon;
    Polygons last  = loops;
    while (! last.empty()) {
        last = offset2(last, -(distance + min_spacing/2), +min_spacing/2);
        loops.insert(loops.end(), last.begin(), last.end());
    }

    // generate paths from the outermost to the innermost, to avoid
    // adhesion problems of the first central tiny loops
    loops = union_pt_chained(loops, false);
    
    // split paths using a nearest neighbor search
    size_t iPathFirst = polylines_out.size();
    Point last_pos(0, 0);
    for (const Polygon &loop : loops) {
        polylines_out.push_back(loop.split_at_index(this->layer_id % loop.points.size()));
        last_pos = polylines_out.back().last_point();
    }
}

} // namespace Slic3r
