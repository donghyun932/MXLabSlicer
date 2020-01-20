#ifndef slic3r_FillHoneycomb_hpp_
#define slic3r_FillHoneycomb_hpp_

#include <map>

#include "../libslic3r.h"

#include "FillBase.hpp"

namespace Slic3r {

class FillHoneycomb : public Fill
{
public:
    virtual ~FillHoneycomb() {}

protected:
    virtual Fill* clone() const { return new FillHoneycomb(*this); };
	virtual void _fill_surface_single(
	    const FillParams                &params, 
	    unsigned int                     thickness_layers,
	    const std::pair<float, Point>   &direction, 
	    ExPolygon                       &expolygon, 
	    Polylines                       &polylines_out);

	// Caching the 
	struct CacheID 
	{
		CacheID(float adensity, coordf_t aspacing) : 
			density(adensity), spacing(aspacing) {}
		float		density;
		coordf_t	spacing;
		bool operator<(const CacheID &other) const 
			{ return (density < other.density) || (density == other.density && spacing < other.spacing); }
		bool operator==(const CacheID &other) const 
			{ return density == other.density && spacing == other.spacing; }
	};
	struct CacheData
	{
		coord_t	distance;
        coord_t hex_side;
        coord_t hex_width;
        coord_t	pattern_height;
        coord_t y_short;
        coord_t x_offset;
        coord_t	y_offset;
        Point	hex_center;
    };
    typedef std::map<CacheID, CacheData> Cache;
	Cache cache;

    virtual float _layer_angle(size_t idx) const { return float(M_PI/3.) * (idx % 3); }
};

} // namespace Slic3r

#endif // slic3r_FillHoneycomb_hpp_
