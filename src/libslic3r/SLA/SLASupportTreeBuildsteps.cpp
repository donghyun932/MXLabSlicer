#include "SLASupportTreeBuildsteps.hpp"

#include <libnest2d/optimizers/nlopt/genetic.hpp>
#include <libnest2d/optimizers/nlopt/subplex.hpp>
#include <boost/log/trivial.hpp>

namespace Slic3r {
namespace sla {

SupportTreeBuildsteps::SupportTreeBuildsteps(SupportTreeBuilder &   builder,
                                           const SupportableMesh &sm)
    : m_cfg(sm.cfg)
    , m_mesh(sm.emesh)
    , m_support_pts(sm.pts)
    , m_support_nmls(sm.pts.size(), 3)
    , m_builder(builder)
    , m_points(sm.pts.size(), 3)
    , m_thr(builder.ctl().cancelfn)
{
    // Prepare the support points in Eigen/IGL format as well, we will use
    // it mostly in this form.
    
    long i = 0;
    for (const SupportPoint &sp : m_support_pts) {
        m_points.row(i)(X) = double(sp.pos(X));
        m_points.row(i)(Y) = double(sp.pos(Y));
        m_points.row(i)(Z) = double(sp.pos(Z));
        ++i;
    }
}

bool SupportTreeBuildsteps::execute(SupportTreeBuilder &   builder,
                                   const SupportableMesh &sm)
{
    if(sm.pts.empty()) return false;
    
    SupportTreeBuildsteps alg(builder, sm);
    
    // Let's define the individual steps of the processing. We can experiment
    // later with the ordering and the dependencies between them.
    enum Steps {
        BEGIN,
        FILTER,
        PINHEADS,
        CLASSIFY,
        ROUTING_GROUND,
        ROUTING_NONGROUND,
        CASCADE_PILLARS,
        HEADLESS,
        MERGE_RESULT,
        DONE,
        ABORT,
        NUM_STEPS
        //...
    };
    
    // Collect the algorithm steps into a nice sequence
    std::array<std::function<void()>, NUM_STEPS> program = {
        [] () {
            // Begin...
            // Potentially clear up the shared data (not needed for now)
        },
        
        std::bind(&SupportTreeBuildsteps::filter, &alg),
        
        std::bind(&SupportTreeBuildsteps::add_pinheads, &alg),
        
        std::bind(&SupportTreeBuildsteps::classify, &alg),
        
        std::bind(&SupportTreeBuildsteps::routing_to_ground, &alg),
        
        std::bind(&SupportTreeBuildsteps::routing_to_model, &alg),
        
        std::bind(&SupportTreeBuildsteps::interconnect_pillars, &alg),
        
        std::bind(&SupportTreeBuildsteps::routing_headless, &alg),
        
        std::bind(&SupportTreeBuildsteps::merge_result, &alg),
        
        [] () {
            // Done
        },
        
        [] () {
            // Abort
        }
    };
    
    Steps pc = BEGIN;
    
    if(sm.cfg.ground_facing_only) {
        program[ROUTING_NONGROUND] = []() {
            BOOST_LOG_TRIVIAL(info)
                << "Skipping model-facing supports as requested.";
        };
        program[HEADLESS] = []() {
            BOOST_LOG_TRIVIAL(info) << "Skipping headless stick generation as"
                                       " requested.";
        };
    }
    
    // Let's define a simple automaton that will run our program.
    auto progress = [&builder, &pc] () {
        static const std::array<std::string, NUM_STEPS> stepstr {
            "Starting",
            "Filtering",
            "Generate pinheads",
            "Classification",
            "Routing to ground",
            "Routing supports to model surface",
            "Interconnecting pillars",
            "Processing small holes",
            "Merging support mesh",
            "Done",
            "Abort"
        };
        
        static const std::array<unsigned, NUM_STEPS> stepstate {
            0,
            10,
            30,
            50,
            60,
            70,
            80,
            85,
            99,
            100,
            0
        };
        
        if(builder.ctl().stopcondition()) pc = ABORT;
        
        switch(pc) {
        case BEGIN: pc = FILTER; break;
        case FILTER: pc = PINHEADS; break;
        case PINHEADS: pc = CLASSIFY; break;
        case CLASSIFY: pc = ROUTING_GROUND; break;
        case ROUTING_GROUND: pc = ROUTING_NONGROUND; break;
        case ROUTING_NONGROUND: pc = CASCADE_PILLARS; break;
        case CASCADE_PILLARS: pc = HEADLESS; break;
        case HEADLESS: pc = MERGE_RESULT; break;
        case MERGE_RESULT: pc = DONE; break;
        case DONE:
        case ABORT: break;
        default: ;
        }
        
        builder.ctl().statuscb(stepstate[pc], stepstr[pc]);
    };
    
    // Just here we run the computation...
    while(pc < DONE) {
        progress();
        program[pc]();
    }
    
    return pc == ABORT;
}

EigenMesh3D::hit_result SupportTreeBuildsteps::pinhead_mesh_intersect(
    const Vec3d &s, const Vec3d &dir, double r_pin, double r_back, double width)
{
    static const size_t SAMPLES = 8;
    
    // method based on:
    // https://math.stackexchange.com/questions/73237/parametric-equation-of-a-circle-in-3d-space
    
    // We will shoot multiple rays from the head pinpoint in the direction
    // of the pinhead robe (side) surface. The result will be the smallest
    // hit distance.
    
    // Move away slightly from the touching point to avoid raycasting on the
    // inner surface of the mesh.
    Vec3d v = dir;     // Our direction (axis)
    Vec3d c = s + width * dir;
    const double& sd = m_cfg.safety_distance_mm;
    
    // Two vectors that will be perpendicular to each other and to the
    // axis. Values for a(X) and a(Y) are now arbitrary, a(Z) is just a
    // placeholder.
    Vec3d a(0, 1, 0), b;
    
    // The portions of the circle (the head-back circle) for which we will
    // shoot rays.
    std::array<double, SAMPLES> phis;
    for(size_t i = 0; i < phis.size(); ++i) phis[i] = i*2*PI/phis.size();
    
    auto& m = m_mesh;
    using HitResult = EigenMesh3D::hit_result;
    
    // Hit results
    std::array<HitResult, SAMPLES> hits;
    
    // We have to address the case when the direction vector v (same as
    // dir) is coincident with one of the world axes. In this case two of
    // its components will be completely zero and one is 1.0. Our method
    // becomes dangerous here due to division with zero. Instead, vector
    // 'a' can be an element-wise rotated version of 'v'
    auto chk1 = [] (double val) {
        return std::abs(std::abs(val) - 1) < 1e-20;
    };
    
    if(chk1(v(X)) || chk1(v(Y)) || chk1(v(Z))) {
        a = {v(Z), v(X), v(Y)};
        b = {v(Y), v(Z), v(X)};
    }
    else {
        a(Z) = -(v(Y)*a(Y)) / v(Z); a.normalize();
        b = a.cross(v);
    }

    // Now a and b vectors are perpendicular to v and to each other.
    // Together they define the plane where we have to iterate with the
    // given angles in the 'phis' vector
    ccr::enumerate(
        phis.begin(), phis.end(),
        [&hits, &m, sd, r_pin, r_back, s, a, b, c](double phi, size_t i) {
           double sinphi = std::sin(phi);
           double cosphi = std::cos(phi);
        
           // Let's have a safety coefficient for the radiuses.
           double rpscos = (sd + r_pin) * cosphi;
           double rpssin = (sd + r_pin) * sinphi;
           double rpbcos = (sd + r_back) * cosphi;
           double rpbsin = (sd + r_back) * sinphi;
        
           // Point on the circle on the pin sphere
           Vec3d ps(s(X) + rpscos * a(X) + rpssin * b(X),
                    s(Y) + rpscos * a(Y) + rpssin * b(Y),
                    s(Z) + rpscos * a(Z) + rpssin * b(Z));
        
           // Point ps is not on mesh but can be inside or
           // outside as well. This would cause many problems
           // with ray-casting. To detect the position we will
           // use the ray-casting result (which has an is_inside
           // predicate).
        
           // This is the point on the circle on the back sphere
           Vec3d p(c(X) + rpbcos * a(X) + rpbsin * b(X),
                   c(Y) + rpbcos * a(Y) + rpbsin * b(Y),
                   c(Z) + rpbcos * a(Z) + rpbsin * b(Z));
        
           Vec3d n = (p - ps).normalized();
           auto  q = m.query_ray_hit(ps + sd * n, n);
        
           if (q.is_inside()) { // the hit is inside the model
               if (q.distance() > r_pin + sd) {
                   // If we are inside the model and the hit
                   // distance is bigger than our pin circle
                   // diameter, it probably indicates that the
                   // support point was already inside the
                   // model, or there is really no space
                   // around the point. We will assign a zero
                   // hit distance to these cases which will
                   // enforce the function return value to be
                   // an invalid ray with zero hit distance.
                   // (see min_element at the end)
                   hits[i] = HitResult(0.0);
               } else {
                   // re-cast the ray from the outside of the
                   // object. The starting point has an offset
                   // of 2*safety_distance because the
                   // original ray has also had an offset
                   auto q2 = m.query_ray_hit(
                       ps + (q.distance() + 2 * sd) * n, n);
                   hits[i] = q2;
               }
           } else
               hits[i] = q;
        });

    auto mit = std::min_element(hits.begin(), hits.end());
    
    return *mit;
}

EigenMesh3D::hit_result SupportTreeBuildsteps::bridge_mesh_intersect(
    const Vec3d &s, const Vec3d &dir, double r, bool ins_check)
{
    static const size_t SAMPLES = 8;
    
    // helper vector calculations
    Vec3d a(0, 1, 0), b;
    const double& sd = m_cfg.safety_distance_mm;
    
    // INFO: for explanation of the method used here, see the previous
    // method's comments.
    
    auto chk1 = [] (double val) {
        return std::abs(std::abs(val) - 1) < 1e-20;
    };
    
    if(chk1(dir(X)) || chk1(dir(Y)) || chk1(dir(Z))) {
        a = {dir(Z), dir(X), dir(Y)};
        b = {dir(Y), dir(Z), dir(X)};
    }
    else {
        a(Z) = -(dir(Y)*a(Y)) / dir(Z); a.normalize();
        b = a.cross(dir);
    }
    
    // circle portions
    std::array<double, SAMPLES> phis;
    for(size_t i = 0; i < phis.size(); ++i) phis[i] = i*2*PI/phis.size();
    
    auto& m = m_mesh;
    using HitResult = EigenMesh3D::hit_result;
    
    // Hit results
    std::array<HitResult, SAMPLES> hits;
    
    ccr::enumerate(
        phis.begin(), phis.end(),
        [&m, a, b, sd, dir, r, s, ins_check, &hits] (double phi, size_t i) {
            double sinphi = std::sin(phi);
            double cosphi = std::cos(phi);
            
            // Let's have a safety coefficient for the radiuses.
            double rcos = (sd + r) * cosphi;
            double rsin = (sd + r) * sinphi;
            
            // Point on the circle on the pin sphere
            Vec3d p (s(X) + rcos * a(X) + rsin * b(X),
                    s(Y) + rcos * a(Y) + rsin * b(Y),
                    s(Z) + rcos * a(Z) + rsin * b(Z));
            
            auto hr = m.query_ray_hit(p + sd*dir, dir);
            
            if(ins_check && hr.is_inside()) {
                if(hr.distance() > 2 * r + sd) hits[i] = HitResult(0.0);
                else {
                    // re-cast the ray from the outside of the object
                    auto hr2 =
                        m.query_ray_hit(p + (hr.distance() + 2*sd)*dir, dir);
                    
                    hits[i] = hr2;
                }
            } else hits[i] = hr;
        });
    
    auto mit = std::min_element(hits.begin(), hits.end());
    
    return *mit;
}

bool SupportTreeBuildsteps::interconnect(const Pillar &pillar,
                                         const Pillar &nextpillar)
{
    // We need to get the starting point of the zig-zag pattern. We have to
    // be aware that the two head junctions are at different heights. We
    // may start from the lowest junction and call it a day but this
    // strategy would leave unconnected a lot of pillar duos where the
    // shorter pillar is too short to start a new bridge but the taller
    // pillar could still be bridged with the shorter one.
    bool was_connected = false;
    
    Vec3d supper = pillar.startpoint();
    Vec3d slower = nextpillar.startpoint();
    Vec3d eupper = pillar.endpoint();
    Vec3d elower = nextpillar.endpoint();
    
    double zmin = m_builder.ground_level + m_cfg.base_height_mm;
    eupper(Z) = std::max(eupper(Z), zmin);
    elower(Z) = std::max(elower(Z), zmin);
    
    // The usable length of both pillars should be positive
    if(slower(Z) - elower(Z) < 0) return false;
    if(supper(Z) - eupper(Z) < 0) return false;
    
    double pillar_dist = distance(Vec2d{slower(X), slower(Y)},
                                  Vec2d{supper(X), supper(Y)});
    double bridge_distance = pillar_dist / std::cos(-m_cfg.bridge_slope);
    double zstep = pillar_dist * std::tan(-m_cfg.bridge_slope);
    
    if(pillar_dist < 2 * m_cfg.head_back_radius_mm ||
        pillar_dist > m_cfg.max_pillar_link_distance_mm) return false;
    
    if(supper(Z) < slower(Z)) supper.swap(slower);
    if(eupper(Z) < elower(Z)) eupper.swap(elower);
    
    double startz = 0, endz = 0;
    
    startz = slower(Z) - zstep < supper(Z) ? slower(Z) - zstep : slower(Z);
    endz = eupper(Z) + zstep > elower(Z) ? eupper(Z) + zstep : eupper(Z);
    
    if(slower(Z) - eupper(Z) < std::abs(zstep)) {
        // no space for even one cross
        
        // Get max available space
        startz = std::min(supper(Z), slower(Z) - zstep);
        endz = std::max(eupper(Z) + zstep, elower(Z));
        
        // Align to center
        double available_dist = (startz - endz);
        double rounds = std::floor(available_dist / std::abs(zstep));
        startz -= 0.5 * (available_dist - rounds * std::abs(zstep));
    }
    
    auto pcm = m_cfg.pillar_connection_mode;
    bool docrosses =
        pcm == PillarConnectionMode::cross ||
        (pcm == PillarConnectionMode::dynamic &&
         pillar_dist > 2*m_cfg.base_radius_mm);
    
    // 'sj' means starting junction, 'ej' is the end junction of a bridge.
    // They will be swapped in every iteration thus the zig-zag pattern.
    // According to a config parameter, a second bridge may be added which
    // results in a cross connection between the pillars.
    Vec3d sj = supper, ej = slower; sj(Z) = startz; ej(Z) = sj(Z) + zstep;
    
    // TODO: This is a workaround to not have a faulty last bridge
    while(ej(Z) >= eupper(Z) /*endz*/) {
        if(bridge_mesh_intersect(sj, dirv(sj, ej), pillar.r) >= bridge_distance)
        {
            m_builder.add_crossbridge(sj, ej, pillar.r);
            was_connected = true;
        }
        
        // double bridging: (crosses)
        if(docrosses) {
            Vec3d sjback(ej(X), ej(Y), sj(Z));
            Vec3d ejback(sj(X), sj(Y), ej(Z));
            if (sjback(Z) <= slower(Z) && ejback(Z) >= eupper(Z) &&
                bridge_mesh_intersect(sjback, dirv(sjback, ejback),
                                      pillar.r) >= bridge_distance) {
                // need to check collision for the cross stick
                m_builder.add_crossbridge(sjback, ejback, pillar.r);
                was_connected = true;
            }
        }
        
        sj.swap(ej);
        ej(Z) = sj(Z) + zstep;
    }
    
    return was_connected;
}

bool SupportTreeBuildsteps::connect_to_nearpillar(const Head &head,
                                                  long        nearpillar_id)
{
    auto nearpillar = [this, nearpillar_id]() -> const Pillar& {
        return m_builder.pillar(nearpillar_id);
    };
    
    if (m_builder.bridgecount(nearpillar()) > m_cfg.max_bridges_on_pillar) 
        return false;
    
    Vec3d headjp = head.junction_point();
    Vec3d nearjp_u = nearpillar().startpoint();
    Vec3d nearjp_l = nearpillar().endpoint();
    
    double r = head.r_back_mm;
    double d2d = distance(to_2d(headjp), to_2d(nearjp_u));
    double d3d = distance(headjp, nearjp_u);
    
    double hdiff = nearjp_u(Z) - headjp(Z);
    double slope = std::atan2(hdiff, d2d);
    
    Vec3d bridgestart = headjp;
    Vec3d bridgeend = nearjp_u;
    double max_len = m_cfg.max_bridge_length_mm;
    double max_slope = m_cfg.bridge_slope;
    double zdiff = 0.0;
    
    // check the default situation if feasible for a bridge
    if(d3d > max_len || slope > -max_slope) {
        // not feasible to connect the two head junctions. We have to search
        // for a suitable touch point.
        
        double Zdown = headjp(Z) + d2d * std::tan(-max_slope);
        Vec3d touchjp = bridgeend; touchjp(Z) = Zdown;
        double D = distance(headjp, touchjp);
        zdiff = Zdown - nearjp_u(Z);
        
        if(zdiff > 0) {
            Zdown -= zdiff;
            bridgestart(Z) -= zdiff;
            touchjp(Z) = Zdown;
            
            double t = bridge_mesh_intersect(headjp, {0,0,-1}, r);
            
            // We can't insert a pillar under the source head to connect
            // with the nearby pillar's starting junction
            if(t < zdiff) return false;
        }
        
        if(Zdown <= nearjp_u(Z) && Zdown >= nearjp_l(Z) && D < max_len)
            bridgeend(Z) = Zdown;
        else
            return false;
    }
    
    // There will be a minimum distance from the ground where the
    // bridge is allowed to connect. This is an empiric value.
    double minz = m_builder.ground_level + 2 * m_cfg.head_width_mm;
    if(bridgeend(Z) < minz) return false;
    
    double t = bridge_mesh_intersect(bridgestart,
                                     dirv(bridgestart, bridgeend), r);
    
    // Cannot insert the bridge. (further search might not worth the hassle)
    if(t < distance(bridgestart, bridgeend)) return false;
    
    std::lock_guard<ccr::BlockingMutex> lk(m_bridge_mutex);
    
    if (m_builder.bridgecount(nearpillar()) < m_cfg.max_bridges_on_pillar) {
        // A partial pillar is needed under the starting head.
        if(zdiff > 0) {
            m_builder.add_pillar(head.id, bridgestart, r);
            m_builder.add_junction(bridgestart, r);
            m_builder.add_bridge(bridgestart, bridgeend, head.r_back_mm);
        } else {
            m_builder.add_bridge(head.id, bridgeend);
        }
        
        m_builder.increment_bridges(nearpillar());
    } else return false;
    
    return true;
}

bool SupportTreeBuildsteps::search_pillar_and_connect(const Head &head)
{
    PointIndex spindex = m_pillar_index.guarded_clone();
    
    long nearest_id = ID_UNSET;
    
    Vec3d querypoint = head.junction_point();
    
    while(nearest_id < 0 && !spindex.empty()) { m_thr();
        // loop until a suitable head is not found
        // if there is a pillar closer than the cluster center
        // (this may happen as the clustering is not perfect)
        // than we will bridge to this closer pillar
        
        Vec3d qp(querypoint(X), querypoint(Y), m_builder.ground_level);
        auto qres = spindex.nearest(qp, 1);
        if(qres.empty()) break;
        
        auto ne = qres.front();
        nearest_id = ne.second;
        
        if(nearest_id >= 0) {
            if(size_t(nearest_id) < m_builder.pillarcount()) {
                if(!connect_to_nearpillar(head, nearest_id)) {
                    nearest_id = ID_UNSET;    // continue searching
                    spindex.remove(ne);       // without the current pillar
                }
            }
        }
    }
    
    return nearest_id >= 0;
}

void SupportTreeBuildsteps::create_ground_pillar(const Vec3d &jp,
                                                 const Vec3d &sourcedir,
                                                 double       radius,
                                                 long         head_id)
{
    // People were killed for this number (seriously)
    static const double SQR2 = std::sqrt(2.0);
    static const Vec3d  DOWN = {0.0, 0.0, -1.0};
    
    double gndlvl       = m_builder.ground_level;
    Vec3d  endp         = {jp(X), jp(Y), gndlvl};
    double sd           = m_cfg.pillar_base_safety_distance_mm;
    long   pillar_id    = ID_UNSET;
    double min_dist     = sd + m_cfg.base_radius_mm + EPSILON;
    double dist         = 0;
    bool   can_add_base = true;
    bool   normal_mode  = true;
    
    if (m_cfg.object_elevation_mm < EPSILON
        && (dist = std::sqrt(m_mesh.squared_distance(endp))) < min_dist) {
        // Get the distance from the mesh. This can be later optimized
        // to get the distance in 2D plane because we are dealing with
        // the ground level only.
        
        normal_mode     = false;
        double mind     = min_dist - dist;
        double azimuth  = std::atan2(sourcedir(Y), sourcedir(X));
        double sinpolar = std::sin(PI - m_cfg.bridge_slope);
        double cospolar = std::cos(PI - m_cfg.bridge_slope);
        double cosazm   = std::cos(azimuth);
        double sinazm   = std::sin(azimuth);
        
        auto dir = Vec3d(cosazm * sinpolar, sinazm * sinpolar, cospolar)
                       .normalized();
        
        using namespace libnest2d::opt;
        StopCriteria scr;
        scr.stop_score = min_dist;
        SubplexOptimizer solver(scr);
        
        auto result = solver.optimize_max(
            [this, dir, jp, gndlvl](double mv) {
                Vec3d endpt = jp + SQR2 * mv * dir;
                endpt(Z)    = gndlvl;
                return std::sqrt(m_mesh.squared_distance(endpt));
            },
            initvals(mind), bound(0.0, 2 * min_dist));
        
        mind           = std::get<0>(result.optimum);
        endp         = jp + SQR2 * mind * dir;
        Vec3d pgnd   = {endp(X), endp(Y), gndlvl};
        can_add_base = result.score > min_dist;
        
        double gnd_offs = m_mesh.ground_level_offset();
        auto abort_in_shame =
            [gnd_offs, &normal_mode, &can_add_base, &endp, jp, gndlvl]()
        {
            normal_mode  = true;
            can_add_base = false;   // Nothing left to do, hope for the best
            endp         = {jp(X), jp(Y), gndlvl - gnd_offs };
        };
        
        // We have to check if the bridge is feasible.
        if (bridge_mesh_intersect(jp, dir, radius) < (endp - jp).norm())
            abort_in_shame();
        else {
            // If the new endpoint is below ground, do not make a pillar
            if (endp(Z) < gndlvl)
                endp = endp - SQR2 * (gndlvl - endp(Z)) * dir; // back off
            else {
                
                auto hit = bridge_mesh_intersect(endp, DOWN, radius);
                if (!std::isinf(hit.distance())) abort_in_shame();
                
                pillar_id = m_builder.add_pillar(endp, pgnd, radius);
                
                if (can_add_base)
                    m_builder.add_pillar_base(pillar_id, m_cfg.base_height_mm,
                                              m_cfg.base_radius_mm);
            }
            
            m_builder.add_bridge(jp, endp, radius);
            m_builder.add_junction(endp, radius);
            
            // Add a degenerated pillar and the bridge.
            // The degenerate pillar will have zero length and it will
            // prevent from queries of head_pillar() to have non-existing
            // pillar when the head should have one.
            if (head_id >= 0)
                m_builder.add_pillar(head_id, jp, radius);
        }
    }
    
    if (normal_mode) {
        pillar_id = head_id >= 0 ? m_builder.add_pillar(head_id, endp, radius) :
                                   m_builder.add_pillar(jp, endp, radius);

        if (can_add_base)
            m_builder.add_pillar_base(pillar_id, m_cfg.base_height_mm,
                                      m_cfg.base_radius_mm);
    }
    
    if(pillar_id >= 0) // Save the pillar endpoint in the spatial index
        m_pillar_index.guarded_insert(endp, unsigned(pillar_id));
}

void SupportTreeBuildsteps::filter()
{
    // Get the points that are too close to each other and keep only the
    // first one
    auto aliases = cluster(m_points, D_SP, 2);
    
    PtIndices filtered_indices;
    filtered_indices.reserve(aliases.size());
    m_iheads.reserve(aliases.size());
    m_iheadless.reserve(aliases.size());
    for(auto& a : aliases) {
        // Here we keep only the front point of the cluster.
        filtered_indices.emplace_back(a.front());
    }
    
    // calculate the normals to the triangles for filtered points
    auto nmls = sla::normals(m_points, m_mesh, m_cfg.head_front_radius_mm,
                             m_thr, filtered_indices);
    
    // Not all of the support points have to be a valid position for
    // support creation. The angle may be inappropriate or there may
    // not be enough space for the pinhead. Filtering is applied for
    // these reasons.
    
    using libnest2d::opt::bound;
    using libnest2d::opt::initvals;
    using libnest2d::opt::GeneticOptimizer;
    using libnest2d::opt::StopCriteria;
    
    ccr::SpinningMutex mutex;
    auto addfn = [&mutex](PtIndices &container, unsigned val) {
        std::lock_guard<ccr::SpinningMutex> lk(mutex);
        container.emplace_back(val);
    };
    
    auto filterfn = [this, &nmls, addfn](unsigned fidx, size_t i) {
        m_thr();
        
        auto n = nmls.row(Eigen::Index(i));
        
        // for all normals we generate the spherical coordinates and
        // saturate the polar angle to 45 degrees from the bottom then
        // convert back to standard coordinates to get the new normal.
        // Then we just create a quaternion from the two normals
        // (Quaternion::FromTwoVectors) and apply the rotation to the
        // arrow head.
        
        double z       = n(2);
        double r       = 1.0; // for normalized vector
        double polar   = std::acos(z / r);
        double azimuth = std::atan2(n(1), n(0));
        
        // skip if the tilt is not sane
        if(polar >= PI - m_cfg.normal_cutoff_angle) {
            
            // We saturate the polar angle to 3pi/4
            polar = std::max(polar, 3*PI / 4);
            
            // save the head (pinpoint) position
            Vec3d hp = m_points.row(fidx);
            
            double w = m_cfg.head_width_mm +
                       m_cfg.head_back_radius_mm +
                       2*m_cfg.head_front_radius_mm;
            
            double pin_r = double(m_support_pts[fidx].head_front_radius);
            
            // Reassemble the now corrected normal
            auto nn = Vec3d(std::cos(azimuth) * std::sin(polar),
                            std::sin(azimuth) * std::sin(polar),
                            std::cos(polar)).normalized();
            
            // check available distance
            EigenMesh3D::hit_result t
                = pinhead_mesh_intersect(hp, // touching point
                                         nn, // normal
                                         pin_r,
                                         m_cfg.head_back_radius_mm,
                                         w);
            
            if(t.distance() <= w) {
                
                // Let's try to optimize this angle, there might be a
                // viable normal that doesn't collide with the model
                // geometry and its very close to the default.
                
                StopCriteria stc;
                stc.max_iterations = m_cfg.optimizer_max_iterations;
                stc.relative_score_difference = m_cfg.optimizer_rel_score_diff;
                stc.stop_score = w; // space greater than w is enough
                GeneticOptimizer solver(stc);
                solver.seed(0); // we want deterministic behavior
                
                auto oresult = solver.optimize_max(
                    [this, pin_r, w, hp](double plr, double azm)
                    {
                        auto dir = Vec3d(std::cos(azm) * std::sin(plr),
                                         std::sin(azm) * std::sin(plr),
                                         std::cos(plr)).normalized();
                        
                        double score = pinhead_mesh_intersect(
                            hp, dir, pin_r, m_cfg.head_back_radius_mm, w);
                        
                        return score;
                    },
                    initvals(polar, azimuth), // start with what we have
                    bound(3 * PI / 4,
                          PI),     // Must not exceed the tilt limit
                    bound(-PI, PI) // azimuth can be a full search
                    );
                
                if(oresult.score > w) {
                    polar = std::get<0>(oresult.optimum);
                    azimuth = std::get<1>(oresult.optimum);
                    nn = Vec3d(std::cos(azimuth) * std::sin(polar),
                               std::sin(azimuth) * std::sin(polar),
                               std::cos(polar)).normalized();
                    t = oresult.score;
                }
            }
            
            // save the verified and corrected normal
            m_support_nmls.row(fidx) = nn;
            
            if (t.distance() > w) {
                // Check distance from ground, we might have zero elevation.
                if (hp(Z) + w * nn(Z) < m_builder.ground_level) {
                    addfn(m_iheadless, fidx);
                } else {
                    // mark the point for needing a head.
                    addfn(m_iheads, fidx);
                }
            } else if (polar >= 3 * PI / 4) {
                // Headless supports do not tilt like the headed ones
                // so the normal should point almost to the ground.
                addfn(m_iheadless, fidx);
            }
        }
    };
    
    ccr::enumerate(filtered_indices.begin(), filtered_indices.end(), filterfn);
    
    m_thr();
}

void SupportTreeBuildsteps::add_pinheads()
{
    for (unsigned i : m_iheads) {
        m_thr();
        m_builder.add_head(
            i,
            m_cfg.head_back_radius_mm,
            m_support_pts[i].head_front_radius,
            m_cfg.head_width_mm,
            m_cfg.head_penetration_mm,
            m_support_nmls.row(i),         // dir
            m_support_pts[i].pos.cast<double>() // displacement
            );
    }
}

void SupportTreeBuildsteps::classify()
{
    // We should first get the heads that reach the ground directly
    PtIndices ground_head_indices;
    ground_head_indices.reserve(m_iheads.size());
    m_iheads_onmodel.reserve(m_iheads.size());
    
    // First we decide which heads reach the ground and can be full
    // pillars and which shall be connected to the model surface (or
    // search a suitable path around the surface that leads to the
    // ground -- TODO)
    for(unsigned i : m_iheads) {
        m_thr();
        
        auto& head = m_builder.head(i);
        Vec3d n(0, 0, -1);
        double r = head.r_back_mm;
        Vec3d headjp = head.junction_point();
        
        // collision check
        auto hit = bridge_mesh_intersect(headjp, n, r);
        
        if(std::isinf(hit.distance())) ground_head_indices.emplace_back(i);
        else if(m_cfg.ground_facing_only)  head.invalidate();
        else m_iheads_onmodel.emplace_back(std::make_pair(i, hit));
    }
    
    // We want to search for clusters of points that are far enough
    // from each other in the XY plane to not cross their pillar bases
    // These clusters of support points will join in one pillar,
    // possibly in their centroid support point.
    
    auto pointfn = [this](unsigned i) {
        return m_builder.head(i).junction_point();
    };
    
    auto predicate = [this](const PointIndexEl &e1,
                            const PointIndexEl &e2) {
        double d2d = distance(to_2d(e1.first), to_2d(e2.first));
        double d3d = distance(e1.first, e2.first);
        return d2d < 2 * m_cfg.base_radius_mm
               && d3d < m_cfg.max_bridge_length_mm;
    };

    m_pillar_clusters = cluster(ground_head_indices, pointfn, predicate,
                                m_cfg.max_bridges_on_pillar);
}

void SupportTreeBuildsteps::routing_to_ground()
{
    const double pradius = m_cfg.head_back_radius_mm;
    
    ClusterEl cl_centroids;
    cl_centroids.reserve(m_pillar_clusters.size());
    
    for (auto &cl : m_pillar_clusters) {
        m_thr();
        
        // place all the centroid head positions into the index. We
        // will query for alternative pillar positions. If a sidehead
        // cannot connect to the cluster centroid, we have to search
        // for another head with a full pillar. Also when there are two
        // elements in the cluster, the centroid is arbitrary and the
        // sidehead is allowed to connect to a nearby pillar to
        // increase structural stability.
        
        if (cl.empty()) continue;
        
        // get the current cluster centroid
        auto &      thr    = m_thr;
        const auto &points = m_points;
        long        lcid   = cluster_centroid(
            cl, [&points](size_t idx) { return points.row(long(idx)); },
            [thr](const Vec3d &p1, const Vec3d &p2) {
                thr();
                return distance(Vec2d(p1(X), p1(Y)), Vec2d(p2(X), p2(Y)));
            });
        
        assert(lcid >= 0);
        unsigned hid = cl[size_t(lcid)]; // Head ID
        
        cl_centroids.emplace_back(hid);
        
        Head &h = m_builder.head(hid);
        h.transform();
        
        create_ground_pillar(h.junction_point(), h.dir, h.r_back_mm, h.id);
    }
    
    // now we will go through the clusters ones again and connect the
    // sidepoints with the cluster centroid (which is a ground pillar)
    // or a nearby pillar if the centroid is unreachable.
    size_t ci = 0;
    for (auto cl : m_pillar_clusters) {
        m_thr();
        
        auto cidx = cl_centroids[ci++];
        
        // TODO: don't consider the cluster centroid but calculate a
        // central position where the pillar can be placed. this way
        // the weight is distributed more effectively on the pillar.
        
        auto centerpillarID = m_builder.head_pillar(cidx).id;
        
        for (auto c : cl) {
            m_thr();
            if (c == cidx) continue;
            
            auto &sidehead = m_builder.head(c);
            sidehead.transform();
            
            if (!connect_to_nearpillar(sidehead, centerpillarID) &&
                !search_pillar_and_connect(sidehead)) {
                Vec3d pstart = sidehead.junction_point();
                // Vec3d pend = Vec3d{pstart(X), pstart(Y), gndlvl};
                // Could not find a pillar, create one
                create_ground_pillar(pstart, sidehead.dir, pradius, sidehead.id);
            }
        }
    }
}

void SupportTreeBuildsteps::routing_to_model()
{   
    // We need to check if there is an easy way out to the bed surface.
    // If it can be routed there with a bridge shorter than
    // min_bridge_distance.
    
    // First we want to index the available pillars. The best is to connect
    // these points to the available pillars
    
    auto routedown = [this](Head& head, const Vec3d& dir, double dist)
    {
        head.transform();
        Vec3d endp = head.junction_point() + dist * dir;
        m_builder.add_bridge(head.id, endp);
        m_builder.add_junction(endp, head.r_back_mm);
        
        this->create_ground_pillar(endp, dir, head.r_back_mm);
    };
    
    std::vector<unsigned> modelpillars;
    ccr::SpinningMutex mutex;

    auto onmodelfn =
    [this, routedown, &modelpillars, &mutex]
        (const std::pair<unsigned, EigenMesh3D::hit_result> &el, size_t)
    {        
        m_thr();
        unsigned idx = el.first;
        EigenMesh3D::hit_result hit = el.second;
        
        auto& head = m_builder.head(idx);
        Vec3d hjp = head.junction_point();
        
        // /////////////////////////////////////////////////////////////////
        // Search nearby pillar
        // /////////////////////////////////////////////////////////////////
        
        if(search_pillar_and_connect(head)) { head.transform(); return; }
        
        // /////////////////////////////////////////////////////////////////
        // Try straight path
        // /////////////////////////////////////////////////////////////////
        
        // Cannot connect to nearby pillar. We will try to search for
        // a route to the ground.
        
        double t = bridge_mesh_intersect(hjp, head.dir, head.r_back_mm);
        double d = 0, tdown = 0;
        Vec3d dirdown(0.0, 0.0, -1.0);
        
        t = std::min(t, m_cfg.max_bridge_length_mm);
        
        while(d < t && !std::isinf(tdown = bridge_mesh_intersect(
                                        hjp + d*head.dir,
                                        dirdown, head.r_back_mm))) {
            d += head.r_back_mm;
        }
        
        if(std::isinf(tdown)) { // we heave found a route to the ground
            routedown(head, head.dir, d); return;
        }
        
        // /////////////////////////////////////////////////////////////////
        // Optimize bridge direction
        // /////////////////////////////////////////////////////////////////
        
        // Straight path failed so we will try to search for a suitable
        // direction out of the cavity.
        
        // Get the spherical representation of the normal. its easier to
        // work with.
        double z = head.dir(Z);
        double r = 1.0;     // for normalized vector
        double polar = std::acos(z / r);
        double azimuth = std::atan2(head.dir(Y), head.dir(X));
        
        using libnest2d::opt::bound;
        using libnest2d::opt::initvals;
        using libnest2d::opt::GeneticOptimizer;
        using libnest2d::opt::StopCriteria;
        
        StopCriteria stc;
        stc.max_iterations = m_cfg.optimizer_max_iterations;
        stc.relative_score_difference = m_cfg.optimizer_rel_score_diff;
        stc.stop_score = 1e6;
        GeneticOptimizer solver(stc);
        solver.seed(0); // we want deterministic behavior
        
        double r_back = head.r_back_mm;
        
        auto oresult = solver.optimize_max(
            [this, hjp, r_back](double plr, double azm)
            {
                Vec3d n = Vec3d(std::cos(azm) * std::sin(plr),
                                std::sin(azm) * std::sin(plr),
                                std::cos(plr)).normalized();
                return bridge_mesh_intersect(hjp, n, r_back);
            },
            initvals(polar, azimuth),  // let's start with what we have
            bound(3*PI/4, PI),  // Must not exceed the slope limit
            bound(-PI, PI)      // azimuth can be a full range search
            );
        
        d = 0; t = oresult.score;
        
        polar = std::get<0>(oresult.optimum);
        azimuth = std::get<1>(oresult.optimum);
        Vec3d bridgedir = Vec3d(std::cos(azimuth) * std::sin(polar),
                                std::sin(azimuth) * std::sin(polar),
                                std::cos(polar)).normalized();
        
        t = std::min(t, m_cfg.max_bridge_length_mm);
        
        while(d < t && !std::isinf(tdown = bridge_mesh_intersect(
                                        hjp + d*bridgedir,
                                        dirdown,
                                        head.r_back_mm))) {
            d += head.r_back_mm;
        }
        
        if(std::isinf(tdown)) { // we heave found a route to the ground
            routedown(head, bridgedir, d); return;
        }
        
        // /////////////////////////////////////////////////////////////////
        // Route to model body
        // /////////////////////////////////////////////////////////////////
        
        double zangle = std::asin(hit.direction()(Z));
        zangle = std::max(zangle, PI/4);
        double h = std::sin(zangle) * head.fullwidth();
        
        // The width of the tail head that we would like to have...
        h = std::min(hit.distance() - head.r_back_mm, h);
        
        if(h > 0) {
            Vec3d endp{hjp(X), hjp(Y), hjp(Z) - hit.distance() + h};
            auto center_hit = m_mesh.query_ray_hit(hjp, dirdown);
            
            double hitdiff = center_hit.distance() - hit.distance();
            Vec3d hitp = std::abs(hitdiff) < 2*head.r_back_mm?
                             center_hit.position() : hit.position();
            
            head.transform();

            long pillar_id = m_builder.add_pillar(head.id, endp, head.r_back_mm);
            Pillar &pill = m_builder.pillar(pillar_id);

            Vec3d taildir = endp - hitp;
            double dist = distance(endp, hitp) + m_cfg.head_penetration_mm;
            double w = dist - 2 * head.r_pin_mm - head.r_back_mm;
            
            if (w < 0.) {
                BOOST_LOG_TRIVIAL(error) << "Pinhead width is negative!";
                w = 0.;
            }
                
            Head tailhead(head.r_back_mm,
                          head.r_pin_mm,
                          w,
                          m_cfg.head_penetration_mm,
                          taildir,
                          hitp);
            
            tailhead.transform();
            pill.base = tailhead.mesh;
            
            // Experimental: add the pillar to the index for cascading
            std::lock_guard<ccr::SpinningMutex> lk(mutex);
            modelpillars.emplace_back(unsigned(pill.id));
            return;
        }
        
        // We have failed to route this head.
        BOOST_LOG_TRIVIAL(warning)
            << "Failed to route model facing support point."
            << " ID: " << idx;
        head.invalidate();
    };

    ccr::enumerate(m_iheads_onmodel.begin(), m_iheads_onmodel.end(), onmodelfn);
    
    for(auto pillid : modelpillars) {
        auto& pillar = m_builder.pillar(pillid);
        m_pillar_index.insert(pillar.endpoint(), pillid);
    }
}

void SupportTreeBuildsteps::interconnect_pillars()
{
    // Now comes the algorithm that connects pillars with each other.
    // Ideally every pillar should be connected with at least one of its
    // neighbors if that neighbor is within max_pillar_link_distance
    
    // Pillars with height exceeding H1 will require at least one neighbor
    // to connect with. Height exceeding H2 require two neighbors.
    double H1 = m_cfg.max_solo_pillar_height_mm;
    double H2 = m_cfg.max_dual_pillar_height_mm;
    double d = m_cfg.max_pillar_link_distance_mm;
    
    //A connection between two pillars only counts if the height ratio is
    // bigger than 50%
    double min_height_ratio = 0.5;
    
    std::set<unsigned long> pairs;
    
    // A function to connect one pillar with its neighbors. THe number of
    // neighbors is given in the configuration. This function if called
    // for every pillar in the pillar index. A pair of pillar will not
    // be connected multiple times this is ensured by the 'pairs' set which
    // remembers the processed pillar pairs
    auto cascadefn =
        [this, d, &pairs, min_height_ratio, H1] (const PointIndexEl& el)
    {
        Vec3d qp = el.first;    // endpoint of the pillar
        
        const Pillar& pillar = m_builder.pillar(el.second); // actual pillar
        
        // Get the max number of neighbors a pillar should connect to
        unsigned neighbors = m_cfg.pillar_cascade_neighbors;
        
        // connections are already enough for the pillar
        if(pillar.links >= neighbors) return;
        
        // Query all remaining points within reach
        auto qres = m_pillar_index.query([qp, d](const PointIndexEl& e){
            return distance(e.first, qp) < d;
        });
        
        // sort the result by distance (have to check if this is needed)
        std::sort(qres.begin(), qres.end(),
                  [qp](const PointIndexEl& e1, const PointIndexEl& e2){
                      return distance(e1.first, qp) < distance(e2.first, qp);
                  });
        
        for(auto& re : qres) { // process the queried neighbors
            
            if(re.second == el.second) continue; // Skip self
            
            auto a = el.second, b = re.second;
            
            // Get unique hash for the given pair (order doesn't matter)
            auto hashval = pairhash(a, b);
            
            // Search for the pair amongst the remembered pairs
            if(pairs.find(hashval) != pairs.end()) continue;
            
            const Pillar& neighborpillar = m_builder.pillar(re.second);
            
            // this neighbor is occupied, skip
            if(neighborpillar.links >= neighbors) continue;
            
            if(interconnect(pillar, neighborpillar)) {
                pairs.insert(hashval);
                
                // If the interconnection length between the two pillars is
                // less than 50% of the longer pillar's height, don't count
                if(pillar.height < H1 ||
                    neighborpillar.height / pillar.height > min_height_ratio)
                    m_builder.increment_links(pillar);
                
                if(neighborpillar.height < H1 ||
                    pillar.height / neighborpillar.height > min_height_ratio)
                    m_builder.increment_links(neighborpillar);
                
            }
            
            // connections are enough for one pillar
            if(pillar.links >= neighbors) break;
        }
    };
    
    // Run the cascade for the pillars in the index
    m_pillar_index.foreach(cascadefn);
    
    // We would be done here if we could allow some pillars to not be
    // connected with any neighbors. But this might leave the support tree
    // unprintable.
    //
    // The current solution is to insert additional pillars next to these
    // lonely pillars. One or even two additional pillar might get inserted
    // depending on the length of the lonely pillar.
    
    size_t pillarcount = m_builder.pillarcount();
    
    // Again, go through all pillars, this time in the whole support tree
    // not just the index.
    for(size_t pid = 0; pid < pillarcount; pid++) {
        auto pillar = [this, pid]() { return m_builder.pillar(pid); };
        
        // Decide how many additional pillars will be needed:
        
        unsigned needpillars = 0;
        if (pillar().bridges > m_cfg.max_bridges_on_pillar)
            needpillars = 3;
        else if (pillar().links < 2 && pillar().height > H2) {
            // Not enough neighbors to support this pillar
            needpillars = 2;
        } else if (pillar().links < 1 && pillar().height > H1) {
            // No neighbors could be found and the pillar is too long.
            needpillars = 1;
        }
        
        needpillars = std::max(pillar().links, needpillars) - pillar().links;
        if (needpillars == 0) continue;
        
        // Search for new pillar locations:
        
        bool   found    = false;
        double alpha    = 0; // goes to 2Pi
        double r        = 2 * m_cfg.base_radius_mm;
        Vec3d  pillarsp = pillar().startpoint();
        
        // temp value for starting point detection
        Vec3d sp(pillarsp(X), pillarsp(Y), pillarsp(Z) - r);
        
        // A vector of bool for placement feasbility
        std::vector<bool>  canplace(needpillars, false);
        std::vector<Vec3d> spts(needpillars); // vector of starting points
        
        double gnd      = m_builder.ground_level;
        double min_dist = m_cfg.pillar_base_safety_distance_mm +
                          m_cfg.base_radius_mm + EPSILON;
        
        while(!found && alpha < 2*PI) {
            for (unsigned n = 0;
                 n < needpillars && (!n || canplace[n - 1]);
                 n++)
            {
                double a = alpha + n * PI / 3;
                Vec3d  s = sp;
                s(X) += std::cos(a) * r;
                s(Y) += std::sin(a) * r;
                spts[n] = s;
                
                // Check the path vertically down
                auto hr = bridge_mesh_intersect(s, {0, 0, -1}, pillar().r);
                Vec3d gndsp{s(X), s(Y), gnd};
                
                // If the path is clear, check for pillar base collisions
                canplace[n] = std::isinf(hr.distance()) &&
                              std::sqrt(m_mesh.squared_distance(gndsp)) >
                                  min_dist;
            }
            
            found = std::all_of(canplace.begin(), canplace.end(),
                                [](bool v) { return v; });
            
            // 20 angles will be tried...
            alpha += 0.1 * PI;
        }
        
        std::vector<long> newpills;
        newpills.reserve(needpillars);

        if (found)
            for (unsigned n = 0; n < needpillars; n++) {
                Vec3d  s = spts[n];
                Pillar p(s, Vec3d(s(X), s(Y), gnd), pillar().r);
                p.add_base(m_cfg.base_height_mm, m_cfg.base_radius_mm);

                if (interconnect(pillar(), p)) {
                    Pillar &pp = m_builder.pillar(m_builder.add_pillar(p));
                    m_pillar_index.insert(pp.endpoint(), unsigned(pp.id));

                    m_builder.add_junction(s, pillar().r);
                    double t = bridge_mesh_intersect(pillarsp,
                                                     dirv(pillarsp, s),
                                                     pillar().r);
                    if (distance(pillarsp, s) < t)
                        m_builder.add_bridge(pillarsp, s, pillar().r);

                    if (pillar().endpoint()(Z) > m_builder.ground_level)
                        m_builder.add_junction(pillar().endpoint(),
                                               pillar().r);

                    newpills.emplace_back(pp.id);
                    m_builder.increment_links(pillar());
                    m_builder.increment_links(pp);
                }
            }

        if(!newpills.empty()) {
            for(auto it = newpills.begin(), nx = std::next(it);
                 nx != newpills.end(); ++it, ++nx) {
                const Pillar& itpll = m_builder.pillar(*it);
                const Pillar& nxpll = m_builder.pillar(*nx);
                if(interconnect(itpll, nxpll)) {
                    m_builder.increment_links(itpll);
                    m_builder.increment_links(nxpll);
                }
            }
            
            m_pillar_index.foreach(cascadefn);
        }
    }
}

void SupportTreeBuildsteps::routing_headless()
{
    // For now we will just generate smaller headless sticks with a sharp
    // ending point that connects to the mesh surface.
    
    // We will sink the pins into the model surface for a distance of 1/3 of
    // the pin radius
    for(unsigned i : m_iheadless) {
        m_thr();
        
        const auto R = double(m_support_pts[i].head_front_radius);
        const double HWIDTH_MM = m_cfg.head_penetration_mm;
        
        // Exact support position
        Vec3d sph = m_support_pts[i].pos.cast<double>();
        Vec3d n = m_support_nmls.row(i);   // mesh outward normal
        Vec3d sp = sph - n * HWIDTH_MM;     // stick head start point
        
        Vec3d dir = {0, 0, -1};
        Vec3d sj = sp + R * n;              // stick start point
        
        // This is only for checking
        double idist = bridge_mesh_intersect(sph, dir, R, true);
        double realdist = ray_mesh_intersect(sj, dir);
        double dist = realdist;
        
        if (std::isinf(dist)) dist = sph(Z) - m_builder.ground_level;
        
        if(std::isnan(idist) || idist < 2*R || std::isnan(dist) || dist < 2*R) {
            BOOST_LOG_TRIVIAL(warning) << "Can not find route for headless"
                                       << " support stick at: "
                                       << sj.transpose();
            continue;
        }
        
        bool use_endball = !std::isinf(realdist);
        Vec3d ej = sj + (dist + HWIDTH_MM) * dir;
        m_builder.add_compact_bridge(sp, ej, n, R, use_endball);
    }
}

}
}
