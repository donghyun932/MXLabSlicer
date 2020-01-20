#include "GCodeTimeEstimator.hpp"
#include "Utils.hpp"
#include <boost/bind.hpp>
#include <cmath>

#include <Shiny/Shiny.h>

#include <boost/nowide/fstream.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/algorithm/string/predicate.hpp>

static const float MMMIN_TO_MMSEC = 1.0f / 60.0f;
static const float MILLISEC_TO_SEC = 0.001f;
static const float INCHES_TO_MM = 25.4f;

static const float DEFAULT_FEEDRATE = 1500.0f; // from Prusa Firmware (Marlin_main.cpp)
static const float DEFAULT_ACCELERATION = 1500.0f; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_RETRACT_ACCELERATION = 1500.0f; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_AXIS_MAX_FEEDRATE[] = { 500.0f, 500.0f, 12.0f, 120.0f }; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_AXIS_MAX_ACCELERATION[] = { 9000.0f, 9000.0f, 500.0f, 10000.0f }; // Prusa Firmware 1_75mm_MK2
static const float DEFAULT_AXIS_MAX_JERK[] = { 10.0f, 10.0f, 0.4f, 2.5f }; // from Prusa Firmware (Configuration.h)
static const float DEFAULT_MINIMUM_FEEDRATE = 0.0f; // from Prusa Firmware (Configuration_adv.h)
static const float DEFAULT_MINIMUM_TRAVEL_FEEDRATE = 0.0f; // from Prusa Firmware (Configuration_adv.h)
static const float DEFAULT_EXTRUDE_FACTOR_OVERRIDE_PERCENTAGE = 1.0f; // 100 percent

static const float PREVIOUS_FEEDRATE_THRESHOLD = 0.0001f;

#if ENABLE_MOVE_STATS
static const std::string MOVE_TYPE_STR[Slic3r::GCodeTimeEstimator::Block::Num_Types] =
{
    "Noop",
    "Retract",
    "Unretract",
    "Tool_change",
    "Move",
    "Extrude"
};
#endif // ENABLE_MOVE_STATS

namespace Slic3r {
    void GCodeTimeEstimator::Feedrates::reset()
    {
        feedrate = 0.0f;
        safe_feedrate = 0.0f;
        ::memset(axis_feedrate, 0, Num_Axis * sizeof(float));
        ::memset(abs_axis_feedrate, 0, Num_Axis * sizeof(float));
    }

    float GCodeTimeEstimator::Block::Trapezoid::acceleration_time(float acceleration) const
    {
        return acceleration_time_from_distance(feedrate.entry, accelerate_until, acceleration);
    }

    float GCodeTimeEstimator::Block::Trapezoid::cruise_time() const
    {
        return (feedrate.cruise != 0.0f) ? cruise_distance() / feedrate.cruise : 0.0f;
    }

    float GCodeTimeEstimator::Block::Trapezoid::deceleration_time(float acceleration) const
    {
        return acceleration_time_from_distance(feedrate.cruise, (distance - decelerate_after), -acceleration);
    }

    float GCodeTimeEstimator::Block::Trapezoid::cruise_distance() const
    {
        return decelerate_after - accelerate_until;
    }

    float GCodeTimeEstimator::Block::Trapezoid::acceleration_time_from_distance(float initial_feedrate, float distance, float acceleration)
    {
        return (acceleration != 0.0f) ? (speed_from_distance(initial_feedrate, distance, acceleration) - initial_feedrate) / acceleration : 0.0f;
    }

    float GCodeTimeEstimator::Block::Trapezoid::speed_from_distance(float initial_feedrate, float distance, float acceleration)
    {
        // to avoid invalid negative numbers due to numerical imprecision 
        float value = std::max(0.0f, sqr(initial_feedrate) + 2.0f * acceleration * distance);
        return ::sqrt(value);
    }

    GCodeTimeEstimator::Block::Block()
    {
    }

    float GCodeTimeEstimator::Block::move_length() const
    {
        float length = ::sqrt(sqr(delta_pos[X]) + sqr(delta_pos[Y]) + sqr(delta_pos[Z]));
        return (length > 0.0f) ? length : std::abs(delta_pos[E]);
    }

    float GCodeTimeEstimator::Block::is_extruder_only_move() const
    {
        return (delta_pos[X] == 0.0f) && (delta_pos[Y] == 0.0f) && (delta_pos[Z] == 0.0f) && (delta_pos[E] != 0.0f);
    }

    float GCodeTimeEstimator::Block::is_travel_move() const
    {
        return delta_pos[E] == 0.0f;
    }

    float GCodeTimeEstimator::Block::acceleration_time() const
    {
        return trapezoid.acceleration_time(acceleration);
    }

    float GCodeTimeEstimator::Block::cruise_time() const
    {
        return trapezoid.cruise_time();
    }

    float GCodeTimeEstimator::Block::deceleration_time() const
    {
        return trapezoid.deceleration_time(acceleration);
    }

    float GCodeTimeEstimator::Block::cruise_distance() const
    {
        return trapezoid.cruise_distance();
    }

    void GCodeTimeEstimator::Block::calculate_trapezoid()
    {
        float distance = move_length();

        trapezoid.distance = distance;
        trapezoid.feedrate = feedrate;

        float accelerate_distance = std::max(0.0f, estimate_acceleration_distance(feedrate.entry, feedrate.cruise, acceleration));
        float decelerate_distance = std::max(0.0f, estimate_acceleration_distance(feedrate.cruise, feedrate.exit, -acceleration));
        float cruise_distance = distance - accelerate_distance - decelerate_distance;

        // Not enough space to reach the nominal feedrate.
        // This means no cruising, and we'll have to use intersection_distance() to calculate when to abort acceleration 
        // and start braking in order to reach the exit_feedrate exactly at the end of this block.
        if (cruise_distance < 0.0f)
        {
            accelerate_distance = clamp(0.0f, distance, intersection_distance(feedrate.entry, feedrate.exit, acceleration, distance));
            cruise_distance = 0.0f;
            trapezoid.feedrate.cruise = Trapezoid::speed_from_distance(feedrate.entry, accelerate_distance, acceleration);
        }

        trapezoid.accelerate_until = accelerate_distance;
        trapezoid.decelerate_after = accelerate_distance + cruise_distance;
    }

    float GCodeTimeEstimator::Block::max_allowable_speed(float acceleration, float target_velocity, float distance)
    {
        // to avoid invalid negative numbers due to numerical imprecision 
        float value = std::max(0.0f, sqr(target_velocity) - 2.0f * acceleration * distance);
        return ::sqrt(value);
    }

    float GCodeTimeEstimator::Block::estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
    {
        return (acceleration == 0.0f) ? 0.0f : (sqr(target_rate) - sqr(initial_rate)) / (2.0f * acceleration);
    }

    float GCodeTimeEstimator::Block::intersection_distance(float initial_rate, float final_rate, float acceleration, float distance)
    {
        return (acceleration == 0.0f) ? 0.0f : (2.0f * acceleration * distance - sqr(initial_rate) + sqr(final_rate)) / (4.0f * acceleration);
    }

#if ENABLE_MOVE_STATS
    GCodeTimeEstimator::MoveStats::MoveStats()
        : count(0)
        , time(0.0f)
    {
    }
#endif // ENABLE_MOVE_STATS

    const std::string GCodeTimeEstimator::Normal_First_M73_Output_Placeholder_Tag = "; _TE_NORMAL_FIRST_M73_OUTPUT_PLACEHOLDER";
    const std::string GCodeTimeEstimator::Silent_First_M73_Output_Placeholder_Tag = "; _TE_SILENT_FIRST_M73_OUTPUT_PLACEHOLDER";
    const std::string GCodeTimeEstimator::Normal_Last_M73_Output_Placeholder_Tag = "; _TE_NORMAL_LAST_M73_OUTPUT_PLACEHOLDER";
    const std::string GCodeTimeEstimator::Silent_Last_M73_Output_Placeholder_Tag = "; _TE_SILENT_LAST_M73_OUTPUT_PLACEHOLDER";

    const std::string GCodeTimeEstimator::Color_Change_Tag = "PRINT_COLOR_CHANGE";

    GCodeTimeEstimator::GCodeTimeEstimator(EMode mode)
        : m_mode(mode)
    {
        reset();
        set_default();
    }

    void GCodeTimeEstimator::add_gcode_line(const std::string& gcode_line)
    {
        PROFILE_FUNC();
        m_parser.parse_line(gcode_line, 
            [this](GCodeReader &reader, const GCodeReader::GCodeLine &line)
        { this->_process_gcode_line(reader, line); });
    }

    void GCodeTimeEstimator::add_gcode_block(const char *ptr)
    {
        PROFILE_FUNC();
        GCodeReader::GCodeLine gline;
        auto action = [this](GCodeReader &reader, const GCodeReader::GCodeLine &line)
        { this->_process_gcode_line(reader, line); };
        for (; *ptr != 0;) {
            gline.reset();
            ptr = m_parser.parse_line(ptr, gline, action);
        }
    }

    void GCodeTimeEstimator::calculate_time(bool start_from_beginning)
    {
        PROFILE_FUNC();
        if (start_from_beginning)
        {
            _reset_time();
            m_last_st_synchronized_block_id = -1;
        }
        _calculate_time();

        if (m_needs_color_times && (m_color_time_cache != 0.0f))
            m_color_times.push_back(m_color_time_cache);

#if ENABLE_MOVE_STATS
        _log_moves_stats();
#endif // ENABLE_MOVE_STATS
    }

    void GCodeTimeEstimator::calculate_time_from_text(const std::string& gcode)
    {
        reset();

        m_parser.parse_buffer(gcode,
            [this](GCodeReader &reader, const GCodeReader::GCodeLine &line)
        { this->_process_gcode_line(reader, line); });

        _calculate_time();

        if (m_needs_color_times && (m_color_time_cache != 0.0f))
            m_color_times.push_back(m_color_time_cache);

#if ENABLE_MOVE_STATS
        _log_moves_stats();
#endif // ENABLE_MOVE_STATS
    }

    void GCodeTimeEstimator::calculate_time_from_file(const std::string& file)
    {
        reset();

        m_parser.parse_file(file, boost::bind(&GCodeTimeEstimator::_process_gcode_line, this, _1, _2));
        _calculate_time();

        if (m_needs_color_times && (m_color_time_cache != 0.0f))
            m_color_times.push_back(m_color_time_cache);

#if ENABLE_MOVE_STATS
        _log_moves_stats();
#endif // ENABLE_MOVE_STATS
    }

    void GCodeTimeEstimator::calculate_time_from_lines(const std::vector<std::string>& gcode_lines)
    {
        reset();

        auto action = [this](GCodeReader &reader, const GCodeReader::GCodeLine &line)
        { this->_process_gcode_line(reader, line); };
        for (const std::string& line : gcode_lines)
            m_parser.parse_line(line, action);
        _calculate_time();

        if (m_needs_color_times && (m_color_time_cache != 0.0f))
            m_color_times.push_back(m_color_time_cache);

#if ENABLE_MOVE_STATS
        _log_moves_stats();
#endif // ENABLE_MOVE_STATS
    }

    bool GCodeTimeEstimator::post_process(const std::string& filename, float interval_sec, const PostProcessData* const normal_mode, const PostProcessData* const silent_mode)
    {
        boost::nowide::ifstream in(filename);
        if (!in.good())
            throw std::runtime_error(std::string("Time estimator post process export failed.\nCannot open file for reading.\n"));

        std::string path_tmp = filename + ".postprocess";

        FILE* out = boost::nowide::fopen(path_tmp.c_str(), "wb");
        if (out == nullptr)
            throw std::runtime_error(std::string("Time estimator post process export failed.\nCannot open file for writing.\n"));

        std::string normal_time_mask = "M73 P%s R%s\n";
        std::string silent_time_mask = "M73 Q%s S%s\n";
        char line_M73[64];

        std::string gcode_line;
        // buffer line to export only when greater than 64K to reduce writing calls
        std::string export_line;

        // helper function to write to disk
        auto write_string = [&](const std::string& str) {
            fwrite((const void*)export_line.c_str(), 1, export_line.length(), out);
            if (ferror(out))
            {
                in.close();
                fclose(out);
                boost::nowide::remove(path_tmp.c_str());
                throw std::runtime_error(std::string("Time estimator post process export failed.\nIs the disk full?\n"));
            }
            export_line.clear();
        };

        GCodeReader parser;
        unsigned int g1_lines_count = 0;
        int normal_g1_line_id = 0;
        float normal_last_recorded_time = 0.0f;
        int silent_g1_line_id = 0;
        float silent_last_recorded_time = 0.0f;

        // helper function to process g1 lines
        auto process_g1_line = [&](const PostProcessData* const data, const GCodeReader::GCodeLine& line, int& g1_line_id, float& last_recorded_time, const std::string& time_mask) {
            if (data == nullptr)
                return;

            assert((g1_line_id >= (int)data->g1_line_ids.size()) || (data->g1_line_ids[g1_line_id].first >= g1_lines_count));
            const Block* block = nullptr;
            if (g1_line_id < (int)data->g1_line_ids.size())
            {
                const G1LineIdToBlockId& map_item = data->g1_line_ids[g1_line_id];
                if (map_item.first == g1_lines_count)
                {
                    if (line.has_e() && (map_item.second < (unsigned int)data->blocks.size()))
                        block = &data->blocks[map_item.second];
                    ++g1_line_id;
                }
            }

            if ((block != nullptr) && (block->elapsed_time != -1.0f))
            {
                float block_remaining_time = data->time - block->elapsed_time;
                if (std::abs(last_recorded_time - block_remaining_time) > interval_sec)
                {
                    sprintf(line_M73, time_mask.c_str(), std::to_string((int)(100.0f * block->elapsed_time / data->time)).c_str(), _get_time_minutes(block_remaining_time).c_str());
                    gcode_line += line_M73;

                    last_recorded_time = block_remaining_time;
                }
            }
        };

        while (std::getline(in, gcode_line))
        {
            if (!in.good())
            {
                fclose(out);
                throw std::runtime_error(std::string("Time estimator post process export failed.\nError while reading from file.\n"));
            }

            // check tags
            // remove color change tag
            if (gcode_line == "; " + Color_Change_Tag)
                continue;

            // replaces placeholders for initial line M73 with the real lines
            if ((normal_mode != nullptr) && (gcode_line == Normal_First_M73_Output_Placeholder_Tag))
            {
                sprintf(line_M73, normal_time_mask.c_str(), "0", _get_time_minutes(normal_mode->time).c_str());
                gcode_line = line_M73;
            }
            else if ((silent_mode != nullptr) && (gcode_line == Silent_First_M73_Output_Placeholder_Tag))
            {
                sprintf(line_M73, silent_time_mask.c_str(), "0", _get_time_minutes(silent_mode->time).c_str());
                gcode_line = line_M73;
            }
            // replaces placeholders for final line M73 with the real lines
            else if ((normal_mode != nullptr) && (gcode_line == Normal_Last_M73_Output_Placeholder_Tag))
            {
                sprintf(line_M73, normal_time_mask.c_str(), "100", "0");
                gcode_line = line_M73;
            }
            else if ((silent_mode != nullptr) && (gcode_line == Silent_Last_M73_Output_Placeholder_Tag))
            {
                sprintf(line_M73, silent_time_mask.c_str(), "100", "0");
                gcode_line = line_M73;
            }
            else
                gcode_line += "\n";

            // add remaining time lines where needed
            parser.parse_line(gcode_line,
                [&](GCodeReader& reader, const GCodeReader::GCodeLine& line)
                {
                    if (line.cmd_is("G1"))
                    {
                        ++g1_lines_count;
                        process_g1_line(silent_mode, line, silent_g1_line_id, silent_last_recorded_time, silent_time_mask);
                        process_g1_line(normal_mode, line, normal_g1_line_id, normal_last_recorded_time, normal_time_mask);
                    }
                });

            export_line += gcode_line;
            if (export_line.length() > 65535)
                write_string(export_line);
        }

        if (!export_line.empty())
            write_string(export_line);

        fclose(out);
        in.close();

        if (rename_file(path_tmp, filename))
            throw std::runtime_error(std::string("Failed to rename the output G-code file from ") + path_tmp + " to " + filename + '\n' +
                "Is " + path_tmp + " locked?" + '\n');

        return true;
    }

    void GCodeTimeEstimator::set_axis_position(EAxis axis, float position)
    {
        m_state.axis[axis].position = position;
    }

    void GCodeTimeEstimator::set_axis_origin(EAxis axis, float position)
    {
        m_state.axis[axis].origin = position;
    }

    void GCodeTimeEstimator::set_axis_max_feedrate(EAxis axis, float feedrate_mm_sec)
    {
        m_state.axis[axis].max_feedrate = feedrate_mm_sec;
    }

    void GCodeTimeEstimator::set_axis_max_acceleration(EAxis axis, float acceleration)
    {
        m_state.axis[axis].max_acceleration = acceleration;
    }

    void GCodeTimeEstimator::set_axis_max_jerk(EAxis axis, float jerk)
    {
        m_state.axis[axis].max_jerk = jerk;
    }

    float GCodeTimeEstimator::get_axis_position(EAxis axis) const
    {
        return m_state.axis[axis].position;
    }

    float GCodeTimeEstimator::get_axis_origin(EAxis axis) const
    {
        return m_state.axis[axis].origin;
    }

    float GCodeTimeEstimator::get_axis_max_feedrate(EAxis axis) const
    {
        return m_state.axis[axis].max_feedrate;
    }

    float GCodeTimeEstimator::get_axis_max_acceleration(EAxis axis) const
    {
        return m_state.axis[axis].max_acceleration;
    }

    float GCodeTimeEstimator::get_axis_max_jerk(EAxis axis) const
    {
        return m_state.axis[axis].max_jerk;
    }

    void GCodeTimeEstimator::set_feedrate(float feedrate_mm_sec)
    {
        m_state.feedrate = feedrate_mm_sec;
    }

    float GCodeTimeEstimator::get_feedrate() const
    {
        return m_state.feedrate;
    }

    void GCodeTimeEstimator::set_acceleration(float acceleration_mm_sec2)
    {
        m_state.acceleration = (m_state.max_acceleration == 0) ? 
            acceleration_mm_sec2 : 
            // Clamp the acceleration with the maximum.
            std::min(m_state.max_acceleration, acceleration_mm_sec2);
    }

    float GCodeTimeEstimator::get_acceleration() const
    {
        return m_state.acceleration;
    }

    void GCodeTimeEstimator::set_max_acceleration(float acceleration_mm_sec2)
    {
        m_state.max_acceleration = acceleration_mm_sec2;
        if (acceleration_mm_sec2 > 0)
            m_state.acceleration = acceleration_mm_sec2;
    }

    float GCodeTimeEstimator::get_max_acceleration() const
    {
        return m_state.max_acceleration;
    }

    void GCodeTimeEstimator::set_retract_acceleration(float acceleration_mm_sec2)
    {
        m_state.retract_acceleration = acceleration_mm_sec2;
    }

    float GCodeTimeEstimator::get_retract_acceleration() const
    {
        return m_state.retract_acceleration;
    }

    void GCodeTimeEstimator::set_minimum_feedrate(float feedrate_mm_sec)
    {
        m_state.minimum_feedrate = feedrate_mm_sec;
    }

    float GCodeTimeEstimator::get_minimum_feedrate() const
    {
        return m_state.minimum_feedrate;
    }

    void GCodeTimeEstimator::set_minimum_travel_feedrate(float feedrate_mm_sec)
    {
        m_state.minimum_travel_feedrate = feedrate_mm_sec;
    }

    float GCodeTimeEstimator::get_minimum_travel_feedrate() const
    {
        return m_state.minimum_travel_feedrate;
    }

    void GCodeTimeEstimator::set_filament_load_times(const std::vector<double> &filament_load_times)
    {
        m_state.filament_load_times.clear();
        for (double t : filament_load_times)
            m_state.filament_load_times.push_back((float)t);
    }

    void GCodeTimeEstimator::set_filament_unload_times(const std::vector<double> &filament_unload_times)
    {
        m_state.filament_unload_times.clear();
        for (double t : filament_unload_times)
            m_state.filament_unload_times.push_back((float)t);
    }

    float GCodeTimeEstimator::get_filament_load_time(unsigned int id_extruder)
    {
        return
            (m_state.filament_load_times.empty() || id_extruder == m_state.extruder_id_unloaded) ? 
                0 :
                (m_state.filament_load_times.size() <= id_extruder) ?
                    m_state.filament_load_times.front() : 
                    m_state.filament_load_times[id_extruder];
    }

    float GCodeTimeEstimator::get_filament_unload_time(unsigned int id_extruder)
    {
        return
            (m_state.filament_unload_times.empty() || id_extruder == m_state.extruder_id_unloaded) ? 
                0 :
                (m_state.filament_unload_times.size() <= id_extruder) ?
                    m_state.filament_unload_times.front() : 
                    m_state.filament_unload_times[id_extruder];
    }

    void GCodeTimeEstimator::set_extrude_factor_override_percentage(float percentage)
    {
        m_state.extrude_factor_override_percentage = percentage;
    }

    float GCodeTimeEstimator::get_extrude_factor_override_percentage() const
    {
        return m_state.extrude_factor_override_percentage;
    }

    void GCodeTimeEstimator::set_dialect(GCodeFlavor dialect)
    {
        m_state.dialect = dialect;
    }

    GCodeFlavor GCodeTimeEstimator::get_dialect() const
    {
        PROFILE_FUNC();
        return m_state.dialect;
    }

    void GCodeTimeEstimator::set_units(GCodeTimeEstimator::EUnits units)
    {
        m_state.units = units;
    }

    GCodeTimeEstimator::EUnits GCodeTimeEstimator::get_units() const
    {
        return m_state.units;
    }

    void GCodeTimeEstimator::set_global_positioning_type(GCodeTimeEstimator::EPositioningType type)
    {
        m_state.global_positioning_type = type;
    }

    GCodeTimeEstimator::EPositioningType GCodeTimeEstimator::get_global_positioning_type() const
    {
        return m_state.global_positioning_type;
    }

    void GCodeTimeEstimator::set_e_local_positioning_type(GCodeTimeEstimator::EPositioningType type)
    {
        m_state.e_local_positioning_type = type;
    }

    GCodeTimeEstimator::EPositioningType GCodeTimeEstimator::get_e_local_positioning_type() const
    {
        return m_state.e_local_positioning_type;
    }

    int GCodeTimeEstimator::get_g1_line_id() const
    {
        return m_state.g1_line_id;
    }

    void GCodeTimeEstimator::increment_g1_line_id()
    {
        ++m_state.g1_line_id;
    }

    void GCodeTimeEstimator::reset_g1_line_id()
    {
        m_state.g1_line_id = 0;
    }

    void GCodeTimeEstimator::set_extruder_id(unsigned int id)
    {
        m_state.extruder_id = id;
    }

    unsigned int GCodeTimeEstimator::get_extruder_id() const
    {
        return m_state.extruder_id;
    }

    void GCodeTimeEstimator::reset_extruder_id()
    {
        // Set the initial extruder ID to unknown. For the multi-material setup it means
        // that all the filaments are parked in the MMU and no filament is loaded yet.
        m_state.extruder_id = m_state.extruder_id_unloaded;
    }

    void GCodeTimeEstimator::add_additional_time(float timeSec)
    {
        PROFILE_FUNC();
        m_state.additional_time += timeSec;
    }

    void GCodeTimeEstimator::set_additional_time(float timeSec)
    {
        m_state.additional_time = timeSec;
    }

    float GCodeTimeEstimator::get_additional_time() const
    {
        return m_state.additional_time;
    }

    void GCodeTimeEstimator::set_default()
    {
        set_units(Millimeters);
        set_dialect(gcfRepRap);
        set_global_positioning_type(Absolute);
        set_e_local_positioning_type(Absolute);

        set_feedrate(DEFAULT_FEEDRATE);
        // Setting the maximum acceleration to zero means that the there is no limit and the G-code
        // is allowed to set excessive values.
        set_max_acceleration(0);
        set_acceleration(DEFAULT_ACCELERATION);
        set_retract_acceleration(DEFAULT_RETRACT_ACCELERATION);
        set_minimum_feedrate(DEFAULT_MINIMUM_FEEDRATE);
        set_minimum_travel_feedrate(DEFAULT_MINIMUM_TRAVEL_FEEDRATE);
        set_extrude_factor_override_percentage(DEFAULT_EXTRUDE_FACTOR_OVERRIDE_PERCENTAGE);
        
        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            EAxis axis = (EAxis)a;
            set_axis_max_feedrate(axis, DEFAULT_AXIS_MAX_FEEDRATE[a]);
            set_axis_max_acceleration(axis, DEFAULT_AXIS_MAX_ACCELERATION[a]);
            set_axis_max_jerk(axis, DEFAULT_AXIS_MAX_JERK[a]);
        }

        m_state.filament_load_times.clear();
        m_state.filament_unload_times.clear();
    }

    void GCodeTimeEstimator::reset()
    {
        _reset_time();
#if ENABLE_MOVE_STATS
        _moves_stats.clear();
#endif // ENABLE_MOVE_STATS
        _reset_blocks();
        _reset();
    }

    float GCodeTimeEstimator::get_time() const
    {
        return m_time;
    }

    std::string GCodeTimeEstimator::get_time_dhms() const
    {
        return _get_time_dhms(get_time());
    }

    std::string GCodeTimeEstimator::get_time_minutes() const
    {
        return _get_time_minutes(get_time());
    }

    std::vector<float> GCodeTimeEstimator::get_color_times() const
    {
        return m_color_times;
    }

    std::vector<std::string> GCodeTimeEstimator::get_color_times_dhms(bool include_remaining) const
    {
        std::vector<std::string> ret;
        float total_time = 0.0f;
        for (float t : m_color_times)
        {
            std::string time = _get_time_dhms(t);
            if (include_remaining)
            {
                time += " (";
                time += _get_time_dhms(m_time - total_time);
                time += ")";
            }
            total_time += t;
            ret.push_back(time);
        }
        return ret;
    }

    std::vector<std::string> GCodeTimeEstimator::get_color_times_minutes(bool include_remaining) const
    {
        std::vector<std::string> ret;
        float total_time = 0.0f;
        for (float t : m_color_times)
        {
            std::string time = _get_time_minutes(t);
            if (include_remaining)
            {
                time += " (";
                time += _get_time_minutes(m_time - total_time);
                time += ")";
            }
            total_time += t;
        }
        return ret;
    }

    // Return an estimate of the memory consumed by the time estimator.
	size_t GCodeTimeEstimator::memory_used() const
    {
        size_t out = sizeof(*this);
		out += SLIC3R_STDVEC_MEMSIZE(this->m_blocks, Block);
		out += SLIC3R_STDVEC_MEMSIZE(this->m_g1_line_ids, G1LineIdToBlockId);
        return out;
    }

    void GCodeTimeEstimator::_reset()
    {
        m_curr.reset();
        m_prev.reset();

        set_axis_position(X, 0.0f);
        set_axis_position(Y, 0.0f);
        set_axis_position(Z, 0.0f);
        set_axis_origin(X, 0.0f);
        set_axis_origin(Y, 0.0f);
        set_axis_origin(Z, 0.0f);

        if (get_e_local_positioning_type() == Absolute)
            set_axis_position(E, 0.0f);

        set_additional_time(0.0f);

        reset_extruder_id();
        reset_g1_line_id();
        m_g1_line_ids.clear();

        m_last_st_synchronized_block_id = -1;

        m_needs_color_times = false;
        m_color_times.clear();
        m_color_time_cache = 0.0f;
    }

    void GCodeTimeEstimator::_reset_time()
    {
        m_time = 0.0f;
    }

    void GCodeTimeEstimator::_reset_blocks()
    {
        m_blocks.clear();
    }

    void GCodeTimeEstimator::_calculate_time()
    {
        PROFILE_FUNC();
        _forward_pass();
        _reverse_pass();
        _recalculate_trapezoids();

        m_time += get_additional_time();
        m_color_time_cache += get_additional_time();

        for (int i = m_last_st_synchronized_block_id + 1; i < (int)m_blocks.size(); ++i)
        {
            Block& block = m_blocks[i];
            float block_time = 0.0f;
            block_time += block.acceleration_time();
            block_time += block.cruise_time();
            block_time += block.deceleration_time();
            m_time += block_time;
            block.elapsed_time = m_time;

#if ENABLE_MOVE_STATS
            MovesStatsMap::iterator it = _moves_stats.find(block.move_type);
            if (it == _moves_stats.end())
                it = _moves_stats.insert(MovesStatsMap::value_type(block.move_type, MoveStats())).first;

            it->second.count += 1;
            it->second.time += block_time;
#endif // ENABLE_MOVE_STATS

            m_color_time_cache += block_time;
        }

        m_last_st_synchronized_block_id = (int)m_blocks.size() - 1;
        // The additional time has been consumed (added to the total time), reset it to zero.
        set_additional_time(0.);
    }

    void GCodeTimeEstimator::_process_gcode_line(GCodeReader&, const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();

        // processes 'special' comments contained in line
        if (_process_tags(line))
            return;

        std::string cmd = line.cmd();
        if (cmd.length() > 1)
        {
            switch (::toupper(cmd[0]))
            {
            case 'G':
                {
                    switch (::atoi(&cmd[1]))
                    {
                    case 1: // Move
                        {
                            _processG1(line);
                            break;
                        }
                    case 4: // Dwell
                        {
                            _processG4(line);
                            break;
                        }
                    case 20: // Set Units to Inches
                        {
                            _processG20(line);
                            break;
                        }
                    case 21: // Set Units to Millimeters
                        {
                            _processG21(line);
                            break;
                        }
                    case 28: // Move to Origin (Home)
                        {
                            _processG28(line);
                            break;
                        }
                    case 90: // Set to Absolute Positioning
                        {
                            _processG90(line);
                            break;
                        }
                    case 91: // Set to Relative Positioning
                        {
                            _processG91(line);
                            break;
                        }
                    case 92: // Set Position
                        {
                            _processG92(line);
                            break;
                        }
                    }

                    break;
                }
            case 'M':
                {
                    switch (::atoi(&cmd[1]))
                    {
                    case 1: // Sleep or Conditional stop
                        {
                            _processM1(line);
                            break;
                        }
                    case 82: // Set extruder to absolute mode
                        {
                            _processM82(line);
                            break;
                        }
                    case 83: // Set extruder to relative mode
                        {
                            _processM83(line);
                            break;
                        }
                    case 109: // Set Extruder Temperature and Wait
                        {
                            _processM109(line);
                            break;
                        }
                    case 201: // Set max printing acceleration
                        {
                            _processM201(line);
                            break;
                        }
                    case 203: // Set maximum feedrate
                        {
                            _processM203(line);
                            break;
                        }
                    case 204: // Set default acceleration
                        {
                            _processM204(line);
                            break;
                        }
                    case 205: // Advanced settings
                        {
                            _processM205(line);
                            break;
                        }
                    case 221: // Set extrude factor override percentage
                        {
                            _processM221(line);
                            break;
                        }
                    case 566: // Set allowable instantaneous speed change
                        {
                            _processM566(line);
                            break;
                        }
                    case 702: // MK3 MMU2: Process the final filament unload.
                        {
                            _processM702(line);
                            break;
                        }
                    }

                    break;
                }
            case 'T': // Select Tools
                {
                    _processT(line);
                    break;
                }
            }
        }
    }

    void GCodeTimeEstimator::_processG1(const GCodeReader::GCodeLine& line)
    {
        auto axis_absolute_position = [this](GCodeTimeEstimator::EAxis axis, const GCodeReader::GCodeLine& lineG1) -> float
        {
            float current_absolute_position = get_axis_position(axis);
            float current_origin = get_axis_origin(axis);
            float lengthsScaleFactor = (get_units() == GCodeTimeEstimator::Inches) ? INCHES_TO_MM : 1.0f;

            bool is_relative = (get_global_positioning_type() == Relative);
            if (axis == E)
                is_relative |= (get_e_local_positioning_type() == Relative);

            if (lineG1.has(Slic3r::Axis(axis)))
            {
                float ret = lineG1.value(Slic3r::Axis(axis)) * lengthsScaleFactor;
                return is_relative ? current_absolute_position + ret : ret + current_origin;
            }
            else
                return current_absolute_position;
        };

        PROFILE_FUNC();
        increment_g1_line_id();

        // updates axes positions from line
        float new_pos[Num_Axis];
        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            new_pos[a] = axis_absolute_position((EAxis)a, line);
        }

        // updates feedrate from line, if present
        if (line.has_f())
            set_feedrate(std::max(line.f() * MMMIN_TO_MMSEC, get_minimum_feedrate()));

        // fills block data
        Block block;

        // calculates block movement deltas
        float max_abs_delta = 0.0f;
        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            block.delta_pos[a] = new_pos[a] - get_axis_position((EAxis)a);
            max_abs_delta = std::max(max_abs_delta, std::abs(block.delta_pos[a]));
        }

        // is it a move ?
        if (max_abs_delta == 0.0f)
            return;

        // calculates block feedrate
        m_curr.feedrate = std::max(get_feedrate(), block.is_travel_move() ? get_minimum_travel_feedrate() : get_minimum_feedrate());

        float distance = block.move_length();
        float invDistance = 1.0f / distance;

        float min_feedrate_factor = 1.0f;
        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            m_curr.axis_feedrate[a] = m_curr.feedrate * block.delta_pos[a] * invDistance;
            if (a == E)
                m_curr.axis_feedrate[a] *= get_extrude_factor_override_percentage();

            m_curr.abs_axis_feedrate[a] = std::abs(m_curr.axis_feedrate[a]);
            if (m_curr.abs_axis_feedrate[a] > 0.0f)
                min_feedrate_factor = std::min(min_feedrate_factor, get_axis_max_feedrate((EAxis)a) / m_curr.abs_axis_feedrate[a]);
        }
        
        block.feedrate.cruise = min_feedrate_factor * m_curr.feedrate;

        if (min_feedrate_factor < 1.0f)
        {
            for (unsigned char a = X; a < Num_Axis; ++a)
            {
                m_curr.axis_feedrate[a] *= min_feedrate_factor;
                m_curr.abs_axis_feedrate[a] *= min_feedrate_factor;
            }
        }

        // calculates block acceleration
        float acceleration = block.is_extruder_only_move() ? get_retract_acceleration() : get_acceleration();

        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            float axis_max_acceleration = get_axis_max_acceleration((EAxis)a);
            if (acceleration * std::abs(block.delta_pos[a]) * invDistance > axis_max_acceleration)
                acceleration = axis_max_acceleration;
        }

        block.acceleration = acceleration;

        // calculates block exit feedrate
        m_curr.safe_feedrate = block.feedrate.cruise;

        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            float axis_max_jerk = get_axis_max_jerk((EAxis)a);
            if (m_curr.abs_axis_feedrate[a] > axis_max_jerk)
                m_curr.safe_feedrate = std::min(m_curr.safe_feedrate, axis_max_jerk);
        }

        block.feedrate.exit = m_curr.safe_feedrate;

        // calculates block entry feedrate
        float vmax_junction = m_curr.safe_feedrate;
        if (!m_blocks.empty() && (m_prev.feedrate > PREVIOUS_FEEDRATE_THRESHOLD))
        {
            bool prev_speed_larger = m_prev.feedrate > block.feedrate.cruise;
            float smaller_speed_factor = prev_speed_larger ? (block.feedrate.cruise / m_prev.feedrate) : (m_prev.feedrate / block.feedrate.cruise);
            // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
            vmax_junction = prev_speed_larger ? block.feedrate.cruise : m_prev.feedrate;

            float v_factor = 1.0f;
            bool limited = false;

            for (unsigned char a = X; a < Num_Axis; ++a)
            {
                // Limit an axis. We have to differentiate coasting from the reversal of an axis movement, or a full stop.
                float v_exit = m_prev.axis_feedrate[a];
                float v_entry = m_curr.axis_feedrate[a];

                if (prev_speed_larger)
                    v_exit *= smaller_speed_factor;

                if (limited)
                {
                    v_exit *= v_factor;
                    v_entry *= v_factor;
                }

                // Calculate the jerk depending on whether the axis is coasting in the same direction or reversing a direction.
                float jerk =
                    (v_exit > v_entry) ?
                    (((v_entry > 0.0f) || (v_exit < 0.0f)) ?
                    // coasting
                    (v_exit - v_entry) :
                    // axis reversal
                    std::max(v_exit, -v_entry)) :
                    // v_exit <= v_entry
                    (((v_entry < 0.0f) || (v_exit > 0.0f)) ?
                    // coasting
                    (v_entry - v_exit) :
                    // axis reversal
                    std::max(-v_exit, v_entry));

                float axis_max_jerk = get_axis_max_jerk((EAxis)a);
                if (jerk > axis_max_jerk)
                {
                    v_factor *= axis_max_jerk / jerk;
                    limited = true;
                }
            }

            if (limited)
                vmax_junction *= v_factor;

            // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
            // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
            float vmax_junction_threshold = vmax_junction * 0.99f;

            // Not coasting. The machine will stop and start the movements anyway, better to start the segment from start.
            if ((m_prev.safe_feedrate > vmax_junction_threshold) && (m_curr.safe_feedrate > vmax_junction_threshold))
                vmax_junction = m_curr.safe_feedrate;
        }

        float v_allowable = Block::max_allowable_speed(-acceleration, m_curr.safe_feedrate, distance);
        block.feedrate.entry = std::min(vmax_junction, v_allowable);

        block.max_entry_speed = vmax_junction;
        block.flags.nominal_length = (block.feedrate.cruise <= v_allowable);
        block.flags.recalculate = true;
        block.safe_feedrate = m_curr.safe_feedrate;

        // calculates block trapezoid
        block.calculate_trapezoid();

        // updates previous
        m_prev = m_curr;

        // updates axis positions
        for (unsigned char a = X; a < Num_Axis; ++a)
        {
            set_axis_position((EAxis)a, new_pos[a]);
        }

#if ENABLE_MOVE_STATS
        // detects block move type
        block.move_type = Block::Noop;

        if (block.delta_pos[E] < 0.0f)
        {
            if ((block.delta_pos[X] != 0.0f) || (block.delta_pos[Y] != 0.0f) || (block.delta_pos[Z] != 0.0f))
                block.move_type = Block::Move;
            else
                block.move_type = Block::Retract;
        }
        else if (block.delta_pos[E] > 0.0f)
        {
            if ((block.delta_pos[X] == 0.0f) && (block.delta_pos[Y] == 0.0f) && (block.delta_pos[Z] == 0.0f))
                block.move_type = Block::Unretract;
            else if ((block.delta_pos[X] != 0.0f) || (block.delta_pos[Y] != 0.0f))
                block.move_type = Block::Extrude;
        }
        else if ((block.delta_pos[X] != 0.0f) || (block.delta_pos[Y] != 0.0f) || (block.delta_pos[Z] != 0.0f))
            block.move_type = Block::Move;
#endif // ENABLE_MOVE_STATS

        // adds block to blocks list
        m_blocks.emplace_back(block);
        m_g1_line_ids.emplace_back(G1LineIdToBlockIdMap::value_type(get_g1_line_id(), (unsigned int)m_blocks.size() - 1));
    }

    void GCodeTimeEstimator::_processG4(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        GCodeFlavor dialect = get_dialect();

        float value;
        if (line.has_value('P', value))
            add_additional_time(value * MILLISEC_TO_SEC);

        // see: http://reprap.org/wiki/G-code#G4:_Dwell
        if ((dialect == gcfRepetier) ||
            (dialect == gcfMarlin) ||
            (dialect == gcfSmoothie) ||
            (dialect == gcfRepRap))
        {
            if (line.has_value('S', value))
                add_additional_time(value);
        }

        _simulate_st_synchronize();
    }

    void GCodeTimeEstimator::_processG20(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        set_units(Inches);
    }

    void GCodeTimeEstimator::_processG21(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        set_units(Millimeters);
    }

    void GCodeTimeEstimator::_processG28(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        // TODO
    }

    void GCodeTimeEstimator::_processG90(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        set_global_positioning_type(Absolute);
    }

    void GCodeTimeEstimator::_processG91(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        set_global_positioning_type(Relative);
    }

    void GCodeTimeEstimator::_processG92(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        float lengthsScaleFactor = (get_units() == Inches) ? INCHES_TO_MM : 1.0f;
        bool anyFound = false;

        if (line.has_x())
        {
            set_axis_origin(X, get_axis_position(X) - line.x() * lengthsScaleFactor);
            anyFound = true;
        }

        if (line.has_y())
        {
            set_axis_origin(Y, get_axis_position(Y) - line.y() * lengthsScaleFactor);
            anyFound = true;
        }

        if (line.has_z())
        {
            set_axis_origin(Z, get_axis_position(Z) - line.z() * lengthsScaleFactor);
            anyFound = true;
        }

        if (line.has_e())
        {
            set_axis_origin(E, get_axis_position(E) - line.e() * lengthsScaleFactor);
            anyFound = true;
        }
        else
            _simulate_st_synchronize();

        if (!anyFound)
        {
            for (unsigned char a = X; a < Num_Axis; ++a)
            {
                set_axis_origin((EAxis)a, get_axis_position((EAxis)a));
            }
        }
    }

    void GCodeTimeEstimator::_processM1(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        _simulate_st_synchronize();
    }

    void GCodeTimeEstimator::_processM82(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        set_e_local_positioning_type(Absolute);
    }

    void GCodeTimeEstimator::_processM83(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        set_e_local_positioning_type(Relative);
    }

    void GCodeTimeEstimator::_processM109(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        // TODO
    }

    void GCodeTimeEstimator::_processM201(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        GCodeFlavor dialect = get_dialect();

        // see http://reprap.org/wiki/G-code#M201:_Set_max_printing_acceleration
        float factor = ((dialect != gcfRepRap) && (get_units() == GCodeTimeEstimator::Inches)) ? INCHES_TO_MM : 1.0f;

        if (line.has_x())
            set_axis_max_acceleration(X, line.x() * factor);

        if (line.has_y())
            set_axis_max_acceleration(Y, line.y() * factor);

        if (line.has_z())
            set_axis_max_acceleration(Z, line.z() * factor);

        if (line.has_e())
            set_axis_max_acceleration(E, line.e() * factor);
    }

    void GCodeTimeEstimator::_processM203(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        GCodeFlavor dialect = get_dialect();

        // see http://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate
        if (dialect == gcfRepetier)
            return;

        // see http://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate
        // http://smoothieware.org/supported-g-codes
        float factor = (dialect == gcfMarlin || dialect == gcfSmoothie) ? 1.0f : MMMIN_TO_MMSEC;

        if (line.has_x())
            set_axis_max_feedrate(X, line.x() * factor);

        if (line.has_y())
            set_axis_max_feedrate(Y, line.y() * factor);

        if (line.has_z())
            set_axis_max_feedrate(Z, line.z() * factor);

        if (line.has_e())
            set_axis_max_feedrate(E, line.e() * factor);
    }

    void GCodeTimeEstimator::_processM204(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        float value;
        if (line.has_value('S', value)) {
            // Legacy acceleration format. This format is used by the legacy Marlin, MK2 or MK3 firmware,
            // and it is also generated by Slic3r to control acceleration per extrusion type
            // (there is a separate acceleration settings in Slicer for perimeter, first layer etc).
            set_acceleration(value);
            if (line.has_value('T', value))
                set_retract_acceleration(value);
        } else {
            // New acceleration format, compatible with the upstream Marlin.
            if (line.has_value('P', value))
                set_acceleration(value);
            if (line.has_value('R', value))
                set_retract_acceleration(value);
            if (line.has_value('T', value)) {
                // Interpret the T value as the travel acceleration in the new Marlin format.
                //FIXME Prusa3D firmware currently does not support travel acceleration value independent from the extruding acceleration value.
                // set_travel_acceleration(value);
            }
        }
    }

    void GCodeTimeEstimator::_processM205(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        if (line.has_x())
        {
            float max_jerk = line.x();
            set_axis_max_jerk(X, max_jerk);
            set_axis_max_jerk(Y, max_jerk);
        }

        if (line.has_y())
            set_axis_max_jerk(Y, line.y());

        if (line.has_z())
            set_axis_max_jerk(Z, line.z());

        if (line.has_e())
            set_axis_max_jerk(E, line.e());

        float value;
        if (line.has_value('S', value))
            set_minimum_feedrate(value);

        if (line.has_value('T', value))
            set_minimum_travel_feedrate(value);
    }

    void GCodeTimeEstimator::_processM221(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        float value_s;
        float value_t;
        if (line.has_value('S', value_s) && !line.has_value('T', value_t))
            set_extrude_factor_override_percentage(value_s * 0.01f);
    }

    void GCodeTimeEstimator::_processM566(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        if (line.has_x())
            set_axis_max_jerk(X, line.x() * MMMIN_TO_MMSEC);

        if (line.has_y())
            set_axis_max_jerk(Y, line.y() * MMMIN_TO_MMSEC);

        if (line.has_z())
            set_axis_max_jerk(Z, line.z() * MMMIN_TO_MMSEC);

        if (line.has_e())
            set_axis_max_jerk(E, line.e() * MMMIN_TO_MMSEC);
    }

    void GCodeTimeEstimator::_processM702(const GCodeReader::GCodeLine& line)
    {
        PROFILE_FUNC();
        if (line.has('C')) {
            // MK3 MMU2 specific M code:
            // M702 C is expected to be sent by the custom end G-code when finalizing a print.
            // The MK3 unit shall unload and park the active filament into the MMU2 unit.
            add_additional_time(get_filament_unload_time(get_extruder_id()));
            reset_extruder_id();
            _simulate_st_synchronize();
        }
    }

    void GCodeTimeEstimator::_processT(const GCodeReader::GCodeLine& line)
    {
        std::string cmd = line.cmd();
        if (cmd.length() > 1)
        {
            unsigned int id = (unsigned int)::strtol(cmd.substr(1).c_str(), nullptr, 10);
            if (get_extruder_id() != id)
            {
                // Specific to the MK3 MMU2: The initial extruder ID is set to -1 indicating
                // that the filament is parked in the MMU2 unit and there is nothing to be unloaded yet.
                add_additional_time(get_filament_unload_time(get_extruder_id()));
                set_extruder_id(id);
                add_additional_time(get_filament_load_time(get_extruder_id()));
                _simulate_st_synchronize();
            }
        }
    }

    bool GCodeTimeEstimator::_process_tags(const GCodeReader::GCodeLine& line)
    {
        std::string comment = line.comment();

        // color change tag
        size_t pos = comment.find(Color_Change_Tag);
        if (pos != comment.npos)
        {
            _process_color_change_tag();
            return true;
        }

        return false;
    }

    void GCodeTimeEstimator::_process_color_change_tag()
    {
        PROFILE_FUNC();
        m_needs_color_times = true;
        _calculate_time();
        if (m_color_time_cache != 0.0f)
        {
            m_color_times.push_back(m_color_time_cache);
            m_color_time_cache = 0.0f;
        }
    }

    void GCodeTimeEstimator::_simulate_st_synchronize()
    {
        PROFILE_FUNC();
        _calculate_time();
    }

    void GCodeTimeEstimator::_forward_pass()
    {
        PROFILE_FUNC();
        if (m_blocks.size() > 1)
        {
            for (int i = m_last_st_synchronized_block_id + 1; i < (int)m_blocks.size() - 1; ++i)
            {
                _planner_forward_pass_kernel(m_blocks[i], m_blocks[i + 1]);
            }
        }
    }

    void GCodeTimeEstimator::_reverse_pass()
    {
        PROFILE_FUNC();
        if (m_blocks.size() > 1)
        {
            for (int i = (int)m_blocks.size() - 1; i >= m_last_st_synchronized_block_id + 2; --i)
            {
                _planner_reverse_pass_kernel(m_blocks[i - 1], m_blocks[i]);
            }
        }
    }

    void GCodeTimeEstimator::_planner_forward_pass_kernel(Block& prev, Block& curr)
    {
        PROFILE_FUNC();
        // If the previous block is an acceleration block, but it is not long enough to complete the
        // full speed change within the block, we need to adjust the entry speed accordingly. Entry
        // speeds have already been reset, maximized, and reverse planned by reverse planner.
        // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
        if (!prev.flags.nominal_length)
        {
            if (prev.feedrate.entry < curr.feedrate.entry)
            {
                float entry_speed = std::min(curr.feedrate.entry, Block::max_allowable_speed(-prev.acceleration, prev.feedrate.entry, prev.move_length()));

                // Check for junction speed change
                if (curr.feedrate.entry != entry_speed)
                {
                    curr.feedrate.entry = entry_speed;
                    curr.flags.recalculate = true;
                }
            }
        }
    }

    void GCodeTimeEstimator::_planner_reverse_pass_kernel(Block& curr, Block& next)
    {
        // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
        // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
        // check for maximum allowable speed reductions to ensure maximum possible planned speed.
        if (curr.feedrate.entry != curr.max_entry_speed)
        {
            // If nominal length true, max junction speed is guaranteed to be reached. Only compute
            // for max allowable speed if block is decelerating and nominal length is false.
            if (!curr.flags.nominal_length && (curr.max_entry_speed > next.feedrate.entry))
                curr.feedrate.entry = std::min(curr.max_entry_speed, Block::max_allowable_speed(-curr.acceleration, next.feedrate.entry, curr.move_length()));
            else
                curr.feedrate.entry = curr.max_entry_speed;

            curr.flags.recalculate = true;
        }
    }

    void GCodeTimeEstimator::_recalculate_trapezoids()
    {
        PROFILE_FUNC();
        Block* curr = nullptr;
        Block* next = nullptr;

        for (int i = m_last_st_synchronized_block_id + 1; i < (int)m_blocks.size(); ++i)
        {
            Block& b = m_blocks[i];

            curr = next;
            next = &b;

            if (curr != nullptr)
            {
                // Recalculate if current block entry or exit junction speed has changed.
                if (curr->flags.recalculate || next->flags.recalculate)
                {
                    // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                    Block block = *curr;
                    block.feedrate.exit = next->feedrate.entry;
                    block.calculate_trapezoid();
                    curr->trapezoid = block.trapezoid;
                    curr->flags.recalculate = false; // Reset current only to ensure next trapezoid is computed
                }
            }
        }

        // Last/newest block in buffer. Always recalculated.
        if (next != nullptr)
        {
            Block block = *next;
            block.feedrate.exit = next->safe_feedrate;
            block.calculate_trapezoid();
            next->trapezoid = block.trapezoid;
            next->flags.recalculate = false;
        }
    }

    std::string GCodeTimeEstimator::_get_time_dhms(float time_in_secs)
    {
        int days = (int)(time_in_secs / 86400.0f);
        time_in_secs -= (float)days * 86400.0f;
        int hours = (int)(time_in_secs / 3600.0f);
        time_in_secs -= (float)hours * 3600.0f;
        int minutes = (int)(time_in_secs / 60.0f);
        time_in_secs -= (float)minutes * 60.0f;

        char buffer[64];
        if (days > 0)
            ::sprintf(buffer, "%dd %dh %dm %ds", days, hours, minutes, (int)time_in_secs);
        else if (hours > 0)
            ::sprintf(buffer, "%dh %dm %ds", hours, minutes, (int)time_in_secs);
        else if (minutes > 0)
            ::sprintf(buffer, "%dm %ds", minutes, (int)time_in_secs);
        else
            ::sprintf(buffer, "%ds", (int)time_in_secs);

        return buffer;
    }

    std::string GCodeTimeEstimator::_get_time_minutes(float time_in_secs)
    {
        return std::to_string((int)(::roundf(time_in_secs / 60.0f)));
    }

#if ENABLE_MOVE_STATS
    void GCodeTimeEstimator::_log_moves_stats() const
    {
        float moves_count = 0.0f;
        for (const MovesStatsMap::value_type& move : _moves_stats)
        {
            moves_count += (float)move.second.count;
        }

        for (const MovesStatsMap::value_type& move : _moves_stats)
        {
            std::cout << MOVE_TYPE_STR[move.first];
            std::cout << ": count " << move.second.count << " (" << 100.0f * (float)move.second.count / moves_count << "%)";
            std::cout << " - time: " << move.second.time << "s (" << 100.0f * move.second.time / m_time << "%)";
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
#endif // ENABLE_MOVE_STATS
}
