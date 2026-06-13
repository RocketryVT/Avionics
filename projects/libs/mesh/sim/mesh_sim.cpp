#include "mesh.hpp"

#include "Fusion.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const char* message)
{
    if (!condition) {
        std::printf("FAIL: %s\n", message);
        ++g_failures;
    }
}

class SimBus;

class SimRadio final : public SIGMA2::Radio {
public:
    SimRadio(SimBus& bus, uint8_t radio_id, const char* radio_name)
        : bus_(bus)
        , radio_id_(radio_id)
        , name_(radio_name)
    {}

    const char* name() const override { return name_; }
    int begin(const SIGMA2::RadioConfig&) override { return 0; }
    int transmit(const uint8_t* data, size_t len) override;
    bool receive(SIGMA2::RadioRx& out) override;

    void push(const uint8_t* data,
              size_t len,
              uint8_t from_radio_id,
              uint32_t received_ms)
    {
        SIGMA2::RadioRx rx = {};
        std::memcpy(rx.data, data, len);
        rx.len = len;
        rx.radio_id = from_radio_id;
        rx.rssi_dbm = -62;
        rx.snr_q4 = 32;
        rx.received_ms = received_ms;
        inbox_.push_back(rx);
    }

    uint8_t id() const { return radio_id_; }
    void set_now(uint32_t now_ms);

private:
    SimBus& bus_;
    uint8_t radio_id_;
    const char* name_;
    uint32_t now_ms_ = 0;
    std::deque<SIGMA2::RadioRx> inbox_;
};

class SimBus {
public:
    struct Pending {
        uint8_t from = 0;
        uint8_t to = 0;
        uint8_t data[SIGMA2::MAX_FRAME] = {};
        size_t len = 0;
        uint32_t delivery_ms = 0;
    };

    void add_radio(SimRadio& radio)
    {
        radios_.push_back(&radio);
    }

    void set_delay(uint32_t base_delay_ms, uint32_t jitter_ms)
    {
        base_delay_ms_ = base_delay_ms;
        jitter_ms_ = jitter_ms;
    }

    void link(uint8_t a, uint8_t b)
    {
        links_[a][b] = true;
        links_[b][a] = true;
    }

    int transmit(uint8_t from, const uint8_t* data, size_t len)
    {
        for (SimRadio* radio : radios_) {
            if (radio->id() != from && links_[from][radio->id()]) {
                Pending pending = {};
                pending.from = from;
                pending.to = radio->id();
                std::memcpy(pending.data, data, len);
                pending.len = len;
                const int32_t delay =
                    static_cast<int32_t>(base_delay_ms_) + next_jitter_ms();
                pending.delivery_ms = now_ms_ +
                    static_cast<uint32_t>(delay > 0 ? delay : 0);
                pending_.push_back(pending);
            }
        }
        return 0;
    }

    void set_now(uint32_t now_ms)
    {
        now_ms_ = now_ms;
        for (std::size_t i = 0; i < pending_.size();) {
            const Pending& pending = pending_[i];
            if (pending.delivery_ms > now_ms_) {
                ++i;
                continue;
            }

            if (SimRadio* radio = radio_by_id(pending.to)) {
                radio->push(pending.data, pending.len, pending.from, now_ms_);
            }
            pending_.erase(pending_.begin() + static_cast<long>(i));
        }
    }

private:
    SimRadio* radio_by_id(uint8_t id)
    {
        for (SimRadio* radio : radios_) {
            if (radio->id() == id) {
                return radio;
            }
        }
        return nullptr;
    }

    int32_t next_jitter_ms()
    {
        if (jitter_ms_ == 0u) {
            return 0;
        }

        rng_ = rng_ * 1664525u + 1013904223u;
        const uint32_t range = 2u * jitter_ms_ + 1u;
        return
            static_cast<int32_t>(rng_ % range) - static_cast<int32_t>(jitter_ms_);
    }

    std::vector<SimRadio*> radios_;
    std::deque<Pending> pending_;
    bool links_[8][8] = {};
    uint32_t now_ms_ = 0;
    uint32_t base_delay_ms_ = 0;
    uint32_t jitter_ms_ = 0;
    uint32_t rng_ = 0x43524f53u;
};

void SimRadio::set_now(uint32_t now_ms)
{
    now_ms_ = now_ms;
    bus_.set_now(now_ms);
}

int SimRadio::transmit(const uint8_t* data, size_t len)
{
    return bus_.transmit(radio_id_, data, len);
}

bool SimRadio::receive(SIGMA2::RadioRx& out)
{
    if (inbox_.empty()) {
        return false;
    }

    out = inbox_.front();
    inbox_.pop_front();
    return true;
}

struct SimNode {
    SimRadio radio;
    mesh::Mesh mesh;

    SimNode(SimBus& bus,
            uint8_t radio_id,
            const char* radio_name,
            SIGMA2::NodeID node_id,
            SIGMA2::NodeID dst)
        : radio(bus, radio_id, radio_name)
        , mesh(config(node_id, dst))
    {
        mesh.set_primary_radio(radio);
        check(mesh.begin(), "mesh begin failed");
    }

    static SIGMA2::MeshConfig config(SIGMA2::NodeID node_id,
                                     SIGMA2::NodeID dst)
    {
        SIGMA2::MeshConfig cfg = {};
        cfg.node_id = node_id;
        cfg.default_destination = dst;
        cfg.controlled_flooding_enabled = true;
        return cfg;
    }
};

void pump(std::array<SimNode*, 3> nodes, uint32_t& now_ms, int steps)
{
    for (int i = 0; i < steps; ++i) {
        now_ms += 10;
        for (SimNode* node : nodes) {
            node->radio.set_now(now_ms);
            (void) node->mesh.poll_receive(now_ms);
        }
        for (SimNode* node : nodes) {
            int state = 0;
            (void) node->mesh.transmit_one(&state);
        }
    }
}

void test_multihop_delivery()
{
    SimBus bus;
    SimNode nose(bus, 0, "nose", SIGMA2::NodeID::NOSE_CONE,
                 SIGMA2::NodeID::ANTENNA_TRACKER);
    SimNode relay(bus, 1, "relay", SIGMA2::NodeID::MOBILE_NODE1,
                  SIGMA2::NodeID::ANTENNA_TRACKER);
    SimNode gs(bus, 2, "gs", SIGMA2::NodeID::ANTENNA_TRACKER,
               SIGMA2::NodeID::NOSE_CONE);

    bus.add_radio(nose.radio);
    bus.add_radio(relay.radio);
    bus.add_radio(gs.radio);
    bus.link(0, 1);
    bus.link(1, 2);

    SIGMA2::MeshSnapshot snapshot = {};
    snapshot.have_gps = true;
    snapshot.boot_ms = 1000;
    snapshot.gps.lat = 38.0;
    snapshot.gps.lon = -77.0;
    snapshot.gps.alt_m = 123.0;
    snapshot.gps.satellites = 9;
    snapshot.gps.fix_type = 3;

    nose.mesh.tick_1hz(snapshot);

    uint32_t now_ms = 1000;
    pump({&nose, &relay, &gs}, now_ms, 8);

    check(relay.mesh.stats().forwarded == 1, "relay forwarded one frame");
    check(gs.mesh.received_queued() == 1, "ground station received one frame");

    SIGMA2::MeshReceivedFrame received = {};
    check(gs.mesh.read_received(received), "ground station read received frame");
    check(received.header.node_src == SIGMA2::NodeID::NOSE_CONE,
          "received src is nose");
    check(received.header.node_dst == SIGMA2::NodeID::ANTENNA_TRACKER,
          "received dst is ground station");
    check(received.header.type == SIGMA2::PacketType::GPS,
          "received packet type is GPS");
    check(received.header.ttl == SIGMA2::DEFAULT_TTL - 1u,
          "received TTL decremented by relay");

    SIGMA2::DecodedFrame decoded = {};
    check(SIGMA2::deserialize_frame(received.rx.data, received.rx.len, decoded) ==
              SIGMA2::DecodeStatus::Ok,
          "delivered frame still validates");

    SIGMA2::TRANSMIT_PACKETS::GPSNav gps = {};
    check(SIGMA2::deserialize_packet_payload(decoded, gps),
          "GPS payload deserializes");
    check(gps.num_sv == 9, "GPS payload satellite count preserved");

    pump({&nose, &relay, &gs}, now_ms, 8);
    check(gs.mesh.received_queued() == 0, "duplicate loop not redelivered");
}

void make_frame(SIGMA2::Serializer& serializer,
                SIGMA2::NodeID dst,
                uint8_t* out,
                size_t& len,
                uint32_t timestamp_ms,
                uint8_t ttl = SIGMA2::DEFAULT_TTL)
{
    SIGMA2::TRANSMIT_PACKETS::TimeSync ts = {};
    ts.t1.timestamp_ms = timestamp_ms;
    ts.t1.utc_ms = timestamp_ms + 100;
    ts.t1.flags = SIGMA2::DATA_VALID_FLAG::TIME_VALID;
    len = serializer.serialize_packet(ts, dst, out, SIGMA2::MAX_FRAME,
                                      timestamp_ms, {0}, ttl);
}

SIGMA2::RadioRx rx_from_frame(const uint8_t* frame, size_t len)
{
    SIGMA2::RadioRx rx = {};
    std::memcpy(rx.data, frame, len);
    rx.len = len;
    rx.radio_id = 7;
    rx.received_ms = 5000;
    return rx;
}

void test_duplicate_suppression()
{
    SimBus bus;
    SimNode relay(bus, 0, "relay", SIGMA2::NodeID::MOBILE_NODE1,
                  SIGMA2::NodeID::ANTENNA_TRACKER);

    SIGMA2::Serializer serializer(SIGMA2::NodeID::NOSE_CONE);
    uint8_t frame[SIGMA2::MAX_FRAME] = {};
    size_t len = 0;
    make_frame(serializer, SIGMA2::NodeID::ANTENNA_TRACKER, frame, len, 2000);

    const SIGMA2::RadioRx rx = rx_from_frame(frame, len);
    check(relay.mesh.ingest_received_frame(rx), "first frame handled");
    check(!relay.mesh.ingest_received_frame(rx), "duplicate frame suppressed");
    check(relay.mesh.stats().rx_duplicate == 1, "duplicate stat incremented");
    check(relay.mesh.stats().forwarded == 1, "duplicate was not forwarded twice");
}

void test_ttl_stops_forwarding()
{
    SimBus bus;
    SimNode relay(bus, 0, "relay", SIGMA2::NodeID::MOBILE_NODE1,
                  SIGMA2::NodeID::ANTENNA_TRACKER);

    SIGMA2::Serializer serializer(SIGMA2::NodeID::NOSE_CONE);
    uint8_t frame[SIGMA2::MAX_FRAME] = {};
    size_t len = 0;
    make_frame(serializer, SIGMA2::NodeID::ANTENNA_TRACKER, frame, len, 3000, 1);

    const SIGMA2::RadioRx rx = rx_from_frame(frame, len);
    check(!relay.mesh.ingest_received_frame(rx), "ttl=1 remote frame not handled");
    check(relay.mesh.queued() == 0, "ttl=1 remote frame not forwarded");
    check(relay.mesh.stats().forwarded == 0, "ttl=1 forward stat unchanged");
}

void test_bad_crc_rejected()
{
    SimBus bus;
    SimNode relay(bus, 0, "relay", SIGMA2::NodeID::MOBILE_NODE1,
                  SIGMA2::NodeID::ANTENNA_TRACKER);

    SIGMA2::Serializer serializer(SIGMA2::NodeID::NOSE_CONE);
    uint8_t frame[SIGMA2::MAX_FRAME] = {};
    size_t len = 0;
    make_frame(serializer, SIGMA2::NodeID::ANTENNA_TRACKER, frame, len, 4000);
    frame[SIGMA2::HEADER::WIRE_SIZE] ^= 0x55u;

    const SIGMA2::RadioRx rx = rx_from_frame(frame, len);
    check(!relay.mesh.ingest_received_frame(rx), "bad CRC frame rejected");
    check(relay.mesh.stats().rx_err == 1, "bad CRC increments rx_err");
}

struct TrajectoryPoint {
    double t_s = 0.0;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;
    double n_m = 0.0;
    double e_m = 0.0;
    double u_m = 0.0;
    double vn_mps = 0.0;
    double ve_mps = 0.0;
    double vu_mps = 0.0;
};

struct PredictionStats {
    std::size_t count = 0;
    double delayed_sum_sq_m = 0.0;
    double predicted_sum_sq_m = 0.0;
    double delayed_max_m = 0.0;
    double predicted_max_m = 0.0;
    double pointing_max_deg = 0.0;
    double latency_sum_s = 0.0;
    double latency_min_s = std::numeric_limits<double>::infinity();
    double latency_max_s = 0.0;
};

struct PhaseStats {
    const char* name = "";
    double start_s = 0.0;
    double end_s = 0.0;
    std::size_t count = 0;
    double predicted_sum_sq_m = 0.0;
    double predicted_max_m = 0.0;
    double pointing_max_deg = 0.0;
};

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

double to_double(const std::string& s)
{
    return std::strtod(s.c_str(), nullptr);
}

double deg_to_rad(double deg)
{
    return deg * 3.14159265358979323846 / 180.0;
}

double rad_to_deg(double rad)
{
    return rad * 180.0 / 3.14159265358979323846;
}

double distance3(double n0, double e0, double u0,
                 double n1, double e1, double u1)
{
    const double dn = n0 - n1;
    const double de = e0 - e1;
    const double du = u0 - u1;
    return std::sqrt(dn * dn + de * de + du * du);
}

double pointing_error_deg(const TrajectoryPoint& truth,
                          double pred_n,
                          double pred_e,
                          double pred_u)
{
    const double true_norm = std::sqrt(truth.n_m * truth.n_m +
                                       truth.e_m * truth.e_m +
                                       truth.u_m * truth.u_m);
    const double pred_norm = std::sqrt(pred_n * pred_n +
                                       pred_e * pred_e +
                                       pred_u * pred_u);
    if (true_norm < 1.0 || pred_norm < 1.0) {
        return 0.0;
    }

    double c = (truth.n_m * pred_n + truth.e_m * pred_e + truth.u_m * pred_u) /
               (true_norm * pred_norm);
    c = std::clamp(c, -1.0, 1.0);
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

std::vector<TrajectoryPoint> load_openrocket_csv(const char* path)
{
    std::ifstream in(path);
    std::vector<TrajectoryPoint> points;
    if (!in) {
        std::printf("Could not open OpenRocket CSV: %s\n", path);
        return points;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if ((line[0] < '0' || line[0] > '9') && line[0] != '-') {
            continue;
        }

        const std::vector<std::string> f = split_csv_line(line);
        if (f.size() < 9) {
            continue;
        }

        TrajectoryPoint p = {};
        p.t_s = to_double(f[0]);
        p.alt_m = to_double(f[1]);
        p.lat_deg = to_double(f[7]);
        p.lon_deg = to_double(f[8]);
        points.push_back(p);
    }

    if (points.size() < 3) {
        return points;
    }

    constexpr double EARTH_RADIUS_M = 6371000.0;
    const double lat0 = deg_to_rad(points.front().lat_deg);
    const double lon0 = deg_to_rad(points.front().lon_deg);
    const double cos_lat0 = std::cos(lat0);

    for (TrajectoryPoint& p : points) {
        p.n_m = (deg_to_rad(p.lat_deg) - lat0) * EARTH_RADIUS_M;
        p.e_m = (deg_to_rad(p.lon_deg) - lon0) * EARTH_RADIUS_M * cos_lat0;
        p.u_m = p.alt_m;
    }

    for (std::size_t i = 0; i < points.size(); ++i) {
        const std::size_t lo = (i == 0) ? i : i - 1u;
        const std::size_t hi = (i + 1u >= points.size()) ? i : i + 1u;
        const double dt = points[hi].t_s - points[lo].t_s;
        if (dt <= 0.0) {
            continue;
        }
        points[i].vn_mps = (points[hi].n_m - points[lo].n_m) / dt;
        points[i].ve_mps = (points[hi].e_m - points[lo].e_m) / dt;
        points[i].vu_mps = (points[hi].u_m - points[lo].u_m) / dt;
    }

    return points;
}

TrajectoryPoint sample_trajectory(const std::vector<TrajectoryPoint>& points,
                                  double t_s)
{
    if (t_s <= points.front().t_s) {
        return points.front();
    }
    if (t_s >= points.back().t_s) {
        return points.back();
    }

    const auto hi = std::lower_bound(
        points.begin(), points.end(), t_s,
        [](const TrajectoryPoint& p, double t) { return p.t_s < t; });
    const auto lo = hi - 1;
    const double span = hi->t_s - lo->t_s;
    const double a = span > 0.0 ? (t_s - lo->t_s) / span : 0.0;

    TrajectoryPoint out = {};
    out.t_s = t_s;
    out.lat_deg = lo->lat_deg + (hi->lat_deg - lo->lat_deg) * a;
    out.lon_deg = lo->lon_deg + (hi->lon_deg - lo->lon_deg) * a;
    out.alt_m = lo->alt_m + (hi->alt_m - lo->alt_m) * a;
    out.n_m = lo->n_m + (hi->n_m - lo->n_m) * a;
    out.e_m = lo->e_m + (hi->e_m - lo->e_m) * a;
    out.u_m = lo->u_m + (hi->u_m - lo->u_m) * a;
    out.vn_mps = lo->vn_mps + (hi->vn_mps - lo->vn_mps) * a;
    out.ve_mps = lo->ve_mps + (hi->ve_mps - lo->ve_mps) * a;
    out.vu_mps = lo->vu_mps + (hi->vu_mps - lo->vu_mps) * a;
    return out;
}

SIGMA2::MeshSnapshot snapshot_from_trajectory(const TrajectoryPoint& p,
                                              uint32_t boot_ms)
{
    SIGMA2::MeshSnapshot snapshot = {};
    snapshot.boot_ms = boot_ms;
    snapshot.have_nav = true;
    snapshot.nav.lat = p.lat_deg;
    snapshot.nav.lon = p.lon_deg;
    snapshot.nav.alt_fused_m = static_cast<float>(p.u_m);
    snapshot.nav.vel_ned_ms[0] = static_cast<float>(p.vn_mps);
    snapshot.nav.vel_ned_ms[1] = static_cast<float>(p.ve_mps);
    snapshot.nav.vel_ned_ms[2] = static_cast<float>(-p.vu_mps);
    snapshot.nav.q[0] = 1.0f;
    snapshot.nav.frame = SIGMA2::CoordinateFrame::NED;
    snapshot.nav.nav_source = SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_POS |
                              SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_VEL |
                              SIGMA2::TRANSMIT_PACKETS::NavSource::BARO_ALT |
                              SIGMA2::TRANSMIT_PACKETS::NavSource::BARO_VEL;
    snapshot.nav.flags = SIGMA2::DATA_VALID_FLAG::GPS_VALID |
                         SIGMA2::DATA_VALID_FLAG::BARO_VALID;
    snapshot.nav.state = SIGMA2::FLIGHT_STATE::BOOST;
    return snapshot;
}

void update_prediction_stats(const std::vector<TrajectoryPoint>& trajectory,
                             const SIGMA2::MeshReceivedFrame& received,
                             uint32_t now_ms,
                             const TrajectoryPoint& origin,
                             PredictionStats& stats,
                             std::array<PhaseStats, 3>& phases)
{
    if (received.header.type != SIGMA2::PacketType::NAV_STATE) {
        return;
    }

    SIGMA2::DecodedFrame decoded = {};
    if (SIGMA2::deserialize_frame(received.rx.data, received.rx.len, decoded) !=
        SIGMA2::DecodeStatus::Ok) {
        check(false, "trajectory received frame validates");
        return;
    }

    SIGMA2::TRANSMIT_PACKETS::NavState nav = {};
    if (!SIGMA2::deserialize_packet_payload(decoded, nav)) {
        check(false, "trajectory NavState payload deserializes");
        return;
    }

    const double source_t_s = static_cast<double>(received.header.timestamp_ms) * 0.001;
    const double now_s = static_cast<double>(now_ms) * 0.001;
    const double latency_s = std::max(0.0, now_s - source_t_s);
    if (now_s < 1.0) {
        return;
    }

    constexpr double EARTH_RADIUS_M = 6371000.0;
    const double lat0 = deg_to_rad(origin.lat_deg);
    const double lon0 = deg_to_rad(origin.lon_deg);
    const double lat = deg_to_rad(static_cast<double>(nav.lat_deg_e7) * 1.0e-7);
    const double lon = deg_to_rad(static_cast<double>(nav.lon_deg_e7) * 1.0e-7);
    const double obs_n = (lat - lat0) * EARTH_RADIUS_M;
    const double obs_e = (lon - lon0) * EARTH_RADIUS_M * std::cos(lat0);
    const double obs_u = static_cast<double>(nav.alt_fused_cm) * 0.01;
    const double vel_n = static_cast<double>(nav.vel_n_cms) * 0.01;
    const double vel_e = static_cast<double>(nav.vel_e_cms) * 0.01;
    const double vel_u = -static_cast<double>(nav.vel_d_cms) * 0.01;

    const double pred_n = obs_n + vel_n * latency_s;
    const double pred_e = obs_e + vel_e * latency_s;
    const double pred_u = obs_u + vel_u * latency_s;

    const TrajectoryPoint truth = sample_trajectory(trajectory, now_s);
    const double delayed_err =
        distance3(obs_n, obs_e, obs_u, truth.n_m, truth.e_m, truth.u_m);
    const double predicted_err =
        distance3(pred_n, pred_e, pred_u, truth.n_m, truth.e_m, truth.u_m);
    const double pointing_err =
        pointing_error_deg(truth, pred_n, pred_e, pred_u);

    ++stats.count;
    stats.delayed_sum_sq_m += delayed_err * delayed_err;
    stats.predicted_sum_sq_m += predicted_err * predicted_err;
    stats.delayed_max_m = std::max(stats.delayed_max_m, delayed_err);
    stats.predicted_max_m = std::max(stats.predicted_max_m, predicted_err);
    stats.pointing_max_deg = std::max(stats.pointing_max_deg, pointing_err);
    stats.latency_sum_s += latency_s;
    stats.latency_min_s = std::min(stats.latency_min_s, latency_s);
    stats.latency_max_s = std::max(stats.latency_max_s, latency_s);

    for (PhaseStats& phase : phases) {
        if (now_s >= phase.start_s && now_s < phase.end_s) {
            ++phase.count;
            phase.predicted_sum_sq_m += predicted_err * predicted_err;
            phase.predicted_max_m = std::max(phase.predicted_max_m, predicted_err);
            phase.pointing_max_deg =
                std::max(phase.pointing_max_deg, pointing_err);
            break;
        }
    }
}

void test_delayed_trajectory_prediction(const char* csv_path)
{
    const std::vector<TrajectoryPoint> trajectory = load_openrocket_csv(csv_path);
    check(trajectory.size() > 100, "OpenRocket trajectory loaded");
    if (trajectory.size() <= 100) {
        return;
    }

    SimBus bus;
    bus.set_delay(1000, 250);
    SimNode rocket(bus, 0, "rocket", SIGMA2::NodeID::NOSE_CONE,
                   SIGMA2::NodeID::ANTENNA_TRACKER);
    SimNode tracker(bus, 1, "tracker", SIGMA2::NodeID::ANTENNA_TRACKER,
                    SIGMA2::NodeID::NOSE_CONE);
    bus.add_radio(rocket.radio);
    bus.add_radio(tracker.radio);
    bus.link(0, 1);

    PredictionStats stats = {};
    std::array<PhaseStats, 3> phases = {{
        {"burn_0_4s", 0.0, 4.0},
        {"fast_4_10s", 4.0, 10.0},
        {"rest_10s_plus", 10.0, std::numeric_limits<double>::infinity()},
    }};
    const TrajectoryPoint origin = trajectory.front();
    const double end_s = trajectory.back().t_s;
    uint32_t next_tx_ms = 0;

    for (uint32_t now_ms = 0; now_ms <= static_cast<uint32_t>(end_s * 1000.0) + 2000u;
         now_ms += 20u) {
        rocket.radio.set_now(now_ms);
        tracker.radio.set_now(now_ms);

        if (now_ms <= static_cast<uint32_t>(end_s * 1000.0) &&
            now_ms >= next_tx_ms) {
            const TrajectoryPoint sample =
                sample_trajectory(trajectory, static_cast<double>(now_ms) * 0.001);
            rocket.mesh.tick_1hz(snapshot_from_trajectory(sample, now_ms));
            next_tx_ms += 100u;
        }

        (void) rocket.mesh.poll_receive(now_ms);
        (void) tracker.mesh.poll_receive(now_ms);

        int state = 0;
        (void) rocket.mesh.transmit_one(&state);
        (void) tracker.mesh.transmit_one(&state);

        SIGMA2::MeshReceivedFrame received = {};
        while (tracker.mesh.read_received(received)) {
            update_prediction_stats(trajectory, received, now_ms, origin, stats,
                                    phases);
        }
    }

    const double delayed_rms =
        std::sqrt(stats.delayed_sum_sq_m / static_cast<double>(stats.count));
    const double predicted_rms =
        std::sqrt(stats.predicted_sum_sq_m / static_cast<double>(stats.count));
    const double mean_latency =
        stats.latency_sum_s / static_cast<double>(stats.count);

    std::printf(
        "trajectory prediction: samples=%zu latency=%.3f..%.3fs mean=%.3fs "
        "delayed_rms=%.2fm delayed_max=%.2fm predicted_rms=%.2fm "
        "predicted_max=%.2fm max_pointing=%.2fdeg\n",
        stats.count,
        stats.latency_min_s,
        stats.latency_max_s,
        mean_latency,
        delayed_rms,
        stats.delayed_max_m,
        predicted_rms,
        stats.predicted_max_m,
        stats.pointing_max_deg);

    for (const PhaseStats& phase : phases) {
        if (phase.count == 0u) {
            continue;
        }
        const double rms =
            std::sqrt(phase.predicted_sum_sq_m /
                      static_cast<double>(phase.count));
        std::printf("  phase %-12s samples=%zu predicted_rms=%.2fm "
                    "predicted_max=%.2fm max_pointing=%.2fdeg\n",
                    phase.name,
                    phase.count,
                    rms,
                    phase.predicted_max_m,
                    phase.pointing_max_deg);
    }

    check(stats.count > 100, "trajectory prediction received enough samples");
    check(predicted_rms < delayed_rms, "prediction improves RMS position error");
    check(stats.pointing_max_deg < 20.0, "prediction pointing error stays bounded");
}

void test_synthetic_fusion_yaw()
{
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 0,
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    constexpr float dt = 0.01f;
    constexpr float yaw_rate_dps = 45.0f;
    constexpr int samples = 500;

    for (int i = 0; i < samples; ++i) {
        const FusionVector gyro = {.axis = {.x = 0.0f, .y = 0.0f, .z = yaw_rate_dps}};
        const FusionVector accel = {.axis = {.x = 0.0f, .y = 0.0f, .z = 1.0f}};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, dt);
    }

    const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(q);
    // FusionAhrsUpdateNoMagnetometer intentionally zeros heading during the
    // 3-second startup period, so yaw is only expected to accumulate after that.
    constexpr float startup_s = 3.0f;
    const double expected_yaw_deg =
        yaw_rate_dps * ((dt * static_cast<float>(samples)) - startup_s);
    const double yaw_err_deg = std::fabs(euler.angle.yaw - expected_yaw_deg);

    std::printf("fusion synthetic yaw: expected=%.2fdeg roll=%.2f pitch=%.2f "
                "yaw=%.2f q=[%.4f %.4f %.4f %.4f] yaw_err=%.2fdeg\n",
                expected_yaw_deg,
                euler.angle.roll,
                euler.angle.pitch,
                euler.angle.yaw,
                q.element.w,
                q.element.x,
                q.element.y,
                q.element.z,
                yaw_err_deg);

    check(yaw_err_deg < 2.0, "Fusion synthetic yaw quaternion tracks gyro");
}

struct AdsReplayStats {
    std::size_t samples = 0;
    std::size_t accel_ignored = 0;
    std::size_t mag_ignored = 0;
    double max_accel_g = 0.0;
    double max_roll_abs_deg = 0.0;
    double max_pitch_abs_deg = 0.0;
    double max_yaw_abs_deg = 0.0;
    FusionEuler final_euler = FUSION_EULER_ZERO;
    FusionQuaternion final_q = FUSION_QUATERNION_IDENTITY;
};

bool parse_ads_row(const std::vector<std::string>& f,
                   uint64_t& time_us,
                   FusionVector& accel,
                   FusionVector& mag)
{
    if (f.size() < 17) {
        return false;
    }
    char* end = nullptr;
    const unsigned long long parsed_time = std::strtoull(f[0].c_str(), &end, 10);
    if (!end || *end != '\0') {
        return false;
    }

    time_us = static_cast<uint64_t>(parsed_time);
    mag.axis.x = static_cast<float>(to_double(f[11]));
    mag.axis.y = static_cast<float>(to_double(f[12]));
    mag.axis.z = static_cast<float>(to_double(f[13]));
    accel.axis.x = static_cast<float>(to_double(f[14]));
    accel.axis.y = static_cast<float>(to_double(f[15]));
    accel.axis.z = static_cast<float>(to_double(f[16]));
    return std::isfinite(accel.axis.x) && std::isfinite(accel.axis.y) &&
           std::isfinite(accel.axis.z) && std::isfinite(mag.axis.x) &&
           std::isfinite(mag.axis.y) && std::isfinite(mag.axis.z);
}

void test_ads_fusion_replay(const char* csv_path)
{
    std::ifstream in(csv_path);
    check(static_cast<bool>(in), "ADS CSV opened");
    if (!in) {
        return;
    }

    std::string line;
    if (!std::getline(in, line)) {
        check(false, "ADS CSV has header");
        return;
    }

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .recoveryTriggerPeriod = 500,
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    AdsReplayStats stats = {};
    bool have_prev = false;
    uint64_t prev_us = 0;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const std::vector<std::string> f = split_csv_line(line);
        uint64_t time_us = 0;
        FusionVector accel = FUSION_VECTOR_ZERO;
        FusionVector mag = FUSION_VECTOR_ZERO;
        if (!parse_ads_row(f, time_us, accel, mag)) {
            continue;
        }

        float dt = 0.01f;
        if (have_prev && time_us > prev_us) {
            dt = static_cast<float>(static_cast<double>(time_us - prev_us) * 1.0e-6);
            if (dt <= 0.0f || dt > 0.1f) {
                dt = 0.01f;
            }
        }
        prev_us = time_us;
        have_prev = true;

        // The real log does not include gyro columns, so zero gyro is explicit.
        const FusionVector gyro = FUSION_VECTOR_ZERO;
        FusionAhrsUpdate(&ahrs, gyro, accel, mag, dt);

        const FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs);
        if (flags.accelerationRecovery) {
            ++stats.accel_ignored;
        }
        if (flags.magneticRecovery) {
            ++stats.mag_ignored;
        }

        const double accel_norm =
            std::sqrt(static_cast<double>(accel.axis.x) * accel.axis.x +
                      static_cast<double>(accel.axis.y) * accel.axis.y +
                      static_cast<double>(accel.axis.z) * accel.axis.z);
        stats.max_accel_g = std::max(stats.max_accel_g, accel_norm);

        const FusionEuler euler =
            FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        stats.max_roll_abs_deg =
            std::max(stats.max_roll_abs_deg, std::fabs(static_cast<double>(euler.angle.roll)));
        stats.max_pitch_abs_deg =
            std::max(stats.max_pitch_abs_deg, std::fabs(static_cast<double>(euler.angle.pitch)));
        stats.max_yaw_abs_deg =
            std::max(stats.max_yaw_abs_deg, std::fabs(static_cast<double>(euler.angle.yaw)));
        stats.final_euler = euler;
        stats.final_q = FusionAhrsGetQuaternion(&ahrs);
        ++stats.samples;
    }

    const double accel_ignored_pct =
        stats.samples == 0 ? 0.0 :
        100.0 * static_cast<double>(stats.accel_ignored) /
        static_cast<double>(stats.samples);
    const double mag_ignored_pct =
        stats.samples == 0 ? 0.0 :
        100.0 * static_cast<double>(stats.mag_ignored) /
        static_cast<double>(stats.samples);

    std::printf("fusion ADS replay: samples=%zu max_accel=%.2fg "
                "accel_recovery=%.1f%% mag_recovery=%.1f%% "
                "max_abs_euler=[r %.1f p %.1f y %.1f] "
                "final_euler=[r %.1f p %.1f y %.1f] "
                "final_q=[%.4f %.4f %.4f %.4f] "
                "(zero gyro: diagnostic only)\n",
                stats.samples,
                stats.max_accel_g,
                accel_ignored_pct,
                mag_ignored_pct,
                stats.max_roll_abs_deg,
                stats.max_pitch_abs_deg,
                stats.max_yaw_abs_deg,
                stats.final_euler.angle.roll,
                stats.final_euler.angle.pitch,
                stats.final_euler.angle.yaw,
                stats.final_q.element.w,
                stats.final_q.element.x,
                stats.final_q.element.y,
                stats.final_q.element.z);

    check(stats.samples > 1000, "ADS Fusion replay processed enough rows");
}

} // namespace

int main(int argc, char** argv)
{
    const char* trajectory_csv =
        (argc > 1) ? argv[1] :
        "../../../../flight_data/OpenRocket/2026/IREC2026SIM.csv";
    const char* ads_csv =
        (argc > 2) ? argv[2] :
        "../../../../flight_data/test_launches/2026/RAW/Test_Launch_2_OH/ads.csv";

    test_multihop_delivery();
    test_duplicate_suppression();
    test_ttl_stops_forwarding();
    test_bad_crc_rejected();
    test_delayed_trajectory_prediction(trajectory_csv);
    test_synthetic_fusion_yaw();
    test_ads_fusion_replay(ads_csv);

    if (g_failures != 0) {
        std::printf("mesh_sim failed: %d failure(s)\n", g_failures);
        return 1;
    }

    std::printf("mesh_sim passed\n");
    return 0;
}
