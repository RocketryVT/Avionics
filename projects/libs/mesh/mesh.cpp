#include "mesh.hpp"

#include <cstring>

namespace mesh {

namespace {

int32_t deg_to_e7(double deg) {
    const double scaled = deg * 10000000.0;
    return static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

int32_t metres_to_cm(float metres) {
    const float scaled = metres * 100.0f;
    return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int16_t metres_per_second_to_cms(float metres_per_second) {
    const float scaled = metres_per_second * 100.0f;
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int16_t metres_per_second_squared_to_mg(float metres_per_second_squared) {
    constexpr float MPS2_PER_G = 9.80665f;
    const float scaled = (metres_per_second_squared / MPS2_PER_G) * 1000.0f;
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int16_t quat_to_q15(float q) {
    if (q > 0.9999695f) {
        q = 0.9999695f;
    } else if (q < -1.0f) {
        q = -1.0f;
    }
    const float scaled = q * 32768.0f;
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

} // namespace

Mesh::Mesh(const SIGMA2::MeshConfig& config)
    : config_(config)
    , serializer_(config.node_id)
{}

void Mesh::set_primary_radio(SIGMA2::Radio& radio)
{
    primary_ = &radio;
    ready_ = false;
}

bool Mesh::begin()
{
    if (!primary_) {
        return false;
    }
    const int state = primary_->begin(config_.radio);
    ready_ = (state == 0);
    return ready_;
}

bool Mesh::ready() const
{
    return ready_;
}

bool Mesh::poll_receive(uint32_t now_ms)
{
    if (!ready_ || !primary_) {
        return false;
    }

    SIGMA2::RadioRx rx = {};
    if (!primary_->receive(rx)) {
        return false;
    }
    if (rx.received_ms == 0u) {
        rx.received_ms = now_ms;
    }
    return ingest_received_frame(rx);
}

bool Mesh::ingest_received_frame(const SIGMA2::RadioRx& rx)
{
    SIGMA2::DecodedFrame decoded = {};
    const SIGMA2::DecodeStatus status =
        SIGMA2::deserialize_frame(rx.data, rx.len, decoded);
    if (status != SIGMA2::DecodeStatus::Ok) {
        ++stats_.rx_err;
        return false;
    }

    if (decoded.header.node_src == config_.node_id) {
        ++stats_.rx_self;
        return false;
    }

    if (decoded.header.ttl == 0u) {
        ++stats_.rx_ttl_expired;
        return false;
    }

    if (!mark_seen(decoded.header, rx.received_ms)) {
        ++stats_.rx_duplicate;
        return false;
    }

    ++stats_.rx_ok;

    bool handled = false;
    if (should_deliver(decoded.header)) {
        if (enqueue_received(rx, decoded)) {
            ++stats_.delivered;
            handled = true;
        } else {
            ++stats_.delivery_dropped;
        }
    }

    if (should_forward(decoded.header)) {
        if (forward_received(rx, decoded)) {
            ++stats_.forwarded;
            handled = true;
        } else {
            ++stats_.forward_dropped;
        }
    }

    return handled;
}

void Mesh::tick_1hz(const SIGMA2::MeshSnapshot& snapshot)
{
    ++tick_count_;

    if (snapshot.have_nav) {
        enqueue_nav_state(snapshot.nav, snapshot.boot_ms);
    }

    if (snapshot.have_gps) {
        enqueue_gps_nav(snapshot.gps, snapshot.boot_ms);
    }

    if (snapshot.have_gps &&
        config_.time_sync_period_ticks != 0u &&
        (tick_count_ % config_.time_sync_period_ticks) == 0u) {
        for (uint8_t i = 0; i < config_.time_sync_burst_count; ++i) {
            enqueue_time_sync(snapshot.gps, snapshot.boot_ms);
        }
    }
}

bool Mesh::transmit_one(int* radio_state)
{
    if (radio_state) {
        *radio_state = 0;
    }
    if (!ready_ || !primary_ || count_ == 0u) {
        return false;
    }

    const Frame& frame = queue_[tail_];
    const int state = primary_->transmit(frame.buf, frame.len);
    if (radio_state) {
        *radio_state = state;
    }

    tail_ = (tail_ + 1u) % QUEUE_DEPTH;
    --count_;

    if (state == 0) {
        ++stats_.ok;
    } else {
        ++stats_.err;
    }
    return true;
}

std::size_t Mesh::queued() const
{
    return count_;
}

std::size_t Mesh::received_queued() const
{
    return rx_count_;
}

bool Mesh::read_received(SIGMA2::MeshReceivedFrame& out)
{
    if (rx_count_ == 0u) {
        return false;
    }

    out = rx_queue_[rx_tail_];
    rx_tail_ = (rx_tail_ + 1u) % RX_QUEUE_DEPTH;
    --rx_count_;
    return true;
}

bool Mesh::enqueue(const uint8_t* data, std::size_t len)
{
    if (!data || len == 0u || len > SIGMA2::MAX_FRAME) {
        return false;
    }
    if (count_ >= QUEUE_DEPTH) {
        ++stats_.dropped;
        return false;
    }

    Frame& frame = queue_[head_];
    std::memcpy(frame.buf, data, len);
    frame.len = static_cast<uint16_t>(len);
    head_ = (head_ + 1u) % QUEUE_DEPTH;
    ++count_;
    return true;
}

bool Mesh::enqueue_received(const SIGMA2::RadioRx& rx,
                            const SIGMA2::DecodedFrame& decoded)
{
    if (rx_count_ >= RX_QUEUE_DEPTH) {
        return false;
    }

    SIGMA2::MeshReceivedFrame& event = rx_queue_[rx_head_];
    event.rx = rx;
    event.header = decoded.header;
    rx_head_ = (rx_head_ + 1u) % RX_QUEUE_DEPTH;
    ++rx_count_;
    return true;
}

bool Mesh::mark_seen(const SIGMA2::HEADER& header, uint32_t now_ms)
{
    const std::size_t bucket_idx = dedupe_bucket(header);
    SeenFrame* bucket = seen_[bucket_idx];

    for (std::size_t i = 0; i < DEDUPE_WAYS; ++i) {
        const SeenFrame& seen = bucket[i];
        if (seen.valid &&
            seen.src == header.node_src &&
            seen.type == header.type &&
            seen.seq == header.seq) {
            return false;
        }
    }

    std::size_t insert_idx = 0;
    bool found_empty = false;
    uint32_t oldest_seen_ms = bucket[0].last_seen_ms;

    for (std::size_t i = 0; i < DEDUPE_WAYS; ++i) {
        if (!bucket[i].valid) {
            insert_idx = i;
            found_empty = true;
            break;
        }
        if (!found_empty && bucket[i].last_seen_ms < oldest_seen_ms) {
            oldest_seen_ms = bucket[i].last_seen_ms;
            insert_idx = i;
        }
    }

    SeenFrame& slot = bucket[insert_idx];
    slot.valid = true;
    slot.src = header.node_src;
    slot.type = header.type;
    slot.seq = header.seq;
    slot.last_seen_ms = now_ms;
    return true;
}

std::size_t Mesh::dedupe_bucket(const SIGMA2::HEADER& header) const
{
    uint32_t h = static_cast<uint8_t>(header.node_src);
    h = (h * 16777619u) ^ static_cast<uint8_t>(header.type);
    h = (h * 16777619u) ^ static_cast<uint8_t>(header.seq & 0xFFu);
    h = (h * 16777619u) ^ static_cast<uint8_t>((header.seq >> 8) & 0xFFu);
    return static_cast<std::size_t>(h % DEDUPE_BUCKETS);
}

bool Mesh::should_deliver(const SIGMA2::HEADER& header) const
{
    return header.node_dst == config_.node_id ||
           SIGMA2::is_broadcast(header.node_dst);
}

bool Mesh::should_forward(const SIGMA2::HEADER& header) const
{
    if (!config_.controlled_flooding_enabled) {
        return false;
    }
    if (header.node_src == config_.node_id) {
        return false;
    }
    if (header.ttl <= 1u) {
        return false;
    }
    if (header.node_dst == config_.node_id) {
        return false;
    }
    return true;
}

bool Mesh::forward_received(const SIGMA2::RadioRx& rx,
                            const SIGMA2::DecodedFrame& decoded)
{
    SIGMA2::HEADER forwarded = decoded.header;
    --forwarded.ttl;

    uint8_t frame[SIGMA2::MAX_FRAME];
    std::size_t written = 0;
    if (!SIGMA2::serialize_frame(forwarded,
                                 rx.data + SIGMA2::HEADER::WIRE_SIZE,
                                 decoded.payload_len,
                                 frame,
                                 sizeof(frame),
                                 written)) {
        return false;
    }

    return enqueue(frame, written);
}

void Mesh::enqueue_nav_state(const SIGMA2::NavSnapshot& nav, uint32_t boot_ms)
{
    SIGMA2::TRANSMIT_PACKETS::NavState ns = {};
    ns.nav_source = nav.nav_source;
    ns.frame = nav.frame;
    ns.lat_deg_e7 = deg_to_e7(nav.lat);
    ns.lon_deg_e7 = deg_to_e7(nav.lon);
    ns.alt_fused_cm = metres_to_cm(nav.alt_fused_m);
    ns.vel_n_cms = metres_per_second_to_cms(nav.vel_ned_ms[0]);
    ns.vel_e_cms = metres_per_second_to_cms(nav.vel_ned_ms[1]);
    ns.vel_d_cms = metres_per_second_to_cms(nav.vel_ned_ms[2]);
    ns.acc_n_mg = metres_per_second_squared_to_mg(nav.acc_ned_mss[0]);
    ns.acc_e_mg = metres_per_second_squared_to_mg(nav.acc_ned_mss[1]);
    ns.acc_d_mg = metres_per_second_squared_to_mg(nav.acc_ned_mss[2]);
    ns.q[0] = quat_to_q15(nav.q[0]);
    ns.q[1] = quat_to_q15(nav.q[1]);
    ns.q[2] = quat_to_q15(nav.q[2]);
    ns.q[3] = quat_to_q15(nav.q[3]);
    ns.state = nav.state;
    ns.flags = nav.flags;

    uint8_t frame[SIGMA2::MAX_FRAME];
    const std::size_t n = serializer_.serialize_packet(
        ns,
        config_.default_destination,
        frame,
        sizeof(frame),
        boot_ms);
    enqueue(frame, n);
}

void Mesh::enqueue_gps_nav(const SIGMA2::GpsFix& gps, uint32_t boot_ms)
{
    SIGMA2::TRANSMIT_PACKETS::GPSNav gn = {};
    gn.gps_tow_ms = gps.utc_ms;
    gn.lat_deg_e7 = deg_to_e7(gps.lat);
    gn.lon_deg_e7 = deg_to_e7(gps.lon);
    gn.alt_msl_cm = metres_to_cm(static_cast<float>(gps.alt_m));
    gn.vel_n_cms = metres_per_second_to_cms(gps.vel_ned_ms[0]);
    gn.vel_e_cms = metres_per_second_to_cms(gps.vel_ned_ms[1]);
    gn.vel_d_cms = metres_per_second_to_cms(gps.vel_ned_ms[2]);
    gn.h_acc_cm = gps.h_acc_cm;
    gn.v_acc_cm = gps.v_acc_cm;
    gn.s_acc_cms = gps.s_acc_cms;
    gn.fix_type = gps.fix_type;
    gn.num_sv = gps.satellites;
    gn.flags = gps.flags | SIGMA2::DATA_VALID_FLAG::GPS_VALID;

    uint8_t frame[SIGMA2::MAX_FRAME];
    const std::size_t n = serializer_.serialize_packet(
        gn,
        config_.default_destination,
        frame,
        sizeof(frame),
        boot_ms);
    enqueue(frame, n);
}

void Mesh::enqueue_time_sync(const SIGMA2::GpsFix& gps, uint32_t boot_ms)
{
    SIGMA2::TRANSMIT_PACKETS::TimeSync ts = {};
    ts.t1.timestamp_ms = boot_ms;
    ts.t1.utc_ms = gps.utc_ms;
    ts.t1.flags = SIGMA2::DATA_VALID_FLAG::GPS_VALID |
                  SIGMA2::DATA_VALID_FLAG::TIME_VALID;

    uint8_t frame[SIGMA2::MAX_FRAME];
    const std::size_t n = serializer_.serialize_packet(
        ts,
        config_.default_destination,
        frame,
        sizeof(frame),
        boot_ms);
    enqueue(frame, n);
}

} // namespace mesh
