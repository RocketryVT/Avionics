#pragma once

#include "SIGMA2/SIGMA2.hpp"
#include "SIGMA2/packets/gps_packets.hpp"
#include "SIGMA2/packets/mesh_packets.hpp"

#include <cstddef>
#include <cstdint>

namespace mesh {

class Mesh {
public:
    explicit Mesh(const SIGMA2::MeshConfig& config);

    void set_primary_radio(SIGMA2::Radio& radio);
    [[nodiscard]] bool begin();
    [[nodiscard]] bool ready() const;

    // Called by the application scheduler. Builds packets from the newest
    // sensor/fusion snapshot and stages them for radio transmission.
    void tick_1hz(const SIGMA2::MeshSnapshot& snapshot);

    // Polls the primary radio once and ingests a received frame if available.
    [[nodiscard]] bool poll_receive(uint32_t now_ms);

    // Ingests one received radio frame. Valid frames are deduped, delivered
    // locally when applicable, and forwarded with decremented TTL when allowed.
    [[nodiscard]] bool ingest_received_frame(const SIGMA2::RadioRx& rx);

    // Transmits one queued packet if any are pending.
    [[nodiscard]] bool transmit_one(int* radio_state = nullptr);

    [[nodiscard]] bool read_received(SIGMA2::MeshReceivedFrame& out);
    [[nodiscard]] const SIGMA2::MeshTxStats& stats() const { return stats_; }
    [[nodiscard]] std::size_t queued() const;
    [[nodiscard]] std::size_t received_queued() const;

private:
    struct Frame {
        uint8_t buf[SIGMA2::MAX_FRAME];
        uint16_t len = 0;
    };

#ifndef SIGMA2_MESH_QUEUE_DEPTH
#define SIGMA2_MESH_QUEUE_DEPTH 16
#endif
#ifndef SIGMA2_MESH_RX_QUEUE_DEPTH
#define SIGMA2_MESH_RX_QUEUE_DEPTH 8
#endif
#ifndef SIGMA2_MESH_DEDUPE_BUCKETS
#define SIGMA2_MESH_DEDUPE_BUCKETS 16
#endif
#ifndef SIGMA2_MESH_DEDUPE_WAYS
#define SIGMA2_MESH_DEDUPE_WAYS 4
#endif

    static constexpr std::size_t QUEUE_DEPTH = SIGMA2_MESH_QUEUE_DEPTH;
    static constexpr std::size_t RX_QUEUE_DEPTH = SIGMA2_MESH_RX_QUEUE_DEPTH;
    static constexpr std::size_t DEDUPE_BUCKETS = SIGMA2_MESH_DEDUPE_BUCKETS;
    static constexpr std::size_t DEDUPE_WAYS = SIGMA2_MESH_DEDUPE_WAYS;

    struct SeenFrame {
        bool valid = false;
        SIGMA2::NodeID src = SIGMA2::NodeID::UNDEFINED;
        SIGMA2::PacketType type = SIGMA2::PacketType::UNKNOWN;
        uint16_t seq = 0;
        uint32_t last_seen_ms = 0;
    };

    SIGMA2::MeshConfig config_;
    SIGMA2::Serializer serializer_;
    SIGMA2::Radio* primary_ = nullptr;
    bool ready_ = false;
    SIGMA2::MeshTxStats stats_ = {};
    Frame queue_[QUEUE_DEPTH] = {};
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t count_ = 0;
    uint32_t tick_count_ = 0;
    SIGMA2::MeshReceivedFrame rx_queue_[RX_QUEUE_DEPTH] = {};
    std::size_t rx_head_ = 0;
    std::size_t rx_tail_ = 0;
    std::size_t rx_count_ = 0;
    SeenFrame seen_[DEDUPE_BUCKETS][DEDUPE_WAYS] = {};

    bool enqueue(const uint8_t* data, std::size_t len);
    bool enqueue_received(const SIGMA2::RadioRx& rx, const SIGMA2::DecodedFrame& decoded);
    bool mark_seen(const SIGMA2::HEADER& header, uint32_t now_ms);
    std::size_t dedupe_bucket(const SIGMA2::HEADER& header) const;
    bool should_deliver(const SIGMA2::HEADER& header) const;
    bool should_forward(const SIGMA2::HEADER& header) const;
    bool forward_received(const SIGMA2::RadioRx& rx, const SIGMA2::DecodedFrame& decoded);
    void enqueue_nav_state(const SIGMA2::NavSnapshot& nav, uint32_t boot_ms);
    void enqueue_gps_nav(const SIGMA2::GpsFix& gps, uint32_t boot_ms);
    void enqueue_time_sync(const SIGMA2::GpsFix& gps, uint32_t boot_ms);
};

} // namespace mesh
