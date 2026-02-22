#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <concepts>

template <typename T>
concept LoraImpl = requires(T t,
                            std::span<const uint8_t> tx,
                            std::span<uint8_t>       rx,
                            uint32_t freq,
                            uint8_t  bw,
                            uint8_t  sf,
                            uint8_t  cr,
                            int8_t   pwr) {
    { t.initialize()             } -> std::same_as<bool>;
    { t.send(tx)                 } -> std::same_as<bool>;
    { t.receive(rx)              } -> std::same_as<int>;
    { t.set_frequency(freq)      } -> std::same_as<void>;
    { t.set_bandwidth(bw)        } -> std::same_as<void>;
    { t.set_spreading_factor(sf) } -> std::same_as<void>;
    { t.set_coding_rate(cr)      } -> std::same_as<void>;
    { t.set_tx_power(pwr)        } -> std::same_as<void>;
    { t.get_rssi()               } -> std::convertible_to<int16_t>;
    { t.get_snr()                } -> std::convertible_to<int8_t>;
    { t.sleep()                  } -> std::same_as<void>;
    { t.standby()                } -> std::same_as<void>;
};
