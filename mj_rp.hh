/**
 * Copyright (c) 2018 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Declaration of a Least Recently Used replacement policy.
 * The victim is chosen using the last touch timestamp.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_MJ_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_MJ_RP_HH__

#include "mem/cache/replacement_policies/base.hh"

// #include "mem/packet.hh"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <iterator>
#include <unordered_map>

#define LOG2_BLOCK_SIZE 6
#define LLC_NUM_SETS ((1024 * 1024) / (16 * 64))
#define LLC_NUM_WAYS 16
#define LLC_NUM_SETS_LOG_2 10
#define LLC_BLOCK_SIZE 64
#define LLC_BLOCK_SIZE_LOG_2 6

#define INF_ETR 15
#define MAX_ETR_CLOCK 8
#define CONSTANT_FACTOR_F 8

#define INF_RD 127
#define MAX_RD 104

#define PC_SIG_MASK 0x7ff

#define CONSTANT_FACTOR_F 8

#define LOG2_LLC_SIZE 20
#define LOG2_SAMPLED_SETS 4
#define NUM_SAMPLED_CACHE_SETS 16

#define SAMPLED_CACHE_WAYS 5
#define SAMPLED_CACHE_SETS 16
#define SAMPLED_CACHE_TAG_BITS 11
#define LOG2_SAMPLED_CACHE_SETS 4
#define TIMESTAMP_WIDTH 8

namespace gem5
{

struct MJRPParams;

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

class Counters
{
  public:
    void init(uint32_t sets, uint32_t ways)
    {
        etr_entries =
            std::vector<std::vector<int>>(sets, std::vector<int>(ways));
    }

    uint32_t way_to_evict(uint32_t set) const
    {
        assert(set < LLC_NUM_SETS);
        int highest_absolute_value = abs(etr_entries[set][0]);
        uint32_t idx = 0;
        bool is_negative = etr_entries[set][0] < 0;

        for (uint32_t i = 1; i < LLC_NUM_WAYS; i++)
        {
            int curr_abs_value = abs(of(set, i));
            bool is_curr_neg = of(set, i) < 0;
            if (curr_abs_value > highest_absolute_value ||
                (curr_abs_value == highest_absolute_value && is_curr_neg &&
                 !is_negative))
            {
                highest_absolute_value = curr_abs_value;
                idx = i;
                is_negative = is_curr_neg;
            }
        }

        return idx;
    }

    int get_set_timestamp(uint32_t set)
    {
        // assert(set < set_timestamp.size());
        return set_timestamp[set];
    }

    void advance_set_clock(uint32_t set)
    {
        set_timestamp[set] = (set_timestamp[set] + 1) % 256;
    }

    int get_estimated_time_remaining(uint32_t set, uint32_t way) const
    {
        return of(set, way);
    }

    void set_estimated_time_remaining(uint32_t set, uint32_t way, int etr)
    {
        assert(set < LLC_NUM_SETS && way < LLC_NUM_WAYS);
        etr_entries[set][way] = etr;
    }

    void age_block(uint32_t set, uint32_t way)
    {
        assert(set < LLC_NUM_SETS && way < LLC_NUM_WAYS);
        etr_entries[set][way] -= 1;
    }

    int get_current_set_clock(uint32_t set)
    {
        // assert(set < set_timestamp.size());
        return set_current_clock[set];
    }

    void set_current_set_clock(uint32_t set, int clk)
    {
        // assert(set < set_timestamp.size());
        set_current_clock[set] = clk;
    }

    void reset_set_clock(uint32_t set)
    {
        assert(set < LLC_NUM_SETS);
        set_current_clock[set] = 0;
    }

  private:
    int of(uint32_t set, uint32_t way) const
    {
        assert(set < LLC_NUM_SETS && way < LLC_NUM_WAYS);
        return etr_entries[set][way];
    }

  private:
    std::vector<std::vector<int>> etr_entries;
    std::unordered_map<uint32_t, int> set_timestamp;
    std::unordered_map<uint32_t, int> set_current_clock;
};

class ReuseDistancePredictor
{
  public:
    uint8_t get_value_of(uint16_t sig)
    {
        if (entries.find(sig) != std::end(entries))
            return entries[sig];
        else
            return std::numeric_limits<uint8_t>::max();
    }

    void set_value_of(uint16_t sig, uint8_t diff) { entries[sig] = diff; }

  private:
    std::unordered_map<uint16_t, uint8_t> entries;
};

struct SampledCacheEntry
{
    static SampledCacheEntry& def()
    {
        static SampledCacheEntry ret = SampledCacheEntry();
        ret.valid = true;
        return ret;
    }
    bool valid;
    int last_access_timestamp;
    uint64_t last_pc_signature;
    uint64_t tag;
};

class SampledCache
{
  public:
    bool is_set_to_sample(uint32_t set)
    {
        int mask_length = LLC_NUM_SETS_LOG_2 - LOG2_SAMPLED_SETS;
        int mask = (1 << mask_length) - 1;
        return (set & mask) ==
               ((set >> (LLC_NUM_SETS_LOG_2 - mask_length)) & mask);
    }

    SampledCacheEntry& get_invalid_entry(uint32_t set)
    {
        for (int i = 0; i < SAMPLED_CACHE_WAYS; i++)
        {
            if (!entries[set][i].valid)
                return entries[set][i];
        }
        return SampledCacheEntry::def();
    }

    bool valid;
    void init()
    {
        warn("ENTER Sampled Cache INIT\n");
        std::vector<int> sampled_sets;
        sampled_ways = 5;
        for (uint32_t i = 0; i < LLC_NUM_SETS; i++)
            if (is_set_to_sample(i))
                sampled_sets.push_back(i);

        for (int s : sampled_sets)
        {
            // entries[i] = std::vector<SampledCacheEntry>(n_ways());
            int modifier = 1 << LLC_NUM_SETS_LOG_2;
            for (int j = 0; j < NUM_SAMPLED_CACHE_SETS; j++)
            {
                entries[s + modifier * j] =
                    std::vector<SampledCacheEntry>(SAMPLED_CACHE_WAYS);
            }
        }

        for (auto s : entries)
            warn("SAMPLED SET: %d\n", s.first);

        warn("EXIT Sampled Cache INIT\n");
    }

    /*
        Sampled Cache is indexed using a concatenation of the 5 set id bits
       that identify the 32 sampled sets and the bits [3:0] of the block
       address tag
        TODO: only sample 32 sets.
    */
    uint64_t get_set(uint64_t addr) const
    {
        // uint64_t sampled_set_bits = log2(n_sampled_sets());
        // uint64_t sampled_set_mask = (n_sampled_sets() - 1) <<
        // LOG2_BLOCK_SIZE; return (pc & sampled_set_mask) >> LOG2_BLOCK_SIZE;
        addr = addr >> LOG2_BLOCK_SIZE;
        addr = (addr << (64 - (LOG2_SAMPLED_CACHE_SETS + 10))) >>
               (64 - (LOG2_SAMPLED_CACHE_SETS + 10));
        return addr;
    }

    uint64_t get_tag(uint64_t addr) const
    {
        // uint64_t tag_bits = 64 - LOG2_BLOCK_SIZE - log2(n_sampled_sets());
        // return (pc & ((1 << tag_bits) - 1)) >> tag_bits;
        addr >>= 20;
        addr = (addr << (64 - SAMPLED_CACHE_TAG_BITS)) >>
               (64 - SAMPLED_CACHE_TAG_BITS);
        return addr;
    }

    int is_present(uint64_t tag, uint64_t set)
    {
        warn("ENTER is_present\n");
        warn("set: %d\n", set);
        if (entries.find(set) == std::end(entries))
            return -1;
        for (int i = 0; i < n_ways(); i++)
        {
            if (entries[set][i].valid && tag == entries[set][i].tag)
            {
                warn("EXIT is_present\n");
                return i;
            }
        }
        warn("EXIT is_present\n");
        return -1;
    }

    uint32_t n_ways() const { return sampled_ways; }

    SampledCacheEntry& at(uint32_t set, uint32_t way)
    {
        // assert(way < n_ways());
        warn("ENTER SAMPLED_CACHE_AT: set: %d, way: %d\n", set, way);
        assert(entries.find(set) != std::end(entries));
        assert(entries[set].size() > way);
        auto& ret = entries[set][way];
        warn("ENTER SAMPLED_CACHE_AT\n");
        return ret;
    }

  private:
    std::unordered_map<uint32_t, std::vector<SampledCacheEntry>> entries;
    uint32_t sampled_ways;
};

class MJRP : public Base
{
  protected:
    /** LRU-specific implementation of replacement data. */
    struct MJReplData : public ReplacementData
    {
        /** Tick on which the entry was last touched. */
        Tick lastTouchTick;
        bool valid;
        uint64_t generatingPCSignature;
        uint64_t blkAddr;
        uint64_t set;
        uint64_t way;
        MJReplData() : lastTouchTick(0), valid(false) {}
        MJReplData(int s, int w)
            : lastTouchTick(0), valid(false), set(s), way(w)
        {
        }
    };

  public:
    typedef MJRPParams Params;
    MJRP(const Params& p);
    ~MJRP() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(
        const std::shared_ptr<ReplacementData>& replacement_data) override;

    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data)
        const override;
    void touch(const std::shared_ptr<ReplacementData>& replacement_data,
               const PacketPtr pkt) override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data)
        const override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
               const PacketPtr pkt) override;

    /**
     * Find replacement victim using MJRP timestamps.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry*
    getVictim(const ReplacementCandidates& candidates) const override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
    std::shared_ptr<ReplacementData> instantiateEntry(int set,
                                                      int way) override;

    // void penalize_block(uint64_t set, uint64_t way)
    // {
    //     if (!sampled_cache.at(set, way).valid)
    //         return;
    //     if (rdp.get_value_of(sampled_cache.at(set, way).last_pc_signature)
    //     !=
    //         std::numeric_limits<uint8_t>::max())
    //     {
    //         rdp.set_value_of(
    //             sampled_cache.at(set, way).last_pc_signature,
    //             rdp.get_value_of(
    //                 sampled_cache.at(set, way).last_pc_signature) +
    //                 1);
    //         if (rdp.get_value_of(
    //                 sampled_cache.at(set, way).last_pc_signature) > INF_RD)
    //             rdp.set_value_of(sampled_cache.at(set,
    //             way).last_pc_signature,
    //                              INF_RD);
    //     }
    //     sampled_cache.at(set, way).valid = false;
    // }

  private:
    SampledCache sampled_cache;
    ReuseDistancePredictor rdp;
    Counters etr_counters;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_MJRP_RP_HH__
