/**
 * Copyright (c) 2018-2020 Inria
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

#include "mem/cache/replacement_policies/mj_rp.hh"

#include <cassert>
#include <cmath>
#include <memory>

#include "params/MJRP.hh"
#include "sim/cur_tick.hh"

#ifdef warn
#undef warn
#define warn(...)
#endif

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

static uint64_t murmur_hash(uint64_t pc)
{
    pc ^= pc >> 33;
    pc *= 0xff51afd7ed558ccdL;
    pc ^= pc >> 33;
    pc *= 0xc4ceb9fe1a85ec53L;
    pc ^= pc >> 33;
    return pc;
}

static uint64_t get_pc_signature(uint64_t pc, bool is_hit)
{
    return murmur_hash((pc << 1) | (is_hit)) & (PC_SIG_MASK);
}

MJRP::MJRP(const Params& p) : Base(p)
{
    sampled_cache.init();
    etr_counters.init(LLC_NUM_SETS, LLC_NUM_WAYS);
}

void MJRP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    auto set = std::static_pointer_cast<MJReplData>(replacement_data)->set;
    auto way = std::static_pointer_cast<MJReplData>(replacement_data)->way;

    if (sampled_cache.is_set_to_sample(set))
    {
        auto fullAddr =
            std::static_pointer_cast<MJReplData>(replacement_data)->blkAddr;
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way == -1)
            return;
        auto& entry = sampled_cache.at(sampled_set, sampled_way);
        if (!entry.valid)
            return;
        entry.valid = false;
        rdp.set_value_of(entry.last_pc_signature, MAX_RD);
        etr_counters.set_estimated_time_remaining(set, way, INF_ETR);
    }
}

void MJRP::touch(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    panic("CAN'T OPERATE WITHOUT ACCESS PACKET INFORMATION");
}

void MJRP::touch(const std::shared_ptr<ReplacementData>& replacement_data,
                 const PacketPtr pkt)
{
    warn("ENTER TOUCH\n");
    auto castedData = std::static_pointer_cast<MJReplData>(replacement_data);
    uint64_t ip = 0;
    if (pkt->req->hasPC())
        ip = pkt->req->getPC();
    auto fullAddr = pkt->getAddr();
    auto set = castedData->set;
    auto way = castedData->way;
    uint16_t pcSignature = get_pc_signature(ip, true);
    castedData->generatingPCSignature = pcSignature;
    if (sampled_cache.is_set_to_sample(set))
    {
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way == -1)
            goto out;

        auto& entry = sampled_cache.at(sampled_set, sampled_way);
        auto last_sig = entry.last_pc_signature;
        auto last_access_timestamp = entry.last_access_timestamp;
        auto sset_timestamp = etr_counters.get_set_timestamp(set);
        int diff = abs(last_access_timestamp - sset_timestamp);

        if (diff <= 127)
        {
            int rdp_value = rdp.get_value_of(last_sig);
            int new_val = 0;
            if (rdp_value == std::numeric_limits<uint8_t>::max())
                new_val = diff;
            else
                new_val =
                    rdp_value + ((rdp_value > diff) ? std::min(1, diff / 16)
                                                    : -std::min(1, diff / 16));

            rdp.set_value_of(last_sig, new_val);
            sampled_cache.at(sampled_set, sampled_way).valid = false;
        }

        int lru_way = -1;
        int lru_reuse_distance = -1;
        for (int i = 0; i < SAMPLED_CACHE_WAYS; i++)
        {
            uint64_t last_timestamp =
                sampled_cache.at(sampled_set, i).last_access_timestamp;
            int set_timestamp = etr_counters.get_set_timestamp(sampled_set);
            int reuse_distance = 0;
            if (set_timestamp > last_timestamp)
                reuse_distance = set_timestamp - last_timestamp;
            else
            {
                reuse_distance = set_timestamp + (1 << TIMESTAMP_WIDTH);
                reuse_distance -= last_timestamp;
            }
            if (reuse_distance > INF_RD)
            {
                lru_way = i;
                lru_reuse_distance = INF_RD + 1;
                if (sampled_cache.at(sampled_set, i).valid)
                {
                    if (rdp.get_value_of(sampled_cache.at(sampled_set, i)
                                             .last_pc_signature) !=
                        std::numeric_limits<uint8_t>::max())
                    {
                        rdp.set_value_of(
                            sampled_cache.at(sampled_set, i).last_pc_signature,
                            rdp.get_value_of(sampled_cache.at(sampled_set, i)
                                                 .last_pc_signature) +
                                1);
                        if (rdp.get_value_of(sampled_cache.at(sampled_set, i)
                                                 .last_pc_signature) > INF_RD)
                            rdp.set_value_of(sampled_cache.at(sampled_set, i)
                                                 .last_pc_signature,
                                             INF_RD);
                    }
                    sampled_cache.at(sampled_set, i).valid = false;
                }
            }
            else if (reuse_distance > lru_reuse_distance)
            {
                lru_way = i;
                lru_reuse_distance = reuse_distance;
            }
            if (!sampled_cache.at(sampled_set, i).valid)
            {
                lru_way = i;
                lru_reuse_distance = INF_RD + 1;
            }
        }
        if (sampled_cache.at(sampled_set, lru_way).valid)
        {
            if (rdp.get_value_of(sampled_cache.at(sampled_set, lru_way)
                                     .last_pc_signature) !=
                std::numeric_limits<uint8_t>::max())
            {
                rdp.set_value_of(
                    sampled_cache.at(sampled_set, lru_way).last_pc_signature,
                    rdp.get_value_of(sampled_cache.at(sampled_set, lru_way)
                                         .last_pc_signature) +
                        1);
                if (rdp.get_value_of(sampled_cache.at(sampled_set, lru_way)
                                         .last_pc_signature) > INF_RD)
                    rdp.set_value_of(sampled_cache.at(sampled_set, lru_way)
                                         .last_pc_signature,
                                     INF_RD);
            }
            sampled_cache.at(sampled_set, lru_way).valid = false;
        }

        auto& invalidEntry = sampled_cache.get_invalid_entry(sampled_set);
        if (!invalidEntry.valid)
        {
            invalidEntry.valid = true;
            invalidEntry.last_pc_signature = pcSignature;
            invalidEntry.tag = sampled_tag;
            invalidEntry.last_access_timestamp =
                etr_counters.get_set_timestamp(set);
        }
        etr_counters.advance_set_clock(set);
    }
out:

    if (etr_counters.get_current_set_clock(set) == CONSTANT_FACTOR_F)
    {
        for (int i = 0; i < LLC_NUM_WAYS; i++)
        {
            if (i == way)
                continue;
            /* WE ONLY AGE NON-SCANNING LINES. Those are the lines that we
             * denote as not being accessed again, and they priorizied as
             * choices for eviction. */
            if (etr_counters.get_estimated_time_remaining(set, i) < INF_ETR)
                etr_counters.age_block(set, i);
        }
        etr_counters.reset_set_clock(set);
    }
    etr_counters.set_current_set_clock(
        set, etr_counters.get_current_set_clock(set) + 1);

    /*
     * On insertion and promotion, a line’s ETR is initialized with its
     * predicted reuse distance obtained from the RDP "Mockingjay tracks
     * coarse-grained reuse distances (and corresponding ETRs), which are
     * obtained by dividing the precise value by a constant factor f, where f
     * is set to 8 in our evaluation" This ensures that non-scanning lines' ETR
     * values are effectively decrement every 8 accesses by 1.
     */
    if (rdp.get_value_of(pcSignature) == std::numeric_limits<uint8_t>::max())
    {
        etr_counters.set_estimated_time_remaining(set, way, 0);
    }
    else
    {
        if (rdp.get_value_of(pcSignature) > MAX_RD)
            etr_counters.set_estimated_time_remaining(set, way, INF_ETR);
        else
            etr_counters.set_estimated_time_remaining(
                set, way, rdp.get_value_of(pcSignature) / CONSTANT_FACTOR_F);
    }

    warn("EXIT TOUCH\n");
    return;
}

void MJRP::reset(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    panic("CAN'T OPERATE WITHOUT ACCESS PACKET INFORMATION");
}

void MJRP::reset(const std::shared_ptr<ReplacementData>& replacement_data,
                 const PacketPtr pkt)
{
    warn("ENTER RESET\n");
    std::static_pointer_cast<MJReplData>(replacement_data)->lastTouchTick =
        curTick();
    std::static_pointer_cast<MJReplData>(replacement_data)
        ->generatingPCSignature =
        pkt->req->hasPC() ? get_pc_signature(pkt->req->getPC(), false) : 0;
    std::static_pointer_cast<MJReplData>(replacement_data)->blkAddr =
        pkt->getAddr();

    auto castedData = std::static_pointer_cast<MJReplData>(replacement_data);
    uint64_t ip = 0;
    if (pkt->req->hasPC())
        ip = pkt->req->getPC();
    auto fullAddr = pkt->getAddr();
    auto set = castedData->set;
    auto way = castedData->way;
    uint16_t pcSignature = get_pc_signature(ip, false);
    castedData->generatingPCSignature = pcSignature;
    if (sampled_cache.is_set_to_sample(set))
    {
        auto sampled_set = sampled_cache.get_set(fullAddr);
        auto sampled_tag = sampled_cache.get_tag(fullAddr);
        auto sampled_way = sampled_cache.is_present(sampled_tag, sampled_set);
        if (sampled_way == -1)
            goto out;

        auto& entry = sampled_cache.at(sampled_set, sampled_way);
        auto last_sig = entry.last_pc_signature;
        auto last_access_timestamp = entry.last_access_timestamp;
        auto sset_timestamp = etr_counters.get_set_timestamp(set);
        int diff = abs(last_access_timestamp - sset_timestamp);

        if (diff <= 127)
        {
            int rdp_value = rdp.get_value_of(last_sig);
            int new_val = 0;
            if (rdp_value == std::numeric_limits<uint8_t>::max())
                new_val = diff;
            else
                new_val =
                    rdp_value + ((rdp_value > diff) ? std::min(1, diff / 16)
                                                    : -std::min(1, diff / 16));

            rdp.set_value_of(last_sig, new_val);
            sampled_cache.at(sampled_set, sampled_way).valid = false;
        }

        int lru_way = -1;
        int lru_rd = -1;
        for (int i = 0; i < SAMPLED_CACHE_WAYS; i++)
        {
            uint64_t last_timestamp =
                sampled_cache.at(sampled_set, i).last_access_timestamp;
            int set_timestamp = etr_counters.get_set_timestamp(set);
            int reuse_distance = 0;
            if (set_timestamp > last_timestamp)
                reuse_distance = set_timestamp - last_timestamp;
            else
                reuse_distance =
                    set_timestamp + (1 << TIMESTAMP_WIDTH) - last_timestamp;

            if (reuse_distance > INF_RD)
            {
                lru_way = i;
                lru_rd = INF_RD + 1;
                if (sampled_cache.at(sampled_set, i).valid)
                {
                    if (rdp.get_value_of(sampled_cache.at(sampled_set, i)
                                             .last_pc_signature) !=
                        std::numeric_limits<uint8_t>::max())
                    {
                        rdp.set_value_of(
                            sampled_cache.at(sampled_set, i).last_pc_signature,
                            rdp.get_value_of(sampled_cache.at(sampled_set, i)
                                                 .last_pc_signature) +
                                1);
                        if (rdp.get_value_of(sampled_cache.at(sampled_set, i)
                                                 .last_pc_signature) > INF_RD)
                            rdp.set_value_of(sampled_cache.at(sampled_set, i)
                                                 .last_pc_signature,
                                             INF_RD);
                    }
                    sampled_cache.at(sampled_set, i).valid = false;
                }
            }
            else if (reuse_distance > lru_rd)
            {
                lru_way = i;
                lru_rd = reuse_distance;
            }

            if (!sampled_cache.at(sampled_set, i).valid)
            {
                lru_way = i;
                lru_rd = INF_RD + 1;
            }
        }
        if (sampled_cache.at(sampled_set, lru_way).valid)
        {
            if (rdp.get_value_of(sampled_cache.at(sampled_set, lru_way)
                                     .last_pc_signature) !=
                std::numeric_limits<uint8_t>::max())
            {
                rdp.set_value_of(
                    sampled_cache.at(sampled_set, lru_way).last_pc_signature,
                    rdp.get_value_of(sampled_cache.at(sampled_set, lru_way)
                                         .last_pc_signature) +
                        1);
                if (rdp.get_value_of(sampled_cache.at(sampled_set, lru_way)
                                         .last_pc_signature) > INF_RD)
                    rdp.set_value_of(sampled_cache.at(sampled_set, lru_way)
                                         .last_pc_signature,
                                     INF_RD);
            }
            sampled_cache.at(sampled_set, lru_way).valid = false;
        }

        auto& invalidEntry = sampled_cache.get_invalid_entry(sampled_set);
        if (!invalidEntry.valid)
        {
            invalidEntry.valid = true;
            invalidEntry.last_pc_signature = pcSignature;
            invalidEntry.tag = sampled_tag;
            invalidEntry.last_access_timestamp =
                etr_counters.get_set_timestamp(set);
        }
        etr_counters.advance_set_clock(set);
    }
out:
    if (etr_counters.get_current_set_clock(set) == CONSTANT_FACTOR_F)
    {
        for (int i = 0; i < LLC_NUM_WAYS; i++)
        {
            if (i == way)
                continue;
            /* WE ONLY AGE NON-SCANNING LINES. Those are the lines that we
             * denote as not being accessed again, and they priorizied as
             * choices for eviction. */
            if (etr_counters.get_estimated_time_remaining(set, i) < INF_ETR)
                etr_counters.age_block(set, i);
        }
        etr_counters.reset_set_clock(set);
    }
    etr_counters.set_current_set_clock(
        set, etr_counters.get_current_set_clock(set) + 1);

    /*
     * On insertion and promotion, a line’s ETR is initialized with its
     * predicted reuse distance obtained from the RDP "Mockingjay tracks
     * coarse-grained reuse distances (and corresponding ETRs), which are
     * obtained by dividing the precise value by a constant factor f, where f
     * is set to 8 in our evaluation" This ensures that non-scanning lines' ETR
     * values are effectively decrement every 8 accesses by 1.
     */
    if (rdp.get_value_of(pcSignature) == std::numeric_limits<uint8_t>::max())
    {
        etr_counters.set_estimated_time_remaining(set, way, 0);
    }
    else
    {
        if (rdp.get_value_of(pcSignature) > MAX_RD)
            etr_counters.set_estimated_time_remaining(set, way, INF_ETR);
        else
            etr_counters.set_estimated_time_remaining(
                set, way, rdp.get_value_of(pcSignature) / CONSTANT_FACTOR_F);
    }

    warn("EXIT RESET\n");
}

ReplaceableEntry*
MJRP::getVictim(const ReplacementCandidates& candidates) const
{
    warn("ENTER VICTIM\n");
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    uint32_t way_to_evict = etr_counters.way_to_evict(candidates[0]->getSet());
    ReplaceableEntry* victim = candidates[0];
    ReplaceableEntry* lruCandidate = candidates[0];
    for (const auto& candidate : candidates)
    {
        if (candidate->getWay() == way_to_evict)
        {
            victim = candidate;
        }

        if (std::static_pointer_cast<MJReplData>(candidate->replacementData)
                ->lastTouchTick <
            std::static_pointer_cast<MJReplData>(victim->replacementData)
                ->lastTouchTick)
        {
            lruCandidate = candidate;
        }

        std::static_pointer_cast<MJReplData>(candidate->replacementData)->way =
            candidate->getWay();
        std::static_pointer_cast<MJReplData>(candidate->replacementData)->set =
            candidate->getSet();
    }

    if (victim->getWay() != way_to_evict)
    {
        victim = lruCandidate;
        warn("PICKING LRU\n");
    }

    warn("EXIT VICTIM\n");
    return victim;
}

std::shared_ptr<ReplacementData> MJRP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new MJReplData());
}

std::shared_ptr<ReplacementData> MJRP::instantiateEntry(int i, int j)
{
    return std::shared_ptr<ReplacementData>(new MJReplData(i, j));
}

} // namespace replacement_policy
} // namespace gem5
