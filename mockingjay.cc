#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <cassert>
#include <deque>

#include "cache.h"
#include "util.h"

extern std::array<CACHE*, NUM_CACHES> caches;
#define LLC_INDEX 0

#define INF_RD 127
#define MAX_RD 104
#define PC_SIG_MASK ((uint64_t)(0x03FF))
#define HISTORY_LENGTH 32

uint32_t LLC_NUM_WAYS, LLC_NUM_SETS;

/*
  - Upon insertion into the cache, a line’s predicted reuse distance is converted to an ETA
  - ETA values are used to make eviction decisions for future cache misses
  - Cache insertions that are predicted to have ETA values larger than any existing line in the
    set are bypassed, which means that they are not inserted in the cache
  - The ETR value is initialized to the line’s predicted reuse distance and is decremented each time some other line in the set is accessed
  - ETR values are allowed to go below zero (indicating that it wasn't used before its predicted reuse distance)
  - Upon eviction, our policy evicts the line with the largest absolute ETR value, which is the line furthest from its ETA. Thus, the evicted
    line is either predicted to be reused furthest in the future or it was predicted to be reused furthest in the past.
  - Ties are broken by evicting lines with a negative ETR in favor of lines with a positive ETR.
*/

class Counters
{
public:

  void init(uint32_t sets, uint32_t ways)
  {
    etr_entries = std::vector<std::vector<int>>(sets, std::vector<int>(ways));
    set_timestamp = std::vector<int>(sets);
  }

  uint32_t way_to_evict(uint32_t set) const
  {
    assert(set < LLC_NUM_SETS);
    int highest_absolute_value = abs(etr_entries[set][0]);
    uint32_t idx = 0;
    bool is_negative = etr_entries[set][0] < 0;

    for (uint32_t i = 1; i < LLC_NUM_WAYS; i++) {
      int curr_abs_value = abs(of(set, i));
      bool is_curr_neg = of(set, i) < 0;
      if (curr_abs_value > highest_absolute_value ||
          (curr_abs_value == highest_absolute_value && is_curr_neg && !is_negative))
      {
        highest_absolute_value = curr_abs_value;
        idx = i;
        is_negative = is_curr_neg;
      }
    }

    return idx;
  }

  int get_set_timestamp(uint32_t set) const
  {
    assert(set < set_timestamp.size());
    return set_timestamp[set];
  }

private:

  int of(uint32_t set, uint32_t way) const
  {
    assert(set < LLC_NUM_SETS && way < LLC_NUM_WAYS);
    return etr_entries[set][way];
  }

private:
  std::vector<std::vector<int>> etr_entries;
  std::vector<int> set_timestamp;
};


/*
  - Indexed by the PC signature
  - Stores the predicted reuse distance for block corresponding to signatures.
  - PC signature is a hash of the 11 least significant bits of the program counter with a bit indicating
    whether the cache access was a hit or a miss
  - Each entry in the RDP is a 7-bit saturating counter representing the number of set accesses before a cache
    line is predicted to be reused
  - Mockingjay uses separate predictor entries for loads by a PC that hit in the cache and for loads by that 
    same PC that miss in the cache (PC signature)
*/
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

  void set_value_of(uint16_t sig, uint8_t diff)
  {
    entries[sig] = diff;
  }
private:
  std::unordered_map<uint16_t, uint8_t> entries;
};

// The Sampled Cache maintains only tags, not data. Sampled Cache maps block addresses to their last access
// timestamp and their last PC signature
struct SampledCacheEntry
{
  bool valid;

  // FIXME: Assuming a larger timestamp so we don't have to check overflow. The paper demonstrates the usage of 1 byte
  int last_access_timestamp;
  uint64_t last_pc_signature;
  uint64_t tag;
};

class SampledCache
{
public:
  bool is_set_to_sample(uint32_t set)
  {
    // TODO: figure out how to sample sets
    return true;
  }

  void init() {
    sampled_sets = 0;
    sampled_ways = 5;
    for (uint32_t i = 0; i < LLC_NUM_SETS; i++) {
      if (is_set_to_sample(i)) {
        entries[i] = std::vector<SampledCacheEntry>(n_ways());
        sampled_sets++;
      }
    }
  }

  /*
  Sampled Cache is indexed using a concatenation of the 5 set id bits that identify the 32 sampled
  sets and the bits [3:0] of the block address tag
  TODO: only sample 32 sets.
  */
  uint64_t get_set(uint64_t pc) const
  {
    CACHE*& llc = caches[LLC_INDEX];
    uint64_t sampled_set_bits = lg2(n_sampled_sets());
    uint64_t sampled_set_mask = (n_sampled_sets() - 1) << LOG2_BLOCK_SIZE;
    return (pc & sampled_set_mask) >> LOG2_BLOCK_SIZE;
  }

  uint64_t get_tag(uint64_t pc) const
  {
    uint64_t tag_bits = 64 - LOG2_BLOCK_SIZE - lg2(n_sampled_sets());
    return (pc & ((1 << tag_bits) - 1)) >> tag_bits;
  }

  int is_present(uint64_t tag, uint64_t set)
  {
    for (int i = 0; i < n_ways(); i++) {
      if (entries[set][i].valid && tag == entries[set][i].tag)
        return i;
    }
    return -1;
  }

  uint32_t n_ways()         const { return sampled_ways; }
  uint32_t n_sampled_sets() const { return sampled_sets; }

  const SampledCacheEntry& at(uint32_t set, uint32_t way) {
    assert(set < n_sampled_sets() && way < n_ways());
    assert(is_set_to_sample(set));
    return entries[set][way];
  }

private:
  std::unordered_map<uint32_t, std::vector<SampledCacheEntry>> entries;
  uint32_t sampled_ways;
  uint32_t sampled_sets;
};

static SampledCache sampled_cache;
static ReuseDistancePredictor rdp;
static Counters etr_counters;

uint64_t get_pc_signature(uint64_t pc, bool is_hit)
{
  return ((pc << 1) | is_hit) & PC_SIG_MASK;
}

void CACHE::initialize_replacement() {
  LLC_NUM_SETS = NUM_SET;
  LLC_NUM_WAYS = NUM_WAY;
  sampled_cache.init();
  etr_counters.init(LLC_NUM_SETS, LLC_NUM_WAYS);
}

// find replacement victim
uint32_t CACHE::find_victim(uint32_t cpu, uint64_t instr_id, uint32_t set, const BLOCK* current_set, uint64_t ip, uint64_t full_addr, uint32_t type)
{
  return 0;
  for (uint32_t i = 0; i < LLC_NUM_WAYS; i++)
  {
    if (!current_set[i].valid)
      return i;
  }

  uint32_t way_to_evict = etr_counters.way_to_evict(set);

  return way_to_evict;
}

// called on every cache hit and cache fill
void CACHE::update_replacement_state(uint32_t cpu, uint32_t set, uint32_t way, uint64_t full_addr, uint64_t ip, uint64_t victim_addr, uint32_t type,
                                     uint8_t hit)
{
  return;
  if (hit && type == WRITEBACK)
    return;
  uint16_t pc_sig = get_pc_signature(ip, hit);
  if (sampled_cache.is_set_to_sample(set)) {
    auto set = sampled_cache.get_set(full_addr);
    auto tag = sampled_cache.get_tag(full_addr);
    auto way = sampled_cache.is_present(tag, set);
    if (way == -1) goto out;

    auto &entry = sampled_cache.at(set, way);
    auto last_sig = entry.last_pc_signature;
    auto last_access_timestamp = entry.last_access_timestamp;
    int diff = abs(last_access_timestamp - etr_counters.get_set_timestamp(set));
    
    if (diff <= 127) {
      int rdp_value = rdp.get_value_of(pc_sig);
      int new_val = 0;

      if (rdp_value == std::numeric_limits<uint8_t>::max()) new_val = diff;
      else new_val = rdp_value + (rdp_value > diff) ? std::min(1, diff / 16) : -std::min(1, diff / 16);

      rdp.set_value_of(pc_sig, new_val);      
    }
  }
out:


  return;
}

void CACHE::replacement_final_stats() {
}
