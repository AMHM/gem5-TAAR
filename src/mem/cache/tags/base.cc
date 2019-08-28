/*
 * Copyright (c) 2013,2016,2018 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Erik Hallnor
 *          Ron Dreslinski
 */

/**
 * @file
 * Definitions of BaseTags.
 */

#include "mem/cache/tags/base.hh"

#include <cassert>

#include "base/types.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/request.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

BaseTags::BaseTags(const Params *p)
    : ClockedObject(p), blkSize(p->block_size), blkMask(blkSize - 1),
      size(p->size), lookupLatency(p->tag_latency),
      system(p->system), indexingPolicy(p->indexing_policy),
      warmupBound((p->warmup_percentage/100.0) * (p->size / p->block_size)),
      warmedUp(false), numBlocks(p->size / p->block_size),
      dataBlks(new uint8_t[p->size]) // Allocate data storage in one big chunk
{
}

ReplaceableEntry*
BaseTags::findBlockBySetAndWay(int set, int way) const
{
    return indexingPolicy->getEntry(set, way);
}

CacheBlk*
BaseTags::findBlock(Addr addr, bool is_secure, int HW, char command) const
{
    //AMHM Start
    
    /*The layout of considered 16-way set associative cache is as follow
     * |HW less than 100: Way 0, way 1, way 2 and way 3                                         |
     * |HW between 101 and 250: way 4, way 5, way 6, way 7, way 8, way 9, way 10, and way 11    |
     * |HW between 251 and 400: way 12 and way 13                                               |
     * |HW over 401: way 14 and way 15                                                          |    
    */
    if(name()=="system.l2.tags"){
        //assert(assoc == 16); FlexRel is only implemented for 16-way set associative configurations!
        // Extract block tag
        Addr tag = extractTag(addr);
        
        bool found = 0;
        CacheBlk* blk = nullptr;
        
        // Find possible entries that may contain the given address
        const std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(addr);

        for (const auto& location : entries) {
            blk = static_cast<CacheBlk*>(location);
            if ((blk->tag == tag) && blk->isValid() &&
                (blk->isSecure() == is_secure)) {
                found = 1;
                break;
            }
      }
        if ((found == 1) && (command == 'w')) {
            int way = blk->getWay();
            if((HW < 100) && (way < 4)) {return blk;} else
            if((HW >= 101) && (HW <= 250) && (way >= 4) && (way < 12)) {return blk;} else
            if((HW >= 251) && (HW <= 400) && (way >= 12) && (way < 14)) {return blk;} else
            if((HW >= 401) && (way >= 14) && (way < 16)) {return blk;} else {
                blk->invalidate();                
                return nullptr;}
        } else if (found) {
            return blk;
        } else
            return nullptr;
          
    } else{
        // Extract block tag
        Addr tag = extractTag(addr);

        // Find possible entries that may contain the given address
        const std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(addr);

        // Search for block
        for (const auto& location : entries) {
            CacheBlk* blk = static_cast<CacheBlk*>(location);
            if ((blk->tag == tag) && blk->isValid() &&
                (blk->isSecure() == is_secure)) {
                return blk;
            }
      } 
    }

    // Did not find block
    return nullptr;
}

void
BaseTags::insertBlock(const Addr addr, const bool is_secure,
                      const int src_master_ID, const uint32_t task_ID,
                      CacheBlk *blk)
{
    assert(!blk->isValid());

    // Previous block, if existed, has been removed, and now we have
    // to insert the new one
    // Deal with what we are bringing in
    assert(src_master_ID < system->maxMasters());
    occupancies[src_master_ID]++;

    // Insert block with tag, src master id and task id
    blk->insert(extractTag(addr), is_secure, src_master_ID, task_ID);

    // Check if cache warm up is done
    if (!warmedUp && tagsInUse.value() >= warmupBound) {
        warmedUp = true;
        warmupCycle = curTick();
    }

    // We only need to write into one tag and one data block.
    tagAccesses += 1;
    dataAccesses += 1;
}

Addr
BaseTags::extractTag(const Addr addr) const
{
    return indexingPolicy->extractTag(addr);
}

void
BaseTags::cleanupRefsVisitor(CacheBlk &blk)
{
    if (blk.isValid()) {
        totalRefs += blk.refCount;
        ++sampledRefs;
    }
}

void
BaseTags::cleanupRefs()
{
    forEachBlk([this](CacheBlk &blk) { cleanupRefsVisitor(blk); });
}

void
BaseTags::computeStatsVisitor(CacheBlk &blk)
{
    if (blk.isValid()) {
        assert(blk.task_id < ContextSwitchTaskId::NumTaskId);
        occupanciesTaskId[blk.task_id]++;
        assert(blk.tickInserted <= curTick());
        Tick age = curTick() - blk.tickInserted;

        int age_index;
        if (age / SimClock::Int::us < 10) { // <10us
            age_index = 0;
        } else if (age / SimClock::Int::us < 100) { // <100us
            age_index = 1;
        } else if (age / SimClock::Int::ms < 1) { // <1ms
            age_index = 2;
        } else if (age / SimClock::Int::ms < 10) { // <10ms
            age_index = 3;
        } else
            age_index = 4; // >10ms

        ageTaskId[blk.task_id][age_index]++;
    }
}

void
BaseTags::computeStats()
{
    for (unsigned i = 0; i < ContextSwitchTaskId::NumTaskId; ++i) {
        occupanciesTaskId[i] = 0;
        for (unsigned j = 0; j < 5; ++j) {
            ageTaskId[i][j] = 0;
        }
    }

    forEachBlk([this](CacheBlk &blk) { computeStatsVisitor(blk); });
}

std::string
BaseTags::print()
{
    std::string str;

    auto print_blk = [&str](CacheBlk &blk) {
        if (blk.isValid())
            str += csprintf("\tBlock: %s\n", blk.print());
    };
    forEachBlk(print_blk);

    if (str.empty())
        str = "no valid tags\n";

    return str;
}

void
BaseTags::regStats()
{
    ClockedObject::regStats();

    using namespace Stats;

    tagsInUse
        .name(name() + ".tagsinuse")
        .desc("Cycle average of tags in use")
        ;

    totalRefs
        .name(name() + ".total_refs")
        .desc("Total number of references to valid blocks.")
        ;

    sampledRefs
        .name(name() + ".sampled_refs")
        .desc("Sample count of references to valid blocks.")
        ;

    avgRefs
        .name(name() + ".avg_refs")
        .desc("Average number of references to valid blocks.")
        ;

    avgRefs = totalRefs/sampledRefs;

    warmupCycle
        .name(name() + ".warmup_cycle")
        .desc("Cycle when the warmup percentage was hit.")
        ;

    occupancies
        .init(system->maxMasters())
        .name(name() + ".occ_blocks")
        .desc("Average occupied blocks per requestor")
        .flags(nozero | nonan)
        ;
    for (int i = 0; i < system->maxMasters(); i++) {
        occupancies.subname(i, system->getMasterName(i));
    }

    avgOccs
        .name(name() + ".occ_percent")
        .desc("Average percentage of cache occupancy")
        .flags(nozero | total)
        ;
    for (int i = 0; i < system->maxMasters(); i++) {
        avgOccs.subname(i, system->getMasterName(i));
    }

    avgOccs = occupancies / Stats::constant(numBlocks);

    occupanciesTaskId
        .init(ContextSwitchTaskId::NumTaskId)
        .name(name() + ".occ_task_id_blocks")
        .desc("Occupied blocks per task id")
        .flags(nozero | nonan)
        ;

    ageTaskId
        .init(ContextSwitchTaskId::NumTaskId, 5)
        .name(name() + ".age_task_id_blocks")
        .desc("Occupied blocks per task id")
        .flags(nozero | nonan)
        ;

    percentOccsTaskId
        .name(name() + ".occ_task_id_percent")
        .desc("Percentage of cache occupancy per task id")
        .flags(nozero)
        ;

    percentOccsTaskId = occupanciesTaskId / Stats::constant(numBlocks);

    tagAccesses
        .name(name() + ".tag_accesses")
        .desc("Number of tag accesses")
        ;

    dataAccesses
        .name(name() + ".data_accesses")
        .desc("Number of data accesses")
        ;
    
    //AMHM Start
    numberOfReadWay0
        .name(name() + ".numberOfReadWay0")
        .desc("AMHM: Number of read operations in way 0")
        ;
    
    numberOfReadWay1
        .name(name() + ".numberOfReadWay1")
        .desc("AMHM: Number of read operations in way 1")
        ;
    
    numberOfReadWay2
        .name(name() + ".numberOfReadWay2")
        .desc("AMHM: Number of read operations in way 2")
        ;
    
    numberOfReadWay3
        .name(name() + ".numberOfReadWay3")
        .desc("AMHM: Number of read operations in way 3")
        ;
    
    numberOfReadWay4
        .name(name() + ".numberOfReadWay4")
        .desc("AMHM: Number of read operations in way 4")
        ;
    
    numberOfReadWay5
        .name(name() + ".numberOfReadWay5")
        .desc("AMHM: Number of read operations in way 5")
        ;
    
    numberOfReadWay6
        .name(name() + ".numberOfReadWay6")
        .desc("AMHM: Number of read operations in way 6")
        ;
    
    numberOfReadWay7
        .name(name() + ".numberOfReadWay7")
        .desc("AMHM: Number of read operations in way 7")
        ;
    
    numberOfReadWay8
        .name(name() + ".numberOfReadWay8")
        .desc("AMHM: Number of read operations in way 8")
        ;
    
    numberOfReadWay9
        .name(name() + ".numberOfReadWay9")
        .desc("AMHM: Number of read operations in way 9")
        ;
    
    numberOfReadWay10
        .name(name() + ".numberOfReadWay10")
        .desc("AMHM: Number of read operations in way 10")
        ;
    
    numberOfReadWay11
        .name(name() + ".numberOfReadWay11")
        .desc("AMHM: Number of read operations in way 11")
        ;
    
    numberOfReadWay12
        .name(name() + ".numberOfReadWay12")
        .desc("AMHM: Number of read operations in way 12")
        ;
    
    numberOfReadWay13
        .name(name() + ".numberOfReadWay13")
        .desc("AMHM: Number of read operations in way 13")
        ;
    
    numberOfReadWay14
        .name(name() + ".numberOfReadWay14")
        .desc("AMHM: Number of read operations in way 14")
        ;
    
    numberOfReadWay15
        .name(name() + ".numberOfReadWay15")
        .desc("AMHM: Number of read operations in way 15")
        ;
    
    numberOfWriteWay0
        .name(name() + ".numberOfWriteWay0")
        .desc("AMHM: Number of write operations in way 0")
        ;
    
    numberOfWriteWay1
        .name(name() + ".numberOfWriteWay1")
        .desc("AMHM: Number of write operations in way 1")
        ;
    
    numberOfWriteWay2
        .name(name() + ".numberOfWriteWay2")
        .desc("AMHM: Number of write operations in way 2")
        ;
    
    numberOfWriteWay3
        .name(name() + ".numberOfWriteWay3")
        .desc("AMHM: Number of write operations in way 3")
        ;
    
    numberOfWriteWay4
        .name(name() + ".numberOfWriteWay4")
        .desc("AMHM: Number of write operations in way 4")
        ;
    
    numberOfWriteWay5
        .name(name() + ".numberOfWriteWay5")
        .desc("AMHM: Number of write operations in way 5")
        ;
    
    numberOfWriteWay6
        .name(name() + ".numberOfWriteWay6")
        .desc("AMHM: Number of write operations in way 6")
        ;
    
    numberOfWriteWay7
        .name(name() + ".numberOfWriteWay7")
        .desc("AMHM: Number of write operations in way 7")
        ;
    
    numberOfWriteWay8
        .name(name() + ".numberOfWriteWay8")
        .desc("AMHM: Number of write operations in way 8")
        ;
    
    numberOfWriteWay9
        .name(name() + ".numberOfWriteWay9")
        .desc("AMHM: Number of write operations in way 9")
        ;
    
    numberOfWriteWay10
        .name(name() + ".numberOfWriteWay10")
        .desc("AMHM: Number of write operations in way 10")
        ;
    
    numberOfWriteWay11
        .name(name() + ".numberOfWriteWay11")
        .desc("AMHM: Number of write operations in way 11")
        ;
    
    numberOfWriteWay12
        .name(name() + ".numberOfWriteWay12")
        .desc("AMHM: Number of write operations in way 12")
        ;
    
    numberOfWriteWay13
        .name(name() + ".numberOfWriteWay13")
        .desc("AMHM: Number of write operations in way 13")
        ;
    
    numberOfWriteWay14
        .name(name() + ".numberOfWriteWay14")
        .desc("AMHM: Number of write operations in way 14")
        ;
    
    numberOfWriteWay15
        .name(name() + ".numberOfWriteWay15")
        .desc("AMHM: Number of write operations in way 15")
        ;
    
    averageBitToggleRatio
        .init(512)
        .name(name() + ".averageBitToggleRatio")
        .desc("AMHM: Average nano-seconds between two consecutive writes (bit level)")
        .flags(nozero | nonan)
        ;
    minBitToggleRatio
        .init(512)
        .name(name() + ".minBitToggleRatio")
        .desc("AMHM: Min nano-seconds between two consecutive writes (bit level)")
        .flags(nozero | nonan)
        ;
    maxBitToggleRatio
        .init(512)
        .name(name() + ".maxBitToggleRatio")
        .desc("AMHM: Max nano-seconds between two consecutive writes (bit level)")
        .flags(nozero | nonan)
        ;
    averageBlockToggleRatio
        .name(name() + ".averageBlockToggleRatio")
        .desc("AMHM: Average nano-seconds between two consecutive writes (Block level)")
        ;
    minBlockToggleRatio
        .name(name() + ".minBlockToggleRatio")
        .desc("AMHM: Min nano-seconds between two consecutive writes (Block level)")
        ;
    maxBlockToggleRatio
        .name(name() + ".maxBlockToggleRatio")
        .desc("AMHM: Max nano-seconds between two consecutive writes (Block level)s")
        ;
    //AMHM End

    registerDumpCallback(new BaseTagsDumpCallback(this));
    registerExitCallback(new BaseTagsCallback(this));
}
