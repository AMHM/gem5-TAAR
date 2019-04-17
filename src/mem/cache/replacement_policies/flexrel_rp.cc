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
 *
 * Authors: Amir Mahdi Hosseini Monazzah
 */

#include "mem/cache/replacement_policies/flexrel_rp.hh"

#include <cassert>
#include <memory>

#include "params/FLEXRELRP.hh"

FLEXRELRP::FLEXRELRP(const Params *p)
    : BaseReplacementPolicy(p)
{
}

void
FLEXRELRP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    // Reset last touch timestamp
    std::static_pointer_cast<FLEXRELReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
FLEXRELRP::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<FLEXRELReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
FLEXRELRP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<FLEXRELReplData>(
        replacement_data)->lastTouchTick = curTick();
}

ReplaceableEntry*
FLEXRELRP::getVictim(const ReplacementCandidates& candidates, int HW) const
{
    //AMHM Start
    
    /*The layout of considered 16-way set associative cache is as follow
     * |HW less than 100: Way 0, way 1, way 2 and way 3                                         |
     * |HW between 101 and 250: way 4, way 5, way 6, way 7, way 8, way 9, way 10, and way 11    |
     * |HW between 251 and 400: way 12 and way 13                                               |
     * |HW over 401: way 14 and way 15                                                          |    
    */
    assert(candidates.size() == 16); //FlexRel Only works in 16-way configuration!
    
    ReplaceableEntry* victim = candidates[15];
    
    if (HW <= 100){
        victim = candidates[0];
        for (int i = 0; i < 4; i++) {
            // Update victim entry if necessary
            if (std::static_pointer_cast<FLEXRELReplData>(
                        candidates[i]->replacementData)->lastTouchTick <
                    std::static_pointer_cast<FLEXRELReplData>(
                        victim->replacementData)->lastTouchTick) {
                victim = candidates[i];
            }
        }
    } else if ((HW >= 101) && (HW <=250)){
        victim = candidates[4];
        for (int i = 4; i < 12; i++) {
            // Update victim entry if necessary
            if (std::static_pointer_cast<FLEXRELReplData>(
                        candidates[i]->replacementData)->lastTouchTick <
                    std::static_pointer_cast<FLEXRELReplData>(
                        victim->replacementData)->lastTouchTick) {
                victim = candidates[i];
            }
        }
    } else if ((HW >= 251) && (HW <=400)){
        victim = candidates[12];
        for (int i = 12; i < 14; i++) {
            // Update victim entry if necessary
            if (std::static_pointer_cast<FLEXRELReplData>(
                        candidates[i]->replacementData)->lastTouchTick <
                    std::static_pointer_cast<FLEXRELReplData>(
                        victim->replacementData)->lastTouchTick) {
                victim = candidates[i];
            }
        }
    } else {
        victim = candidates[14];
        for (int i = 14; i < 16; i++) {
            // Update victim entry if necessary
            if (std::static_pointer_cast<FLEXRELReplData>(
                        candidates[i]->replacementData)->lastTouchTick <
                    std::static_pointer_cast<FLEXRELReplData>(
                        victim->replacementData)->lastTouchTick) {
                victim = candidates[i];
            }
        }
    }
    return victim;
    //AMHM End
}

std::shared_ptr<ReplacementData>
FLEXRELRP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new FLEXRELReplData());
}

FLEXRELRP*
FLEXRELRPParams::create()
{
    return new FLEXRELRP(this);
}
