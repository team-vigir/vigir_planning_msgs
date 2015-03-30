//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_PLANNING_MSGS_CONSTRAINTS_CONVERSION_H
#define VIGIR_PLANNING_MSGS_CONSTRAINTS_CONVERSION_H

#include <vigir_planning_msgs/JointPositionConstraint.h>
#include <moveit_msgs/JointConstraint.h>

namespace vigir_planning_msgs
{


static inline bool toMoveitConstraint (const vigir_planning_msgs::JointPositionConstraint &input,
                                       moveit_msgs::JointConstraint &output)
{
  if (input.joint_max < input.joint_min)
  {
    return false;
  }else if (input.joint_max == input.joint_min){
    output.position = input.joint_max;
    output.tolerance_above = 0.0;
    output.tolerance_below = 0.0;
  }else{
    // position is mean between max and min limits for now
    output.position = (input.joint_max - input.joint_min) * 0.5;
    output.tolerance_above = input.joint_max - output.position;
    output.tolerance_below = output.tolerance_above;
  }

  return true;
}

static inline bool toVigirConstraint (const moveit_msgs::JointConstraint &input,
                                      vigir_planning_msgs::JointPositionConstraint &output)
{
  if ((input.tolerance_above < 0.0) || (input.tolerance_below < 0.0))
    return false;

  output.joint_max = input.position + input.tolerance_above;
  output.joint_min = input.position - input.tolerance_below;

  return true;
}

}
#endif
