//
// Created by jinjie on 24/10/21.
//

#ifndef WRENCH_EST_I_TERM_H
#define WRENCH_EST_I_TERM_H

#include "wrench_est_base.h"
#include "i_term.h"

/* dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include "aerial_robot_control/ITermConfig.h"
#include "aerial_robot_msgs/DynamicReconfigureLevels.h"

using ITermDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::ITermConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class WrenchEstITerm : public WrenchEstBase
{
public:
  void initParams(ros::NodeHandle& nh_ctrl, double ctrl_loop_du) override;

  void update(const tf::Vector3& pos, const tf::Vector3& pos_ref, const tf::Quaternion& q,
              const tf::Quaternion& q_ref) override;

protected:
  void cfgCallback(ITermConfig& config, uint32_t level);

private:
  std::array<ITerm, 6> pos_i_term_;  // for x, y, z, roll, pitch, yaw

  std::vector<boost::shared_ptr<ITermDynamicConfig>> reconf_servers_;
};

}  // namespace nmpc
}  // namespace aerial_robot_control

#endif  // WRENCH_EST_I_TERM_H
