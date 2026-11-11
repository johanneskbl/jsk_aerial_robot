//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_MT_NEURAL_SERVO_MPC_CONTROLLER_H
#define TILT_MT_NEURAL_SERVO_MPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"

/* LibTorch */
// #include <torch/torch.h>
// #include <torch/script.h>

/* MLP metadata loading */
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace aerial_robot_control
{

namespace nmpc
{

/**
 * @brief MPC controller using a neural network in the dynamics model
 * The state, input and cost are the same as in TiltMtServoNMPC.
 * Additionally, the neural model is loaded and, if the MLP is linearized, the linearization is updated as solver
 * parameters in every control loop.
 */
class TiltMtNeuralServoMPC : public TiltMtServoNMPC
{
public:
  TiltMtNeuralServoMPC() = default;
  ~TiltMtNeuralServoMPC() override = default;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  // For neural model linearization
  std::string neural_model_name_;
  std::string neural_model_instance_;
  std::string metadata_json_path_;
  std::string results_dir_;
  json metadata_;

  // std::shared_ptr<torch::jit::script::Module> neural_module_;
  // torch::Device device_mlp_{torch::Device(torch::kCPU)};

  std::vector<int> state_feats_;
  std::vector<int> u_feats_;
  std::vector<int> y_reg_dims_;
  bool input_transform_{ false };
  int64_t batch_size_{ -1 };
  int64_t N_in_{ -1 };
  int64_t N_out_{ -1 };

  bool linearize_mlp_{ false };
  int linearize_order_{ -1 };
  int linearize_start_idx_{ -1 };
  int linearize_end_idx_{ -1 };
  std::vector<int> lin_param_idx_;
  std::vector<double> lin_params_;
  int num_lin_params_{ -1 };
  bool use_gpu_{ false };
  size_t off_x0_, off_y0_, off_J0_, off_H0_;
  size_t stride_x0_, stride_y0_, stride_J0_, stride_H0_;
  std::vector<float> x0_vec_f32_;
  std::vector<double> x0_vec_, y0_vec_, J0_vec_, H0_vec_;

  int delay_horizon_{ -1 };

  /* Initialization methods */
  void initNeuralModel();

  /* Control methods */
  void prepareMPCParams() override;
  void updateLinearizationParams();
  // std::pair<torch::Tensor, torch::Tensor> linearize(const torch::Tensor& x0,
  //                                                   const torch::Tensor& y0);
  // void flattenTensors(const torch::Tensor& J0,
  //                     const torch::Tensor& H0);
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_NEURAL_SERVO_MPC_CONTROLLER_H
