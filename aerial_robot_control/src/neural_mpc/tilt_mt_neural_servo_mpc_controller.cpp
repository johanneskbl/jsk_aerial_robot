//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/neural_mpc/tilt_mt_neural_servo_mpc_controller.h"

using namespace aerial_robot_control;

namespace data_utils
{
  std::vector<int> parseIntArray(const std::string& s) {
    // Expected input:
    // std::string "[1, 2, 3]"
    std::vector<int> result;
    std::string trimmed = s.substr(1, s.size() - 2); // strip [ and ]
    std::stringstream ss(trimmed);
    std::string token;
    while (std::getline(ss, token, ',')) {
        result.push_back(std::stoi(token));
    }
    return result;
  }
}  // namespace data_utils

/* Runtime analysis
On PC:
  Nominal:
      - controlCore: 1.2-2.0 ms
  Neural (neuralmodel_185):
      - controlCore: 10.0-10.8 ms
  Neural (neuralmodel_185 & linearized order 1):
      - controlCore: 2.0-3.0 ms
          - construction: 0.00 ms
          - convert: 0.03 ms
          - forward pass: 0.13-0.2 ms (peak 0.7 ms)
          - linearization: 0.11-0.2 ms
          - flatten: 0.03-0.06 ms
          - set params: 0.001 ms
  Auxiliary:
      - sendCmd: 0.01 ms
      - recording: 0.02 ms

On Laptop:
  Nominal:
      - controlCore: 2-3 ms
  Neural (neuralmodel_185):
      - controlCore: 15 ms
  Neural (neuralmodel_185 & linearized order 1):
      - controlCore: 10-44 ms (high variance)
          - construction:
          - convert:
          - forward pass:
          - linearization:
          - flatten:
          - set params:
  Auxiliary:
      - sendCmd: 0.00 ms
      - recording: 0.00 ms

On Jetson:
  Nominal:
      - controlCore: 7.5 ms
  Neural (neuralmodel_185):
      - controlCore:
  Neural (neuralmodel_185 & linearized order 1):
      - controlCore:
          - construction:
          - convert:
          - forward pass:
          - linearization:
          - flatten:
          - set params:
  Auxiliary:
      - sendCmd:
      - recording: 0.03 ms
*/

void nmpc::TiltMtNeuralServoMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                            boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                            double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  /* Initialize neural model and load its metadata */
  initNeuralModel();
}

/**
 * @brief Load the metadata of the neural model and, if the MLP is linearized, the neural model itself
 */
void nmpc::TiltMtNeuralServoMPC::initNeuralModel()
{
  return;
  // ROS_INFO("==========================");
  // results_dir_ = ros::package::getPath("aerial_robot_control") + "/scripts/neural_mpc/results/model_fitting/";
  // metadata_json_path_ = results_dir_ + "metadata.json";
  
  // try
  // {
  //   std::ifstream metadata_json_(metadata_json_path_);
  //   metadata_ = json::parse(metadata_json_);
  // }
  // catch (const std::exception& e)
  // {
  //   ROS_ERROR("[NEURAL][MPC] Failed to load neural model metadata from '%s'", metadata_json_path_.c_str());
  //   linearize_mlp_ = false;
  //   return;
  // }
  
  // // Extract neural model metadata
  // // NOTE: At build time, the solver is rebuilt with options from scripts/neural_mpc/config/configurations.py
  // // These options are written AT BUILD TIME to metadata.json in Python, and we read them here AT RUNTIME in C++
  // try
  // {
  //   // General
  //   neural_model_name_ = metadata_["runtime_options"]["model_name"];
  //   neural_model_instance_ = metadata_["runtime_options"]["model_instance"];
    
  //   linearize_mlp_ = metadata_["runtime_options"]["linearize_mlp"];
  //   linearize_order_ = metadata_["runtime_options"]["linearize_order"];
  //   linearize_start_idx_ = metadata_["runtime_options"]["linearize_start_idx"];
  //   linearize_end_idx_ = metadata_["runtime_options"]["linearize_end_idx"];
  //   use_gpu_ = metadata_["runtime_options"]["use_gpu"];

  //   const auto& cfg_ = metadata_[neural_model_name_][neural_model_instance_]["ModelFitConfig"];
  //   state_feats_ = data_utils::parseIntArray(cfg_["state_feats"]);
  //   u_feats_ = data_utils::parseIntArray(cfg_["u_feats"]);
  //   y_reg_dims_ = data_utils::parseIntArray(cfg_["y_reg_dims"]);
  //   input_transform_ = cfg_["input_transform"];
  //   batch_size_ = static_cast<int64_t>(NN_ + 1);
  //   N_in_ = static_cast<int64_t>(state_feats_.size() + u_feats_.size());
  //   N_out_ = static_cast<int64_t>(y_reg_dims_.size());

  //   // Linearization
  //   if (linearize_mlp_)
  //   {
  //     num_lin_params_ = linearize_end_idx_ - linearize_start_idx_;
  //     lin_param_idx_.resize(num_lin_params_);
  //     lin_params_.resize(num_lin_params_, 0.0);
  //     std::iota(lin_param_idx_.begin(), lin_param_idx_.end(), linearize_start_idx_);

  //     x0_vec_f32_.resize(static_cast<size_t>(batch_size_) * N_in_,  0.0f);
  //     x0_vec_.resize(static_cast<size_t>(batch_size_) * N_in_,  0.0);
  //     y0_vec_.resize(static_cast<size_t>(batch_size_) * N_out_, 0.0);
  //     J0_vec_.resize(static_cast<size_t>(batch_size_) * N_out_ * N_in_, 0.0);
  //     H0_vec_.resize(static_cast<size_t>(batch_size_) * N_out_ * N_in_ * N_in_, 0.0);
  //     off_x0_ = 0;
  //     off_y0_ = off_x0_ + static_cast<size_t>(N_in_);
  //     off_J0_ = off_y0_ + static_cast<size_t>(N_out_);
  //     off_H0_ = off_J0_ + static_cast<size_t>(N_out_) * N_in_;
  //     stride_x0_ = static_cast<size_t>(N_in_);
  //     stride_y0_ = static_cast<size_t>(N_out_);
  //     stride_J0_ = static_cast<size_t>(N_out_) * static_cast<size_t>(N_in_);
  //     stride_H0_ = static_cast<size_t>(N_out_) * static_cast<size_t>(N_in_) * static_cast<size_t>(N_in_);

  //     if (off_H0_ + (linearize_order_ >= 2 ? stride_H0_ : 0) != static_cast<size_t>(num_lin_params_))
  //     {
  //       ROS_ERROR("[NEURAL][MPC] Mismatch of linearization parameters size. Expected %zu but got %zu. Disabling linearization.", 
  //                 off_H0_ + (linearize_order_ >= 2 ? stride_H0_ : 0), static_cast<size_t>(num_lin_params_));
  //       linearize_mlp_ = false;
  //     }
  //   }

  //   // Delayed model
  //   delay_horizon_ = metadata_[neural_model_name_][neural_model_instance_]["NetworkConfig"]["delay_horizon"];

  //   ROS_INFO("[NEURAL][MPC] Using neural model %s/%s.", neural_model_name_.c_str(), neural_model_instance_.c_str());
  // }
  // catch (const std::exception& e)
  // {
  //   ROS_ERROR("[NEURAL][MPC] Invalid/unsupported metadata format for %s/%s: %s", neural_model_name_.c_str(),
  //             neural_model_instance_.c_str(), e.what());
  //   linearize_mlp_ = false;
  // }

  // Load neural model using LibTorch
  // if (linearize_mlp_)
  // {
  //   try
  //   {
  //     if (use_gpu_)
  //     {
  //       if (torch::cuda::is_available())
  //       {
  //         device_mlp_ = torch::Device(torch::kCUDA);
  //       }
  //       else
  //       {
  //         ROS_WARN("[NEURAL][MPC] GPU requested but CUDA is not available. Falling back to CPU.");
  //       }
  //     }

  //     torch::jit::script::Module neural_model_ = torch::jit::load(results_dir_ + neural_model_name_ + "/" + neural_model_instance_ + "_scripted.pt", device_mlp_);
  //     neural_model_.eval();
  //     neural_module_ = std::make_shared<torch::jit::script::Module>(std::move(neural_model_));
  //     ROS_INFO("[NEURAL][MPC] Successfully loaded neural model!");
  //     ROS_INFO("[NEURAL][MPC] Linearization is ENABLED with order %d", linearize_order_);
  //     if (input_transform_)
  //       ROS_INFO("[NEURAL][MPC] Input transform is ENABLED.");
  //     else
  //       ROS_INFO("[NEURAL][MPC] Input transform is DISABLED.");
  //   }
  //   catch (const std::exception& e)
  //   {
  //     ROS_ERROR("[NEURAL][MPC] Error loading the neural model: %s. Disabling linearization. Exception: %s", neural_model_instance_.c_str(), e.what());
  //     linearize_mlp_ = false;
  //   }
  // }
  // else
  // {
  //   ROS_INFO("[NEURAL][MPC] Linearization is DISABLED.");
  // }

  // // Delayed model
  // if (delay_horizon_ > 0)
  // {
  //   ROS_INFO("[NEURAL][MPC] Loaded a delayed model (delay_horizon = %d).", delay_horizon_);
  // }

  // if (linearize_mlp_)
  // {
  //   if (linearize_start_idx_ < 0 || linearize_end_idx_ < 0 || linearize_end_idx_ <= linearize_start_idx_ || linearize_end_idx_ > NP_)
  //   {
  //     ROS_ERROR("[NEURAL][MPC] Invalid linearization index range [%d,%d). Disabling linearization.", linearize_start_idx_,
  //               linearize_end_idx_);
  //     linearize_mlp_ = false;
  //   }
  //   if (linearize_order_ < 1 || linearize_order_ > 2)
  //   {
  //     ROS_ERROR("[NEURAL][MPC] Invalid linearization order %d. Only 1 (Jacobian) or 2 (Hessian) are supported. Disabling linearization.",
  //               linearize_order_);
  //     linearize_mlp_ = false;
  //   }
  // }
  // ROS_INFO("==========================");
}

/**
 * @brief Prepare the parameters of the MPC solver, including the linearization of the neural model
 */
void nmpc::TiltMtNeuralServoMPC::prepareMPCParams()
{
  TiltMtServoNMPC::prepareMPCParams();

  // Update linearization parameters
  // if (linearize_mlp_)
  // {
  //   updateLinearizationParams();
  // }
}

/**
 * @brief Linearize the neural model around the predicted trajectory and set the linearization as solver parameters
 */
void nmpc::TiltMtNeuralServoMPC::updateLinearizationParams()
{
  return;
  // try
  // {
  //   double start_time = ros::Time::now().toSec();
  //   // 1) Assemble batched neural network input x0
  //   // NOTE: batch_size_ = NN_ + 1
  //   for (int j = 0; j < batch_size_; ++j)
  //   {
  //     const auto& state_j_raw = mpc_solver_ptr_->xo_.at(j);

  //     // For terminal node, use the control value from N-1
  //     const auto& u_cmd_j = mpc_solver_ptr_->uo_.at(j < NN_ ? j : NN_ - 1);

  //     // Input transform
  //     std::vector<double> state_j = state_j_raw;
  //     if (input_transform_)
  //     {
  //       tf::Quaternion q(state_j[6], state_j[7], state_j[8], state_j[9]);  // (w,x,y,z) order
  //       tf::Vector3 v_w(state_j[3], state_j[4], state_j[5]);
  //       tf::Vector3 v_b = tf::quatRotate(q.inverse(), v_w);
  //       state_j[3] = v_b.x();
  //       state_j[4] = v_b.y();
  //       state_j[5] = v_b.z();
  //     }

  //     float* row_j = x0_vec_f32_.data() + j * N_in_;
  //     int idx = 0;
  //     for (int feat_idx : state_feats_) row_j[idx++] = static_cast<float>(state_j[feat_idx]);
  //     for (int feat_idx : u_feats_)     row_j[idx++] = static_cast<float>(u_cmd_j[feat_idx]);
  //   }

  //   double constr_time = ros::Time::now().toSec();

  //   // 2) Create torch::Tensor
  //   // NOTE: from_blob() does NOT copy; the buffer must remain valid for the lifetime of the tensor
  //   torch::Tensor x0 = torch::from_blob(
  //     x0_vec_f32_.data(),
  //     { batch_size_, N_in_ },
  //     torch::TensorOptions()
  //       .dtype(torch::kFloat32))
  //     .to(device_mlp_);  // First construct on CPU and then move to device to avoid issues with split memory-access

  //   // Repeat x0 such that each "virtual batch element" corresponds to one (b, i) pair
  //   // IDEA: A single extended forward call is cheaper than N_out_ grad() calls
  //   // Basically we repeat every row of x0 N_out_ times. From the model forward call we receive corresponding N_out_ duplicate outputs for each set of N_out_ rows
  //   torch::Tensor x0_rep = x0.repeat_interleave(N_out_, /*dim=*/0)  // (B*N_out_, N_in_)
  //                            .requires_grad_(true);

  //   // Wrap the Tensor in an IValue vector
  //   std::vector<torch::jit::IValue> inputs_rep;
  //   inputs_rep.push_back(x0_rep);

  //   double convert_time = ros::Time::now().toSec();
    
  //   // 3) Forward pass
  //   // NOTE: We need to call forward() on the entire repeated input to ensure that the gradients are correctly tracked w.r.t. the entire repeated input
  //   // NOTE: y0_rep does NOT need to be label transformed since mlp_out is transformed inside the MPC formulation
  //   torch::Tensor y0_rep = neural_module_->forward(inputs_rep).toTensor();
  //   TORCH_CHECK(y0_rep.dim() == 2 && y0_rep.size(0) == batch_size_ * N_out_ && y0_rep.size(1) == N_out_,
  //         "Expected y0_rep shape (", batch_size_ * N_out_, ", ", N_out_, "), got ", y0_rep.sizes());

  //   double forward_time = ros::Time::now().toSec();

  //   // 4) Linearize
  //   auto [J0, H0] = linearize(x0_rep, y0_rep);

  //   double linearize_time = ros::Time::now().toSec();

  //   // 5) Cast and Flatten Input, Output, Jacobian and Hessian to 1-D std::vector<double> in Fortran order to send to acados
  //   // NOTE: acados expects double
  //   x0_vec_.assign(x0_vec_f32_.begin(), x0_vec_f32_.end());  // move to std::vector<double>
  //   torch::Tensor y0 = y0_rep.view({batch_size_, N_out_, N_out_}).select(1, 0);  // extract (B, N_out_)
  //   auto y0_f64 = y0.detach().cpu().to(torch::kFloat64).contiguous();  // cast to double
  //   std::memcpy(y0_vec_.data(),  // move to std::vector<double>
  //               y0_f64.data_ptr<double>(),
  //               y0_vec_.size() * sizeof(double));
  //   flattenTensors(J0, H0);

  //   double cast_time = ros::Time::now().toSec();

  //   // 6) Set per-stage parameters from the batched linearization
  //   for (int j = 0; j <= NN_; ++j)
  //   {
  //     std::memcpy(lin_params_.data() + off_x0_,
  //                 x0_vec_.data() + j * stride_x0_,
  //                 stride_x0_ * sizeof(double));

  //     std::memcpy(lin_params_.data() + off_y0_,
  //                 y0_vec_.data() + j * stride_y0_,
  //                 stride_y0_ * sizeof(double));

  //     std::memcpy(lin_params_.data() + off_J0_,
  //                 J0_vec_.data() + j * stride_J0_,
  //                 stride_J0_ * sizeof(double));

  //     if (linearize_order_ >= 2)
  //       std::memcpy(lin_params_.data() + off_H0_,
  //                   H0_vec_.data() + j * stride_H0_,
  //                   stride_H0_ * sizeof(double));

  //     mpc_solver_ptr_->setParamSparseOneStage(j, lin_param_idx_, lin_params_);
  //   }

  //   double set_param_time = ros::Time::now().toSec();

  //   ROS_INFO_THROTTLE(1.0, "[NEURAL][MPC] Linearization times (ms): constr=%.2f, convert=%.2f, forward=%.2f, linearize=%.2f, cast=%.2f, set_param=%.2f",
  //                     (constr_time - start_time) * 1000,
  //                     (convert_time - constr_time) * 1000,
  //                     (forward_time - convert_time) * 1000,
  //                     (linearize_time - forward_time) * 1000,
  //                     (cast_time - linearize_time) * 1000,
  //                     (set_param_time - cast_time) * 1000);
  // }
  // catch (const std::exception& e)
  // {
  //   ROS_ERROR_THROTTLE(1.0, "[NEURAL][MPC] Torch error during batched linearization: %s", e.what());
  //   return;
  // }
  // return;
}

// std::pair<torch::Tensor, torch::Tensor> nmpc::TiltMtNeuralServoMPC::linearize(const torch::Tensor& x0_rep,
//                                                                                   const torch::Tensor& y0_rep)
// {
//   TORCH_CHECK(x0_rep.requires_grad(), "[TORCH] Input tensor must have requires_grad=true");
//   TORCH_CHECK(y0_rep.requires_grad(), "[TORCH] Output tensor must have requires_grad=true");

//   bool compute_graph = (linearize_order_ >= 2);

//   // ── Jacobian ──────────────────────────────────────────────
//   // Compute full batched Jacobian in a single backward pass.
//   //
//   // Build a (B*N_out_, N_out_) identity with batch_size_ one-hot
//   // vectors as columns so a single grad() returns all rows at once.
//   //
//   // With this the "repeated-input" work-around we avoid retain_graph
//   // completely and only call grad() once.
//   // ──────────────────────────────────────────────────────────

//   // grad_outputs: select output i for virtual-batch element (b*N_out_+i).
//   // Shape: (B*N_out_, N_out_) - block-diagonal identity tiled B times.
//   torch::Tensor eye_block = torch::eye(N_out_,
//       torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_))
//       .repeat({batch_size_, 1});  // (B*N_out_, N_out_)

//   auto J_flat = torch::autograd::grad(  // (B*N_out_, N_in_)
//       { y0_rep },
//       { x0_rep },
//       { eye_block },
//       /*retain_graph=*/compute_graph,  // keep only if Hessian needed
//       /*create_graph=*/compute_graph)[0];

//   torch::Tensor J0 = J_flat.view({batch_size_, N_out_, N_in_});

//   // ── Hessian ───────────────────────────────────────────────
//   // We need H[b,i,j,k] = d^2 y[b,i] / dx[b,j]dx[b,k].
//   // J_flat already contains dJ[b,i] / dx which is treated as a scalar
//   // function of x0_rep. We now differentiate each column k of
//   // J_flat w.r.t. x0_rep using the same identity trick.
//   // ──────────────────────────────────────────────────────────
//   torch::Tensor H0;
//   if (linearize_order_ >= 2)
//   {
//     // eye_k: (B*N_out_, N_in_) - for each k, selects column k of J_flat
//     // We tile torch::eye(N_in_) for all B*N_out_ virtual elements:
//     //   shape: (N_in_, B*N_out_, N_in_)  -> iterate k in outer loop

//     std::vector<torch::Tensor> H_cols;
//     H_cols.reserve(N_in_);

//     torch::Tensor eye_in = torch::eye(N_in_,
//         torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_));

//     for (int k = 0; k < N_in_; ++k)
//     {
//       // grad_outputs selects column k of J_flat for every (b,i) element.
//       // Shape: (B*N_out_, N_in_) - k-th unit vector, tiled.
//       torch::Tensor grad_outputs_k = eye_in[k]
//           .unsqueeze(0)                              // (1, N_in_)
//           .expand({batch_size_ * N_out_, N_in_});    // (B*N_out_, N_in_)

//       bool last_k = (k == N_in_ - 1);
//       auto H_k = torch::autograd::grad(  // (B*N_out_, N_in_) 
//           { J_flat },
//           { x0_rep },
//           { grad_outputs_k },
//           /*retain_graph=*/!last_k,
//           /*create_graph=*/false)[0];

//       H_cols.push_back(H_k.view({batch_size_, N_out_, N_in_}));
//     }
//     H0 = torch::stack(H_cols, /*dim=*/3);
//   }
//   else
//   {
//     H0 = torch::zeros({batch_size_, N_out_, N_in_, N_in_},
//          torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_));
//   }

//   return {J0, H0};
// }

// // =========== NAIVE APPROACH (loop over N_out_ dim) =============
// //   std::vector<torch::Tensor> J_cols;
// //   J_cols.reserve(N_out_);
  
// //   for (int i = 0; i < N_out_; ++i)
// //   {
// //     torch::Tensor jac_grad_outputs = torch::zeros_like(y0); // (B, N_out_)
// //     jac_grad_outputs.select(1, i).fill_(1.0f);

// //     bool last_i = (i == N_out_ - 1);
// //     auto grads = torch::autograd::grad(
// //         { y0 },
// //         { x0 },
// //         { jac_grad_outputs },
// //         /*retain_graph=*/(!last_i || compute_graph),
// //         /*create_graph=*/compute_graph);

// //     J_cols.push_back(grads[0]);  // (B, N_in_)
// //   }
// //   // J0 shape: (B, N_out_, N_in_)
// //   torch::Tensor J0 = torch::stack(J_cols, /*dim=*/1);

// //   torch::Tensor H0;
// //   if (linearize_order_ >= 2)
// //   {
// //     std::vector<torch::Tensor> H_out_cols;
// //     H_out_cols.reserve(N_out_);

// //     torch::Tensor hess_grad_outputs = torch::ones({batch_size_}, 
// //         torch::TensorOptions().dtype(torch::kFloat32).device(device_mlp_));

// //     for (int i = 0; i < N_out_; ++i)
// //     {
// //       std::vector<torch::Tensor> H_in_cols;
// //       H_in_cols.reserve(N_in_);
      
// //       for (int k = 0; k < N_in_; ++k)
// //       {
// //         bool last_grad = (i == N_out_ - 1) && (k == N_in_ - 1);
        
// //         // Differentiate J0[:, i, k] wrt x0
// //         // J0.select(1, i) gives (B, N_in_), then .select(1, k) gives (B)
// //         auto H_k = torch::autograd::grad(  // (B, N_in_)
// //             { J0.select(1, i).select(1, k) }, 
// //             { x0 },
// //             { hess_grad_outputs },
// //             /*retain_graph=*/!last_grad,
// //             /*create_graph=*/false)[0];
// //         H_in_cols.push_back(H_k);
// //       }
// //       // Stack along dim=1 to get (B, N_in_, N_in_)
// //       H_out_cols.push_back(torch::stack(H_in_cols, 1));
// //     }
// //     // Stack along dim=1 to get (B, N_out_, N_in_, N_in_)
// //     H0 = torch::stack(H_out_cols, 1);
// //   }

// /**
//  * Flatten J0 and H0 to 1-D std::vector<float> using CasADi / Fortran
//  * (column-major) order.
//  *
//  * J0 shape: (B, N_out, N_in)
//  *   CasADi sees a (N_out x N_in) matrix -> column-major -> i (output) varies
//  *   fastest.
//  *   Flat index: i + N_out_ * j
//  *   Equivalent torch op: J0[b].t().contiguous().flatten()
//  *
//  * H0 shape: (B, N_out, N_in, N_in)
//  *   CasADi sees a (N_out*N_in x N_in) matrix -> column-major.
//  *   The "row" axis is the composite (i, j) pair, with i varying fastest.
//  *   Flat index: i + N_out_*j + N_out_*N_in_*k
//  *   Equivalent torch op: H0[b].permute({2,1,0}).contiguous().flatten()
//  *     (permute k->dim0, j->dim1, i->dim2, then flatten with i varying fastest)
//  */
// void nmpc::TiltMtNeuralServoMPC::flattenTensors(const torch::Tensor& J0,
//                                                     const torch::Tensor& H0)
// {
//     // ---- Jacobian: (B, N_out, N_in) -> B × (N_out*N_in,) Fortran-flat ----
//     // Transpose last two dims so memory order is [j, i] -> flatten gives
//     // column-major [i0, i1, …, iN | next col …]
//     torch::Tensor J0_flat = J0
//         .transpose(1, 2)           // (B, N_in, N_out) - i varies fastest
//         .contiguous()              // force contiguous before data_ptr
//         .reshape({batch_size_, N_out_ * N_in_});  // (B, N_out*N_in)

//     // ---- Hessian: (B, N_out, N_in, N_in) -> B × (N_out*N_in*N_in,) Fortran-flat ----
//     // CasADi matrix shape: rows = N_out*N_in  (i fast, j slow)
//     //                      cols = N_in        (k)
//     // Column-major: i varies fastest, then j, then k.
//     // Achieve with permute [b, i, j, k] -> [b, i, k, j] then flatten.
//     // keep the N_out axis (i) in place as dim-1 so each output block stays contiguous,
//     // and only swap the two N_in axes so j varies fastest within each block.
//     torch::Tensor H0_flat = H0
//         .permute({0, 1, 3, 2})    // (B, N_out_i, N_in_k, N_in_j) 
//         .contiguous()
//         .reshape({batch_size_, N_out_ * N_in_ * N_in_});  // (B, N_out*N_in*N_in)

//     const int64_t j_numel = J0_flat.numel();
//     const int64_t h_numel = H0_flat.numel();

//     if (j_numel != J0_vec_.size() || h_numel != H0_vec_.size())
//     {
//         ROS_ERROR("[NEURAL][MPC] Size mismatch when flattening Jacobian/Hessian: J0 numel = %ld, "
//                   "H0 numel = %ld, but J0_vec_ size = %zu, H0_vec_ size = %zu",
//                   j_numel, h_numel, J0_vec_.size(), H0_vec_.size());
//         return;
//     }

//     std::memcpy(J0_vec_.data(), J0_flat.cpu().to(torch::kFloat64).data_ptr<double>(), j_numel * sizeof(double));
//     std::memcpy(H0_vec_.data(), H0_flat.cpu().to(torch::kFloat64).data_ptr<double>(), h_numel * sizeof(double));
//     return;
// }


// ===================================================================================
// TODO: Potentially adapt allocation with new model?
// ===================================================================================

/* Plugin registration */
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtNeuralServoMPC, aerial_robot_control::ControlBase)
