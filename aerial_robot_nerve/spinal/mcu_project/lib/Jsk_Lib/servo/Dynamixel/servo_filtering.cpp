/*
 * servo_filtering.cpp
 *
 * Created on: 2026-11-22
 * Low-pass filtered servo angle and its derivative as the velocity for servo/states (and so joint_states), for each
 * sent servo, updated once per complete read in DynamixelSerial::update() (dt = GET_POS_DU = 5 ms):
 * - Angle: implementation of the alpha-beta tracking filter of the ESC RPM (AM32 fork, rpm_filter.c) on the present
 *   position.
 * - Velocity: Savitzky-Golay derivative of the present position over 60 ms, as for the eRPM rate of the AM32 fork
 *   (rpm_rate.c). It replaces the present velocity that the servo measures itself.
 * The results replace the measured values in ServoData.
 *
 * Angle: model and filter
 *   Constant-rate model with the measurement z = angle, the position state x = filtered value
 *   and the velocity state v = change per read:
 *     predict:     x- = x + v
 *     innovation:  r  = z - x-
 *     correct:     x  = x- + alpha*r
 *                  v  = v + beta*r
 *   The velocity state v is only internal: it lets x follow a constant rate of change without lag, which makes this
 *   more accurate than a moving average filter.
 *
 * Angle: gain design
 *   Steady-state Kalman gains (Kalata relation) for white acceleration and white measurement noise:
 *     alpha = 1 - p^2,  beta = 2*(1 - p)^2,  p = exp(-2*pi*f*dt)
 *   The frequency f is 15 Hz as for the RPM. p keeps the poles at the same continuous-time decay for any dt
 *   (damping ~0.7), so the angle gets the same delay as the RPM measurement (on the AM32 fork), and the smoothing
 *   factors follow from the longer dt: alpha = 0.610 and beta = 0.282 at 5 ms (RPM: 0.180 and 0.018 at 1.05 ms).
 *   Group delay at 5 / 10 Hz: 1.0 / 3.4 ms (RPM: 1.5 / 4.6 ms), a step settles within 2 % after 40 ms (RPM: 38 ms).
 *   Through this filter the RMS of the second difference of the angle drops by 43-53 %.
 *
 * Velocity: Savitzky-Golay derivative, quadratic fit over the last N = 2m + 1 = 13 angles (60 ms), differentiated at
 * the centre
 *   1. Fit samples: The angles get the index j in [-m,...,m] counted from the centre sample: j = 0 is m reads old,
 *      j = m is the newest. A quadratic is fitted to them by least squares:
 *        p(j) = c0 + c1*j + c2*j^2,   minimising  sum_j (z_j - p(j))^2
 *      The window is symmetric about j = 0, so the normal equations decouple. The odd coefficient depends only on
 *      the odd moment of the data:
 *        c1 = sum_j j*z_j / sum_j j^2
 *   2. Differentiate at the centre. dp/dj = c1 + 2*c2*j, so the slope at j = 0 is c1 [ticks per read]. c0 and c2 do
 *      not enter it, which is why they are not computed. Per second and in the unit of present_velocity
 *      (0.229 rev/min = 15.63 ticks/s at 4096 ticks per revolution):
 *        velocity = c1 / (dt * 15.63)
 *   Together this is a fixed weighted sum of the angles, weights j / (dt * sum j^2): a linear-phase FIR filter.
 *   Its delay is exactly m reads (6 * 5 = 30 ms) at all frequencies.
 *   The servo derives its present velocity from the same encoder. On the 2026-11-21 flight it behaves like a ~40 ms
 *   moving average of the derivative: delay 21.5 ms, gain 0.75 / 0.53 / 0.33 at 10 / 15 / 20 Hz. The largest fast
 *   content, at 10-20 Hz, is real servo vibration: it matches the angle derivative 95 % (coherence), and real sensor
 *   noise only starts above 20 Hz. The 60 ms fit attenuates it more (gain 0.64 / 0.33 / 0.07), so the RMS error
 *   against a zero-delay velocity (angle derivative, zero-phase 10 Hz low-pass) drops from 0.074 to 0.062 rad/s
 *   (-16 %, rounded to the unit of present_velocity), at 30 instead of 21.5 ms of delay.
 */

#include "servo_filtering.h"

// Filter frequency f in Hz, the same as for the RPM of the ESCs. Lower f: less noise, more lag.
#define SERVO_FILTER_FREQUENCY 15.0

// Savitzky-Golay window of N = 2m + 1 angles, j in [-m,...,m], spanning 2m * dt. A longer window smooths more but
// delays the velocity by m * dt.
#define SERVO_SG_WINDOW_MS 60
#define SERVO_SG_HALF_WINDOW (SERVO_SG_WINDOW_MS / 2 / GET_POS_DU) // m = 6 at dt = 5 ms
#define SERVO_SG_WINDOW (2 * SERVO_SG_HALF_WINDOW + 1)
// Scaling factor: sum_j j^2 over j in [-m,...,m] = m(m + 1)(2m + 1) / 3, 182 for m = 6
#define SERVO_SG_SUM_J2 (SERVO_SG_HALF_WINDOW * (SERVO_SG_HALF_WINDOW + 1) * (2 * SERVO_SG_HALF_WINDOW + 1) / 3)
// Unit of present_velocity, 0.229 rev/min, in ticks per second (4096 ticks per revolution)
#define SERVO_VELOCITY_UNIT (0.229 * 4096.0 / 60.0)

namespace ServoFiltering {
  namespace {
    // Gains for f at dt = GET_POS_DU, see the top of this file
    const float p_ = std::exp(-2.0 * M_PI * SERVO_FILTER_FREQUENCY * GET_POS_DU / 1000.0);
    const float alpha_ = 1.0f - p_ * p_;
    const float beta_ = 2.0f * (1.0f - p_) * (1.0f - p_);
    // Velocity per unit of sum_j j*z_j: c1 = sum / sum_j j^2, divided by dt in s and by the unit of present_velocity
    const float sg_scale_ = 1000.0 / (SERVO_SG_SUM_J2 * GET_POS_DU * SERVO_VELOCITY_UNIT);

    // Alpha-beta filter of one signal
    class AlphaBetaFilter
    {
    public:
      // Start on the measurement at rest (x = z, v = 0), so a later update only sees the motion of one read
      void reset(int32_t z)
      {
        x_ = z;
        v_ = 0;
      }

      // One filter step per read, see the equations at the top of this file. Returns the filtered value, rounded.
      int32_t update(int32_t z)
      {
        // Predict x- = x + v
        float predicted = x_ + v_;
        // Residual r = z - x-
        float residual = z - predicted;
        // Correct x = x- + alpha*r and v = v + beta*r
        x_ = predicted + alpha_ * residual;
        v_ += beta_ * residual;
        return std::lround(x_);
      }

    private:
      float x_; // x, filtered value
      float v_; // v, change per read
    };

    // Savitzky-Golay derivative of one signal
    class SavitzkyGolayDerivative
    {
    public:
      // Start with every sample of the window on the measurement, i.e. at a velocity of zero
      void reset(int32_t z)
      {
        for (uint8_t i = 0; i < SERVO_SG_WINDOW; i++) {
          window_[i] = z;
        }
        oldest_ = 0;
      }

      // One estimation step per read: enter the sample, fit the window and differentiate at its centre, see the top of
      // this file. Returns the velocity in the unit of present_velocity, rounded.
      int32_t update(int32_t z)
      {
        // Enter the sample in place of the oldest one, which moves the window on by one sample
        window_[oldest_] = z;
        oldest_ = (oldest_ + 1 == SERVO_SG_WINDOW) ? 0 : oldest_ + 1;

        // Fit p(j) = c0 + c1*j + c2*j^2 to the window, of which only c1 = sum_j j*z_j / sum_j j^2 is needed.
        // Since sum_j j = 0, summing j * (z_j - z_0) gives the same result, so the terms stay small.
        uint8_t centre = oldest_ + SERVO_SG_HALF_WINDOW;
        if (centre >= SERVO_SG_WINDOW) {
          centre -= SERVO_SG_WINDOW;
        }
        const int32_t z_centre = window_[centre];
        int32_t sum_jz = 0;
        uint8_t i = oldest_; // j = -m
        for (int32_t j = -SERVO_SG_HALF_WINDOW; j <= SERVO_SG_HALF_WINDOW; j++) {
          sum_jz += j * (window_[i] - z_centre);
          i = (i + 1 == SERVO_SG_WINDOW) ? 0 : i + 1;
        }
        // Differentiate p at the centre, p'(0) = c1 per read, and convert: velocity = c1 / (dt * unit), rounded
        return std::lround(sum_jz * sg_scale_);
      }

    private:
      int32_t window_[SERVO_SG_WINDOW]; // Ring buffer of the last N angles, window_[oldest_] is j = -m
      uint8_t oldest_;
    };

    AlphaBetaFilter angle_[MAX_SERVO_NUM];
    SavitzkyGolayDerivative velocity_[MAX_SERVO_NUM];
    bool started_[MAX_SERVO_NUM]; // The filters of the servo run, else the next read starts them
  }

  void update(std::array<ServoData, MAX_SERVO_NUM>& servo, unsigned int servo_num)
  {
    for (unsigned int i = 0; i < servo_num; i++) {
      ServoData& s = servo[i];
      if (!s.send_data_flag_) {
        started_[i] = false; // Not sent: start on the measurement again once it is
      } else if (!started_[i]) {
        angle_[i].reset(s.present_position_);
        velocity_[i].reset(s.present_position_);
        s.present_velocity = 0; // The window starts at rest
        started_[i] = true;
      } else {
        s.present_velocity = velocity_[i].update(s.present_position_); // From the measured angle, before it is filtered
        s.present_position_ = angle_[i].update(s.present_position_);
      }
    }
  }
}
