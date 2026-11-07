import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

RPM_TO_RAD_S = 2 * np.pi / 60  # [rad/s] per [RPM]
KRPM_TO_RAD_S = 1000 * RPM_TO_RAD_S  # [rad/s] per [kRPM]


def rms(residual):
    return np.sqrt(np.mean(residual ** 2))


def analyze_data(folder_path, file_name, has_telemetry, set_voltage, order, if_reverse):
    file_path = folder_path + file_name
    column_names = []
    if not has_telemetry:
        column_names = ["PWM", "fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "State"]
    else:
        column_names = ["PWM", "fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature", "voltage", "State"]

    data = pd.read_csv(file_path, sep=' ', names=column_names)

    # Filter out the rows where State is 'done'
    data = data[data['PWM'] != 'done']

    # convert PWM from string to int
    data['PWM'] = data['PWM'].astype(int)

    # Filter the DataFrame for 'valid' state
    valid_data = data[data['State'] == 'valid']

    if not has_telemetry:
        columns_to_average = ["fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency"]
    else:
        columns_to_average = ["fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature", "voltage"]

    if if_reverse:
        valid_data.loc[:, ['fz', 'mz']] *= -1

    # Group by 'PWM' and calculate the average for each specified column
    average_values = valid_data.groupby('PWM')[columns_to_average].mean()

    # Resetting index to make 'PWM' a column again
    average_values.reset_index(inplace=True)

    # Add a column for power
    if has_telemetry:
        average_values['Power'] = average_values['voltage'] * average_values['currency']
        print("Average voltage: {}".format(average_values['voltage'].mean()))
    else:
        average_values['Power'] = set_voltage * average_values['currency']

    # Add a column for PWM ratio
    average_values['PWM_Ratio_%'] = average_values['PWM'] / 2000 * 100

    if has_telemetry:
        # Add a column for kRPM^2 (kilorevolutions per minute squared)
        average_values['kRPM^2'] = (average_values['RPM'] / 1000) ** 2
        # Add a column for Dshot ratio in percentage
        average_values['Dshot_Ratio_%'] = ((average_values['PWM'] - 1000) / 1000) * 100

    ''' plot the data '''

    # Convert series to numpy arrays

    if has_telemetry:
        kRPM2 = average_values['kRPM^2'].to_numpy()
        kRPM = average_values['RPM'].to_numpy() / 1000

    fz = average_values['fz'].to_numpy()
    mz = average_values['mz'].to_numpy()
    currency = average_values['currency'].to_numpy()
    PWM_ratio = average_values['PWM_Ratio_%'].to_numpy()

    # Perform the fittings
    if has_telemetry:
        # Linear fit for x:kRPM^2 y:fz
        slope_kRPM2, intercept_kRPM2 = np.polyfit(kRPM2, fz, 1)
        fit_eq_kRPM2 = "fz = {:.4f} * kRPM^2 + {:.4f}".format(slope_kRPM2, intercept_kRPM2)
        print("Fitting Equation (x:kRPM^2 y:fz): {}".format(fit_eq_kRPM2))
        print("RMS residual: {:.4f}  # N".format(rms(fz - (slope_kRPM2 * kRPM2 + intercept_kRPM2))))

        # Linear fit for x: pwm_ratio y: kRPM
        slope_kRPM_PWM_ratio, intercept_kRPM_PWM_ratio = np.polyfit(PWM_ratio, kRPM, 1)
        fit_eq_kRPM_PWM_ratio = "kRPM = {:.4f} * PWM_Ratio_% + {:.4f}".format(slope_kRPM_PWM_ratio, intercept_kRPM_PWM_ratio)
        print("Fitting Equation (x:PWM_Ratio_% y:kRPM): {}".format(fit_eq_kRPM_PWM_ratio))
        print("RMS residual: {:.4f}  # kRPM".format(rms(kRPM - (slope_kRPM_PWM_ratio * PWM_ratio + intercept_kRPM_PWM_ratio))))

    # Linear fit for x:fz y:mz, note that the intercept must be 0. Only y=kx form.
    slope_mz_fz, intercept_mz_fz = np.polyfit(fz, mz, 1)
    fit_eq_mz_fz = "mz = {:.4f} * fz".format(slope_mz_fz)
    print("Fitting Equation (x:fz y:mz): {}".format(fit_eq_mz_fz))
    # Residual of the printed and plotted model mz = slope * fz, i.e. without the intercept of the fit
    print("RMS residual: {:.6f}  # N*m".format(rms(mz - slope_mz_fz * fz)))

    # 2nd-order polynomial fit for x:fz y:currency
    coeffs_currency_fz = np.polyfit(fz, currency, 2)
    fit_eq_currency_fz = "currency = {:.4f} * fz^2 + {:.4f} * fz + {:.4f}".format(coeffs_currency_fz[0], coeffs_currency_fz[1], coeffs_currency_fz[2])
    print("Fitting Equation (x:fz y:currency): {}".format(fit_eq_currency_fz))
    print("RMS residual: {:.4f}  # A".format(rms(currency - np.polyval(coeffs_currency_fz, fz))))

    # n-th order polynomial fit for x:PWM_ratio y:fz
    coeffs_PWM_ratio_fz = np.polyfit(PWM_ratio, fz, order)
    fit_eq_PWM_ratio_fz = "fz = "
    for i in range(order + 1):
        fit_eq_PWM_ratio_fz += "\n{:.4e} * PWM_Ratio_%^{} +".format(coeffs_PWM_ratio_fz[i], order - i)
    print()
    print("Fitting Equation (x:PWM_Ratio_% y:fz): {}".format(fit_eq_PWM_ratio_fz))
    for i in range(order + 1):
        print("polynomial{}: {:.8f}  # x10^{}".format(order - i, pow(10, order - i) * coeffs_PWM_ratio_fz[i], order -i))
    print("RMS residual: {:.4f}  # N".format(rms(fz - np.polyval(coeffs_PWM_ratio_fz, PWM_ratio))))
    print()

    # n-th order polynomial fit for x:fz y:PWM_ratio
    coeffs_fz_PWM_ratio = np.polyfit(fz, PWM_ratio, order)
    fit_eq_fz_PWM_ratio = "PWM_ratio_% = "
    for i in range(order + 1):
        fit_eq_fz_PWM_ratio += "\n{:.4e} * fz^{} + ".format(coeffs_fz_PWM_ratio[i], order - i)
    print("Fitting Equation (x:fz y:PWM_Ratio_%): {}".format(fit_eq_fz_PWM_ratio))
    print("voltage: {}".format(set_voltage))
    print("max_thrust: {:.4f} # N".format(np.max(fz)))
    for i in range(order + 1):
        print("polynomial{}: {:.8f}  # x10^{}".format(order - i, pow(10, order - i) * coeffs_fz_PWM_ratio[i], order - i))
    print("RMS residual: {:.4f}  # %".format(rms(PWM_ratio - np.polyval(coeffs_fz_PWM_ratio, fz))))
    print()

    # Create 3x3 subplots
    fig, axs = plt.subplots(3, 3, figsize=(12, 18))

    # Plot 1: x:kRPM^2 y:fz
    if has_telemetry:
        print("has telemetry", has_telemetry)
        axs[0, 0].scatter(kRPM2, fz, color='blue', alpha=0.5, label='Data points')
        axs[0, 0].plot(kRPM2, slope_kRPM2 * kRPM2 + intercept_kRPM2, color='red', label='Fitted line')
        axs[0, 0].set_title('fz vs kRPM^2')
        axs[0, 0].set_xlabel('kRPM^2')
        axs[0, 0].set_ylabel('fz (N)')
        axs[0, 0].grid()
        axs[0, 0].text(0.95, 0.05, fit_eq_kRPM2, horizontalalignment='right', verticalalignment='bottom',
                       transform=axs[0, 0].transAxes, fontsize=10, color='green')

    # Plot 2: x:fz y:mz
    axs[0, 1].scatter(fz, mz, color='blue', alpha=0.5, label='Data points')
    axs[0, 1].plot(fz, slope_mz_fz * fz, color='red', label='Fitted line')
    axs[0, 1].set_title('mz vs fz')
    axs[0, 1].set_xlabel('fz (N)')
    axs[0, 1].set_ylabel('mz (N*m)')
    axs[0, 1].grid()
    axs[0, 1].text(0.95, 0.05, fit_eq_mz_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[0, 1].transAxes, fontsize=10, color='green')

    # Plot 3: x:fz y:currency
    axs[1, 0].scatter(fz, currency, color='blue', alpha=0.5, label='Data points')
    axs[1, 0].plot(np.sort(fz), np.polyval(coeffs_currency_fz, np.sort(fz)), color='red', label='Fitted polynomial')
    axs[1, 0].set_title('currency vs fz')
    axs[1, 0].set_xlabel('fz (N)')
    axs[1, 0].set_ylabel('currency')
    axs[1, 0].grid()
    axs[1, 0].text(0.95, 0.05, fit_eq_currency_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[1, 0].transAxes, fontsize=10, color='green')

    # Plot 4: fz vs PWM_ratio
    axs[1, 1].scatter(PWM_ratio, fz, color='blue', alpha=0.5, label='Data points')
    axs[1, 1].plot(np.sort(PWM_ratio), np.polyval(coeffs_PWM_ratio_fz, np.sort(PWM_ratio)), color='red',
                   label='Fitted polynomial')
    axs[1, 1].set_title('fz vs PWM_Ratio_%')
    axs[1, 1].set_xlabel('PWM_Ratio_%')
    axs[1, 1].set_ylabel('fz (N)')
    axs[1, 1].grid()
    axs[1, 1].text(0.95, 0.05, fit_eq_PWM_ratio_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[1, 1].transAxes, fontsize=10, color='green')

    # Plot 5: fz vs PWM_ratio^2
    axs[2, 0].scatter(PWM_ratio ** 2, fz, color='blue', alpha=0.5, label='Data points')
    axs[2, 0].set_title('fz vs PWM_Ratio_%^2')
    axs[2, 0].set_xlabel('PWM_Ratio_%^2')
    axs[2, 0].set_ylabel('fz (N)')
    axs[2, 0].grid()

    # Plot 6: kRPM vs PWM_ratio
    if has_telemetry:
        axs[2, 1].scatter(PWM_ratio, kRPM, color='blue', alpha=0.5, label='Data points')
        axs[2, 1].plot(PWM_ratio, slope_kRPM_PWM_ratio * PWM_ratio + intercept_kRPM_PWM_ratio, color='red',
                       label='Fitted line')
        axs[2, 1].set_title('kRPM vs PWM_Ratio_%')
        axs[2, 1].set_xlabel('PWM_Ratio_%')
        axs[2, 1].set_ylabel('kRPM')
        axs[2, 1].grid()
        axs[2, 1].text(0.95, 0.05, fit_eq_kRPM_PWM_ratio, horizontalalignment='right', verticalalignment='bottom',
                       transform=axs[2, 1].transAxes, fontsize=10, color='green')

    # Plot 7: PWM_ratio vs fz
    axs[0, 2].scatter(fz, PWM_ratio, color='blue', alpha=0.5, label='Data points')
    axs[0, 2].plot(np.sort(fz), np.polyval(coeffs_fz_PWM_ratio, np.sort(fz)), color='red',
                   label='Fitted polynomial')
    axs[0, 2].set_title('PWM_Ratio_% vs fz')
    axs[0, 2].set_xlabel('fz (N)')
    axs[0, 2].set_ylabel('PWM_Ratio_%')
    axs[0, 2].grid()
    axs[0, 2].text(0.95, 0.05, fit_eq_fz_PWM_ratio, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[0, 2].transAxes, fontsize=10, color='green')

    # title with the file name
    fig.suptitle(file_name)

    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()


def analyze_data_rpm_mode(folder_path, file_name, if_reverse, zero_samples):
    '''
    Identify the rotor as f_t = k_t * Omega^2 and tau_t = k_tau * Omega^2, with Omega [rad/s] the rotor speed that
    the ESC of the tested motor measures (motor_index of motor_test, motor 0 if all motors are tested), from a one-shot
    test with dshot telemetry in rpm mode (test.launch rpm_mode:=true has_dshot_telemetry:=true).
    Rows of the ramps ('ramp', with pwm_ramp_rate > 0) and of the settling ('raise') are not used.

    Each step (one PWM value, i.e. one target RPM) is reduced to the mean of its 'valid' rows minus the zero offset of
    the force sensor at that time. The calibration right before the step zeroes the sensor (offset 0 at the first row
    of the step), and the last zero_samples 'brake' rows of the step in which the ESC reports RPM 0 (the rotor turns too
    slowly for the ESC to detect, if at all) measure the offset again. In between, the offset is interpolated linearly
    in time (column 'time', the mean time of the rows). This removes a linear drift of the sensor during the step,
    which the rest rows alone would overcorrect, as they come after the valid rows.
    Both fits go through the origin, since the model has no bias.
    '''
    file_path = folder_path + file_name
    column_names = ["PWM", "fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature", "voltage", "State", "time"]

    data = pd.read_csv(file_path, sep=' ', names=column_names)

    # Filter out the row 'done' and convert PWM from string to int
    data = data[data['PWM'] != 'done'].copy()
    data['PWM'] = data['PWM'].astype(int)

    if data['time'].isna().any():
        raise SystemExit("No column 'time': the log is not from motor_test in rpm mode with telemetry and one-shot mode")

    # The force sensor node aims at 100 Hz, but the rate drops if the sensor sends about 1 or 2 kHz
    print("Force sensor rows: {:.1f} per second on average".format(len(data) / (data['time'].iloc[-1] - data['time'].iloc[0])))
    print()

    if if_reverse:
        data.loc[:, ['fz', 'mz']] *= -1

    # Squared rotor speed per row: each step averages Omega^2, not the square of its mean Omega
    data['Omega^2'] = (data['RPM'] * RPM_TO_RAD_S) ** 2

    ''' reduce each step to its zero-corrected means '''

    steps = []
    for pwm, step_data in data.groupby('PWM'):
        valid_data = step_data[step_data['State'] == 'valid']
        if valid_data.empty:
            continue  # e.g. a row logged between the last calibration and the end of the test

        if (valid_data['RPM'] == 0).any():
            print("WARNING: PWM {}: RPM 0 in the valid phase, the motor did not run or no telemetry arrived".format(pwm))

        rest_data = step_data[(step_data['State'] == 'brake') & (step_data['RPM'] == 0)].tail(zero_samples)
        if rest_data.empty:
            # e.g. a test without rpm mode, where the stop command holds the min RPM, or a too short brake_duration
            print("WARNING: PWM {}: the rotor did not come to rest in the brake phase, no zero correction".format(pwm))
            fz_rest, mz_rest = np.nan, np.nan
            fz_zero, mz_zero = 0.0, 0.0
        else:
            if len(rest_data) < zero_samples:
                print("WARNING: PWM {}: only {} rows at rest for the zero reference".format(pwm, len(rest_data)))
            fz_rest, mz_rest = rest_data['fz'].mean(), rest_data['mz'].mean()
            # Offset at the mean time of the valid rows, interpolated between 0 at the first row and the rest rows
            first_time = step_data['time'].iloc[0]
            frac = (valid_data['time'].mean() - first_time) / (rest_data['time'].mean() - first_time)
            fz_zero, mz_zero = frac * fz_rest, frac * mz_rest

        steps.append({"PWM": pwm,
                      "rows": len(valid_data),
                      "RPM": valid_data['RPM'].mean(),
                      "RPM_std": valid_data['RPM'].std(),
                      "Omega^2": valid_data['Omega^2'].mean(),
                      "rest_rows": len(rest_data),
                      "fz_rest": fz_rest,  # offset at the end of the step, i.e. the drift since the calibration
                      "mz_rest": mz_rest,
                      "fz": valid_data['fz'].mean() - fz_zero,
                      "mz": valid_data['mz'].mean() - mz_zero})

    if len(steps) < 2:
        raise SystemExit("Less than two steps with 'valid' rows: is the log from the one-shot mode with telemetry?")

    steps = pd.DataFrame(steps)
    print(steps.to_string(index=False, float_format=lambda value: "{:.5g}".format(value)))
    print()

    ''' fit the data '''

    omega2 = steps['Omega^2'].to_numpy()
    fz = steps['fz'].to_numpy()
    mz = steps['mz'].to_numpy()

    # Least squares through the origin: y = k * Omega^2 with k = sum(y * Omega^2) / sum(Omega^4)
    k_t = np.dot(fz, omega2) / np.dot(omega2, omega2)
    k_tau = np.dot(mz, omega2) / np.dot(omega2, omega2)
    fz_residual = fz - k_t * omega2
    mz_residual = mz - k_tau * omega2

    # Diagnostic: the bias of an affine fit should be close to 0 if the zero correction works and the model holds
    slope_affine, intercept_affine = np.polyfit(omega2, fz, 1)

    if k_t < 0:
        print("WARNING: the thrust points along -z of the force sensor, use --if_reverse 1")

    fit_eq_fz = "fz = {:.4e} * Omega^2".format(k_t)
    print("Fitting Equation (x:Omega^2 y:fz): {}".format(fit_eq_fz))
    print("k_t: {:.4e}  # N/(rad/s)^2".format(k_t))
    print("k_t: {:.4f}  # N/kRPM^2, krpm_square_to_thrust_ratio with krpm_square_to_thrust_bias: 0".format(k_t * KRPM_TO_RAD_S ** 2))
    print("RMS residual: {:.4f}  # N".format(rms(fz_residual)))
    print("Affine fit for comparison: fz = {:.4e} * Omega^2 + {:.4f}  # bias in N, should be close to 0".format(slope_affine, intercept_affine))
    print()

    fit_eq_mz = "mz = {:.4e} * Omega^2".format(k_tau)
    print("Fitting Equation (x:Omega^2 y:mz): {}".format(fit_eq_mz))
    print("k_tau: {:.4e}  # N*m/(rad/s)^2".format(k_tau))
    print("k_tau: {:.6f}  # N*m/kRPM^2".format(k_tau * KRPM_TO_RAD_S ** 2))
    print("RMS residual: {:.6f}  # N*m".format(rms(mz_residual)))
    print()

    # Same quantity as the slope of mz vs fz in analyze_data()
    print("k_tau / k_t: {:.4f}  # m, m_f_rate".format(k_tau / k_t))
    print()

    ''' plot the data '''

    omega2_line = np.linspace(0, np.max(omega2), 100)

    # Create 2x2 subplots
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    # Plot 1: x:Omega^2 y:fz
    axs[0, 0].scatter(omega2, fz, color='blue', alpha=0.5, label='Data points')
    axs[0, 0].plot(omega2_line, k_t * omega2_line, color='red', label='Fitted line')
    axs[0, 0].set_title('fz vs Omega^2')
    axs[0, 0].set_xlabel('Omega^2 ((rad/s)^2)')
    axs[0, 0].set_ylabel('fz (N)')
    axs[0, 0].grid()
    axs[0, 0].legend(loc='upper left')
    axs[0, 0].text(0.95, 0.05, fit_eq_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[0, 0].transAxes, fontsize=10, color='green')

    # Plot 2: x:Omega^2 y:mz
    axs[0, 1].scatter(omega2, mz, color='blue', alpha=0.5, label='Data points')
    axs[0, 1].plot(omega2_line, k_tau * omega2_line, color='red', label='Fitted line')
    axs[0, 1].set_title('mz vs Omega^2')
    axs[0, 1].set_xlabel('Omega^2 ((rad/s)^2)')
    axs[0, 1].set_ylabel('mz (N*m)')
    axs[0, 1].grid()
    axs[0, 1].legend(loc='upper left')
    axs[0, 1].text(0.95, 0.05, fit_eq_mz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[0, 1].transAxes, fontsize=10, color='green')

    # Plot 3: residual of fz vs RPM
    axs[1, 0].scatter(steps['RPM'], fz_residual, color='blue', alpha=0.5)
    axs[1, 0].axhline(0, color='red')
    axs[1, 0].set_title('fz residual vs RPM')
    axs[1, 0].set_xlabel('RPM')
    axs[1, 0].set_ylabel('fz - k_t * Omega^2 (N)')
    axs[1, 0].grid()

    # Plot 4: residual of mz vs RPM
    axs[1, 1].scatter(steps['RPM'], mz_residual, color='blue', alpha=0.5)
    axs[1, 1].axhline(0, color='red')
    axs[1, 1].set_title('mz residual vs RPM')
    axs[1, 1].set_xlabel('RPM')
    axs[1, 1].set_ylabel('mz - k_tau * Omega^2 (N*m)')
    axs[1, 1].grid()

    # title with the file name
    fig.suptitle(file_name)

    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # pass text file name as argument
    parser = argparse.ArgumentParser()
    parser.add_argument('file_name', help='file name')
    parser.add_argument('has_telemetry', help='has telemetry, 0 or 1')
    parser.add_argument("--set_voltage", "-s", help="the voltage set in the test, float", default=0.0)
    parser.add_argument("--order", "-o", help="fitting order of fz vs PWM_ratio", default=2)
    parser.add_argument('--folder_path', "-f", help='path to folder', default='~/.ros/')
    parser.add_argument('--if_reverse', "-r", help='if the force sensor is reversed,0 or 1', default='0')
    parser.add_argument('--rpm_mode', "-m", help='identify f = k_t * Omega^2 and tau = k_tau * Omega^2 from a test in rpm mode, 0 or 1', default='0')
    parser.add_argument('--zero_samples', "-z", help='rpm mode: number of rows at rest at the end of each brake phase for the zero reference, int', default=50)
    args = parser.parse_args()

    if int(args.rpm_mode):
        if not int(args.has_telemetry):
            parser.error("rpm mode needs the measured RPM, i.e. has_telemetry 1")
        analyze_data_rpm_mode(args.folder_path, args.file_name, int(args.if_reverse), int(args.zero_samples))
    else:
        analyze_data(args.folder_path, args.file_name, int(args.has_telemetry), float(args.set_voltage), int(args.order), int(args.if_reverse))
