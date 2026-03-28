import os
from typing import Iterable, List

import pandas as pd


def combine_csv_files(rosbag_dir: str, csv_files: Iterable[str], output_file: str) -> str:
    """Combine multiple CSV files by appending rows in order.

    - Writes a single header row.
    - Enforces identical columns across inputs (same names and order).

    Args:
        rosbag_dir: Directory containing the input CSVs (supports '~').
        csv_files: Iterable of CSV filenames (or paths) to append, in order.
        output_file: Output filename (or path). If relative, it is written under rosbag_dir.

    Returns:
        Absolute path to the written output CSV.
    """

    rosbag_dir_expanded = os.path.expanduser(rosbag_dir)
    csv_list: List[str] = list(csv_files)
    if len(csv_list) < 2:
        raise ValueError("csv_files must contain at least two CSV filenames/paths")

    print(f"Combining CSV files: {csv_list}")

    input_paths: List[str] = []
    for csv_name in csv_list:
        csv_name_expanded = os.path.expanduser(csv_name)
        if os.path.isabs(csv_name_expanded):
            candidate = csv_name_expanded
        else:
            candidate = os.path.join(rosbag_dir_expanded, csv_name_expanded)
        if not os.path.exists(candidate):
            raise FileNotFoundError(f"Input CSV not found: {candidate}")
        input_paths.append(candidate)

    output_expanded = os.path.expanduser(output_file)
    if os.path.isabs(output_expanded):
        output_path = output_expanded
    else:
        output_path = os.path.join(rosbag_dir_expanded, output_expanded)

    if os.path.exists(output_path):
        raise FileExistsError(f"Output CSV already exists: {output_path}")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    dataframes = [pd.read_csv(p) for p in input_paths]
    base_columns = list(dataframes[0].columns)
    for idx, df in enumerate(dataframes[1:], start=1):
        if list(df.columns) != base_columns:
            raise ValueError(
                "CSV column mismatch while combining files. "
                f"File '{input_paths[idx]}' has different columns than '{input_paths[0]}'."
            )

    combined = pd.concat(dataframes, axis=0, ignore_index=True)
    combined.to_csv(output_path, index=False, header=True)
    return os.path.abspath(output_path)


if __name__ == "__main__":
    rosbag_dir = "~/ros/rosbag_files/csv"
    csv_files = [
        "2026-03-28-07-48-33_mode_10_RECORDING_JOYSTICK_7_MIN_success1.csv",
        "2026-03-28-07-48-33_mode_10_RECORDING_JOYSTICK_7_MIN_success2.csv",
    ]
    output_file = "2026-03-28-07-48-33_mode_10_RECORDING_JOYSTICK_7_MIN_success.csv"
    output_path = combine_csv_files(rosbag_dir, csv_files, output_file)
    print(f"Successfully combined CSVs and written to {output_path}!")