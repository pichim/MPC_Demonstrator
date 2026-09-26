"""Summarize both host CSV timing logs, independently of the working directory."""

import csv
import math
from pathlib import Path


def summarize(path):
    print(f"\n{path}")
    totals = {name: [] for name in ("dt", "spi")}
    run_times = None
    per_cycle = True
    previous_cycle = 0
    with path.open(newline="") as source:
        reader = csv.DictReader(source)
        expected = ["cycle", "dt_n", "dt_min_ms", "dt_mean_ms", "dt_max_ms", "spi_n", "spi_min_ms", "spi_mean_ms", "spi_max_ms", "cpu_s", "wall_s"]
        data_fields = ["time_s", "voltage_V", "current_A", "motor_position_rad", "motor_velocity_rad_s", "sent_setpoint", "sent_enable", "sent_mode"]
        if reader.fieldnames not in (expected, expected + data_fields):
            print("Invalid CSV header; regenerate this log.")
            return False
        for row in reader:
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("wrong number of columns")
                if "time_s" in row:
                    if not all(math.isfinite(float(row[key])) for key in data_fields):
                        raise ValueError("non-finite motor data")
                    if float(row["time_s"]) < 0 or row["sent_enable"] not in ("0", "1") or row["sent_mode"] not in ("0", "1"):
                        raise ValueError("invalid timestamp, enable or mode")
                cycle, spi_n, dt_n = (int(row[key]) for key in ("cycle", "spi_n", "dt_n"))
                if spi_n <= 0 or cycle != previous_cycle + spi_n:
                    raise ValueError("missing, duplicated or out-of-order cycles")
                if dt_n != spi_n - int(previous_cycle == 0):
                    raise ValueError("incorrect interval count")
                cpu, wall = float(row["cpu_s"]), float(row["wall_s"])
                if not math.isfinite(cpu + wall) or cpu < 0 or wall <= 0:
                    raise ValueError("invalid CPU/wall time")
                if run_times is not None and run_times != (cpu, wall):
                    raise ValueError("inconsistent run totals")
                per_cycle = per_cycle and int(row["spi_n"]) == 1
                groups = {}
                for name in totals:
                    n = int(row[f"{name}_n"])
                    if n < 0:
                        raise ValueError("negative sample count")
                    if n:
                        low, mean, high = (float(row[f"{name}_{key}_ms"]) for key in ("min", "mean", "max"))
                        if not all(map(math.isfinite, (low, mean, high))) or not 0 <= low <= mean <= high:
                            raise ValueError("invalid timing statistics")
                        groups[name] = (n, low, mean, high)
                for name, group in groups.items():
                    totals[name].append(group)
                run_times = cpu, wall
                previous_cycle = cycle
            except (KeyError, TypeError, ValueError) as error:
                print(f"Invalid CSV at line {reader.line_num}: {error}; regenerate this log.")
                return False
    if run_times is None:
        print("No samples recorded.")
        return True
    for name, rows in totals.items():
        if rows:
            n = sum(r[0] for r in rows)
            p99 = "unavailable (requires N=1)"
            if per_cycle:
                samples = sorted(r[2] for r in rows)
                position = 0.99 * (len(samples) - 1)
                lower = int(position)
                upper = min(lower + 1, len(samples) - 1)
                value = samples[lower] + (position - lower) * (samples[upper] - samples[lower])
                p99 = f"{value:.4f}"
            print(f"  {name}: n={n}, min={min(r[1] for r in rows):.4f}, " f"mean={sum(r[0]*r[2] for r in rows)/n:.4f}, " f"p99={p99}, max={max(r[3] for r in rows):.4f} ms")
    cpu, wall = run_times  # Repeated metadata: count once, never sum across rows.
    print(f"  CPU: {100*cpu/wall:.2f}% of one core " f"({cpu:.4f} CPU seconds / {wall:.4f} wall seconds)")
    return True


if __name__ == "__main__":
    host = Path(__file__).resolve().parents[1]
    valid = True
    for path in (host / "spi_timing_python.txt", host / "spi_timing_cpp.txt"):
        if path.is_file():
            valid = summarize(path) and valid
        else:
            print(f"{path}: not found")
    raise SystemExit(0 if valid else 1)
