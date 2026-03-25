#!/usr/bin/env python3
"""
Plot every channel in burnout_state_machine_log.csv on one figure.

The graph shows:
  • Raw vertical acceleration  (az_g)
  • True barometric altitude   (alt_m)
  • Estimated altitude         (estAlt_m) from the VVE
  • Predicted apogee altitude  (predApogee_m)
  • State‑machine state        (STATE_ARMED … STATE_DESCENT)

The first three share the left‑hand y‑axis (metres), acceleration uses the
right‑hand y‑axis (g’s), and the state is drawn as a light step‑plot on a
third y‑axis offset slightly to the right.

No colours are hard‑coded so the script respects your matplotlib rcParams.
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt


def main(csv_path: str) -> None:
    df = pd.read_csv(csv_path)

    # Convert time to seconds for a tidier x‑axis
    t = df["time"] * 1e-3

    fig, ax_alt = plt.subplots(figsize=(10, 6))

    # ── altitude‑family traces (left axis) ────────────────────────────────
    ax_alt.plot(t, df["altitude"],            label="Altitude (baro)")
    ax_alt.plot(t, df["estAlt_m"],         label="Estimated Altitude (VVE)")
    ax_alt.plot(t, df["predApogee_m"],     label="Predicted Apogee")
    ax_alt.plot(t, df['quadPredApogee_m'], label="Predicted Apogee (Quad)")
    ax_alt.plot(t, df['polyPredApogee_m'], label="Predicted Apogee (Poly)")
    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Altitude / Prediction (m)")

    # ── acceleration trace (first right axis) ────────────────────────────
    ax_acc = ax_alt.twinx()
    # ax_acc.plot(t, df["az_g"], linestyle="--", label="Vertical Accel (g)")
    # ax_acc.set_ylabel("Acceleration (g)")

    # ── state trace (second right axis, slightly offset) ─────────────────
    ax_state = ax_alt.twinx()
    # ax_state.spines["right"].set_position(("axes", 1.15))
    # ax_state.plot(
    #     t, df["state"],
    #     drawstyle="steps-post",
    #     alpha=0.4,
    #     label="State (integer)",
    #     color="red",
    # )
    # ax_state.set_ylabel("State")

    # Plot estVel_mps
    ax_est_vel = ax_alt.twinx()
    # ax_est_vel.spines["right"].set_position(("axes", 1.3))
    # ax_est_vel.plot(
    #     t, df["estVel_mps"],
    #     alpha=0.4,
    #     label="Estimated Velocity (m/s)",
    #     color="orange",
    # )

    # ── shared legend ────────────────────────────────────────────────────
    lines, labels = [], []
    for ax in (ax_alt, ax_acc, ax_state):
        ln, lb = ax.get_legend_handles_labels()
        lines.extend(ln)
        labels.extend(lb)
    ax_alt.legend(lines, labels, loc="upper left")

    plt.title("Flight‑computer log overview")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
   
    csv_path = "test/test_burnout_state_machine/burnout_state_machine_log.csv"

    main(csv_path)
