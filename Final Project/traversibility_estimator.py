#!/usr/bin/env python3
"""
traversability_estimator.py

Bayesian filter for terrain traversability estimation.

DBN Structure:
	Location(x,y)_t  -->  Traversable_t  -->  WheelSlip_t
								^
								|  (temporal persistence: terrain is static)
						  Traversable_{t-1}

Per-cell belief: P(Traversable=True), initialised to 0.5 as specified.

Likelihood (CPT for WheelSlip given Traversable):
	P(slip | Traversable=True)  is proportional to  exp(-K * slip)
	P(slip | Traversable=False) is proportional to  1 - exp(-K * slip)

The single proposed parameter K controls how sharply slip discriminates
between the two states. Larger K = sharper discrimination.

Filtering: each time the robot is in a cell, the slip observation is
used to update that cell's belief via Bayes' rule. Repeated observations
of the same cell sharpen the posterior toward 0 or 1.
"""

import rospy
import numpy as np
import json
import os
from std_msgs.msg import String

import matplotlib
matplotlib.use("Agg")  # non-interactive backend, works without display
import matplotlib.pyplot as plt

# ── World / grid parameters ───────────────────────────────────────────────────
WORLD_X_MIN = -3.5
WORLD_X_MAX =  3.5
WORLD_Y_MIN = -5.0
WORLD_Y_MAX =  1.7
CELL_SIZE   = 0.5

N_COLS = int((WORLD_X_MAX - WORLD_X_MIN) / CELL_SIZE)
N_ROWS = int((WORLD_Y_MAX - WORLD_Y_MIN) / CELL_SIZE)

# ── Likelihood model ──────────────────────────────────────────────────────────
K = 10.0

# ── Output paths ──────────────────────────────────────────────────────────────
MAP_IMAGE_PATH = os.path.expanduser("~/traversability_map.png")


def world_to_grid(x, y):
	col = int((x - WORLD_X_MIN) / CELL_SIZE)
	row = int((y - WORLD_Y_MIN) / CELL_SIZE)
	if 0 <= row < N_ROWS and 0 <= col < N_COLS:
		return row, col
	return None


class TraversabilityEstimator:

	def __init__(self):
		rospy.init_node("traversability_estimator")

		self.grid = np.full((N_ROWS, N_COLS), 0.5)
		self.visit_count = np.zeros((N_ROWS, N_COLS), dtype=int)

		rospy.Subscriber("/terrain_features", String, self.features_callback)
		rospy.Timer(rospy.Duration(5.0), self.print_map)
		rospy.Timer(rospy.Duration(5.0), self.save_map_image)

		rospy.loginfo(f"Traversability grid: {N_ROWS} rows x {N_COLS} cols "
					  f"({CELL_SIZE}m cells), K={K}")
		rospy.loginfo(f"Map image will be saved to {MAP_IMAGE_PATH}")

	def features_callback(self, msg):
		try:
			data = json.loads(msg.data)
		except json.JSONDecodeError:
			return

		slip = float(data.get("slip", 0.0))
		pose = data.get("pose", None)
		if pose is None or len(pose) < 2:
			return

		cell = world_to_grid(pose[0], pose[1])
		if cell is None:
			return

		row, col = cell
		self.bayesian_update(row, col, slip)
		self.visit_count[row, col] += 1

	def bayesian_update(self, row, col, slip):
		prior_true  = self.grid[row, col]
		prior_false = 1.0 - prior_true

		lik_true  = np.exp(-K * slip)
		lik_false = 1.0 - lik_true

		unnorm_true  = lik_true  * prior_true
		unnorm_false = lik_false * prior_false
		normaliser   = unnorm_true + unnorm_false

		if normaliser < 1e-12:
			return

		self.grid[row, col] = unnorm_true / normaliser

	def print_map(self, event=None):
		print("\n" + "=" * 60)
		print("TRAVERSABILITY MAP  P(Traversable=True) per 0.5m cell")
		print(f"Grid: {N_ROWS} rows (y) x {N_COLS} cols (x)")
		print("=" * 60)

		for r in range(N_ROWS - 1, -1, -1):
			row_str = ""
			for c in range(N_COLS):
				if self.visit_count[r, c] == 0:
					row_str += "  --  "
				else:
					row_str += f"{self.grid[r, c]:.3f} "
			y_lo = WORLD_Y_MIN + r * CELL_SIZE
			y_hi = y_lo + CELL_SIZE
			print(f"y[{y_lo:+.1f},{y_hi:+.1f}]  {row_str}")

		print("=" * 60)
		visited = int(np.sum(self.visit_count > 0))
		total   = N_ROWS * N_COLS
		print(f"Cells visited: {visited}/{total}  ({100*visited/total:.1f}%)")

		final = np.where(self.visit_count > 0, np.round(self.grid, 3), -1.0)
		print("\nFinal matrix (row 0 = lowest y, unvisited = -1.0):")
		print(np.array2string(final, precision=3, separator=", ", suppress_small=True))

	def save_map_image(self, event=None):
		"""Render the traversability map as a heatmap PNG."""
		# Build display array: NaN for unvisited so they render as a separate color
		display = np.where(self.visit_count > 0, self.grid, np.nan)

		fig, ax = plt.subplots(figsize=(8, 7))

		# RdYlGn: red = low traversability, green = high
		cmap = plt.cm.RdYlGn.copy()
		cmap.set_bad(color="lightgray")  # color for unvisited NaN cells

		im = ax.imshow(
			display,
			cmap=cmap,
			vmin=0.0,
			vmax=1.0,
			origin="lower",
			extent=[WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX],
			interpolation="nearest",
		)

		# Annotate each cell with its probability value
		for r in range(N_ROWS):
			for c in range(N_COLS):
				if self.visit_count[r, c] > 0:
					x_center = WORLD_X_MIN + (c + 0.5) * CELL_SIZE
					y_center = WORLD_Y_MIN + (r + 0.5) * CELL_SIZE
					val = self.grid[r, c]
					# White text on dark cells, black on light cells
					text_color = "white" if val < 0.3 or val > 0.7 else "black"
					ax.text(x_center, y_center, f"{val:.2f}",
							ha="center", va="center", fontsize=7, color=text_color)

		# Grid lines aligned to cell boundaries
		ax.set_xticks(np.arange(WORLD_X_MIN, WORLD_X_MAX + 0.001, CELL_SIZE), minor=True)
		ax.set_yticks(np.arange(WORLD_Y_MIN, WORLD_Y_MAX + 0.001, CELL_SIZE), minor=True)
		ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.3, alpha=0.5)
		ax.tick_params(which="minor", length=0)

		ax.set_xlabel("X (m)")
		ax.set_ylabel("Y (m)")
		visited = int(np.sum(self.visit_count > 0))
		total   = N_ROWS * N_COLS
		ax.set_title(f"Traversability Map  P(Traversable=True)\n"
					 f"Cells visited: {visited}/{total} ({100*visited/total:.1f}%)")

		cbar = plt.colorbar(im, ax=ax, fraction=0.04, pad=0.04)
		cbar.set_label("P(Traversable=True)")

		plt.tight_layout()
		try:
			plt.savefig(MAP_IMAGE_PATH, dpi=120)
		except Exception as e:
			rospy.logwarn(f"Failed to save map image: {e}")
		plt.close(fig)


if __name__ == "__main__":
	node = TraversabilityEstimator()
	rospy.spin()
