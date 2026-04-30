# Aidan Rohm
# Artificial Intelligence
# Professor Lyons
# Final Project 2 May 2026
"""
traversability_estimator.py

Probabilistic terrain traversability estimation from wheel slip observations.

DBN Structure:
	Location(x,y)_t  -->  Traversable_t  -->  WheelSlip_t
								^
								|  (temporal persistence: terrain is static)
						  Traversable_{t-1}

Per-cell estimate: P(Traversable=True), initialised to 0.5 as specified.

Approach
--------
Since terrain is static and the robot can directly measure WheelSlip in each
cell, we maintain a running mean of slip observations per cell. The mean slip
is then mapped to P(Traversable=True) via a soft likelihood function that
preserves nuance — cells with moderate slip get mid-range probabilities
rather than being slammed to 0 or 1.

Likelihood (CPT for WheelSlip given Traversable):
	P(Traversable=True | mean_slip) = exp(-K * mean_slip)

This is monotonically decreasing in slip:
	mean_slip = 0.000  -->  P(T=True) = 1.000
	mean_slip = 0.020  -->  P(T=True) = 0.819   (smooth driving)
	mean_slip = 0.050  -->  P(T=True) = 0.607   (slightly slippery)
	mean_slip = 0.100  -->  P(T=True) = 0.368   (moderately slippery)
	mean_slip = 0.200  -->  P(T=True) = 0.135   (slippery)
	mean_slip = 0.300  -->  P(T=True) = 0.050   (very slippery)

Filtering: each observation contributes equally to the per-cell mean slip.
Repeated observations give a more accurate mean. Unvisited cells retain
the prior of P(Traversable=True) = 0.5.
"""

import rospy
import numpy as np
import json
import os
from std_msgs.msg import String

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── World / grid parameters ───────────────────────────────────────────────────
WORLD_X_MIN = -3.5
WORLD_X_MAX =  3.5
WORLD_Y_MIN = -5.0
WORLD_Y_MAX =  1.7
CELL_SIZE   = 0.5

N_COLS = int((WORLD_X_MAX - WORLD_X_MIN) / CELL_SIZE)
N_ROWS = int((WORLD_Y_MAX - WORLD_Y_MIN) / CELL_SIZE)

# ── Likelihood parameter ──────────────────────────────────────────────────────
# Maps mean slip to P(Traversable=True) via exp(-K * mean_slip).
# K = 10 gives a graded response across the typical slip range:
#   slip 0.005 -> P = 0.95   (very smooth)
#   slip 0.050 -> P = 0.61   (some slip)
#   slip 0.150 -> P = 0.22   (clearly slippery)
K = 20.0

# ── Output path ───────────────────────────────────────────────────────────────
MAP_IMAGE_PATH = os.path.abspath("traversability_map.png")


def world_to_grid(x, y):
	col = int((x - WORLD_X_MIN) / CELL_SIZE)
	row = int((y - WORLD_Y_MIN) / CELL_SIZE)
	if 0 <= row < N_ROWS and 0 <= col < N_COLS:
		return row, col
	return None


class TraversabilityEstimator:

	def __init__(self):
		rospy.init_node("traversability_estimator")

		# Per-cell accumulators for running-mean slip
		self.slip_sum   = np.zeros((N_ROWS, N_COLS))
		self.slip_count = np.zeros((N_ROWS, N_COLS), dtype=int)

		rospy.Subscriber("/terrain_features", String, self.features_callback)
		rospy.Timer(rospy.Duration(5.0), self.print_and_save)

		rospy.loginfo(f"Traversability grid: {N_ROWS} rows x {N_COLS} cols "
					  f"({CELL_SIZE}m cells), K={K}")
		rospy.loginfo(f"Map image will be saved to: {MAP_IMAGE_PATH}")

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
		self.slip_sum[row, col]   += slip
		self.slip_count[row, col] += 1

	def compute_probability_grid(self):
		"""
		Convert running-mean slip per cell into P(Traversable=True).
		Unvisited cells get the prior 0.5.
		"""
		visited = self.slip_count > 0
		mean_slip = np.where(visited, self.slip_sum / np.maximum(self.slip_count, 1), 0.0)

		# Soft mapping: P(T=True) = exp(-K * mean_slip)
		prob = np.exp(-K * mean_slip)
		prob = np.where(visited, prob, 0.5)  # prior for unvisited
		return prob

	def print_and_save(self, event=None):
		self.print_map()
		self.save_map_image()

	def print_map(self):
		prob = self.compute_probability_grid()

		print("\n" + "=" * 60)
		print("TRAVERSABILITY MAP  P(Traversable=True) per 0.5m cell")
		print(f"Grid: {N_ROWS} rows (y) x {N_COLS} cols (x)")
		print("=" * 60)

		for r in range(N_ROWS - 1, -1, -1):
			row_str = ""
			for c in range(N_COLS):
				if self.slip_count[r, c] == 0:
					row_str += "  --  "
				else:
					row_str += f"{prob[r, c]:.3f} "
			y_lo = WORLD_Y_MIN + r * CELL_SIZE
			y_hi = y_lo + CELL_SIZE
			print(f"y[{y_lo:+.1f},{y_hi:+.1f}]  {row_str}")

		print("=" * 60)
		visited = int(np.sum(self.slip_count > 0))
		total   = N_ROWS * N_COLS
		print(f"Cells visited: {visited}/{total}  ({100*visited/total:.1f}%)")

		final = np.where(self.slip_count > 0, np.round(prob, 3), -1.0)
		print("\nFinal matrix (row 0 = lowest y, unvisited = -1.0):")
		print(np.array2string(final, precision=3, separator=", ", suppress_small=True))

	def save_map_image(self):
		try:
			prob = self.compute_probability_grid()
			display = np.where(self.slip_count > 0, prob, np.nan)

			fig, ax = plt.subplots(figsize=(8, 7))

			cmap = matplotlib.cm.get_cmap("RdYlGn")
			cmap.set_bad(color="lightgray")

			im = ax.imshow(
				display,
				cmap=cmap,
				vmin=0.0,
				vmax=1.0,
				origin="lower",
				extent=[WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX],
				interpolation="nearest",
			)

			for r in range(N_ROWS):
				for c in range(N_COLS):
					if self.slip_count[r, c] > 0:
						x_center = WORLD_X_MIN + (c + 0.5) * CELL_SIZE
						y_center = WORLD_Y_MIN + (r + 0.5) * CELL_SIZE
						val = prob[r, c]
						text_color = "white" if val < 0.3 or val > 0.7 else "black"
						ax.text(x_center, y_center, f"{val:.2f}",
								ha="center", va="center", fontsize=7, color=text_color)

			ax.set_xticks(np.arange(WORLD_X_MIN, WORLD_X_MAX + 0.001, CELL_SIZE), minor=True)
			ax.set_yticks(np.arange(WORLD_Y_MIN, WORLD_Y_MAX + 0.001, CELL_SIZE), minor=True)
			ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.3, alpha=0.5)
			ax.tick_params(which="minor", length=0)

			ax.set_xlabel("X (m)")
			ax.set_ylabel("Y (m)")
			visited = int(np.sum(self.slip_count > 0))
			total   = N_ROWS * N_COLS
			ax.set_title(f"Traversability Map  P(Traversable=True)\n"
						 f"Cells visited: {visited}/{total} ({100*visited/total:.1f}%)")

			cbar = plt.colorbar(im, ax=ax, fraction=0.04, pad=0.04)
			cbar.set_label("P(Traversable=True)")

			plt.tight_layout()
			plt.savefig(MAP_IMAGE_PATH, dpi=120)
			plt.close(fig)

			print(f">>> Saved map image: {MAP_IMAGE_PATH}")
		except Exception as e:
			print(f">>> FAILED to save map image: {e}")
			import traceback
			traceback.print_exc()


if __name__ == "__main__":
	node = TraversabilityEstimator()
	rospy.spin()#!/usr/bin/env python3
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
