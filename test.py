import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splrep, splev

# Define the input points
x_coord = np.array([0, 0.25, 0.5, 0.5])
y_coord = np.array([0, 0.5, 3, 4])
t_span_high = np.array([100, 101.5, 103.9, 104.2])  # High range (unequal interval)
t_span_shifted = t_span_high - 100                 # Shifted high range (unequal interval)
t_span_low = np.array([1, 3, 5, 7])                # Low range (equal interval)

# Fit splines for high range
spline_x_high = splrep(t_span_high, x_coord, s=0)
spline_y_high = splrep(t_span_high, y_coord, s=0)

# Fit splines for shifted high range
spline_x_shifted = splrep(t_span_shifted, x_coord, s=0)
spline_y_shifted = splrep(t_span_shifted, y_coord, s=0)

# Fit splines for low range
spline_x_low = splrep(t_span_low, x_coord, s=0)
spline_y_low = splrep(t_span_low, y_coord, s=0)

# Define interpolation functions
xt_high = lambda t: splev(t, spline_x_high)
yt_high = lambda t: splev(t, spline_y_high)

xt_shifted = lambda t: splev(t, spline_x_shifted)
yt_shifted = lambda t: splev(t, spline_y_shifted)

xt_low = lambda t: splev(t, spline_x_low)
yt_low = lambda t: splev(t, spline_y_low)

# Generate query points for dense sampling
t_query_high = np.linspace(100, 105, 1000)   # High range query
t_query_shifted = np.linspace(0, 5, 1000)    # Shifted high range query
t_query_low = np.linspace(1, 7, 1000)        # Low range query

# Compute interpolated values
x_high = xt_high(t_query_high)
y_high = yt_high(t_query_high)

x_shifted = xt_shifted(t_query_shifted)
y_shifted = yt_shifted(t_query_shifted)

x_low = xt_low(t_query_low)
y_low = yt_low(t_query_low)

fig, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)

# Plotting the results
# plt.figure(figsize=(12, 8))

# Original points
axes[0].plot(x_coord, y_coord, 'ro', label="Original Points")

# High range interpolation (unequal interval)
axes[0].plot(x_high, y_high, 'b--', label="High Range (Unequal Interval)")

# Shifted high range interpolation
axes[0].plot(x_shifted, y_shifted, 'm-.', label="Shifted High Range (Unequal Interval)")

# Low range interpolation (equal interval)
axes[0].plot(x_low, y_low, 'g:', label="Low Range (Equal Interval)")

# Formatting
axes[0].set_title("Comparison of Interpolation for High, Shifted High, and Low Ranges")
axes[0].set_xlabel("X Coordinate")
axes[0].set_ylabel("Y Coordinate")
axes[0].legend()
axes[0].grid(True)
# plt.show()

# Time-based interpolation plots
# plt.figure(figsize=(12, 8))

# High range
axes[1].plot(t_query_high, x_high, label="X (High Range)", linestyle='--', color='b')
axes[1].plot(t_query_high, y_high, label="Y (High Range)", linestyle='-', color='b')

# Formatting
axes[1].set_title("Interpolation Results Over Time")
axes[1].set_xlabel("Time")
axes[1].set_ylabel("Values")
axes[1].legend()
axes[1].grid(True)
# plt.show()


# Time-based interpolation plots
# plt.figure(figsize=(12, 8))
# Shifted high range
axes[2].plot(t_query_shifted, x_shifted, label="X (Shifted High Range)", linestyle='--', color='m')
axes[2].plot(t_query_shifted, y_shifted, label="Y (Shifted High Range)", linestyle='-', color='m')


# Low range
axes[2].plot(t_query_low, x_low, label="X (Low Range)", linestyle='--', color='g')
axes[2].plot(t_query_low, y_low, label="Y (Low Range)", linestyle='-', color='g')

# Formatting
axes[2].set_title("Interpolation Results Over Time")
axes[2].set_xlabel("Time")
axes[2].set_ylabel("Values")
axes[2].legend()
axes[2].grid(True)

# plt.supset_title("Side-by-Side Interpolation Results")
plt.show()
