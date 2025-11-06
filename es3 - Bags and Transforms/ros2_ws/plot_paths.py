import pandas as pd
import matplotlib.pyplot as plt

df_plot = pd.read_csv('out.csv')

# Extract coordinates.
x_vc = df_plot['x_vc']
y_vc = df_plot['y_vc']
x_cs = df_plot['x_cs']
y_cs = df_plot['y_cs']

diff_x = df_plot['diff_x']
diff_y = df_plot['diff_y']

fig, axes = plt.subplots(1, 2, figsize=(10, 8))

# Plot the paths.
axes[0].plot(x_vc, y_vc, label='VC Path', color='blue', linewidth=2, alpha=0.7)
axes[0].plot(x_cs, y_cs, label='CS Position', color='green', linewidth=2, alpha=0.7)

# Highlight last positions.
axes[0].plot(x_cs.iloc[-1], y_cs.iloc[-1], marker='*', markersize=15, color='green', label='Charging Station (CS)', linestyle='None', markeredgecolor='black', markeredgewidth=1.5)
axes[0].plot(x_vc.iloc[-1], y_vc.iloc[-1], marker='o', markersize=10, color='blue', label='Final VC Position', linestyle='None', markeredgecolor='black', markeredgewidth=1)

# Add labels, title, and grid
axes[0].set_title('VC path w.r.t. floor frame')
axes[0].set_xlabel('X')
axes[0].set_ylabel('Y')
axes[0].grid(True, linestyle='--', alpha=0.6)
axes[0].axis('equal')
axes[0].legend()

# Plot the paths.
axes[1].plot(diff_x, diff_y, label='VC Path', color='blue', linewidth=2, alpha=0.7)

# Highlight last positions.
axes[1].plot(0, 0, marker='*', markersize=15, color='green', label='Charging Station (CS)', linestyle='None', markeredgecolor='black', markeredgewidth=1.5)
axes[1].plot(diff_x.iloc[-1], diff_y.iloc[-1], marker='o', markersize=10, color='blue', label='Final VC Position', linestyle='None', markeredgecolor='black', markeredgewidth=1)

# Add labels, title, and grid
axes[1].set_title('VC path w.r.t. CS frame')
axes[1].set_xlabel('X')
axes[1].set_ylabel('Y')
axes[1].grid(True, linestyle='--', alpha=0.6)
axes[1].axis('equal')
axes[1].legend()

# Save the plot
plt.tight_layout()
plt.savefig('paths.png')
