import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.manifold import MDS

def plot_mds_from_file(filename, path_filename):
    try:
        with open(filename, 'r') as f:
            n = int(f.readline().strip()) # depots
            m = int(f.readline().strip()) # demand points
            total_vertices = n + m

            matrix_data = []
            for line in f:
                if line.strip():
                    matrix_data.append([float(x) for x in line.split()])

            dist_matrix = np.array(matrix_data)

            if dist_matrix.shape != (total_vertices, total_vertices):
                print(f"Error: Matrix size {dist_matrix.shape} does not match n+m={total_vertices}")
                return

        mds = MDS(n_components=2,
                  metric_mds=True,
                  n_init=4,
                  init='random',
                  random_state=42,
                  metric="precomputed"
                  )
        coords = mds.fit_transform(dist_matrix)

        depots = coords[:n]
        demands = coords[n:]

        plt.figure(figsize=(10, 7))

        plt.scatter(depots[:, 0], depots[:, 1], c='red', label='Depots (n)', s=100, edgecolors='black', zorder=5)
        plt.scatter(demands[:, 0], demands[:, 1], c='blue', label='Demand Points (m)', s=100, edgecolors='black', zorder=5)

        for i in range(total_vertices):
            label = f"X{i}"
            plt.annotate(label, (coords[i, 0], coords[i, 1]), textcoords="offset points", xytext=(0,10), ha='center', zorder=6)

        colors = ['green', 'purple', 'orange', 'cyan', 'brown'] # TODO add more if needed
        
        try:
            with open(path_filename, 'r') as f:
                uav_count = int(f.readline().strip())

                lines = f.readlines()
                
                uav_idx = 0
                matrix_lines = []
                
                for line in lines:
                    line = line.strip()
                    if not line:
                        if matrix_lines:
                            path_matrix = np.array(matrix_lines)
                            color = colors[uav_idx % len(colors)]
                            
                            rows, cols = np.where(path_matrix > 0.5)
                            
                            for r, c in zip(rows, cols):
                                start_coord = coords[r]
                                end_coord = coords[c]
                                
                                plt.annotate("",
                                             xy=end_coord, xycoords='data',
                                             xytext=start_coord, textcoords='data',
                                             arrowprops=dict(arrowstyle="->", color=color,
                                                             shrinkA=10, shrinkB=10,
                                                             patchA=None, patchB=None,
                                                             connectionstyle="arc3,rad=-0.1",
                                                             lw=2),
                                             zorder=3
                                             )
                                
                            plt.plot([], [], color=color, label=f'UAV {uav_idx} Path', lw=2)
                            
                            uav_idx += 1
                            matrix_lines = []
                            
                    else:
                        matrix_lines.append([float(x) for x in line.split()])
                
                if matrix_lines and uav_idx < uav_count:
                    path_matrix = np.array(matrix_lines)
                    color = colors[uav_idx % len(colors)]
                    rows, cols = np.where(path_matrix > 0.5)
                    for r, c in zip(rows, cols):
                        start_coord = coords[r]
                        end_coord = coords[c]
                        plt.annotate("",
                                     xy=end_coord, xycoords='data',
                                     xytext=start_coord, textcoords='data',
                                     arrowprops=dict(arrowstyle="->", color=color,
                                                     shrinkA=10, shrinkB=10,
                                                     connectionstyle="arc3,rad=-0.1",
                                                     lw=2),
                                     zorder=3
                                     )
                    plt.plot([], [], color=color, label=f'UAV {uav_idx} Path', lw=2)

        except FileNotFoundError:
            print(f"Warning: Path file '{path_filename}' not found. Plotting nodes only.")
        except Exception as path_e:
            print(f"Error parsing path file: {path_e}")

        plt.title('UAV Routing over MDS Coordinates')
        plt.xlabel('Dimension 1')
        plt.ylabel('Dimension 2')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6, zorder=0)

        output_filename = "../../mds_plot.png"
        plt.savefig(output_filename, dpi=300)
        print(f"Plot successfully saved to {output_filename}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    plot_mds_from_file('../../test-data/demandPointsInput.txt', 'uav_path.txt')