import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.manifold import MDS

def plot_mds_from_file(filename):
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

        plt.scatter(depots[:, 0], depots[:, 1], c='red', label='Depots (n)', s=100, edgecolors='black')

        plt.scatter(demands[:, 0], demands[:, 1], c='blue', label='Demand Points (m)', s=100, edgecolors='black')

        for i in range(total_vertices):
            label = f"D{i}" if i < n else f"C{i-n}"
            plt.annotate(label, (coords[i, 0], coords[i, 1]), textcoords="offset points", xytext=(0,10), ha='center')

        plt.title('MDS Visualization: Depots and Demand Points')
        plt.xlabel('Dimension 1')
        plt.ylabel('Dimension 2')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6)

        output_filename = "mds_plot.png"
        plt.savefig(output_filename)
        print(f"Plot successfully saved to {output_filename}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    plot_mds_from_file('demandPointsInput.txt')