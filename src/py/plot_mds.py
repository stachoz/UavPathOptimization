import argparse
import numpy as np
import matplotlib
import os

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.manifold import MDS

def parse_coord_line(line_str):
    line_str = line_str.replace(',', '.')
    
    parts = line_str.split(';')
    
    if len(parts) < 3:
        raise ValueError(f"Expected at least 3 parts (x;y;type) but got {len(parts)} in line: {line_str}")

    x = float(parts[0].strip())
    y = float(parts[1].strip())
    vertex_type = parts[2].strip()
    
    return x, y, vertex_type

def read_vertices_info(filename, file_type, skip_header_lines=0):
    with open(filename, 'r') as f:
        # Skip header lines if specified
        for _ in range(skip_header_lines):
            f.readline()

        if file_type == 'distances':
            try:
                n = int(f.readline().strip())
            except ValueError:
                raise ValueError(f"Error: Expected integer for 'n' on line {skip_header_lines + 1} but got non-integer value in '{filename}'")
            
            try:
                m = int(f.readline().strip())
            except ValueError:
                raise ValueError(f"Error: Expected integer for 'm' on line {skip_header_lines + 2} but got non-integer value in '{filename}'")
            
            total_vertices = n + m

            matrix_data = []
            for line_num_in_file, line in enumerate(f, start=skip_header_lines + 3):
                if line.strip():
                    try:
                        matrix_data.append([float(x) for x in line.split()])
                    except ValueError as e:
                        raise ValueError(f"Error parsing distance matrix value on line {line_num_in_file} in '{filename}': {e}")
            data_matrix = np.array(matrix_data)
            
            if data_matrix.shape != (total_vertices, total_vertices):
                raise ValueError(f"Distance matrix size {data_matrix.shape} does not match expected ({total_vertices}, {total_vertices})")
            
            return n, m, data_matrix

        elif file_type == 'coords':
            coords_list = []
            n_count = 0
            m_count = 0
            
            for line_num_in_file, line in enumerate(f, start=skip_header_lines + 1):
                if line.strip():
                    try:
                        x, y, v_type = parse_coord_line(line)
                        coords_list.append([x, y])
                        if v_type.lower() == 'depot':
                            n_count += 1
                        elif v_type.lower() == 'demand':
                            m_count += 1
                        else:
                            raise ValueError(f"Unknown vertex type '{v_type}' on line {line_num_in_file} in '{filename}'. Expected 'depot' or 'demand'.")
                    except ValueError as e:
                        raise ValueError(f"Error parsing line {line_num_in_file} in coordinate file '{filename}': {e}")
            
            data_matrix = np.array(coords_list)
            
            n = n_count
            m = m_count
            total_vertices = n + m

            if data_matrix.shape[0] != total_vertices:
                raise ValueError(f"Internal error: Mismatch between counted vertices ({total_vertices}) and parsed coordinates ({data_matrix.shape[0]})")
            if data_matrix.shape[1] != 2:
                raise ValueError(f"Coordinate matrix should have 2 columns (X, Y), got {data_matrix.shape[1]}")
            
            return n, m, data_matrix

        else:
            raise ValueError(f"Unknown file_type: {file_type}. Must be 'distances' or 'coords'.")

def plot_uav_routing(filename, file_type, path_filename='uav_path.txt', skip_header_lines=0):
    try:
        n, m, data_for_mds_or_coords = read_vertices_info(filename, file_type, skip_header_lines)
        if data_for_mds_or_coords is None:
            return

        total_vertices = n + m
        coords = None

        if file_type == 'distances':
            mds = MDS(n_components=2,
                      metric_mds=True,
                      n_init=4,
                      init='random',
                      random_state=42,
                      metric="precomputed"
                      )
            coords = mds.fit_transform(data_for_mds_or_coords)
            title_suffix = "MDS"
            xlabel = 'Dimension 1'
            ylabel = 'Dimension 2'
        elif file_type == 'coords':
            coords = data_for_mds_or_coords
            title_suffix = "Provided Coordinates"
            xlabel = 'X Coordinate'
            ylabel = 'Y Coordinate'
        else:
            print(f"Error: Invalid file_type '{file_type}' passed to plotting function.")
            return

        depots = coords[:n]
        demands = coords[n:]

        plt.figure(figsize=(10, 7))

        plt.scatter(depots[:, 0], depots[:, 1], c='red', label='Depots (n)', s=100, edgecolors='black', zorder=5)
        plt.scatter(demands[:, 0], demands[:, 1], c='blue', label='Demand Points (m)', s=100, edgecolors='black', zorder=5)

        for i in range(total_vertices):
            label = f"X{i}"
            plt.annotate(label, (coords[i, 0], coords[i, 1]), textcoords="offset points", xytext=(0,10), ha='center', zorder=6)

        colors = ['green', 'purple', 'orange', 'cyan', 'brown', 'pink', 'gray', 'olive', 'lime', 'teal'] # More colors
        
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
                            path_matrix = np.array(matrix_lines, dtype=float)
                            color = colors[uav_idx % len(colors)]
                            
                            rows, cols = np.where(path_matrix > 0.5) # Check for values close to 1
                            
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
                                
                            plt.plot([], [], color=color, label=f'UAV {uav_idx} Path', lw=2) # For legend
                            
                            uav_idx += 1
                            matrix_lines = []
                            
                    else:
                        matrix_lines.append([float(x) for x in line.split()])
                
                # Handle the last matrix if the file doesn't end with an empty line
                if matrix_lines and uav_idx < uav_count:
                    path_matrix = np.array(matrix_lines, dtype=float)
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
            print(f"Error parsing path file '{path_filename}': {path_e}")

        plt.title(f'UAV Routing over {title_suffix}')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6, zorder=0)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
        output_filename = os.path.join(project_root, "mds_plot.png")
        
        plt.savefig(output_filename, dpi=300)
        print(f"Plot successfully saved to {output_filename}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot UAV routing and node positions.")
    parser.add_argument("filename", help="Path to the input file containing vertex data (distances or coordinates).")
    parser.add_argument("file_type", choices=["distances", "coords"], 
                        help="Type of data in the input file: 'distances' (MDS will be applied) or 'coords' (coordinates are used directly).")
    parser.add_argument("--path_filename", default="uav_path.txt", 
                        help="Path to the file containing UAV routing information (default: uav_path.txt).")
    parser.add_argument("--skip_header", type=int, default=1,
                        help="Number of header lines to skip before reading n and m (for 'distances') or coordinate data (for 'coords') (default: 0).")
    
    args = parser.parse_args()
    
    plot_uav_routing(args.filename, args.file_type, args.path_filename, args.skip_header)