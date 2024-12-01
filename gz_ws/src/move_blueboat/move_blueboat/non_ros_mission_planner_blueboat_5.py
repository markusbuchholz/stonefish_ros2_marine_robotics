import tkinter as tk
from tkinter import messagebox
import numpy as np
import random
import math
import heapq

class CoordinateApp:
    def __init__(self, root):
        self.root = root
        self.root.title("BlueBoat Mission Planner")
        self.grid_size = 50
        self.step_size = 10
        self.node_radius = 5
        self.obstacle_radius = 1.0  # Diameter of 3 units
        self.canvas_size = 500
        self.center_x = self.canvas_size // 2
        self.center_y = self.canvas_size // 2
        self.start = None
        self.goal = None
        self.waypoints = []
        self.obstacles = []
        self.alpha = 1.0
        self.beta = 1.0
        self.gamma = 1.0
        self.dt = 1.0

        self.zones = [
            {'ocean_current': {'direction': 0, 'force': 0}, 'wave': {'direction': 0, 'height': 0}, 'wind': {'direction': 0, 'speed': 0}},
            {'ocean_current': {'direction': 0, 'force': 0}, 'wave': {'direction': 0, 'height': 0}, 'wind': {'direction': 0, 'speed': 0}},
            {'ocean_current': {'direction': 0, 'force': 0}, 'wave': {'direction': 0, 'height': 0}, 'wind': {'direction': 0, 'speed': 0}},
            {'ocean_current': {'direction': 0, 'force': 0}, 'wave': {'direction': 0, 'height': 0}, 'wind': {'direction': 0, 'speed': 0}}
        ]

        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=10, rowspan=10, sticky='nsew')

        self.start_button = tk.Button(root, text="Set Start Point", command=self.set_start)
        self.start_button.grid(row=10, column=0, sticky='nsew')

        self.goal_button = tk.Button(root, text="Set Goal Point", command=self.set_goal)
        self.goal_button.grid(row=10, column=1, sticky='nsew')

        self.waypoint_button = tk.Button(root, text="Add Waypoint", command=self.add_waypoint)
        self.waypoint_button.grid(row=10, column=2, sticky='nsew')

        self.obstacle_button = tk.Button(root, text="Add Obstacle", command=self.add_obstacle)
        self.obstacle_button.grid(row=10, column=3, sticky='nsew')

        self.set_parameters_button = tk.Button(root, text="Set Zone Parameters", command=self.open_zone_parameter_window)
        self.set_parameters_button.grid(row=12, column=1, sticky='nsew')

        self.plan_button = tk.Button(root, text="RRT* Path", command=self.plan_path)
        self.plan_button.grid(row=11, column=0, sticky='nsew')

        self.simple_path_button = tk.Button(root, text="Simple Path", command=self.save_simple_path)
        self.simple_path_button.grid(row=11, column=2, sticky='nsew')

        self.a_star_button = tk.Button(root, text="A* Path", command=self.plan_a_star_path)
        self.a_star_button.grid(row=11, column=1, sticky='nsew')

        self.clear_button = tk.Button(root, text="Clear", command=self.clear_canvas)
        self.clear_button.grid(row=12, column=0, sticky='nsew')

        self.canvas.grid_configure(padx=5, pady=5)
        self.canvas.bind("<Button-1>", self.get_coordinates)
        self.mode = None

    def open_zone_parameter_window(self):
        self.zone_window = tk.Toplevel(self.root)
        self.zone_window.title("Set Zone Parameters")

        for zone_idx in range(4):
            tk.Label(self.zone_window, text=f"Zone {zone_idx+1}").grid(row=zone_idx*7, column=0, sticky='w')

            tk.Label(self.zone_window, text="Ocean Current Direction").grid(row=zone_idx*7 + 1, column=0, sticky='w')
            ocean_current_direction_entry = tk.Entry(self.zone_window)
            ocean_current_direction_entry.grid(row=zone_idx*7 + 1, column=1)
            ocean_current_direction_entry.insert(0, str(self.zones[zone_idx]['ocean_current']['direction']))

            tk.Label(self.zone_window, text="Ocean Current Force").grid(row=zone_idx*7 + 2, column=0, sticky='w')
            ocean_current_force_entry = tk.Entry(self.zone_window)
            ocean_current_force_entry.grid(row=zone_idx*7 + 2, column=1)
            ocean_current_force_entry.insert(0, str(self.zones[zone_idx]['ocean_current']['force']))

            tk.Label(self.zone_window, text="Wave Direction").grid(row=zone_idx*7 + 3, column=0, sticky='w')
            wave_direction_entry = tk.Entry(self.zone_window)
            wave_direction_entry.grid(row=zone_idx*7 + 3, column=1)
            wave_direction_entry.insert(0, str(self.zones[zone_idx]['wave']['direction']))

            tk.Label(self.zone_window, text="Wave Height").grid(row=zone_idx*7 + 4, column=0, sticky='w')
            wave_height_entry = tk.Entry(self.zone_window)
            wave_height_entry.grid(row=zone_idx*7 + 4, column=1)
            wave_height_entry.insert(0, str(self.zones[zone_idx]['wave']['height']))

            tk.Label(self.zone_window, text="Wind Direction").grid(row=zone_idx*7 + 5, column=0, sticky='w')
            wind_direction_entry = tk.Entry(self.zone_window)
            wind_direction_entry.grid(row=zone_idx*7 + 5, column=1)
            wind_direction_entry.insert(0, str(self.zones[zone_idx]['wind']['direction']))

            tk.Label(self.zone_window, text="Wind Speed").grid(row=zone_idx*7 + 6, column=0, sticky='w')
            wind_speed_entry = tk.Entry(self.zone_window)
            wind_speed_entry.grid(row=zone_idx*7 + 6, column=1)
            wind_speed_entry.insert(0, str(self.zones[zone_idx]['wind']['speed']))

            self.zones[zone_idx]['entries'] = {
                'ocean_current_direction': ocean_current_direction_entry,
                'ocean_current_force': ocean_current_force_entry,
                'wave_direction': wave_direction_entry,
                'wave_height': wave_height_entry,
                'wind_direction': wind_direction_entry,
                'wind_speed': wind_speed_entry
            }

        save_button = tk.Button(self.zone_window, text="Save", command=self.save_zone_parameters)
        save_button.grid(row=30, column=0, columnspan=2, sticky='nsew')

    def save_zone_parameters(self):
        try:
            for zone_idx, zone in enumerate(self.zones):
                zone['ocean_current']['direction'] = float(zone['entries']['ocean_current_direction'].get())
                zone['ocean_current']['force'] = float(zone['entries']['ocean_current_force'].get())
                zone['wave']['direction'] = float(zone['entries']['wave_direction'].get())
                zone['wave']['height'] = float(zone['entries']['wave_height'].get())
                zone['wind']['direction'] = float(zone['entries']['wind_direction'].get())
                zone['wind']['speed'] = float(zone['entries']['wind_speed'].get())
            self.plot_environmental_effects()
            self.zone_window.destroy()
        except ValueError:
            messagebox.showerror("Error", "Please enter valid parameters for ocean current, wave, and wind.")

    def plot_environmental_effects(self):
        self.canvas.delete("arrows")  # Remove old arrows
        self.draw_zone_lines()  # Draw zone boundary lines

        zone_width = self.canvas_size // 2
        zone_height = self.canvas_size // 2

        # Visual offset for arrow directions (in degrees)
        visual_offset = 180  # Change this value to adjust the arrow direction

        for zone_idx, zone in enumerate(self.zones):
            zone_x = (zone_idx % 2) * zone_width
            zone_y = (zone_idx // 2) * zone_height

            for i in range(zone_x, zone_x + zone_width, self.grid_size):
                for j in range(zone_y, zone_y + zone_height, self.grid_size):
                    x, y = i + self.grid_size // 2, j + self.grid_size // 2
                    self.draw_arrow(x, y, zone['ocean_current'], "blue", visual_offset)
                    self.draw_arrow(x, y, zone['wave'], "green", visual_offset)
                    self.draw_arrow(x, y, zone['wind'], "red", visual_offset)

    def draw_zone_lines(self):
        self.canvas.create_line(self.canvas_size // 2, 0, self.canvas_size // 2, self.canvas_size, fill="gray", dash=(2, 2))
        self.canvas.create_line(0, self.canvas_size // 2, self.canvas_size, self.canvas_size // 2, fill="gray", dash=(2, 2))

    def draw_arrow(self, x, y, params, color, visual_offset):
        direction = np.radians(params['direction'] + visual_offset)  # Apply visual offset
        length = params['force'] if 'force' in params else params['height'] if 'height' in params else params['speed']
        dx = length * math.cos(direction) * self.grid_size / 5
        dy = -length * math.sin(direction) * self.grid_size / 5
        self.canvas.create_line(x, y, x + dx, y + dy, fill=color, arrow=tk.LAST, tags="arrows")

    def set_start(self):
        self.mode = "start"

    def set_goal(self):
        self.mode = "goal"

    def add_waypoint(self):
        self.mode = "waypoint"

    def add_obstacle(self):
        self.mode = "obstacle"

    def canvas_to_grid(self, x, y):
        grid_x = (x - self.center_x) / self.grid_size
        grid_y = (self.center_y - y) / self.grid_size
        return np.array([grid_x, grid_y])

    def grid_to_canvas(self, grid_x, grid_y):
        x = grid_x * self.grid_size + self.center_x
        y = self.center_y - grid_y * self.grid_size
        return x, y

    def get_coordinates(self, event):
        x, y = event.x, event.y
        grid_x, grid_y = self.canvas_to_grid(x, y)
        coord = (x, y)

        if self.mode == "start":
            self.start = (grid_x, grid_y)
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="green")
        elif self.mode == "goal":
            self.goal = (grid_x, grid_y)
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="red")
        elif self.mode == "waypoint":
            self.waypoints.append((grid_x, grid_y))
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="blue")
        elif self.mode == "obstacle":
            self.obstacles.append((grid_x, grid_y))
            self.canvas.create_oval(coord[0]-self.obstacle_radius*self.grid_size, coord[1]-self.obstacle_radius*self.grid_size,
                                    coord[0]+self.obstacle_radius*self.grid_size, coord[1]+self.obstacle_radius*self.grid_size, fill="black")

    def plan_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        all_points = [self.start] + self.waypoints + [self.goal]
        complete_path = []

        for i in range(len(all_points) - 1):
            segment_path_with_cost = self.rrt_star(all_points[i], all_points[i + 1], without_environmental_cost=False)
            segment_path_without_cost = self.rrt_star(all_points[i], all_points[i + 1], without_environmental_cost=True)
            if segment_path_with_cost is None or segment_path_without_cost is None:
                messagebox.showerror("Error", f"No path found between points {i} and {i+1}.")
                return
            if i != 0:
                segment_path_with_cost = segment_path_with_cost[1:]
                segment_path_without_cost = segment_path_without_cost[1:]
            complete_path.extend(segment_path_with_cost)

        self.draw_path(complete_path)
        self.draw_path(segment_path_without_cost, dash=(5, 2))
        self.save_path(complete_path, "mission_path.txt")
        messagebox.showinfo("Success!", "Path found and saved to mission_path.txt")

    def plan_a_star_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        all_points = [self.start] + self.waypoints + [self.goal]
        complete_path = []

        for i in range(len(all_points) - 1):
            segment_path_with_cost = self.a_star(all_points[i], all_points[i + 1])
            segment_path_without_cost = self.a_star(all_points[i], all_points[i + 1], without_environmental_cost=True)
            if segment_path_with_cost is None or segment_path_without_cost is None:
                messagebox.showerror("Error", f"No path found between points {i} and {i+1}.")
                return
            if i != 0:
                segment_path_with_cost = segment_path_with_cost[1:]
                segment_path_without_cost = segment_path_without_cost[1:]
            complete_path.extend(segment_path_with_cost)

        self.draw_path(complete_path)
        self.draw_path(segment_path_without_cost, dash=(5, 2))
        self.save_path(complete_path, "mission_path.txt")
        messagebox.showinfo("Success!", "A* path found and saved to mission_path.txt")

    def draw_path(self, path, dash=None):
        for i in range(len(path) - 1):
            x1, y1 = self.grid_to_canvas(*path[i])
            x2, y2 = self.grid_to_canvas(*path[i+1])
            self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=2, dash=dash)

    def save_path(self, complete_path, filename):
        with open(filename, "w") as file:
            for point in complete_path:
                x, y = point
                file.write(f"{x}, {y}\n")

    def save_simple_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        with open("mission_path.txt", "w") as file:
            file.write(f"{self.start[0]}, {self.start[1]}\n")
            for waypoint in self.waypoints:
                file.write(f"{waypoint[0]}, {waypoint[1]}\n")
            file.write(f"{self.goal[0]}, {self.goal[1]}\n")
        messagebox.showinfo("Success!", "Simple path saved to mission_path.txt")

    def rrt_star(self, start, goal, without_environmental_cost=False):
        nodes = [start]
        parent = {start: None}
        cost = {start: 0}
        for i in range(10000):
            random_point = (random.uniform(-5, 5), random.uniform(-5, 5))
            nearest_node = self.get_nearest_node(nodes, random_point)
            new_node = self.steer(nearest_node, random_point)
            if not self.is_colliding(nearest_node, new_node):
                new_cost = cost[nearest_node] + self.distance(nearest_node, new_node)
                if not without_environmental_cost:
                    new_cost += self.environmental_cost(nearest_node, new_node)
                nodes.append(new_node)
                parent[new_node] = nearest_node
                cost[new_node] = new_cost
                if self.distance(new_node, goal) < self.step_size / self.grid_size:
                    parent[goal] = new_node
                    return self.extract_path(parent, goal)
        return None

    def get_nearest_node(self, nodes, point):
        return min(nodes, key=lambda node: self.distance(node, point))

    def steer(self, from_node, to_point):
        angle = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
        new_point = (from_node[0] + (self.step_size / self.grid_size) * math.cos(angle), from_node[1] + (self.step_size / self.grid_size) * math.sin(angle))
        return new_point

    def is_colliding(self, from_node, to_node):
        for obstacle in self.obstacles:
            if self.distance_to_circle(obstacle, from_node, to_node) < self.obstacle_radius:
                return True
        return False

    def distance(self, node1, node2):
        return np.linalg.norm(np.array(node1) - np.array(node2))

    def distance_to_circle(self, circle_center, line_start, line_end):
        px, py = circle_center
        ax, ay = line_start
        bx, by = line_end
        line_mag = self.distance(line_start, line_end)
        if line_mag == 0:
            return self.distance(circle_center, line_start)
        u = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / float(line_mag ** 2)
        closest_point = (ax + u * (bx - ax), ay + u * (by - ay))
        return self.distance(circle_center, closest_point)

    def extract_path(self, parent, goal):
        path = [goal]
        current_node = goal
        while parent[current_node] is not None:
            current_node = parent[current_node]
            path.append(current_node)
        path.reverse()
        return path

    def a_star(self, start, goal, without_environmental_cost=False):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.distance(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if self.distance(current, goal) < self.step_size / self.grid_size:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                if not without_environmental_cost:
                    tentative_g_score += self.environmental_cost(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def environmental_cost(self, from_node, to_node):
        current_effect = self.get_ocean_current(from_node)
        wind_effect = self.get_wind_effect(from_node)
        wave_effect = self.get_wave_effect(from_node)
        total_effect = current_effect + wind_effect + wave_effect
        from_node = np.array(from_node)
        to_node = np.array(to_node)
        cost = np.linalg.norm(to_node - from_node + total_effect)
        return self.alpha * cost + self.beta * cost + self.gamma * cost

    def get_ocean_current(self, position):
        zone = self.get_zone(position)
        direction = np.radians(zone['ocean_current']['direction'])
        force = zone['ocean_current']['force']
        return force * np.array([np.cos(direction), np.sin(direction)])

    def get_wind_effect(self, position):
        zone = self.get_zone(position)
        direction = np.radians(zone['wind']['direction'])
        speed = zone['wind']['speed']
        return speed * np.array([np.cos(direction), np.sin(direction)])

    def get_wave_effect(self, position):
        zone = self.get_zone(position)
        direction = np.radians(zone['wave']['direction'])
        height = zone['wave']['height']
        return height * np.array([np.cos(direction), np.sin(direction)])

    def get_neighbors(self, node):
        neighbors = []
        for dx in [-self.step_size / self.grid_size, 0, self.step_size / self.grid_size]:
            for dy in [-self.step_size / self.grid_size, 0, self.step_size / self.grid_size]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (node[0] + dx, node[1] + dy)
                if not self.is_colliding(node, neighbor):
                    neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_zone(self, position):
        x, y = position
        if x < 0 and y < 0:
            return self.zones[0]
        elif x >= 0 and y < 0:
            return self.zones[1]
        elif x < 0 and y >= 0:
            return self.zones[2]
        else:
            return self.zones[3]

    def clear_canvas(self):
        self.canvas.delete("all")
        self.start = None
        self.goal = None
        self.waypoints = []
        self.obstacles = []
        self.plot_environmental_effects()  # Redraw the zone lines

def main():
    root = tk.Tk()
    app = CoordinateApp(root)

    def on_closing():
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
