import tkinter as tk
from tkinter import messagebox
import numpy as np
import random
import math
import subprocess

class CoordinateApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CoralGuide Path Planner")
        self.grid_size = 50
        self.step_size = 10
        self.node_radius = 5
        self.obstacle_radius = 1.0  # Diameter of 3 units
        self.start = None
        self.goal = None
        self.waypoints = []
        self.obstacles = []

        self.canvas = tk.Canvas(root, width=500, height=500, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4)

        self.start_button = tk.Button(root, text="Set Start Point", command=self.set_start)
        self.start_button.grid(row=1, column=0)

        self.goal_button = tk.Button(root, text="Set Goal Point", command=self.set_goal)
        self.goal_button.grid(row=1, column=1)

        self.waypoint_button = tk.Button(root, text="Add Waypoint", command=self.add_waypoint)
        self.waypoint_button.grid(row=1, column=2)

        self.obstacle_button = tk.Button(root, text="Add Obstacle", command=self.add_obstacle)
        self.obstacle_button.grid(row=1, column=3)

        self.plan_button = tk.Button(root, text="Plan Path", command=self.plan_path)
        self.plan_button.grid(row=2, column=0, columnspan=2)

        self.run_ros_button = tk.Button(root, text="Run ROS", command=self.run_ros_program)
        self.run_ros_button.grid(row=2, column=2, columnspan=2)

        self.canvas.bind("<Button-1>", self.get_coordinates)
        self.mode = None

    def set_start(self):
        self.mode = "start"

    def set_goal(self):
        self.mode = "goal"

    def add_waypoint(self):
        self.mode = "waypoint"

    def add_obstacle(self):
        self.mode = "obstacle"

    def get_coordinates(self, event):
        x, y = event.x, event.y
        grid_x, grid_y = x // self.grid_size, y // self.grid_size
        coord = (grid_x * self.grid_size, grid_y * self.grid_size)

        if self.mode == "start":
            self.start = coord
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="green")
        elif self.mode == "goal":
            self.goal = coord
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="red")
        elif self.mode == "waypoint":
            self.waypoints.append(coord)
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="blue")
        elif self.mode == "obstacle":
            self.obstacles.append(coord)
            self.canvas.create_oval(coord[0]-self.obstacle_radius*self.grid_size, coord[1]-self.obstacle_radius*self.grid_size,
                                    coord[0]+self.obstacle_radius*self.grid_size, coord[1]+self.obstacle_radius*self.grid_size, fill="black")

    def plan_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        all_points = [self.start] + self.waypoints + [self.goal]
        complete_path = []

        for i in range(len(all_points) - 1):
            segment_path = self.rrt_star(all_points[i], all_points[i + 1])
            if segment_path is None:
                messagebox.showerror("Error", f"No path found between points {i} and {i+1}.")
                return
            if i != 0:
                segment_path = segment_path[1:]
            complete_path.extend(segment_path)

        self.draw_path(complete_path)
        self.save_path(complete_path, all_points)
        messagebox.showinfo("Success", "Path found and saved to path.txt")

    def draw_path(self, path):
        for i in range(len(path) - 1):
            self.canvas.create_line(path[i][0], path[i][1], path[i+1][0], path[i+1][1], fill="blue", width=2)

    def save_path(self, complete_path, all_points):
        with open("path.txt", "w") as file:
            file.write(f"{all_points[0][0]}, {all_points[0][1]}\n")  # Start point
            for i in range(len(all_points) - 1):
                segment_path = self.rrt_star(all_points[i], all_points[i + 1])
                if segment_path:
                    segment_length = len(segment_path)
                    file.write(f"{segment_path[segment_length // 3][0]}, {segment_path[segment_length // 3][1]}\n")
                    file.write(f"{segment_path[2 * segment_length // 3][0]}, {segment_path[2 * segment_length // 3][1]}\n")
                file.write(f"{all_points[i + 1][0]}, {all_points[i + 1][1]}\n")  # Waypoints and goal

    def rrt_star(self, start, goal):
        nodes = [start]
        parent = {start: None}
        for i in range(10000):  # Increased iterations for better chance to find path
            random_point = (random.randint(0, 500), random.randint(0, 500))
            nearest_node = self.get_nearest_node(nodes, random_point)
            new_node = self.steer(nearest_node, random_point)
            if not self.is_colliding(nearest_node, new_node):
                nodes.append(new_node)
                parent[new_node] = nearest_node
                if self.distance(new_node, goal) < self.step_size:
                    parent[goal] = new_node
                    return self.extract_path(parent, goal)
        return None

    def get_nearest_node(self, nodes, point):
        return min(nodes, key=lambda node: self.distance(node, point))

    def steer(self, from_node, to_point):
        angle = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
        new_point = (from_node[0] + self.step_size * math.cos(angle), from_node[1] + self.step_size * math.sin(angle))
        return new_point

    def is_colliding(self, from_node, to_node):
        for obstacle in self.obstacles:
            if self.distance_to_circle(obstacle, from_node, to_node) < self.obstacle_radius * self.grid_size:
                return True
        return False

    def distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

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

    def run_ros_program(self):
        try:
            subprocess.run(["ros2", "run", "move_blueboat", "run_mission"], check=True)
            messagebox.showinfo("Success", "ROS 2 program executed successfully.")
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Error", f"ROS 2 program failed: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = CoordinateApp(root)
    root.mainloop()
