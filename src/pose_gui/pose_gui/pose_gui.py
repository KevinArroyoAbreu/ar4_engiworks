#!/usr/bin/env python3
# Made with AI (ChatGPT)

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tkinter as tk
from tkinter import filedialog, messagebox
from tf2_ros import Buffer, TransformListener
import re
import time
import os 

class TfViewer(Node):
    def __init__(self):
        super().__init__('pose_gui', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.last_update_time = None  # store timestamp in seconds
        self.last_displayed_tf = None  # to compare with last shown

        self.root = tk.Tk()
        self.root.title("TF Viewer: base_link → ee_link")

        self.header = tk.Label(
            self.root,
            text="Pose Coordinates Viewer",
            font=("Ubuntu", 18, "bold"),
            bg="#0B4B8B",     # Dark blue background
            fg="#E9E9E9",     # White text
            padx=10,
            pady=10
        )
        self.header.pack(fill="x")  # Fill horizontally

        self.red_stripe = tk.Frame(
            self.root,
            bg="firebrick",
            height=12  # You can adjust this for a thicker or thinner stripe
        )
        self.red_stripe.pack(fill="x")

        # Set custom icon here:
        try:
            icon = tk.PhotoImage(file='src/pose_gui/pose_gui_icon.png')
            self.root.iconphoto(True, icon)
        except Exception as e:
            print(f"Warning: could not load icon: {e}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.label = tk.Label(self.root, text="Waiting for TF data...", font=("Courier", 14))
        self.label.pack(padx=20, pady=10)

        # Notices
        notice_label = tk.Label(self.root, text="Execute Move in Rviz and wait for TF to be updated", 
                                fg="firebrick", font=("Courier", 12, "italic"))
        notice_label.pack(pady=10)

        notice_label2 = tk.Label(self.root, text="Showing coords. for base_link → ee_link", 
                                fg="black", font=("Courier", 10, "italic"))
        notice_label2.pack(pady=10)

        # Frame for file selection and pose name input
        frame = tk.Frame(self.root)
        frame.pack(padx=20, pady=10, fill="x")

        # File selection widgets
        tk.Label(frame, text="Target File:", font=("Courier", 10)).grid(row=0, column=0, sticky="w")
        self.file_var = tk.StringVar()
        self.file_entry = tk.Entry(frame, textvariable=self.file_var, width=40)
        self.file_entry.grid(row=0, column=1, sticky="ew", padx=(5, 5))
        browse_button = tk.Button(frame, text="Browse", command=self.browse_file)
        browse_button.grid(row=0, column=2, sticky="ew")

        # Pose name widgets
        tk.Label(frame, text="Pose Name:", font=("Courier", 10)).grid(row=1, column=0, sticky="w", pady=(10, 0))
        self.pose_name_entry = tk.Entry(frame, width=40)
        self.pose_name_entry.grid(row=1, column=1, sticky="ew", padx=(5, 5), pady=(10, 0))

        # Save button
        self.save_button = tk.Button(self.root, text="Save Current Pose", command=self.save_pose)
        self.save_button.pack(pady=10)

        # Add Refresh button
        self.refresh_button = tk.Button(self.root, text="Refresh TF", command=self.update_tf)
        self.refresh_button.pack(pady=5)

        # Status labels
        self.status_label = tk.Label(self.root, text="", font=("Courier", 10), fg="green")
        self.status_label.pack(pady=5)

        self.save_status_label = tk.Label(self.root, text="", font=("Courier", 10), fg="green")
        self.save_status_label.pack(pady=(0, 5))

        self.time_label = tk.Label(self.root, text="Last update: N/A", font=("Courier", 10))
        self.time_label.pack(pady=(0, 5))

        # Make sure frame expands properly
        frame.columnconfigure(1, weight=1)

        self.current_tf = None
        self.timer = self.create_timer(1.0, self.update_tf)
        # Refresh and show last update time
        self.periodic_refresh()
        self.update_time_label()

    def browse_file(self):
        # Use absolute path to avoid issues:
        default_dir = os.path.expanduser('~/ar4/src/programs_cpp/saved_poses')
        file_path = filedialog.askopenfilename(
            title="Select or create file to save poses",
            defaultextension=".txt",
            filetypes=[("All Files", "*.*")],
            initialdir=default_dir
        )
        if file_path:
            self.file_var.set(file_path)

    def update_time_label(self):
        if self.last_update_time is not None:
            elapsed = int(time.time() - self.last_update_time)
            minutes, seconds = divmod(elapsed, 60)
            self.time_label.config(text=f"Last update: {minutes}m {seconds}s ago")
        else:
            self.time_label.config(text="Last update: N/A")

        # Schedule next update in 1 second
        self.root.after(1000, self.update_time_label)

    def periodic_refresh(self):
        self.update_tf()
        # Schedule this method to be called again after 100 ms (0.1 second)
        self.root.after(100, self.periodic_refresh)

    def update_tf(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'ee_link', rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )

            pos = tf.transform.translation
            rot = tf.transform.rotation

            # Create tuple for comparison
            current_tuple = (pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w)

            self.current_tf = tf  # Always update current_tf

            # Only refresh label + timestamp if TF changed
            if current_tuple != self.last_displayed_tf:
                display = (
                    f"Position:\n"
                    f"  x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}\n"
                    f"Orientation (Quaternion):\n"
                    f"  x: {rot.x:.3f}, y: {rot.y:.3f}, z: {rot.z:.3f}, w: {rot.w:.3f}"
                )
                self.label.config(text=display)

                self.last_update_time = time.time()
                self.last_displayed_tf = current_tuple

                self.status_label.config(text="TF updated", fg="green")
            else:
                self.status_label.config(text="TF unchanged", fg="gray")

        except Exception as e:
            self.label.config(text=f"Waiting for TF data...\n{str(e)}")
            self.status_label.config(text="No TF data", fg="red")
            self.current_tf = None

    def save_pose(self):
        if self.current_tf is None:
            messagebox.showwarning("No TF Data", "Cannot save pose — no transform available.")
            return

        pose_name = self.pose_name_entry.get().strip()
        if not pose_name:
            messagebox.showwarning("No Pose Name", "Please enter a pose name.")
            return

        file_path = self.file_var.get().strip()
        if not file_path:
            messagebox.showwarning("No File Selected", "Please select a file to save the pose.")
            return

        versioned_name = self.get_versioned_pose_name(file_path, pose_name)
        if versioned_name != pose_name:
            proceed = messagebox.askyesno(
                "Pose Name Exists",
                f"Pose name '{pose_name}' exists.\nSaving as '{versioned_name}'. Proceed?"
            )
            if not proceed:
                self.save_status_label.config(text="Save cancelled.", fg="orange")
                return

        pos = self.current_tf.transform.translation
        rot = self.current_tf.transform.rotation

        cpp_pose = (
            f"\n//-----------{versioned_name}---------------\n"
            f"geometry_msgs::msg::Pose {versioned_name} = []() {{\n"
            f"    geometry_msgs::msg::Pose {versioned_name};\n"
            f"    {versioned_name}.position.x = {pos.x:.6f};\n"
            f"    {versioned_name}.position.y = {pos.y:.6f};\n"
            f"    {versioned_name}.position.z = {pos.z:.6f};\n"
            f"    {versioned_name}.orientation.x = {rot.x:.6f};\n"
            f"    {versioned_name}.orientation.y = {rot.y:.6f};\n"
            f"    {versioned_name}.orientation.z = {rot.z:.6f};\n"
            f"    {versioned_name}.orientation.w = {rot.w:.6f};\n"
            f"    return {versioned_name};\n"
            f"}}();\n"
        )

        try:
            if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
                # File doesn't exist or is empty — create header boilerplate
                content = (
                    "#ifndef SAVED_POSES_HPP\n"
                    "#define SAVED_POSES_HPP\n\n"
                    "#include <geometry_msgs/msg/pose.hpp>\n\n"
                    f"{cpp_pose}\n"
                    "#endif // SAVED_POSES_HPP\n"
                )
            else:
                with open(file_path, "r") as f:
                    lines = f.readlines()

                insert_index = -1
                for i, line in enumerate(lines):
                    if line.strip().startswith("#endif"):
                        insert_index = i
                        break

                if insert_index == -1:
                    # If #endif is not found, append at the end and add it
                    lines.append(f"{cpp_pose}\n#endif // SAVED_POSES_HPP\n")
                else:
                    lines.insert(insert_index, f"{cpp_pose}\n")

                content = "".join(lines)

            with open(file_path, "w") as f:
                f.write(content)

            self.save_status_label.config(text=f"Saved pose: {versioned_name}", fg="green")
            self.save_status_label.after(3000, lambda: self.save_status_label.config(text=""))

        except Exception as e:
            messagebox.showerror("Error Saving File", f"Failed to save pose:\n{str(e)}")

    def get_versioned_pose_name(self, filepath, pose_name):
        try:
            with open(filepath, "r") as f:
                content = f.read()
        except Exception:
            return pose_name  # File not found or unreadable, just return the original

        # Regex to find pose names EXACTLY like 'pose_name' or 'pose_name_v<number>'
        pattern = re.compile(rf"-----------({re.escape(pose_name)}(?:_v(\d+))?)---------------")

        versions = []
        for match in pattern.finditer(content):
            if match.group(2):
                versions.append(int(match.group(2)))
            else:
                versions.append(0)  # original name without version

        if not versions:
            return pose_name  # no match found, use original name

        max_version = max(versions)
        if max_version == 0:
            # Original name exists, so next is _v1
            return f"{pose_name}_v1"
        else:
            # Return next version number
            return f"{pose_name}_v{max_version + 1}"

    def run_gui(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = TfViewer()
    node.run_gui()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
