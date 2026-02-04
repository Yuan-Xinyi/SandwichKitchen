import tkinter as tk
from tkinter import messagebox
import random
import socket
import sys

class ModernSandwichKitchen:
    """GUI that sends sandwich orders to a robot controller via TCP socket
       and can handle multiple orders, including a clean shutdown."""
    def __init__(self, root):
        self.root = root
        self.root.title("Gourmet Sandwich Animation Terminal")
        self.root.geometry("3000x1500")
        self.root.configure(bg="#FFFFFF")

        # -----------------------------
        # CONNECT TO ROBOT CONTROLLER (RETRY-ROBUST)
        # -----------------------------
        self.robot_host = "127.0.0.1"
        self.robot_port = 5555

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False

        def try_connect():
            try:
                self.sock.connect((self.robot_host, self.robot_port))
                self.connected = True
                print("[INFO] Connected to robot controller.")
            except Exception:
                print("[WARN] Robot not ready yet, will retry in 2s...")
                self.root.after(2000, try_connect)

        try_connect()

        # -----------------------------
        # LOAD BACKGROUND IMAGE
        # -----------------------------
        try:
            full_img = tk.PhotoImage(
                file="experiments/robot/xarm/sandwich_kitchen/cooking_chef_woman_asia.png"
            )
            self.chef_bg = full_img.subsample(2, 2)
        except:
            self.chef_bg = None
            print("Warning: image not found.")

        # -----------------------------
        # RECIPES (for animation only)
        # -----------------------------
        self.recipes = {
            "Basic Sandwich": ["Bread", "Lettuce", "Ham", "Bread"],
            "Double Sandwich": ["Bread", "Lettuce", "Ham", "Lettuce", "Ham", "Bread"],
            "Veggie Sandwich": ["Bread", "Lettuce", "Lettuce", "Bread"],
            "Meat Sandwich": ["Bread", "Ham", "Ham", "Bread"],
            "Gluten-free Sandwich": ["Lettuce", "Ham", "Lettuce"]
        }

        self.colors = {
            "Bread": "#F5E7C6",
            "Lettuce": "#A3D78A",
            "Ham": "#FFAAB8"
        }

        self.font_msg = ("DejaVu Sans", 20)
        self.font_btn = ("DejaVu Sans", 22, "bold")

        # -----------------------------
        # UI LAYOUT
        # -----------------------------
        self.main_frame = tk.Frame(root, bg="#FFFFFF")
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        self.left_ui = tk.Frame(self.main_frame, bg="#FFFFFF", width=600)
        self.left_ui.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.right_kitchen = tk.Frame(
            self.main_frame, bg="#F5F5F5", padx=20, pady=20
        )
        self.right_kitchen.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            self.right_kitchen,
            width=600,
            height=900,
            bg="white",
            highlightthickness=0,
        )
        self.canvas.pack(pady=20, expand=True)

        self.setup_ui()
        self.show_start_screen()

        # Start background check for robot messages
        self.root.after(200, self.check_robot_messages)

    # ---------------------------------------------------
    # COMMUNICATION WITH ROBOT
    # ---------------------------------------------------
    def send_order_to_robot(self, sandwich_name: str):
        if not self.connected:
            messagebox.showerror("Robot Error", "Not connected to robot controller!")
            return

        try:
            msg = sandwich_name + "\n"
            self.sock.sendall(msg.encode("utf-8"))
            print(f"[INFO] Sent order to robot: {sandwich_name}")
        except Exception as e:
            print(f"[ERROR] Failed to send order: {e}")
            self.connected = False

    def send_shutdown_to_robot(self):
        """Tell robot to shut down."""
        if not self.connected:
            self.root.quit()
            return

        try:
            self.sock.sendall(b"SHUTDOWN\n")
            print("[INFO] Sent SHUTDOWN to robot.")
        except Exception as e:
            print(f"[ERROR] Failed to send SHUTDOWN: {e}")

        # Close GUI after short delay
        self.root.after(500, self.root.quit)

    def check_robot_messages(self):
        try:
            self.sock.settimeout(0.01)
            data = self.sock.recv(1024)
            if data:
                msg = data.decode("utf-8").strip()
                print(f"[INFO] Message from robot: {msg}")

                if msg == "DONE":
                    self.complete_from_robot()

        except socket.timeout:
            pass
        except Exception as e:
            print("[WARN] Lost connection to robot:", e)
            self.connected = False

        self.root.after(200, self.check_robot_messages)

    # ---------------------------------------------------
    # UI SETUP
    # ---------------------------------------------------
    def setup_ui(self):
        tk.Label(
            self.left_ui,
            text="SANDWICH KITCHEN.",
            font=("DejaVu Sans", 36, "bold"),
            fg="#1A73E8",
            bg="#FFFFFF",
        ).pack(pady=40)

        self.chat_display = tk.Text(
            self.left_ui,
            font=self.font_msg,
            state="disabled",
            bg="#F8F9FA",
            relief="flat",
            wrap=tk.WORD,
            height=15,
        )
        self.chat_display.pack(expand=True, fill=tk.BOTH, padx=50)

        self.action_frame = tk.Frame(self.left_ui, bg="#FFFFFF", pady=50)
        self.action_frame.pack(fill=tk.X, side=tk.BOTTOM)

    def log(self, sender, msg):
        self.chat_display.config(state="normal")
        self.chat_display.insert(tk.END, f"● {sender}\n{msg}\n\n")
        self.chat_display.see(tk.END)
        self.chat_display.config(state="disabled")

    def show_start_screen(self):
        self.canvas.delete("all")

        if self.chef_bg:
            self.canvas.create_image(300, 900, image=self.chef_bg, anchor=tk.S)

        for w in self.action_frame.winfo_children():
            w.destroy()

        tk.Button(
            self.action_frame,
            text="START ORDERING",
            command=self.greet,
            font=self.font_btn,
            bg="#1A73E8",
            fg="white",
            padx=40,
            pady=20,
        ).pack()

    def greet(self):
        self.log("Staff", "Welcome to Sandwich Kitchen! How can I help you today?")

        for w in self.action_frame.winfo_children():
            w.destroy()

        tk.Button(
            self.action_frame,
            text="Sure, can I see the menu?",
            command=self.handle_see_menu,
            font=("DejaVu Sans", 18),
            width=25,
            pady=10,
        ).pack(pady=5)

        tk.Button(
            self.action_frame,
            text="Umm, what do you recommend?",
            command=self.handle_recommendation,
            font=("DejaVu Sans", 18),
            width=25,
            pady=10,
        ).pack(pady=5)

    def handle_see_menu(self):
        self.log("Customer", "Can I see the menu?")
        self.log("Staff", "Sure! We can do five kinds of sandwiches:")
        self.ask_menu()

    def handle_recommendation(self):
        self.log("Customer", "Emm, what do you recommend?")
        self.log(
            "Staff",
            "Our Double Sandwich is really popular! But feel free to check all of them:",
        )
        self.ask_menu()

    def ask_menu(self):
        for w in self.action_frame.winfo_children():
            w.destroy()

        for name in self.recipes.keys():
            tk.Button(
                self.action_frame,
                text=name,
                command=lambda n=name: self.cook(n),
                font=("DejaVu Sans", 18),
                width=20,
                pady=10,
            ).pack(pady=5)

    def draw_ingredient(self, type, layer_index, total_layers):
        spacing = 80
        cx, cy = 300, 450 - (layer_index * spacing)
        w, h = 350, 60

        if type == "Bread":
            points = [
                cx - w / 2,
                cy + h / 2,
                cx + w / 2,
                cy + h / 2,
                cx,
                cy - h / 2,
            ]
            self.canvas.create_polygon(
                points,
                fill=self.colors["Bread"],
                outline="#F5E7C6",
                width=3,
            )

        elif type == "Ham":
            self.canvas.create_arc(
                cx - w / 2,
                cy - h / 2,
                cx + w / 2,
                cy + h / 2,
                start=0,
                extent=180,
                fill=self.colors["Ham"],
                outline="#FFAAB8",
                width=2,
            )

        elif type == "Lettuce":
            offset = random.randint(-20, 20)
            points = [
                cx - w / 2 + offset,
                cy + h / 2,
                cx + w / 2 + offset,
                cy + h / 2,
                cx + offset,
                cy - h / 2,
            ]
            self.canvas.create_polygon(
                points,
                fill=self.colors["Lettuce"],
                outline="#A3D78A",
                width=2,
            )

    def cook(self, name):
        self.canvas.delete("all")

        for w in self.action_frame.winfo_children():
            w.destroy()

        self.log("Customer", f"I'll have {name}.")
        self.log("Staff", "Coming right up! Look at the kitchen...")

        self.send_order_to_robot(name)

        ingredients = self.recipes[name]
        self.animate_step(ingredients, 0, name)

    def animate_step(self, ingredients, i, name):
        if i < len(ingredients):
            self.draw_ingredient(ingredients[i], i, len(ingredients))
            self.root.after(
                800, lambda: self.animate_step(ingredients, i + 1, name)
            )

    def complete_from_robot(self):
        self.log(
            "Staff",
            "Your sandwich is ready! Please enjoy your meal, welcome next time!",
        )
        messagebox.showinfo("Ready", "Enjoy your sandwich!")

        for w in self.action_frame.winfo_children():
            w.destroy()

        tk.Label(
            self.action_frame,
            text="ORDER FINISHED",
            font=self.font_btn,
            fg="gray",
        ).pack()

        # Button to order again
        tk.Button(
            self.action_frame,
            text="ORDER ANOTHER SANDWICH",
            command=self.show_start_screen,
            font=("DejaVu Sans", 16),
            pady=10,
        ).pack(pady=10)

        # 🔹 NEW BUTTON: SHUTDOWN EVERYTHING
        tk.Button(
            self.action_frame,
            text="Thank you, that was all",
            command=self.send_shutdown_to_robot,
            font=("DejaVu Sans", 16),
            bg="#DDDDDD",
            pady=10,
        ).pack(pady=10)


if __name__ == "__main__":
    root = tk.Tk()
    try:
        root.tk.call("tk", "scaling", 2.0)
    except:
        pass

    app = ModernSandwichKitchen(root)
    root.mainloop()
