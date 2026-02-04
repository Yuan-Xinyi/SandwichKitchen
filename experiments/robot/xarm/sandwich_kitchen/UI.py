import tkinter as tk
from tkinter import messagebox
import random

class ModernSandwichKitchen:
    """Simple GUI that animates sandwich assembly steps and logs a chat-like interaction.
    This class creates a two-column UI: controls and a canvas where a sandwich is
    assembled layer-by-layer above a chef background image.
    """
    def __init__(self, root):
        self.root = root
        self.root.title("Gourmet Sandwich Animation Terminal")
        self.root.geometry("3000x1500")
        self.root.configure(bg="#FFFFFF")
        
        # Core improvement: load and downscale the background image
        try:
            full_img = tk.PhotoImage(file="experiments/robot/xarm/sandwich_kitchen/cooking_chef_woman_asia.png")
            # Subsample image: (2, 2) reduces width and height to 1/2.
            # Increase to (3, 3) if the image still appears too large.
            self.chef_bg = full_img.subsample(2, 2) 
        except:
            self.chef_bg = None
            print("Warning: image not found.")
        
        self.recipes = {
            "Basic Sandwich": ["Bread", "Lettuce", "Ham", "Bread"],
            "Double Sandwich": ["Bread", "Lettuce", "Ham", "Lettuce", "Ham", "Bread"],
            "Veggie Sandwich": ["Bread", "Lettuce", "Lettuce", "Bread"],
            "Meat Sandwich": ["Bread", "Ham", "Ham", "Bread"],
            "Gluten-free Sandwich": ["Lettuce", "Ham", "Lettuce"]
        }
        
        self.colors = {"Bread": "#F5E7C6", "Lettuce": "#A3D78A", "Ham": "#FFAAB8"}
        self.font_msg = ("DejaVu Sans", 20)
        self.font_btn = ("DejaVu Sans", 22, "bold")

        self.main_frame = tk.Frame(root, bg="#FFFFFF")
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        self.left_ui = tk.Frame(self.main_frame, bg="#FFFFFF", width=600)
        self.left_ui.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.right_kitchen = tk.Frame(self.main_frame, bg="#F5F5F5", padx=20, pady=20)
        self.right_kitchen.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.right_kitchen, width=600, height=900, bg="white", highlightthickness=0)
        self.canvas.pack(pady=20, expand=True)

        self.setup_ui()
        self.show_start_screen()

    def setup_ui(self):
        tk.Label(self.left_ui, text="SANDWICH KITCHEN.", font=("DejaVu Sans", 36, "bold"), fg="#1A73E8", bg="#FFFFFF").pack(pady=40)
        self.chat_display = tk.Text(self.left_ui, font=self.font_msg, state='disabled', bg="#F8F9FA", relief="flat", wrap=tk.WORD, height=15)
        self.chat_display.pack(expand=True, fill=tk.BOTH, padx=50)
        self.action_frame = tk.Frame(self.left_ui, bg="#FFFFFF", pady=50)
        self.action_frame.pack(fill=tk.X, side=tk.BOTTOM)

    def log(self, sender, msg):
        self.chat_display.config(state='normal')
        self.chat_display.insert(tk.END, f"● {sender}\n{msg}\n\n")
        self.chat_display.see(tk.END)
        self.chat_display.config(state='disabled')

    def show_start_screen(self):
        self.canvas.delete("all")
        # Core improvement: place the chef background image anchored at the canvas bottom
        if self.chef_bg:
            # anchor=tk.S aligns the image bottom-center to canvas coords (300, 900)
            self.canvas.create_image(300, 900, image=self.chef_bg, anchor=tk.S)
            
        for w in self.action_frame.winfo_children(): w.destroy()
        tk.Button(self.action_frame, text="START ORDERING", command=self.greet, font=self.font_btn, bg="#1A73E8", fg="white", padx=40, pady=20).pack()

    def greet(self):
        self.log("Staff", "Welcome to Sandwich Kitchen! How can I help you today?")
        for w in self.action_frame.winfo_children(): w.destroy()
        tk.Button(self.action_frame, text="Sure, can I see the menu?", command=self.handle_see_menu, font=("DejaVu Sans", 18), width=25, pady=10).pack(pady=5)
        tk.Button(self.action_frame, text="Umm, what do you recommend?", command=self.handle_recommendation, font=("DejaVu Sans", 18), width=25, pady=10).pack(pady=5)

    def handle_see_menu(self):
        self.log("Customer", "Can I see the menu?")
        self.log("Staff", "Sure! We can do five kinds of sandwiches:")
        self.ask_menu()

    def handle_recommendation(self):
        self.log("Customer", "Emm, what do you recommend?")
        self.log("Staff", "Our Double Sandwich is really popular! But feel free to check all of them:")
        self.ask_menu()

    def ask_menu(self):
        for w in self.action_frame.winfo_children(): w.destroy()
        for name in self.recipes.keys():
            tk.Button(self.action_frame, text=name, command=lambda n=name: self.cook(n), font=("DejaVu Sans", 18), width=20, pady=10).pack(pady=5)

    def draw_ingredient(self, type, layer_index, total_layers):
        """Draw a sandwich ingredient layer on the canvas.

        Positioning: the sandwich is assembled in the upper half of the canvas
        with even vertical spacing between layers. `layer_index` 0 is the bottom
        layer and increases upwards.
        """
        spacing = 80
        # Shift cy upward so the sandwich is drawn above the chef image
        cx, cy = 300, 450 - (layer_index * spacing)
        w, h = 350, 60

        if type == "Bread":
            # Draw a triangular/rounded bread slice as a polygon for stylized look
            points = [cx - w / 2, cy + h / 2, cx + w / 2, cy + h / 2, cx, cy - h / 2]
            self.canvas.create_polygon(points, fill=self.colors["Bread"], outline="#F5E7C6", width=3)
        elif type == "Ham":
            # Draw ham as a semi-oval arc
            self.canvas.create_arc(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, start=0, extent=180, fill=self.colors["Ham"], outline="#FFAAB8", width=2)
        elif type == "Lettuce":
            # Lettuce has a small horizontal jitter to make layers look organic
            offset = random.randint(-20, 20)
            points = [cx - w / 2 + offset, cy + h / 2, cx + w / 2 + offset, cy + h / 2, cx + offset, cy - h / 2]
            self.canvas.create_polygon(points, fill=self.colors["Lettuce"], outline="#A3D78A", width=2)

    def cook(self, name):
        for w in self.action_frame.winfo_children(): w.destroy()
        self.log("Customer", f"I'll have {name}.")
        self.log("Staff", "Coming right up! Look at the kitchen...")
        ingredients = self.recipes[name]
        self.animate_step(ingredients, 0, name)

    def animate_step(self, ingredients, i, name):
        if i < len(ingredients):
            self.draw_ingredient(ingredients[i], i, len(ingredients))
            self.root.after(800, lambda: self.animate_step(ingredients, i + 1, name))
        else:
            self.root.after(5000, lambda: self.complete(name))

    def complete(self, name):
        self.log("Staff", f"Your {name} is ready! Please enjoy your meal, welcome next time!")
        messagebox.showinfo("Ready", f"Enjoy your {name}!")
        for w in self.action_frame.winfo_children(): w.destroy()
        tk.Label(self.action_frame, text="ORDER FINISHED", font=self.font_btn, fg="gray").pack()

if __name__ == "__main__":
    root = tk.Tk()
    try:
        root.tk.call('tk', 'scaling', 2.0)
    except:
        pass
    app = ModernSandwichKitchen(root)
    root.mainloop()