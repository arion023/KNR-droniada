import tkinter as tk

def on_key_press(event):
    """Handle key press events."""
    key = event.keysym
    if key == 'w':
        print("Forward")
    elif key == 's':
        print("Backward")
    elif key == 'space':
        print("Stop")

# Set up the tkinter GUI
root = tk.Tk()
root.title("Motor Control Test")

# Bind keys to functions
root.bind('<KeyPress>', on_key_press)

# Instructions label
instructions = tk.Label(root, text="Press 'W' to move forward, 'S' to move backward, 'Space' to stop")
instructions.pack()

# Run the tkinter main loop
root.mainloop()
