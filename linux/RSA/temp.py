import tkinter as tk
from tkinter import font

# -------------------------------------------------
# 1. Define a clean, reusable font family
# -------------------------------------------------
# Try Segoe UI first, fall back to Inter (Linux/macOS), then to a generic sans-serif
BASE_FONT = "Segoe UI"
if BASE_FONT not in font.families():
    # Ubuntu: sudo apt install fonts-inter
    BASE_FONT = "Inter"
if BASE_FONT not in font.families():
    BASE_FONT = "Helvetica"   # ultimate safe fallback

# -------------------------------------------------
# 2. Create font objects (you can reuse them)
# -------------------------------------------------
FONT_TITLE   = (BASE_FONT, 20, "bold")
FONT_HEADER  = (BASE_FONT, 14, "bold")
FONT_NORMAL  = (BASE_FONT, 11)
FONT_SMALL   = (BASE_FONT, 10)
FONT_MONO    = ("Courier New", 10)   # for logs / code

# -------------------------------------------------
# 3. Example UI (copy-paste into your app)
# -------------------------------------------------
root = tk.Tk()
root.title("Professional UI Demo")
root.geometry("600x400")
root.configure(bg="#f5f6fa")

tk.Label(root, text="Robot Rehabilitation System", font=FONT_TITLE,
         bg="#f5f6fa", fg="#2c3e50").pack(pady=20)

tk.Label(root, text="Drag Mode Control", font=FONT_HEADER,
         bg="#f5f6fa", fg="#2c3e50").pack(anchor="w", padx=30)

tk.Button(root, text="Enable Drag Mode", font=FONT_NORMAL,
          bg="#3498db", fg="white", relief="flat",
          padx=20, pady=10).pack(pady=10)

root.mainloop()