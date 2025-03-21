import tkinter as tk

class WindowExample:
    def __init__(self, root):
        self.root = root
        self.root.title("Main Window")

        # Main Window Button
        self.open_window_button = tk.Button(self.root, text="Open New Window", command=self.open_new_window)
        self.open_window_button.pack(padx=20, pady=20)

    def open_new_window(self):
        # Create a new window
        self.new_window = tk.Toplevel(self.root)
        self.new_window.title("New Window")

        # Button to return to the main window
        self.back_button = tk.Button(self.new_window, text="Back to Main Window", command=self.new_window.destroy)
        self.back_button.pack(padx=20, pady=20)

def main():
    root = tk.Tk()
    app = WindowExample(root)
    root.mainloop()

if __name__ == "__main__":
    main()