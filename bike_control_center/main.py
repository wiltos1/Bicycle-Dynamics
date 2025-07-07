
import serial
import keyboard
import time
import tkinter as tk
from tkinter import messagebox, ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import os
import csv
import threading
import queue
import sys
from collections import defaultdict

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# --- STATE ---
ser = None
serial_queue = queue.Queue()
recording_flag = threading.Event()
data_columns = defaultdict(list)
is_old_file_loaded = False  # Flag to track if an old file is loaded

# --- Serial Communication ---
def send_command(cmd):
    if ser and ser.is_open:
        ser.write((cmd + '\n').encode())
        print(f">> Sent: {cmd}")
    else:
        print("❌ Serial not connected.")

def send_discard_command(filename):
    send_command(f"DELETE:{filename}")

def read_serial():
    current_filename = None
    while True:
        try:
            if ser and ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                print(f"<< {line}")
                if line.startswith("FILENAME:"):
                    # Strip leading slash from bike filename
                    raw = line.split(':',1)[1]
                    fname = raw.lstrip('/')
                    current_filename = fname
                    serial_queue.put(("FILENAME", fname))
                    data_columns.clear()
                elif line.startswith("COLUMN:"):
                    try:
                        _, rest = line.split(':', 1)
                        key, value = rest.split(',', 1)
                        value = float(value)
                        data_columns[key].append(value)
                        print(f"📊 Added {key}: {value} to data_columns")

                        # Check if yaw can be computed
                        if all(k in data_columns for k in ('frontOriX', 'rearOriX', 'time_us')):
                            i = len(data_columns['yaw'])  # Next yaw index
                            while (
                                i < len(data_columns['frontOriX']) and
                                i < len(data_columns['rearOriX']) and
                                i < len(data_columns['time_us'])
                            ):
                                f = data_columns['frontOriX'][i]
                                r = data_columns['rearOriX'][i]
                                yaw_val = f - r
                                data_columns['yaw'].append(yaw_val)
                                print(f"🌀 Computed yaw[{i}]: {yaw_val}")
                                i += 1
                    except Exception as e:
                        print(f"⚠️ Bad column line: {line} — {e}")
                elif "DONE" in line:
                    serial_queue.put(("DATA_COMPLETE", current_filename))
        except Exception as e:
            print(f"Serial read error: {e}")
        time.sleep(0.05)

def serial_listener_thread():
    threading.Thread(target=read_serial, daemon=True).start()

# --- Keyboard Controls (toggle on '1') ---
def keyboard_controls_thread(selected_filename, entry_name, update_buttons):
    def loop():
        warning_shown = False
        while True:
            if keyboard.is_pressed('1'):
                if recording_flag.is_set():
                    send_command("STOP")
                    recording_flag.clear()
                else:
                    global is_old_file_loaded
                    if is_old_file_loaded:
                        if not warning_shown:
                            messagebox.showwarning(
                                "Old File Open",
                                "Please close the current file before starting a new recording."
                            )
                            warning_shown = True
                        keyboard.wait('1')
                        continue
                    if selected_filename.get() and not entry_name.get().strip():
                        if not warning_shown:
                            messagebox.showwarning(
                                "Unsaved Ride",
                                "Please save or discard the current ride before starting a new recording."
                            )
                            warning_shown = True
                        keyboard.wait('1')
                        continue
                    warning_shown = False
                    send_command("START")
                    recording_flag.set()
                    is_old_file_loaded = False  # Reset flag for new recording
                    update_buttons()  # Restore Save and Discard
                keyboard.wait('1')
            if keyboard.is_pressed('esc'):
                break
            time.sleep(0.1)
    threading.Thread(target=loop, daemon=True).start()

# --- File loader ---
def load_file(path, entry_name, text_notes):
    data_columns.clear()
    try:
        with open(path) as fh:
            reader = csv.DictReader(fh)
            # Get the first row to extract Ride Name and Notes
            first_row = next(reader)
            ride_name = first_row.get('Ride Name', '')
            notes = first_row.get('Notes', '')
            # Update UI with Ride Name and Notes
            entry_name.delete(0, tk.END)
            entry_name.insert(0, ride_name)
            text_notes.delete('1.0', tk.END)
            text_notes.insert('1.0', notes)
            # Reset file pointer to start for data reading
            fh.seek(0)
            reader = csv.DictReader(fh)
            for row in reader:
                try:
                    data_columns['time_us'].append(float(row['Time (s)'])*1e6)
                    data_columns['yaw'].append(float(row.get('Yaw', '0')))
                    data_columns['frontOriZ'].append(float(row.get('Front Roll', '0')))
                    data_columns['rearOriZ'].append(float(row.get('Rear Roll', '0')))
                except ValueError as e:
                    print(f"⚠️ Skipping invalid row in {path}: {row} — {e}")
            print(f"Loaded data from {path}: {len(data_columns['time_us'])} rows")
    except Exception as e:
        print(f"Error loading file: {e}")
        messagebox.showerror("Error", f"Failed to load file: {e}")

# --- GUI & Logic ---
def launch_control_center():
    app = tk.Tk()
    app.title("Ride Control Center")
    app.state('zoomed')

    selected_filename = tk.StringVar()
    recording_label = tk.StringVar()

    # Matplotlib setup
    fig, ax = plt.subplots(figsize=(10,6))
    canvas = FigureCanvasTkAgg(fig, master=None)

    # Checkbox vars
    var_yaw = tk.BooleanVar(value=True)
    var_fr = tk.BooleanVar(value=True)
    var_rr = tk.BooleanVar(value=True)

    def update_time_plot():
        ax.clear()
        times = data_columns.get('time_us', [])
        if not times:
            canvas.draw()
            return

        base = times[0]
        ts = [(t - base) / 1e6 for t in times]

        print(f"🟦 Plotting time_us: {len(ts)} samples")
        if var_yaw.get():
            print(f"🟨 Yaw enabled: {len(data_columns['yaw'])} samples")
        if var_fr.get():
            print(f"🟩 Front Roll: {len(data_columns['frontOriZ'])} samples")
        if var_rr.get():
            print(f"🟥 Rear Roll: {len(data_columns['rearOriZ'])} samples")

        if var_yaw.get() and data_columns.get('yaw'):
            min_len = min(len(ts), len(data_columns['yaw']))
            print(f"🌀 Plotting Yaw with {min_len} points")
            ax.plot(ts[:min_len], data_columns['yaw'][:min_len], label='Yaw')

        if var_fr.get() and data_columns.get('frontOriZ'):
            min_len = min(len(ts), len(data_columns['frontOriZ']))
            ax.plot(ts[:min_len], data_columns['frontOriZ'][:min_len], label='Front Roll')

        if var_rr.get() and data_columns.get('rearOriZ'):
            min_len = min(len(ts), len(data_columns['rearOriZ']))
            ax.plot(ts[:min_len], data_columns['rearOriZ'][:min_len], label='Rear Roll')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (°)')
        ax.legend()
        canvas.draw()


    def update_recording_label():
        recording_label.set("🔴 Recording in Progress" if recording_flag.is_set() else "")
        try:
            app.after(500, update_recording_label)
        except tk.TclError:
            pass

    def disable_ui():
        print("🔒 Disabling UI and showing loading animation. States - Save: {}, Discard: {}, Close: {}".format(
            btn_save['state'], btn_discard['state'], btn_close['state']))
        btn_save.config(state='disabled')
        btn_discard.config(state='disabled')
        btn_close.config(state='disabled')
        for cb in [chk_yaw, chk_fr, chk_rr]:
            cb.config(state='disabled')
        loading_label.place(relx=0.5, rely=0.5, anchor='center')
        loading_label.lift()
        app.update_idletasks()

    def enable_ui():
        print("🔓 Enabling UI and hiding loading animation. States - Save: {}, Discard: {}, Close: {}".format(
            btn_save['state'], btn_discard['state'], btn_close['state']))
        btn_save.config(state='normal' if not is_old_file_loaded else 'disabled')
        btn_discard.config(state='normal' if not is_old_file_loaded else 'disabled')
        btn_close.config(state='normal' if is_old_file_loaded else 'disabled')
        for cb in [chk_yaw, chk_fr, chk_rr]:
            cb.config(state='normal')
        loading_label.place_forget()
        app.update_idletasks()

    def check_serial_queue():
        try:
            while not serial_queue.empty():
                msg, content = serial_queue.get()
                if msg == "FILENAME":
                    selected_filename.set(content)
                    disable_ui()
                elif msg == "DATA_COMPLETE":
                    app.after(500, lambda: [enable_ui(), update_time_plot()])
        except:
            pass
        try:
            app.after(100, check_serial_queue)
        except tk.TclError:
            pass

    def update_buttons():
        if is_old_file_loaded:
            btn_save.pack_forget()
            btn_discard.pack_forget()
            btn_close.pack(side=tk.LEFT, padx=5)
        else:
            btn_close.pack_forget()
            btn_save.pack(side=tk.LEFT, padx=5)
            btn_discard.pack(side=tk.LEFT)

    def close_file():
        if not selected_filename.get():
            return
        path = os.path.join('logs', selected_filename.get())
        new_ride_name = entry_name.get().strip()
        new_notes = text_notes.get('1.0', tk.END).strip()
        try:
            with open(path, 'r') as fh:
                reader = csv.DictReader(fh)
                rows = list(reader)  # Read all rows
            with open(path, 'w', newline='') as fh:
                writer = csv.DictWriter(fh, fieldnames=reader.fieldnames)
                writer.writeheader()
                for row in rows:
                    row['Ride Name'] = new_ride_name
                    row['Notes'] = new_notes
                    writer.writerow(row)
            print(f"📝 Updated {path} with new Ride Name and Notes")
        except Exception as e:
            print(f"Error updating file: {e}")
            messagebox.showerror("Error", f"Failed to update file: {e}")
        data_columns.clear()
        entry_name.delete(0, tk.END)
        text_notes.delete('1.0', tk.END)
        selected_filename.set('')
        update_time_plot()
        global is_old_file_loaded
        is_old_file_loaded = False
        update_buttons()

    # Layout frames
    left = tk.Frame(app)
    left.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
    right = tk.Frame(app)
    right.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH, padx=10, pady=10)

    # Past rides table
    tk.Label(left, text="📂 Past Rides").pack()
    table = ttk.Treeview(left, columns=('file','name'), show='headings')
    table.heading('file', text='File')
    table.heading('name', text='Ride Name')
    table.pack(fill=tk.BOTH, expand=True)

    def refresh_table():
        for i in table.get_children():
            table.delete(i)
        os.makedirs('logs', exist_ok=True)
        for f in os.listdir('logs'):
            if f.endswith('.csv'):
                with open(os.path.join('logs',f)) as fh:
                    reader = csv.DictReader(fh)
                    try:
                        row = next(reader)
                        table.insert('', 'end', values=(f, row.get('Ride Name','')))
                    except StopIteration:
                        print(f"⚠️ Empty or invalid CSV: {f}")
                    except Exception as e:
                        print(f"⚠️ Error reading {f}: {e}")

    def load_selected(e):
        sel = table.focus()
        if not sel:
            return
        f, _ = table.item(sel)['values']
        selected_filename.set(f)
        load_file(os.path.join('logs',f), entry_name, text_notes)
        update_time_plot()
        global is_old_file_loaded
        is_old_file_loaded = True
        update_buttons()
        enable_ui()  # Ensure UI is enabled, including Close button
        print("🔍 load_selected completed. is_old_file_loaded: {}, Close state: {}".format(is_old_file_loaded, btn_close['state']))

    # Bind double click for loading
    table.bind('<Double-1>', load_selected)

    # Right panel widgets
    tk.Label(right, textvariable=recording_label, font=("Arial",12), fg="red").pack(anchor='w')
    tk.Label(right, text="Press '1' to start/stop recording.", font=("Arial",15), fg="black").pack(anchor='w', pady=(5,10))

    # Plot canvas
    canvas = FigureCanvasTkAgg(fig, master=right)
    canvas.draw()
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(pady=10, expand=True, fill=tk.BOTH)

    # Loading label
    loading_label = ttk.Label(canvas_widget, text="⏳ Loading…", font=("Arial",14), foreground="blue")
    loading_label.place_forget()

    # Checkboxes
    cbf = tk.Frame(right)
    cbf.pack()
    chk_yaw = tk.Checkbutton(cbf, text='Yaw', variable=var_yaw, command=update_time_plot)
    chk_fr  = tk.Checkbutton(cbf, text='Front Roll', variable=var_fr, command=update_time_plot)
    chk_rr  = tk.Checkbutton(cbf, text='Rear Roll', variable=var_rr, command=update_time_plot)
    chk_yaw.pack(side=tk.LEFT)
    chk_fr.pack(side=tk.LEFT)
    chk_rr.pack(side=tk.LEFT)

    file_label_frame = tk.Frame(right)
    file_label_frame.pack(anchor='w', pady=(10, 0))
    tk.Label(file_label_frame, text="Active file:", font=("Arial", 10)).pack(side=tk.LEFT)
    tk.Label(file_label_frame, textvariable=selected_filename, font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(5,0))

    # Ride Name & Notes
    tk.Label(right, text="Ride Name:").pack(anchor='w')
    entry_name = tk.Entry(right, width=50)
    entry_name.pack()
    tk.Label(right, text="Notes:").pack(anchor='w')
    text_notes = tk.Text(right, height=4, width=60)
    text_notes.pack()

    # Save & Discard buttons
    btnf = tk.Frame(right)
    btnf.pack(pady=10)
    btn_save = tk.Button(btnf, text='✅ Save', bg='green', fg='white', command=lambda: on_save(entry_name, text_notes, selected_filename, refresh_table, update_time_plot))
    btn_discard = tk.Button(btnf, text='🗑️ Discard', bg='red', fg='white', command=lambda: on_discard(entry_name, text_notes, selected_filename, update_time_plot))
    btn_close = tk.Button(btnf, text='🚪 Close', bg='gray', fg='white', command=close_file)
    btn_save.pack(side=tk.LEFT, padx=5)
    btn_discard.pack(side=tk.LEFT)
    btn_close.pack_forget()  # Initially hidden

    # Close handling
    def on_close():
        app.destroy()
        sys.exit(0)
    app.protocol("WM_DELETE_WINDOW", on_close)

    # Start threads & loops
    keyboard_controls_thread(selected_filename, entry_name, update_buttons)
    refresh_table()
    app.after(100, check_serial_queue)
    app.after(500, update_recording_label)
    app.mainloop()

# --- Save & Discard outside launch to capture UI elements ---
def on_save(entry, notes, sel_var, refresh_fn, plot_fn):
    rn = entry.get().strip()
    if not rn:
        messagebox.showwarning("Missing Ride Name","Enter ride name before saving.")
        return
    path = os.path.join('logs', sel_var.get())
    os.makedirs('logs', exist_ok=True)
    base = data_columns['time_us'][0] if data_columns['time_us'] else 0
    print(f"📝 Saving to {path} with data_columns: {dict(data_columns)}")
    try:
        with open(path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(["Time (s)","Yaw","Front Roll","Rear Roll","Ride Name","Notes"])
            for i, t0 in enumerate(data_columns['time_us']):
                ts = (t0-base)/1e6
                yaw = data_columns['frontOriX'][i] - data_columns['rearOriX'][i] if i < len(data_columns['frontOriX']) and i < len(data_columns['rearOriX']) else 0.0
                fr = data_columns['frontOriZ'][i] if i < len(data_columns['frontOriZ']) else 0.0
                rr = data_columns['rearOriZ'][i] if i < len(data_columns['rearOriZ']) else 0.0
                w.writerow([ts, yaw, fr, rr, rn, notes.get('1.0', tk.END).strip()])
        messagebox.showinfo("Saved", f"Saved to {path}")
        refresh_fn()
    except Exception as e:
        messagebox.showerror("Error", str(e))
    data_columns.clear()
    entry.delete(0, tk.END)
    notes.delete('1.0', tk.END)
    sel_var.set('')
    plot_fn()

def on_discard(entry, notes, sel_var, plot_fn):
    send_discard_command(sel_var.get())
    messagebox.showinfo("Discarded","Discard command sent.")
    data_columns.clear()
    entry.delete(0, tk.END)
    notes.delete('1.0', tk.END)
    sel_var.set('')
    plot_fn()

if __name__ == '__main__':
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
    except:
        sys.exit(1)
    serial_listener_thread()
    launch_control_center()
