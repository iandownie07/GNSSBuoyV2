import asyncio
import csv
import datetime
import os
from bleak import BleakClient

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import collections

# ---------------- CONFIG ----------------
ADDRESS = "12764788-CC99-3A0B-1607-43255AC43AC0"
CHAR_UUID = "0000FFE1-0000-1000-8000-00805F9B34FB"
CSV_FILENAME = "rotation_test_values.csv"

MAX_POINTS = 300
PLOT_REFRESH_S = 0.1
# ----------------------------------------

# Rolling buffers
times = collections.deque(maxlen=MAX_POINTS)
v1 = collections.deque(maxlen=MAX_POINTS)
v2 = collections.deque(maxlen=MAX_POINTS)
v3 = collections.deque(maxlen=MAX_POINTS)


async def main():
    print("CSV path:", os.path.abspath(CSV_FILENAME))

    # ---------- Matplotlib Setup ----------
    plt.ion()  # interactive mode ON
    fig, ax = plt.subplots(figsize=(10, 5))

    line1, = ax.plot([], [], label="N")
    line2, = ax.plot([], [], label="E")
    line3, = ax.plot([], [], label="D")

    ax.set_xlabel("Time")
    ax.set_ylabel("Velocity (m/s)")
    ax.grid(True)
    ax.legend()

    # Time axis formatting
    locator = mdates.AutoDateLocator(minticks=3, maxticks=8)
    formatter = mdates.AutoDateFormatter(locator)
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(formatter)

    fig.autofmt_xdate()
    plt.show()

    # ---------- Plot Update Function ----------
    def update_plot():
        if len(times) == 0:
            return

        x = list(times)

        line1.set_data(x, list(v1))
        line2.set_data(x, list(v2))
        line3.set_data(x, list(v3))

        ax.relim()
        ax.autoscale_view()

        # redraw efficiently
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    # ---------- Plot Task ----------
    async def plot_task():
        while True:
            update_plot()
            await asyncio.sleep(PLOT_REFRESH_S)

    # ---------- BLE Task ----------
    async def ble_task():
        with open(CSV_FILENAME, "a", newline="") as csv_file:
            writer = csv.writer(csv_file)

            if csv_file.tell() == 0:
                writer.writerow(["Timestamp", "N", "E", "D"])
                csv_file.flush()

            def handle_data(sender, data):
                text = data.decode(errors="ignore").strip()
                timestamp = datetime.datetime.now()

                parts = [p.strip() for p in text.split(",")]
                if len(parts) != 3:
                    print("Invalid:", text)
                    return

                try:
                    n = float(parts[0]) / 1000
                    e = float(parts[1]) / 1000
                    d = float(parts[2]) / 1000
                except ValueError:
                    print("Non-numeric:", parts)
                    return

                # Log CSV
                writer.writerow([timestamp.isoformat(), n, e, d])
                csv_file.flush()

                # Update buffers
                times.append(timestamp)
                v1.append(n)
                v2.append(e)
                v3.append(d)

                print(timestamp.time(), n, e, d)

            async with BleakClient(ADDRESS, timeout=60.0) as client:
                print("Connected:", client.is_connected)

                await asyncio.sleep(1.0)

                await client.start_notify(CHAR_UUID, handle_data)
                print("Listening... Ctrl+C to stop")

                try:
                    while True:
                        await asyncio.sleep(1)
                finally:
                    await client.stop_notify(CHAR_UUID)
                    print("BLE stopped.")

    # ---------- Run Both Tasks ----------
    plot_t = asyncio.create_task(plot_task())
    ble_t = asyncio.create_task(ble_task())

    try:
        await ble_t
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        plot_t.cancel()
        ble_t.cancel()
        await asyncio.gather(plot_t, ble_t, return_exceptions=True)
        print("Done. CSV saved:", CSV_FILENAME)


# ---------------- SCRIPT ENTRY ----------------
if __name__ == "__main__":
    asyncio.run(main())

