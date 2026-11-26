import csv
import socket
import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
CSV_FILE = "encoder_log.csv"

MODE_NAMES = {
    0: "FOLLOW",
    1: "ADVANCE",
    2: "TURN",
    3: "SEARCH",
}

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Dang lang nghe UDP tren cong {UDP_PORT}...")
    print("Hay rut 3 day ADC2 (GPIO 25, 26, 27) tren xe, sau do gui lenh 'udp' hoac giu nut BOOT 1 giay.\n")

    records = []
    expected_count = None

    while True:
        data, addr = sock.recvfrom(1024)
        line = data.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        parts = line.split(",")
        tag = parts[0]

        if tag == "START" and len(parts) >= 2:
            expected_count = int(parts[1])
            records.clear()
            print(f"=== BAT DAU NHAN DU LIEU TU {addr[0]} (Tong mau: {expected_count}) ===")
            print(f"{'STT':>5} | {'Time(s)':>8} | {'SpdL(xung/50ms)':>15} | {'SpdR(xung/50ms)':>15} | {'SpdL(xung/s)':>12} | {'SpdR(xung/s)':>12} | {'Err':>6} | {'Mode'}")
            print("-" * 92)

        elif tag == "DATA" and len(parts) >= 7:
            idx = int(parts[1])
            t_ms = int(parts[2])
            spd_l = int(parts[3])
            spd_r = int(parts[4])
            err = int(parts[5])
            mode = int(parts[6])

            # Tránh ghi trùng nếu gói bị gửi lặp
            if records and records[-1]["idx"] == idx:
                continue

            t_s = t_ms / 1000.0
            pps_l = spd_l * 20   # 50ms * 20 = 1 giây
            pps_r = spd_r * 20
            mode_str = MODE_NAMES.get(mode, str(mode))

            records.append({
                "idx": idx,
                "t_ms": t_ms,
                "t_s": t_s,
                "spd_l_50ms": spd_l,
                "spd_r_50ms": spd_r,
                "pps_l": pps_l,
                "pps_r": pps_r,
                "err": err,
                "mode": mode_str,
            })

            print(f"{idx:5d} | {t_s:8.3f} | {spd_l:15d} | {spd_r:15d} | {pps_l:12d} | {pps_r:12d} | {err:6d} | {mode_str}")

        elif tag == "END":
            print("-" * 92)
            print(f"=== HOAN TAT! Da nhan {len(records)} / {expected_count} mau ===")
            break

    sock.close()

    if not records:
        print("Khong nhan duoc mau du lieu nao.")
        return

    # Lưu ra file CSV
    with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["idx", "t_ms", "t_s", "spd_l_50ms", "spd_r_50ms", "pps_l", "pps_r", "err", "mode"],
        )
        writer.writeheader()
        writer.writerows(records)
    print(f"Da luu file CSV: {CSV_FILE}")

    # Vẽ đồ thị vận tốc 2 bánh
    t_vals = [r["t_s"] for r in records]
    pps_l_vals = [r["pps_l"] for r in records]
    pps_r_vals = [r["pps_r"] for r in records]
    err_vals = [r["err"] for r in records]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax1.plot(t_vals, pps_l_vals, label="Bánh TRÁI (xung/giây)", color="tab:blue", linewidth=1.5)
    ax1.plot(t_vals, pps_r_vals, label="Bánh PHẢI (xung/giây)", color="tab:orange", linewidth=1.5)
    ax1.set_ylabel("Vận tốc thực (xung/s)")
    ax1.set_title("Đồ thị vận tốc thực Encoder 2 bánh & Sai số dò line")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend()

    ax2.plot(t_vals, err_vals, label="Sai số vị trí line (err)", color="tab:green", linewidth=1.2)
    ax2.axhline(0, color="gray", linestyle=":", linewidth=1)
    ax2.set_xlabel("Thời gian chạy (giây)")
    ax2.set_ylabel("Sai số (err)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()