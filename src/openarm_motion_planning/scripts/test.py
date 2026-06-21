import csv
import numpy as np


def generate_wave_csv(filename="wave_right.csv"):
    # Cấu trúc file: segment_idx, left_joint1..7, right_joint1..7
    header = ["segment_idx"] + \
             [f"openarm_left_joint{i + 1}" for i in range(7)] + \
             [f"openarm_right_joint{i + 1}" for i in range(7)]

    # Tay trái luôn giữ ở vị trí Home (0.0)
    left_home = [0.0] * 7

    # --- ĐỊNH NGHĨA CÁC TƯ THẾ CỦA TAY PHẢI ---
    # 1. Vị trí Home
    right_home = [0.0] * 7

    # 2. Đưa tay lên cao (Raise Hand)
    # J1: Xoay vai ra ngoài, J2: Nâng vai lên, J4: Gập cùi chỏ, J5: Xoay cổ tay hướng ra trước
    right_raise = [-0.5, -0.8, 0.0, 1.5, 1.57, 0.0, 0.0]

    # 3. Vẫy vào trong (Gập cổ tay J6)
    right_wave_in = [-0.5, -0.8, 0.0, 1.5, 1.57, -0.7, 0.0]

    # 4. Vẫy ra ngoài (Ngửa cổ tay J6)
    right_wave_out = [-0.5, -0.8, 0.0, 1.5, 1.57, 0.7, 0.0]

    # --- TẠO QUỸ ĐẠO THEO TỪNG PHÂN ĐOẠN (SEGMENTS) ---
    # Code chính của bạn đọc segment từ 0 đến 4.
    # Nếu là task thứ 2 trở đi, nó sẽ bỏ qua 0 và 1, bắt đầu chạy từ 2.

    trajectories = []

    # Segment 0: Home -> Raise
    trajectories.append((0, right_home))
    trajectories.append((0, right_raise))

    # Segment 1: Giữ nguyên Raise (Vì code của bạn nhảy qua đoạn này nếu task > 0)
    trajectories.append((1, right_raise))
    trajectories.append((1, right_raise))

    # Segment 2: Bắt đầu vẫy (Raise -> Wave In)
    trajectories.append((2, right_raise))
    trajectories.append((2, right_wave_in))

    # Segment 3: Vẫy sang bên kia (Wave In -> Wave Out)
    trajectories.append((3, right_wave_in))
    trajectories.append((3, right_wave_out))

    # Segment 4: Vẫy thêm 1 nhịp nữa rồi về lại Raise (Chuẩn bị cho task tiếp theo)
    trajectories.append((4, right_wave_out))
    trajectories.append((4, right_wave_in))
    trajectories.append((4, right_wave_out))
    trajectories.append((4, right_raise))  # Kết thúc ở Raise

    # Ghi ra file CSV
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for seg_idx, right_pose in trajectories:
            row = [seg_idx] + left_home + right_pose
            writer.writerow(row)

    print(f" Đã tạo thành công file: {filename}")


if __name__ == "__main__":
    generate_wave_csv()