# Hướng dẫn thu thập dữ liệu bằng kính VR — Robot OpenArm

Tài liệu này hướng dẫn cách thu thập dữ liệu huấn luyện cho robot OpenArm bằng cách điều khiển
cánh tay robot qua kính VR. Dữ liệu thu được sẽ dùng để huấn luyện mô hình AI điều khiển robot tự động
sau này, nên **thao tác càng chuẩn, càng đều đặn thì mô hình học được càng tốt**.

Tài liệu gồm 2 phần:

- **Phần 1 — Khởi động hệ thống**: dành cho kỹ sư/người phụ trách kỹ thuật, làm **trước** mỗi buổi thu.
- **Phần 2 — Vận hành thu dữ liệu**: dành cho người đeo kính VR thao tác (người thu data),
  làm **lặp lại nhiều lần** trong buổi thu.

Đọc kỹ **Phần 3 — An toàn lao động** trước khi thao tác lần đầu tiên.

---

## Nhiệm vụ cần thực hiện (task)

- **Tên nhiệm vụ**: nhấc — đặt vật (pick and place).
- **Vật thể**: hộp *[ĐIỀN SAU: mô tả/kích thước/màu hộp cụ thể]*.
- **Vị trí A (điểm gắp)**: *[ĐIỀN SAU: mô tả vị trí]*.
- **Vị trí B (điểm đặt)**: *[ĐIỀN SAU: mô tả vị trí đích]*.
- **Thao tác**: bấm nút trên tay cầm VR để robot di chuyển
  đến vị trí A / vị trí B; việc gắp vật và đặt vật vẫn do người thao tác **tự điều khiển cánh tay
  (7 khớp mỗi bên) và bàn tay bằng kính VR (teleop)** sau khi robot đã đến nơi.

[ẢNH: toàn cảnh khu vực thao tác — vị trí A, vị trí B, robot, camera]

---

## Phần 1 — Khởi động hệ thống (người phụ trách kỹ thuật/người thu data)

Hệ thống cần 4 nhóm chương trình chạy cùng lúc: (1) cấu hình + điều khiển cánh tay thật (CAN),
(2) cầu nối kính VR (Dora bridge), (3) 3 camera RealSense, (4) ghi dữ liệu bằng `ros2 bag record`.
Dùng script có sẵn để mở cả 6 terminal (pane) cùng lúc thay vì mở tay từng terminal.

### Bước 1. Mở hệ thống

Mở 1 terminal, chạy:

```bash
bash ~/pnk/ws/scripts/record_session_tmux.sh
```

Lệnh này tự mở ra 6 khung màn hình (pane) chia sẵn:

- **Pane cánh tay (CAN)**: tự chạy lệnh cấu hình cả 2 CAN (`can_configure`) ngay khi mở, sau đó
  **gõ sẵn nhưng chưa chạy** lệnh `ros2 launch openarm_bringup ...` (bringup thật, không dùng
  fake hardware) — chờ xác nhận ở Bước 2 rồi mới bấm Enter.
- **Pane cầu nối VR (Dora bridge)**: lệnh `uv run dora run ...` — **đã gõ sẵn nhưng chưa chạy**,
  chờ xác nhận ở Bước 2.
- **3 pane camera** (`cam_head`, `cam_left`, `cam_right`): mỗi pane tự chạy ngay 1 lệnh
  `ros2 launch realsense2_camera rs_launch.py ...` riêng cho từng camera.
- **Pane ghi dữ liệu**: lệnh `ros2 bag record` ghi toàn bộ topic camera + trạng thái tay + tay
  gắp + `/tf` — **đã gõ sẵn nhưng chưa chạy**, chờ xác nhận ở Bước 2.

> **Lưu ý**: nếu khi script gõ lệnh bringup vào pane cánh tay mà đúng lúc đó terminal đang chờ
> nhập mật khẩu `sudo` cho `can_configure`, dòng lệnh gõ sẵn có thể bị "nuốt" mất. Nếu vào pane
> cánh tay mà không thấy dòng lệnh `ros2 launch openarm_bringup ...` đã gõ sẵn, chỉ cần gõ lại.

[ẢNH: màn hình tmux với 6 pane vừa mở]

### Bước 2. Kiểm tra từng pane trước khi bấm Enter

Thứ tự bấm Enter: **cánh tay → camera (tự chạy, chỉ cần kiểm tra) → cầu nối VR → ghi dữ liệu**.
Ghi dữ liệu là **1 file bag ghi liên tục cho cả buổi thu** (không tách theo từng lần gắp-đặt), nên
chỉ bấm Enter ở pane ghi dữ liệu **đúng 1 lần vào đầu buổi** và dừng (`Ctrl+C`) **đúng 1 lần vào
cuối buổi**.

**Không được bấm Enter ở pane cầu nối VR và pane ghi dữ liệu cho đến khi**:

- Pane cánh tay (CAN) đã bấm Enter chạy bringup, không còn báo lỗi màu đỏ, các controller đã ở
  trạng thái hoạt động (có thể kiểm tra thêm bằng `ros2 control list_controllers` ở một terminal
  khác — các dòng phải hiện `active`).
- Cả 3 pane camera (`cam_head`, `cam_left`, `cam_right`) đã lên hình, không còn báo lỗi kết nối
  camera liên tục.
- Sau đó mới bấm Enter ở pane cầu nối VR, và chờ pane này báo đã kết nối / đang nhận dữ liệu từ
  kính, không còn dòng lỗi kết nối liên tục.

Nếu 1 trong các pane báo lỗi và không tự hết sau khoảng 15–20 giây, **không tiếp tục** — xem
Phần 4 (xử lý sự cố) hoặc gọi người phụ trách kỹ thuật.

[ẢNH: log pane cánh tay khi đã sẵn sàng] [ẢNH: log pane camera khi đã sẵn sàng]
[ẢNH: log pane VR bridge khi đã sẵn sàng]

### Bước 3. Bắt đầu ghi dữ liệu

Khi chắc chắn cánh tay, 3 camera, và cầu nối VR đều đã ổn, bấm vào pane ghi dữ liệu, nhấn
**Enter** để bắt đầu `ros2 bag record`. File bag sẽ ghi liên tục từ lúc này cho đến khi bấm
`Ctrl+C` ở cuối buổi thu — không cần (và không nên) dừng/chạy lại giữa các lần gắp-đặt.

[ẢNH: màn hình pane ghi dữ liệu vừa khởi động xong]

---

## Phần 2 — Vận hành thu dữ liệu bằng kính VR (người thao tác)

Robot ở đây là **1 cụm** gồm phần base AMR (bánh xe) mang cả cánh tay — nhưng có 2 phần chuyển
động **tách biệt nhau về cách điều khiển**, đừng nhầm lẫn:

- **Phần base AMR (bánh xe)** — chỉ di chuyển giữa vị trí A và vị trí B khi bấm nút **A**/**B**
  trên tay cầm VR.
- **Cánh tay (7 khớp mỗi bên) và bàn tay Revo2** — do người thao tác **tự điều khiển hoàn toàn
  bằng kính VR (teleop)** để thực hiện việc gắp/đặt, **sau khi** robot đã dừng hẳn ở đúng vị trí.
  Nút **X**/**Y** không tự động gắp/đặt hộ bạn — chúng chỉ là tín hiệu đánh dấu "bắt đầu giai đoạn
  gắp"/"bắt đầu giai đoạn đặt" cho hệ thống, việc gắp/đặt thật sự vẫn là bạn tự làm bằng tay cầm VR.

> Việc ghi dữ liệu (`ros2 bag record`, Phần 1) vẫn chạy **liên tục suốt cả buổi thu**, không phụ
> thuộc vào các nút A/B/X/Y — các nút này chỉ điều khiển robot và đánh dấu giai đoạn, không dùng
> để bắt đầu/kết thúc ghi bag.

### Ý nghĩa nút bấm

| Nút | Ý nghĩa | Khi nào bấm |
|---|---|---|
| **A** | Robot tự di chuyển đến **vị trí A** (khu vực gắp) | Bắt đầu 1 lần gắp-đặt mới, sau khi đã đặt hộp vào vị trí A |
| **X** | Đánh dấu bắt đầu giai đoạn **gắp (pick)** — sau khi bấm, chờ 5 giây rồi mới tự dùng tay cầm VR để gắp | Ngay khi robot đã dừng hẳn ở vị trí A |
| **B** *(lần 1)* | Robot tự di chuyển đến **vị trí B** (khu vực đặt) | Sau khi bạn đã tự gắp xong vật bằng tay cầm VR và chờ đủ 5 giây |
| **Y** | Đánh dấu bắt đầu giai đoạn **đặt (place)** — sau khi bấm, chờ 5 giây rồi mới tự điều khiển tay cầm VR để đặt | Ngay khi robot đã dừng hẳn ở vị trí B |
| **B** *(lần 2)* | **Kết thúc 1 lần thu (episode)** hiện tại | Sau khi bạn đã tự đặt xong vật bằng tay cầm VR và chờ đủ 5 giây |

[ẢNH: tay cầm VR có đánh dấu rõ 4 nút A/B/X/Y]

> ⚠️ Nút **B** dùng 2 lần trong 1 chu trình với 2 ý nghĩa khác nhau (cho robot chạy đến B, và kết
> thúc episode) tùy đang ở bước nào trong quy trình — làm đúng thứ tự dưới đây, không bấm B ngoài
> 2 thời điểm này để tránh robot di chuyển ngoài ý muốn.

### Quy trình cho MỖI lần gắp-đặt (1 episode)

1. Đặt hộp vào **vị trí A**.
2. Đội kính VR, cầm 2 tay cầm điều khiển, đứng đúng tư thế thao tác, xác nhận không có ai/vật cản
   trên đường đi của robot lẫn trong tầm với của cánh tay (xem Phần 3 — An toàn).
3. Bấm nút **A** — robot tự di chuyển đến vị trí A. Chờ đến khi robot dừng hẳn.
4. Bấm nút **X**.
5. Chờ 5 giây.
6. Tự điều khiển cánh tay + bàn tay bằng kính VR để **gắp** hộp (thao tác teleop bình thường).
7. Sau khi gắp xong, chờ 5 giây.
8. Bấm nút **B** — robot tự di chuyển đến vị trí B. Chờ đến khi robot dừng hẳn.
9. Bấm nút **Y**.
10. Chờ 5 giây.
11. Tự điều khiển cánh tay + bàn tay bằng kính VR để **đặt** hộp xuống (thao tác teleop bình thường).
12. Sau khi đặt xong, chờ 5 giây.
13. Bấm nút **B** — kết thúc lần thu (episode) này.
14. Bê vật vừa đặt xuống trở lại **vị trí A** bằng tay (thao tác thủ công, không qua VR) để chuẩn
    bị cho lần tiếp theo.
    - **Nên thay đổi nhẹ** vị trí/hướng đặt hộp mỗi lần (không đặt y hệt lần trước) — việc này giúp
      dữ liệu đa dạng hơn, mô hình học được tổng quát hơn thay vì chỉ nhớ một tư thế duy nhất.
15. Lặp lại từ bước 3 (tóm tắt: `A → X → 5s → [tự gắp bằng VR] → 5s → B → Y → 5s → [tự đặt bằng
    VR] → 5s → B`) cho đến khi đủ số lần yêu cầu cho buổi thu (hỏi kỹ sư phụ trách con số cụ thể,
    ví dụ 70 lần).
16. Khi đã thu đủ, báo cho kỹ sư phụ trách để dừng ghi dữ liệu (kỹ sư bấm `Ctrl+C` ở pane ghi dữ
    liệu) — người thao tác **không tự dừng** hệ thống.

[ẢNH: minh họa 1 chu trình A (robot di chuyển) → X → gắp bằng VR → B (robot di chuyển) → Y → đặt bằng VR → B]

### Vài lưu ý giúp dữ liệu tốt hơn

- Chỉ bấm **A**/**B** khi chắc chắn đường đi của robot không có người/vật cản — robot sẽ tự di
  chuyển ngay sau khi bấm.
- Bấm **X**/**Y** ngay khi robot dừng hẳn, rồi chờ đủ 5 giây ổn định trước khi bắt đầu gắp/đặt —
  đừng bắt đầu teleop khi robot còn đang rung/chưa dừng hẳn, dữ liệu teleop sẽ không sạch.
- Việc **gắp/đặt vẫn là bạn tự điều khiển tay cầm VR như trước** — cố gắng **đi đường tương tự nhau** giữa
  các lần (không cố tình đi vòng vèo hay dừng giữa chừng quá lâu), tốc độ tự nhiên, không cần quá
  nhanh hay quá chậm.
- **Vị trí đặt hộp ban đầu nên xê dịch chút ít** mỗi lần (xem bước 14).
- Nếu tay cầm/kính bị giật, hình ảnh lag, robot di chuyển bất thường, hoặc cánh tay phản ứng
  chậm/không đúng — **không bấm thêm nút nào và dừng teleop ngay**, báo kỹ sư ngay (xem Phần 3 —
  An toàn).
- Không cần vội — cứ thao tác chậm, chắc chắn còn hơn nhanh nhưng làm hỏng. Vì file bag ghi liên
  tục, một lần gắp-đặt bị lỗi vẫn nằm trong dữ liệu; báo kỹ sư để cùng quyết định có cần cắt bỏ
  đoạn đó khi xử lý dữ liệu sau này hay không.

---

## Phần 3 — An toàn lao động

- **Luôn giữ khoảng cách an toàn** với cánh tay robot khi không trực tiếp điều khiển qua kính VR —
  không đưa tay/đứng trong tầm với của cánh tay lúc đó.
- Trước khi bắt đầu, xác nhận **không có ai khác đứng trong vùng hoạt động** của cánh tay **và
  trên đường di chuyển của robot** giữa vị trí A và vị trí B.
- Mỗi lần bấm nút **A**/**B** khiến **robot (bánh xe) tự di chuyển ngay lập tức**, mang theo cả
  cụm cánh tay — chỉ bấm khi đã nhìn kỹ đường đi, không có người/vật cản, không bấm thử/bấm nhầm.
  Dây kính VR, dây camera phải đủ dài/không vướng vào đường robot di chuyển.
- Chỉ bấm **X**/**Y** và bắt đầu teleop gắp/đặt sau khi robot **đã dừng hẳn** — không thao tác
  tay cầm VR khi robot còn đang di chuyển hoặc rung lắc.
- Biết trước vị trí **nút dừng khẩn cấp** của hệ thống *[ĐIỀN SAU: vị trí nút E-stop vật lý — của
  robot và của cánh tay, nếu là 2 nút riêng]*. Tay cầm VR **không** có nút dừng khẩn cấp chuyển
  động của robot hay cánh tay — muốn dừng ngay lập tức (kể cả khi vừa bấm nhầm nút A/B) phải dùng
  **E-stop phần cứng**. Bấm `Ctrl+C` ở pane ghi dữ liệu chỉ dừng việc ghi bag, không dừng chuyển
  động vật lý của robot (cả phần base lẫn cánh tay).
- Nếu robot hoặc cánh tay di chuyển **giật, bất thường, hoặc có tiếng động lạ** — dừng thao tác
  ngay, dùng **E-stop phần cứng** nếu cần, và **gọi kỹ sư phụ trách ngay lập tức**, không tự sửa
  hay tiếp tục thao tác.
- Không tự ý rút/cắm lại dây CAN, dây camera, hoặc tắt các terminal đang chạy khi chưa được
  hướng dẫn.

*[ĐIỀN SAU: bổ sung quy định an toàn cụ thể của nhà máy nếu có — ví dụ PPE bắt buộc, số điện thoại
liên hệ khẩn cấp, quy trình báo cáo sự cố]*

---

## Phần 4 — Xử lý sự cố cơ bản

| Hiện tượng | Xử lý |
|---|---|
| Cánh tay robot không phản hồi theo kính (teleop) | Dừng thao tác, báo kỹ sư kiểm tra pane cầu nối VR (Dora bridge) |
| Bấm nút **A**/**B** nhưng robot không di chuyển/không phản hồi | **Không bấm lại liên tục** — báo kỹ sư kiểm tra hệ thống điều khiển robot |
| Bấm nút **X**/**Y** nhưng không rõ đã ghi nhận giai đoạn gắp/đặt chưa | Cứ tiếp tục teleop gắp/đặt bình thường, báo kỹ sư kiểm tra sau — không phụ thuộc X/Y để gắp/đặt được vì đó vẫn là thao tác tay |
| robot di chuyển sai (đi nhầm hướng/dừng sai chỗ) ngay sau khi bấm A/B | Dừng ngay bằng **E-stop phần cứng** (xem Phần 3 — An toàn), báo kỹ sư, không bấm thêm nút để "sửa" |
| Một pane camera báo lỗi kết nối liên tục | Không tự khởi động lại — báo kỹ sư kiểm tra dây/serial number camera đó |
| Pane ghi dữ liệu (`ros2 bag record`) bị dừng đột ngột / báo lỗi đỏ | Không tự chạy lại — chụp lại màn hình lỗi và báo kỹ sư |
| Cánh tay di chuyển bất thường/giật | Dừng ngay (xem Phần 3 — An toàn), báo kỹ sư |

Mọi trường hợp không chắc chắn: **dừng lại, không tự xử lý, gọi người phụ trách kỹ thuật.**

*[ĐIỀN SAU: số điện thoại/kênh liên lạc của người phụ trách kỹ thuật]*
