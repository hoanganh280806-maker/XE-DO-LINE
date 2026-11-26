# V6.1 – thử nghiệm cải thiện bám line và góc 90°

Cơ sở: branch `development-v6`, commit `13cac4d`, blob V6 `bffc9a87832904353f63703923efcdb867236d6f`.
Giữ nguyên V6 và main. Sketch mới: `LineFollower_ESP32_V61/LineFollower_ESP32_V61.ino`. Chưa biên dịch trên ESP32 thực tế, chưa kiểm thử xe thật. Không nạp và chạy tốc độ cao ngay. NVS riêng `linebot61`: cần hiệu chuẩn lại `c`.

## Tham khảo (nguyên lý; tự triển khai, không sao chép code)
- https://github.com/pololu/qtr-sensors-arduino/blob/master/usage.md – hiệu chuẩn, ước lượng line.
- https://github.com/pololu/pololu-3pi-plus-32u4-arduino-library/blob/master/examples/LineFollowerPID/LineFollowerPID.ino – PD/PID và giới hạn lệnh.
- https://github.com/pololu/zumo-shield-arduino-library/blob/master/examples/MazeSolver/MazeSolver.ino – tiến qua giao điểm và kiểm tra nhánh thẳng.
- https://github.com/IosifSramec/RoboTEC-Linefollower – PD và điều chỉnh tốc độ theo sai lệch.
- https://github.com/wogen-99/esp32-line-follower-dual-pid – tham khảo kiến trúc cascade, chưa áp dụng cascade hoàn chỉnh ở V6.1.

## Thay đổi
- Giảm base 170→100, max 230→140, turn 230→110, lost 140→90, Kp .05→.04, Kd 1→.2, Ki=0. Các giá trị chỉ là khởi đầu thử nghiệm.
- FOLLOW: hạn chế tốc độ theo sai số và đạo hàm lọc; thấy cạnh ngoài hoặc line bất thường thì giới hạn PWM trung bình; không đảo chiều bánh; không tăng base để đáp ứng độ sửa hướng; tăng tốc PWM chậm hơn giảm tốc (3 và 10 đơn vị/2ms).
- Góc: cần 3 mẫu cùng hướng với 3 mắt liền kề phía ngoài đủ tín hiệu. Có thể bỏ sót góc nếu line hẹp; phải kiểm tra dữ liệu cảm biến trên sân thực.
- ADVANCE: tiến PWM 85 trong 80ms mặc định, hoặc tới số xung `advcnt`; timeout 700ms. Sau đó phanh 90ms và kiểm tra có đường thẳng hay không (ưu tiên đi thẳng nếu nhận line đơn giữa). Không giải mê cung chữ T tự động.
- TURN_COARSE (mode 2) → FINE (5) → ALIGN_BRAKE (6); phải rời line cũ, tìm thấy line mới, phanh 65ms và kiểm tra tâm khi xe phanh. Nếu lệch nhỏ, cho căn lại tối đa 2 lần. Nếu quá 1800ms hoặc vượt 140% `turncnt` đã hiệu chuẩn thì FAULT, không chuyển sang quay vòng SEARCH. Chưa đo góc thực; encoder tuyệt đối chỉ xấp xỉ do trượt bánh, không đo yaw thật.
- SEARCH dùng cho mất line bình thường; có timeout, phanh khi nhìn thấy tâm. Giới hạn góc theo xung chỉ khi đã hiệu chuẩn; `tcnt` là số xung trung bình một vòng 360° để SEARCH, khác `turncnt` là số xung trung bình khi xoay 90°.
- `trim` bù PWM tĩnh (+ tăng trái giảm phải); `zero` hiệu chỉnh vị trí tâm; `synckp`/`syncki` PI chênh lệch tốc độ theo độ lớn xung/50ms, chỉ bật khi chạy tiến gần tâm. Mặc định tắt; giới hạn ±15 PWM. Đây KHÔNG phải cascade PI tốc độ từng bánh. Cần kiểm tra số xung/vòng và chiều encoder trước khi bật. Không dùng nếu hai bánh khác đường kính hoặc độ phân giải encoder khác nhau khi chưa quy đổi.
- Giữ GPIO, manual start, dừng lỗi siêu âm, timeout, lệnh x, UDP. Không bỏ lớp an toàn. HC-SR04 không echo hợp lệ sẽ khóa chạy/dừng như V6; cần bố trí môi trường kiểm tra phù hợp. Kiểm tra dừng vật lý và kê bánh khi dùng test/spin. Phanh phần mềm không bảo đảm xe đứng yên; cần xác minh dòng phanh, ma sát, quán tính và tốc độ.
- Debug 100ms: 8 cảm biến, state, mode, line, err, dist, PWM thực/đặt, encoder, dt, overrun, sync. UDP giữ nguyên START/DATA/END và 7 trường; receiver cũ vẫn nhận, mode mới sẽ hiện số nếu chưa bổ sung tên. Không có PWM trong UDP, lấy từ Serial. Log 50ms có thể bỏ qua chuyển trạng thái rất ngắn. UDP không đảm bảo giao đủ gói.

## Lệnh Serial (115200, Newline; gửi từng dòng)
`x`, `c`, `s`, `p`, `dbg`, `raw`, `save`, `clear`, `test`, `spin`, `udp`, `log` như V6. Chỉnh tham số chỉ khi IDLE. `clear` xóa cấu hình namespace V6.1; cần reset và hiệu chuẩn lại.

| Lệnh | Mặc định | Ý nghĩa |
|---|---:|---|
| base | 100 | PWM cơ bản |
| max | 140 | PWM tối đa toàn cục, trừ phanh điện |
| kp | 0.04 | hệ số P line |
| kd | 0.2 | hệ số D line theo chu kỳ danh định |
| ki | 0 | hệ số I line |
| approach | 85 | trần PWM trung bình khi có tín hiệu cạnh hoặc rủi ro |
| advpwm | 85 | PWM tiến qua góc |
| tadv | 80 | ms tiến qua góc khi advcnt=0 |
| tspd | 110 | PWM xoay thô |
| fine | 75 | PWM xoay tinh, cũng là giới hạn khi SEARCH thấy line |
| lost | 90 | PWM SEARCH |
| tfull | 2200 | thời gian SEARCH nếu tcnt=0 |
| tcnt | 0 | số xung SEARCH một vòng nếu đã hiệu chuẩn |
| advcnt | 0 | số xung trung bình để kết thúc ADVANCE (0: thời gian) |
| turncnt | 0 | số xung trung bình cho góc 90° đã hiệu chuẩn (0: không dùng) |
| trim | 0 | bù tĩnh -25..25 PWM |
| zero | 3500 | vị trí tâm 3200..3800 |
| synckp | 0 | P đồng bộ tốc độ, PWM/(xung/50ms) |
| syncki | 0 | I đồng bộ tốc độ, PWM/[(xung/50ms)·s] |
| corner | 1 | bật xử lý góc |

## Kiểm tra thực tế
1. Verify Arduino IDE ESP32 Dev Module. Kê bánh, hiệu chuẩn `c` (quét cả 8 mắt qua đen/trắng trong 4s). `dbg` xem line nằm trái phải có dấu đúng, kiểm tra chiều bánh bằng `test`; xác minh encoder cả hai có xung, GPIO15 phải đúng dây pha B phải. Không chạy nếu ngược dấu điều khiển.
2. Đường thẳng trước: `x`, `corner 0`, `max 140`, `base 100`, `kp 0.04`, `kd 0.2`, `ki 0`, `trim 0`, `synckp 0`, `syncki 0`, `s`. Có người giám sát, giữ khoảng trống, sẵn sàng x/dừng nguồn. Nếu xe không thắng ma sát thì tăng PWM từng bước 5; nếu PWM nhỏ làm kẹt bánh, không chạy kéo dài.
3. Nếu luôn lệch trái, kiểm tra vị trí cảm biến và tốc độ bánh, thử `trim 3`, rồi `trim 6` nếu cần; lệch phải dùng giá trị âm. Đo lại; không tăng Ki để che lỗi dây/encoder. `zero` chỉ dùng khi thân xe được đặt thẳng đúng tâm line nhưng ADC cho sai số ổn định khác 0. Không chỉnh trim và zero cùng lúc.
4. Bật `corner 1`. Thử riêng từng góc trái/phải với thông số mặc định; nếu vào góc sớm/muộn, đo khoảng cách cảm biến–trục bánh và quãng tiến thật. Điều chỉnh `tadv` từng 10ms và `advpwm` từng 5 PWM. Nếu thiếu lực xoay tinh, tăng `fine` 5 PWM; nếu lố giảm `tspd` 5–10 PWM. Cần dừng giữa các lần đổi tham số; chưa save khi chưa tốt.
5. Chỉ bật `advcnt` và `turncnt` sau khi đo số xung thật trên nền, có giám sát. Đừng dùng xung/vòng trục motor thay cho xung/vòng bánh. `spin` là lệnh bench kê bánh, không dùng đo góc xoay trên sàn khi không có cơ chế an toàn.
6. Gửi log Serial với `state/m/err/pwm/target/enc/dt` hoặc video quan sát, kèm kích thước bánh, khoảng cách 2 bánh, khoảng cách mảng cảm biến–trục bánh. Nếu góc không nhận vì chỉ hai mắt ngoài sáng, gửi mẫu `sv` để chỉnh thuật toán thay vì tăng tốc.
7. Khi `FAULT` hoặc `OBSTACLE_STOP`, dừng và xác định nguyên nhân rồi `x`, đặt lại xe trên line, `s`. Không tự khởi động lại. `save` chỉ sau khi kiểm chứng.

Receiver Python hiện có: thêm các mode vào `MODE_NAMES`: `4: "BRAKE", 5: "FINE", 6: "ALIGN_BRAKE"`. Không cần đổi định dạng UDP. Nếu cần UDP: sao chép WiFiConfig.example.h thành WiFiConfig.h và cấu hình nội bộ, không commit mật khẩu. ESP32 cổ điển dùng Wi-Fi 2.4GHz.

## Kiểm thử host
`python tests/run_tests.py` (g++ C++17). Kiểm thử hai mock LEDC API 2.x/3.x, hiệu chuẩn, ARM, sonar, timeout, corner cùng hướng, phanh trước cua, căn tâm khi phanh, hướng trim, giới hạn PWM, ADVANCE theo xung, mất line khi FINE, giới hạn góc, đồng bộ encoder, lệnh stop/clear/test/spin. Mock không mô phỏng quán tính, ADC, ma sát, động lực học hoặc SDK Arduino-ESP32 thật.
