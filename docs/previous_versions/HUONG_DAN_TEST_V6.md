# Kịch bản thử V6 trên bàn — **không cánh quạt, ngắt ESC**

| Bài test | Động tác | Dòng log mong đợi |
|---|---|---|
| T01 | Bật, đặt sensor 30–50cm | `valid:1 reason:READY`, `accepted` tăng |
| T02 | CH5 OFF; CH6 giữa | `mode:1 shadow:CAPTURE` rồi `TRACK` |
| T03 | Giữ cao độ ổn định | `vz ≈ 0`, `u` gần 0 sau lọc |
| T04 | Hạ 5–10cm chậm | `h` giảm; `u` tăng theo hướng dương sau trễ lọc |
| T05 | Nâng 5–10cm chậm | `h` tăng; `u` âm sau trễ lọc |
| T06 | Đặt ở 20mm hoặc che sensor | `valid:0`, `u:0`, `reason:RANGE_BAD` hoặc `STALE` |
| T07 | Đưa sensor về 30–50cm | `WARMUP` rồi `READY`; shadow chụp lại target |
| T08 | CH6 thấp | `mode:0 shadow:OFF u:0` |
| T09 | Mất kết nối ELRS | `RC:0`; shadow `OFF u:0` |
| T10 | Chỉ nghiêng nhẹ trong vùng 25° | `tilt` <1; `raw_h` thay đổi theo bù nghiêng |

Ghi chú: Việc nhận `valid:1` và `u` phản ứng đúng dấu **không đủ** để chứng minh ALT_HOLD bay an toàn. Chỉ đưa log cho lần tinh chỉnh tiếp theo, không nối `u` vào ESC.
