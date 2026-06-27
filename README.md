# coverageSimulator

Mô phỏng robot bao phủ bản đồ lưới có ràng buộc năng lượng, chi phí quay, cơ chế quay về căn cứ, sạc lại và vật cản động.

Repo này là phần code/simulator của đồ án tốt nghiệp cử nhân ngành Khoa học máy tính. Trọng tâm của project là mô hình hóa bài toán, thiết kế thuật toán lập kế hoạch, mô phỏng và đánh giá hành vi của robot trong các kịch bản đại diện. Project không nhằm chế tạo robot thật, thiết kế phần cứng dò mìn, xử lý cảm biến ngoài thực địa hoặc mô phỏng đầy đủ động lực học cơ khí.

---

## 1. Mục tiêu

Hệ thống mô phỏng một robot tự hành thực hiện nhiệm vụ bao phủ trên bản đồ lưới. Robot phải xử lý đồng thời nhiều ràng buộc:

```text
- bao phủ các ô hợp lệ trên bản đồ;
- tránh vật cản tĩnh;
- phản ứng với vật cản động;
- tính chi phí địa hình;
- tính chi phí quay theo hướng hiện tại;
- kiểm tra năng lượng trước khi chọn mục tiêu;
- quay về căn cứ khi cần;
- sạc lại tại căn cứ;
- phân biệt trạng thái kết thúc nhiệm vụ.
```

Một nhiệm vụ không chỉ được đánh giá bằng tỷ lệ bao phủ. Hệ thống còn xét robot có quay về căn cứ được hay không, đã tiêu thụ bao nhiêu năng lượng, phải quay về/sạc bao nhiêu lần và kết thúc ở trạng thái nào.

---

## 2. Phạm vi mô phỏng

Môi trường được biểu diễn bằng bản đồ lưới hai chiều. Mỗi ô có thể là ô hợp lệ, vật cản tĩnh, ô có chi phí địa hình riêng hoặc ô đang bị vật cản động chiếm tại thời điểm xét.

Trong phạm vi đồ án:

```text
Robot       = robot mô phỏng trên bản đồ lưới
Base        = ô xuất phát, nơi robot quay về để sạc hoặc kết thúc nhiệm vụ
Coverage    = ô hợp lệ đã được robot đi qua trong mô hình
Obstacle    = ô không thể đi qua hoặc tạm thời không an toàn
Energy      = chi phí chuẩn hóa dùng trong lập kế hoạch và thống kê
```

Các giả định này giúp tập trung vào phần thuật toán và mô phỏng. Những bài toán như cảm biến dò mìn, định vị ngoài thực địa, điều khiển motor, cơ khí robot hoặc xây dựng bản đồ trực tuyến không thuộc trọng tâm của repo này.

---

## 3. Chức năng chính

- Đọc bản đồ lưới từ file input trong thư mục `tests/`.
- Mô phỏng một robot duy nhất.
- Hỗ trợ vật cản tĩnh.
- Hỗ trợ vật cản động dạng `Guard` và `Vehicle`.
- Lập kế hoạch bằng Dijkstra có hướng trên trạng thái gồm vị trí và hướng.
- Tính chi phí di chuyển theo ô đi vào.
- Tính chi phí quay theo số lần quay 90 độ.
- Hỗ trợ mô hình toàn hướng lý tưởng thông qua cấu hình `metric_h`, trong đó chi phí quay được bỏ qua.
- Chọn mục tiêu chưa bao phủ theo chi phí đi tới, nhưng chỉ chấp nhận nếu còn đủ năng lượng để quay về căn cứ.
- Áp dụng quỹ dự phòng quay về có điều kiện theo sự tồn tại của vật cản động.
- Tự quay về căn cứ khi năng lượng không còn phù hợp để tiếp tục bao phủ.
- Sạc lại khi về căn cứ.
- Lập kế hoạch lại khi đường đi bị chặn.
- Chờ, nhường đường hoặc tìm đường vòng khi vật cản động ảnh hưởng tới đường về.
- Ghi log hành vi và thống kê benchmark.
- Hiển thị mô phỏng bằng OpenCV.

---

## 4. Mô hình năng lượng

Hệ thống tách năng lượng thành hai phần chính:

```text
movement energy = năng lượng di chuyển giữa các ô kề cạnh
turn energy     = năng lượng đổi hướng trước khi đi sang ô tiếp theo
```

Với mô hình mặc định, một lần quay 90 độ tại ô hiện tại có chi phí bằng một nửa chi phí địa hình của ô đó. Nếu robot quay 180 độ, chi phí quay tương ứng bằng hai lần quay 90 độ.

Trong cấu hình toàn hướng lý tưởng (`metric_h`), chi phí quay được đặt bằng 0. Cấu hình này dùng làm mô hình đối chứng để đánh giá tác động của việc bỏ qua chi phí quay ở mức lập kế hoạch.

---

## 5. Chính sách quay về căn cứ

Tại mỗi lần đánh giá mục tiêu hoặc đánh giá trạng thái năng lượng, hệ thống ước lượng chi phí quay về căn cứ bằng Dijkstra có hướng.

Với một mục tiêu ứng viên `g`, hệ thống xét:

```text
cost_to_target = chi phí từ robot tới mục tiêu g
cost_to_base   = chi phí từ mục tiêu g quay về căn cứ
return_margin  = quỹ dự phòng quay về
```

Mục tiêu chỉ được chấp nhận nếu:

```text
energy >= cost_to_target + cost_to_base + return_margin
```

Chính sách `return_margin` trong phiên bản hiện tại là chính sách có điều kiện:

```text
Nếu kịch bản không có vật cản động:
    return_margin = 0

Nếu kịch bản có vật cản động:
    return_margin = max(MIN_RETURN_MARGIN, proportional_return_margin)
```

Trong đó:

```text
MIN_RETURN_MARGIN = 15
proportional_return_margin ≈ 25% * cost_to_base
```

Ý nghĩa của chính sách này:

- Trong bản đồ tĩnh, đường về được đánh giá trên bản đồ đã biết. Robot chỉ cần đủ năng lượng để đi tới mục tiêu và quay về căn cứ.
- Trong kịch bản có vật cản động, đường về có thể phát sinh chi phí do chờ, đi vòng hoặc lập kế hoạch lại. Khi đó hệ thống giữ thêm quỹ dự phòng để giảm rủi ro robot tiêu hết năng lượng cần thiết cho đường về.

Nếu robot về tới căn cứ với đúng 0 năng lượng, trạng thái này vẫn được xem là quay về thành công vì robot đã tới điểm sạc hoặc điểm kết thúc nhiệm vụ.

---

## 6. State machine

Controller chính nằm trong:

```text
srcs/robot/coverage.cpp
```

Các trạng thái chính:

| State | Ý nghĩa |
|---|---|
| `NORMAL` | Robot đang bao phủ bình thường |
| `ALERT` | Có nguy cơ từ vật cản động hoặc đường hiện tại không còn an toàn |
| `HOLD_SAFE` | Robot tạm dừng an toàn và thử phục hồi/lập kế hoạch lại |
| `RETURN_TO_BASE` | Robot quay về căn cứ |
| `RECHARGING` | Robot đang sạc tại căn cứ |
| `POWER_SAVE` | Robot dừng để bảo toàn trạng thái khi không thể tiếp tục an toàn |
| `WAIT_FOR_COMMAND` | Trạng thái chờ chỉ thị trong tình huống tới hạn |
| `FINAL_PUSH` | Trạng thái tiếp tục có chủ đích khi không ưu tiên bảo toàn |

Luồng trạng thái thường gặp:

```text
NORMAL
  -> RETURN_TO_BASE
  -> RECHARGING
  -> NORMAL
```

Khi có vật cản động:

```text
NORMAL
  -> ALERT
  -> HOLD_SAFE / RETURN_TO_BASE
```

Khi đường về bị ảnh hưởng:

```text
RETURN_TO_BASE
  -> wait / replan
  -> tactical yield
  -> detour
  -> WAIT_FOR_COMMAND / POWER_SAVE
```

---

## 7. Mission outcome

Hệ thống phân biệt kết quả bao phủ và kết quả nhiệm vụ.

| Outcome | Ý nghĩa |
|---|---|
| `MISSION_SUCCESS` | Robot bao phủ xong và đã quay về căn cứ |
| `MISSION_PARTIAL_RETURNED` | Robot chưa bao phủ hết nhưng đã quay về căn cứ |
| `MISSION_PARTIAL_PRESERVED` | Robot chưa hoàn tất nhưng dừng ở trạng thái bảo toàn |
| `MISSION_FAILED` | Robot không còn khả năng hoàn thành hoặc bảo toàn nhiệm vụ theo chính sách hiện tại |

Điểm quan trọng: coverage cao không tự động đồng nghĩa với nhiệm vụ thành công đầy đủ. Với hệ thống này, một nhiệm vụ thành công đầy đủ cần thỏa cả hai điều kiện: bao phủ xong và quay về căn cứ.

---

## 8. Vật cản động

Vật cản động đại diện cho các tác nhân chuyển động trong môi trường mô phỏng.

| Ký hiệu | Loại | Ý nghĩa |
|---|---|---|
| `G` | Guard | Tác nhân tuần tra cục bộ |
| `V` | Vehicle | Tác nhân di chuyển theo hành lang hoặc hướng tuần tra |

Các ràng buộc chính:

```text
- Vật cản động không được khởi tạo trên căn cứ.
- Vật cản động không đi xuyên vật cản tĩnh.
- Vật cản động không chủ động lao vào ô robot đang chiếm.
- Robot không được bắt đầu bước đi vào ô đang bị vật cản động chiếm.
- Nếu đường hiện tại bị ảnh hưởng, robot có thể chờ, lập kế hoạch lại hoặc quay về căn cứ.
```

Vật cản động được cập nhật bởi module môi trường. Robot cập nhật ô cần tránh cho môi trường bằng vị trí hiện tại của mình để giảm nguy cơ xung đột trực tiếp.

---

## 9. Cấu trúc thư mục

```text
srcs/
  main.cpp

  environment/
    dynamic_obstacle.cpp
    environment.cpp
    guard.cpp
    vehicle.cpp

  robot/
    coverage.cpp
    coverage_context.cpp
    coverage_tick.cpp
    coverage_timing.cpp
    coverage_render.cpp

    mission_policy.cpp
    mission_state.cpp
    mission_summary.cpp

    energy_model.cpp
    energy_reserve_policy.cpp
    path_builder.cpp
    path_safety.cpp
    return_to_base.cpp
    robot_lifecycle.cpp
    robot_motion.cpp
    tactical_yield.cpp

  visualization/
    opencv.cpp
    visual_layout.cpp
    visual_assets.cpp
    map_renderer.cpp
    entity_renderer.cpp
    hud_renderer.cpp

  world/
    grid.cpp
    input.cpp
    stats.cpp
    image_utils.cpp
    types.h
```

Luồng chạy chính:

```text
main.cpp
  -> readGrid()
  -> initEnvironment()
  -> executeCoverage()
  -> stopEnvironment()
  -> waitEnvironment()
  -> collectStats()
  -> printStats()
  -> logStats()
```

---

## 10. Input

Chương trình đọc test map từ thư mục:

```text
tests/
```

Khi chương trình hỏi:

```text
Nhap duong dan file input:
```

chỉ cần nhập tên file không có `.txt`.

Ví dụ:

```text
demo_01_open_room
```

Chương trình sẽ đọc:

```text
tests/demo_01_open_room.txt
```

Các ký hiệu thường dùng trong map:

| Ký hiệu | Ý nghĩa |
|---|---|
| `R` | Vị trí xuất phát của robot, đồng thời là căn cứ |
| `0` | Ô trống |
| `1` | Vật cản tĩnh |
| `G` | Guard |
| `V` | Vehicle |

Các ô có chi phí địa hình riêng được biểu diễn bằng giá trị số tương ứng trong input.

---

## 11. Build

Project viết bằng C++17 và dùng OpenCV.

Ví dụ build bằng `g++` trên môi trường có `pkg-config` và OpenCV 4:

```bash
g++ -std=c++17 \
  srcs/main.cpp \
  srcs/environment/*.cpp \
  srcs/robot/*.cpp \
  srcs/world/*.cpp \
  srcs/visualization/*.cpp \
  -I srcs/environment \
  -I srcs/robot \
  -I srcs/world \
  -I srcs/visualization \
  `pkg-config --cflags --libs opencv4` \
  -o coverage_sim
```

Trên Windows hoặc Code::Blocks, cần cấu hình include path và linker path cho OpenCV tương ứng với môi trường cài đặt.

Nếu gặp lỗi:

```text
opencv2/core.hpp: No such file or directory
opencv2/opencv.hpp: No such file or directory
```

thì compiler chưa biết OpenCV include path.

Nếu gặp lỗi:

```text
undefined reference to cv::...
```

thì linker chưa link đúng OpenCV library.

---

## 12. Run

Sau khi build:

```bash
./coverage_sim
```

Sau đó nhập tên test, ví dụ:

```text
demo_01_open_room
```

Với Windows, chạy file `.exe` được build từ IDE hoặc terminal, sau đó nhập tên test tương tự.

---

## 13. Pseudo-code tổng quát

```text
initialize robot at base
initialize environment
mark current cell as covered

while mission is not terminal:
    update dynamic obstacles
    update robot avoidance cell
    estimate cost to base

    if coverage is complete and robot is not at base:
        switch to RETURN_TO_BASE

    if energy indicates return is needed:
        switch to RETURN_TO_BASE

    if state == NORMAL:
        collect uncovered candidate cells
        sort candidates by cost from robot
        for each candidate:
            estimate cost to target
            estimate cost from target to base
            compute return margin
            if energy condition is satisfied:
                build path to candidate
                execute safe movement
                break

    if state == ALERT:
        check path safety
        replan or switch to safer state

    if state == HOLD_SAFE:
        wait briefly and retry recovery/replan

    if state == RETURN_TO_BASE:
        build path to base
        if path is safe and affordable:
            move one step toward base
        else:
            wait, yield, detour, or preserve state

    if state == RECHARGING:
        restore energy
        switch to NORMAL

    if terminal condition is reached:
        record mission outcome
```

Energy-aware target condition:

```text
energy >= cost_to_target + cost_to_base + return_margin
```

Dynamic-aware return margin:

```text
if no dynamic obstacle exists:
    return_margin = 0
else:
    return_margin = max(15, cost_to_base / 4)
```

---

## 14. Metrics / evaluation

Các chỉ số chính dùng để đánh giá:

```text
coverage percentage
covered cells
free cells
steps
total energy used
movement energy
turn energy
remaining energy
revisit count
return count
recharge count
mission outcome
final at base
```

Những chỉ số này giúp đánh giá nhiệm vụ theo nhiều mặt: mức độ bao phủ, chi phí năng lượng, số bước đi lặp, số chu kỳ quay về/sạc và trạng thái kết thúc.

---

## 15. Demo và benchmark

Một số nhóm kịch bản thường dùng trong báo cáo:

| Nhóm | Mục tiêu |
|---|---|
| A | Ảnh hưởng của vị trí căn cứ |
| B | Năng lượng thấp |
| C | Mật độ vật cản cao hoặc vùng khó tiếp cận |
| D | Vật cản động ảnh hưởng đường đi hoặc đường quay về |
| E | Địa hình có trọng số |
| F | Bản đồ lớn hơn |
| H | Mô hình toàn hướng đối chứng (`metric_h`) |

Sau mỗi thay đổi lớn ở thuật toán, nên chạy lại các kịch bản đại diện và đối chiếu các trường:

```text
coverage
steps
energy used
turn energy
returns
recharges
mission outcome
final at base
```

---

## 16. Ghi log và kết quả

Hệ thống ghi nhận kết quả chạy để phục vụ phân tích thực nghiệm. Các log và ảnh kết quả cuối có thể được dùng để kiểm tra lại hành vi của robot trong từng kịch bản.

Báo cáo đồ án không cần đưa toàn bộ log thô vào nội dung chính. Thay vào đó, nên tổng hợp các chỉ số chính trong bảng thực nghiệm và sử dụng một số ảnh minh họa đại diện.

---

## 17. Ghi chú về thiết kế

Một số nguyên tắc thiết kế của repo:

```text
- Planner không chỉ xét đường đi tới mục tiêu, mà còn xét đường quay về căn cứ.
- Quỹ dự phòng 25% chỉ có ý nghĩa khi có rủi ro động.
- Bản đồ tĩnh không cần giữ quỹ dự phòng động.
- Về tới căn cứ với đúng 0 năng lượng vẫn được tính là quay về thành công.
- Vật cản động làm thay đổi trạng thái an toàn của ô theo thời gian.
- Visualization chỉ phản ánh trạng thái mô phỏng, không quyết định logic nhiệm vụ.
- Log và thống kê phải đủ rõ để đối chiếu với báo cáo.
```

---

## 18. Giới hạn hiện tại

- Một robot duy nhất.
- Bản đồ được biết trước trong phạm vi mô phỏng.
- Chuyển động rời rạc theo bản đồ lưới.
- Chưa xử lý cảm biến thật, định vị thật hoặc điều khiển phần cứng.
- Chưa mô phỏng đầy đủ động lực học, trượt bánh, gia tốc hoặc tiêu hao khi chờ.
- Vật cản động được mô hình hóa ở mức tác nhân hợp tác, không phải tác nhân đối kháng.
- Chưa có hệ thống phối hợp nhiều robot.
- Chưa có test runner tự động/headless chính thức.

Các giới hạn này xác định phạm vi hiện tại của simulator. Chúng không làm thay đổi mục tiêu chính của repo: kiểm chứng thuật toán lập kế hoạch bao phủ có xét năng lượng, chi phí quay, quay về căn cứ và vật cản động trên môi trường mô phỏng.

---

## 19. Hướng phát triển

Các hướng mở rộng hợp lý:

- Bổ sung test runner tự động để chạy nhiều kịch bản không cần thao tác thủ công.
- Bổ sung chế độ headless để chạy benchmark không cần giao diện OpenCV.
- Cải tiến chiến lược chọn mục tiêu để giảm revisit trong bản đồ nhiều hành lang/hốc cục bộ.
- Bổ sung nhiều căn cứ hoặc nhiều trạm sạc.
- Bổ sung chi phí chờ, chi phí tăng tốc/giảm tốc hoặc chi phí lập kế hoạch lại.
- Mở rộng mô hình vật cản động bằng dự đoán quỹ đạo ngắn hạn.
- Nghiên cứu phối hợp nhiều robot sau khi mô hình một robot đã ổn định.

Các mở rộng lớn như dynamic map runtime, multi-robot coordination hoặc mô hình cảm biến thật nên được thực hiện theo từng bước nhỏ để tránh làm tăng độ phức tạp ngoài khả năng kiểm chứng.
