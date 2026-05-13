# Coverage Simulator

Mô phỏng bài toán **coverage path planning** cho một robot trên grid map, có vật cản động và ràng buộc năng lượng.

Đây là đồ án tốt nghiệp cử nhân tại Đại học Bách Khoa Hà Nội.

## 1. Bài toán

Bài toán chính là lập kế hoạch cho một robot đi phủ bản đồ dạng lưới.

Robot được giả định có:

- full map;
- góc nhìn bird-view;
- một robot duy nhất;
- vật cản tĩnh biết trước;
- vật cản động trong môi trường;
- năng lượng hữu hạn.

Mục tiêu của đồ án không phải là chứng minh thuật toán tối ưu tuyệt đối, mà là xây dựng một hệ thống mô phỏng có logic rõ ràng, chạy được, giải thích được và có khả năng xử lý các tình huống thực tế như vật cản động, pin thấp, đường về bị chặn, hoặc nhiệm vụ không thể hoàn thành an toàn.

## 2. Ý tưởng chính

Với bài toán coverage đơn giản, robot chỉ cần hỏi:

```text
Ô chưa phủ gần nhất nằm ở đâu?
```

Nhưng khi thêm năng lượng hữu hạn và vật cản động, robot phải ra quyết định phức tạp hơn:

```text
Có nên tiếp tục coverage không?
Có nên quay về base không?
Có nên chờ không?
Có nên replan không?
Có nên đi vòng không?
Có nên dừng để bảo toàn không?
Có nên dùng phần pin cuối để tạo thêm mission value không?
```

Triết lý thiết kế của hệ thống:

```text
Constraint tạo ra tradeoff.
Tradeoff tạo ra quyết định.
Quyết định tạo ra trí tuệ.
```

Robot không được thiết kế để “không bao giờ thất bại”. Robot được thiết kế để nếu thất bại thì thất bại an toàn, có ích, và giữ lại nhiều giá trị nhất có thể.

## 3. Các chức năng chính

- Coverage trên grid map.
- Một robot duy nhất.
- Vật cản tĩnh.
- Vật cản động:
  - guard;
  - vehicle;
  - random walker.
- Planner dùng Dijkstra với unit cost.
- Chọn ô chưa phủ gần nhất làm target.
- Quản lý năng lượng theo từng bước di chuyển.
- Tự quay về base khi pin thấp.
- Sạc lại tại base.
- Né vật cản động.
- Replan khi đường bị chặn.
- Chờ an toàn khi môi trường tạm thời nguy hiểm.
- Xử lý deadlock khi đường về bị chặn.
- Chính sách preserve / heroic trong tình huống critical.
- Hiển thị bằng OpenCV.
- HUD hiển thị trạng thái, năng lượng, số lần return/recharge.
- Trail thể hiện đường robot đã đi và mức độ overlap.
- Bộ map demo/test trong thư mục `tests/`.

## 4. State machine của robot

Controller chính nằm trong `srcs/robot/coverage.cpp`.

Các trạng thái chính:

```text
NORMAL
ALERT
HOLD_SAFE
RETURN_TO_BASE
RECHARGING
POWER_SAVE
WAIT_FOR_COMMAND
FINAL_PUSH
```

Ý nghĩa:

| State | Ý nghĩa |
|---|---|
| `NORMAL` | Robot coverage bình thường |
| `ALERT` | Có vật cản động gần robot hoặc active path bị chặn |
| `HOLD_SAFE` | Môi trường tạm thời không an toàn, robot dừng và thử recover |
| `RETURN_TO_BASE` | Pin thấp, robot quay về base |
| `RECHARGING` | Robot đang sạc tại base |
| `POWER_SAVE` | Preserve mode, robot dừng an toàn |
| `WAIT_FOR_COMMAND` | Tình huống critical, cần directive cấp nhiệm vụ |
| `FINAL_PUSH` | Heroic mode, robot tiếp tục coverage đến khi hết pin nhưng vẫn né vật cản |

## 5. Chính sách năng lượng

Mỗi bước di chuyển tiêu tốn năng lượng.

Robot không dùng ngưỡng phần trăm pin cố định. Thay vào đó, robot ước lượng chi phí quay về base bằng Dijkstra:

```cpp
costToBase = dijkstra distance from robot to base
returnMargin = max(MIN_RETURN_MARGIN, costToBase / RETURN_MARGIN_DIVISOR)
```

Robot bắt đầu quay về base khi:

```cpp
energy <= costToBase + returnMargin
```

Robot rơi vào trạng thái critical khi:

```cpp
energy <= costToBase || energy <= MIN_EMERGENCY_ENERGY
```

Cách này giúp policy phụ thuộc vào vị trí hiện tại. Robot ở xa base sẽ thận trọng hơn robot đang gần base.

## 6. Chính sách thất bại an toàn

Nếu robot đang quay về base nhưng đường về bị vật cản động chặn, robot không lao vào vật cản.

Thay vào đó, robot xử lý theo chuỗi:

```text
RETURN_TO_BASE
→ RETURN_WAIT
→ thử detour
→ nếu không có detour và năng lượng critical
→ WAIT_FOR_COMMAND
→ POWER_SAVE hoặc FINAL_PUSH
```

Directive mặc định hiện tại là:

```cpp
PRESERVE
```

Trong preserve mode, robot dừng an toàn để giữ:

- robot;
- năng lượng còn lại;
- dữ liệu coverage đã thu được;
- khả năng recover trong tương lai.

Có thể đổi directive sang:

```cpp
HEROIC
```

Trong heroic mode, robot bỏ mục tiêu quay về base và dùng phần năng lượng còn lại để tạo thêm mission value. Tuy nhiên, robot vẫn phải né vật cản động.

## 7. Chính sách vật cản động

Vật cản động không được chủ động đâm xuyên qua robot.

Coverage controller cập nhật vị trí robot cho môi trường bằng:

```cpp
setRobotAvoidanceCell(rb.pos);
```

Trước khi di chuyển, dynamic obstacle kiểm tra nếu bước tiếp theo có thể đâm hoặc tiến quá gần robot thì obstacle sẽ dừng/yield.

Chính sách này áp dụng cho:

```text
GUARD
VEHICLE
RANDOM
```

## 8. Giao diện mô phỏng

Giao diện OpenCV hiển thị:

- grid map;
- vật cản tĩnh;
- vật cản động;
- ô đã coverage;
- vị trí robot;
- hướng robot;
- active path;
- trail robot đã đi;
- mức độ overlap trên cạnh;
- trạng thái hiện tại;
- năng lượng còn lại;
- tổng năng lượng đã dùng;
- số lần return;
- số lần recharge;
- active path có tồn tại hay không;
- độ dài path hiện tại.

HUD được đặt ngoài vùng grid nếu còn chỗ. Nếu không đủ chỗ, HUD sẽ không được vẽ để tránh che bản đồ.

## 9. Format input

Chương trình đọc map từ thư mục `tests/`.

Khi chương trình hỏi:

```text
Nhap duong dan file input:
```

chỉ cần nhập tên file không có `.txt`.

Ví dụ nhập:

```text
demo_01_open_room
```

Chương trình sẽ đọc:

```text
tests/demo_01_open_room.txt
```

Các ký hiệu trong map:

| Ký hiệu | Ý nghĩa |
|---|---|
| `R` | Vị trí bắt đầu của robot |
| `0` | Ô trống |
| `1` | Vật cản tĩnh |
| `G` | Guard dynamic obstacle |
| `V` | Vehicle dynamic obstacle |
| `W` | Random dynamic obstacle |

Ví dụ:

```text
111111111111
1R0000000001
100000G00001
100000000001
111111111111
```

## 10. Các demo scenario đề xuất

Một số map demo/test:

```text
demo_01_open_room
demo_02_long_corridor
demo_03_rooms_with_door
demo_04_static_maze
demo_05_guard_alert
demo_06_vehicle_corridor
demo_07_vehicle_detour
demo_08_random_obstacle_room
demo_09_low_energy_large_room
demo_10_blocked_return_preserve
```

Ý nghĩa:

| Scenario | Mục tiêu |
|---|---|
| `demo_01_open_room` | Coverage bình thường trong phòng mở |
| `demo_02_long_corridor` | Hành lang hẹp |
| `demo_03_rooms_with_door` | Coverage qua các phòng nối nhau |
| `demo_04_static_maze` | Điều hướng với vật cản tĩnh |
| `demo_05_guard_alert` | Robot cảnh giác/replan quanh guard |
| `demo_06_vehicle_corridor` | Tương tác với vehicle trong hành lang hẹp |
| `demo_07_vehicle_detour` | Đường bị chặn và có đường vòng |
| `demo_08_random_obstacle_room` | Né random obstacle |
| `demo_09_low_energy_large_room` | Test pin thấp, return và recharge |
| `demo_10_blocked_return_preserve` | Đường về bị chặn, critical và preserve |

## 11. Build

Project viết bằng C++ và dùng OpenCV để hiển thị.

Ví dụ build bằng `g++` trên môi trường có `pkg-config` và OpenCV 4:

```bash
g++ -std=c++17 \
  srcs/main.cpp \
  srcs/environment/*.cpp \
  srcs/robot/*.cpp \
  srcs/world/*.cpp \
  -I srcs/environment \
  -I srcs/robot \
  -I srcs/world \
  `pkg-config --cflags --libs opencv4` \
  -o coverage_sim
```

Trên Windows, cần cấu hình compiler/IDE với:

- OpenCV include path;
- OpenCV library path;
- OpenCV runtime DLL trong `PATH` hoặc cạnh file `.exe`.

## 12. Run

Sau khi build:

```bash
./coverage_sim
```

Sau đó nhập tên test, ví dụ:

```text
demo_01_open_room
```

## 13. Các file quan trọng

```text
srcs/main.cpp
srcs/robot/coverage.cpp
srcs/robot/planner.cpp
srcs/environment/dynamic_obstacle.cpp
srcs/environment/guard.cpp
srcs/environment/vehicle.cpp
srcs/environment/random_walk.cpp
srcs/world/opencv.cpp
srcs/world/input.cpp
srcs/world/stats.cpp
srcs/world/types.h
```

## 14. Giới hạn hiện tại

- Chỉ có một robot.
- Full map được biết trước.
- Chuyển động trên grid.
- Planner dùng Dijkstra unit-cost.
- Dynamic obstacle dùng behavior đơn giản.
- Energy cost hiện chủ yếu gắn với movement.
- Waiting/replanning hiện được mô hình hóa như time/opportunity cost, chưa trừ pin trực tiếp.
- Chưa có đảm bảo tối ưu toàn cục.
- Chưa có multi-robot coordination.

## 15. Hướng phát triển tiếp theo

- Mission summary và stop reason rõ ràng hơn.
- Test runner tự động.
- Headless mode để chạy regression test không cần UI.
- Deterministic obstacle mode để test ổn định hơn.
- Bắt buộc robot quay về base sau khi coverage hoàn tất.
- Tách overlap trong phase coverage khỏi chi phí final return.
- Tactical yield: robot lùi/né vài ô để phá deadlock với vehicle trong hành lang hẹp.
- Cân nhắc energy cost cho waiting, replanning hoặc speed boost.
- Dọn code và split file lớn.

## 16. Triết lý thiết kế

Đồ án này xem coverage không chỉ là bài toán đi qua mọi ô.

Khi năng lượng hữu hạn, mỗi bước đi đều có giá. Mỗi lần đi vòng có giá. Mỗi lần chờ có chi phí cơ hội. Một robot tốt không nên mù quáng đi tiếp cho đến khi hết pin.

Một hệ tự trị trưởng thành không phải là hệ không bao giờ thất bại. Đó là hệ biết thất bại an toàn, giữ lại giá trị, và bảo toàn khả năng phục hồi khi thành công tuyệt đối không còn được đảm bảo.

```text
Làm nhiệm vụ.
Tôn trọng ràng buộc.
Bảo toàn tương lai.
```
