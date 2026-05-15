# Coverage Simulator

Mô phỏng **coverage path planning** cho robot trên bản đồ dạng lưới, có vật cản tĩnh, vật cản động, ràng buộc năng lượng và logic nhiệm vụ theo hướng **mission-aware autonomy**.

Đây là đồ án tốt nghiệp cử nhân tại Đại học Bách Khoa Hà Nội.

---

## 1. Mục tiêu của project

Project này không chỉ mô phỏng robot đi qua mọi ô trống trên bản đồ.

Mục tiêu chính là xây dựng một hệ thống mô phỏng robot trinh sát tự trị trong môi trường nguy hiểm, nơi robot phải cân bằng giữa:

```text
coverage value
survivability
recoverability
energy constraint
safe return
mission outcome
```

Robot không được thiết kế để “không bao giờ thất bại”. Robot được thiết kế để nếu thất bại thì thất bại có kiểm soát, giữ lại nhiều giá trị nhiệm vụ nhất có thể.

```text
Làm nhiệm vụ.
Tôn trọng ràng buộc.
Bảo toàn tương lai.
```

---

## 2. Bối cảnh mô phỏng

Narrative hiện tại của project:

```text
Robot = autonomous reconnaissance robot
Map = khu vực cần khảo sát / trinh sát
Base = command station / recharge station
Dynamic obstacles = allied patrols, vehicles, unknown hazards
Coverage = thu thập dữ liệu khu vực
Return-to-base = xác nhận nhiệm vụ hoàn tất và phục hồi robot
```

Trong hệ thống này:

```text
coverage complete != mission complete
```

Một nhiệm vụ chỉ được xem là thành công hoàn toàn khi robot đã phủ xong bản đồ **và quay về base an toàn**.

---

## 3. Các chức năng chính

- Coverage trên grid map.
- Một robot duy nhất.
- Vật cản tĩnh.
- Vật cản động: guard/patrol, vehicle/convoy, unknown or random hazard.
- Planner dùng Dijkstra với unit-cost.
- Chọn ô chưa phủ gần nhất làm target.
- Quản lý năng lượng theo từng bước di chuyển.
- Ước lượng chi phí quay về base bằng Dijkstra.
- Tự quay về base khi năng lượng thấp.
- Sạc lại khi về base.
- Né vật cản động.
- Replan khi đường bị chặn.
- HOLD_SAFE khi môi trường không an toàn kéo dài.
- Tactical yield khi đường về bị nghẽn.
- POWER_SAVE khi không thể hoàn thành nhiệm vụ tuyệt đối nhưng vẫn cần preserve.
- WAIT_FOR_COMMAND / FINAL_PUSH để mô phỏng directive trong tình huống critical.
- Hiển thị bằng OpenCV với HUD.
- Ghi log thống kê coverage.

---

## 4. State machine của robot

Controller chính nằm trong:

```text
srcs/robot/coverage.cpp
```

Sau refactor, file này chỉ còn vai trò orchestration: khởi tạo robot, chạy tick loop, render frame và gọi các module xử lý mission logic.

Các state chính:

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

| State | Ý nghĩa |
|---|---|
| `NORMAL` | Robot coverage bình thường |
| `ALERT` | Có vật cản động gần robot hoặc active path bị chặn |
| `HOLD_SAFE` | Robot dừng an toàn và thử recover/replan định kỳ |
| `RETURN_TO_BASE` | Robot quay về base vì pin thấp hoặc coverage đã hoàn tất |
| `RECHARGING` | Robot đang sạc tại base |
| `POWER_SAVE` | Robot dừng để bảo toàn năng lượng/dữ liệu khi không thể return an toàn |
| `WAIT_FOR_COMMAND` | Tình huống critical, cần directive |
| `FINAL_PUSH` | Heroic mode: tiếp tục coverage khi preserve không được chọn |

---

## 5. Mission outcome

Project phân biệt coverage result và mission result.

```text
MISSION_SUCCESS
MISSION_PARTIAL_RETURNED
MISSION_PARTIAL_PRESERVED
MISSION_FAILED
```

| Outcome | Ý nghĩa |
|---|---|
| `MISSION_SUCCESS` | Coverage complete và robot đã quay về base |
| `MISSION_PARTIAL_RETURNED` | Robot quay về được nhưng coverage chưa hoàn tất |
| `MISSION_PARTIAL_PRESERVED` | Robot không đạt success tuyệt đối nhưng preserve được mission value / trạng thái an toàn |
| `MISSION_FAILED` | Robot mất khả năng hoàn thành hoặc preserve giá trị nhiệm vụ |

Hiện hệ thống ưu tiên triết lý:

```text
preserve > heroic
```

Heroic behavior chỉ nên dùng khi coverage chưa hoàn tất và hệ thống được cấp directive tương ứng.

---

## 6. Chính sách năng lượng

Robot không dùng một ngưỡng phần trăm pin cố định. Thay vào đó, robot ước lượng chi phí quay về base tại vị trí hiện tại:

```cpp
costToBase = Dijkstra(robot.position, base)
returnMargin = max(MIN_RETURN_MARGIN, costToBase / RETURN_MARGIN_DIVISOR)
```

Robot bắt đầu quay về base khi:

```cpp
energy <= costToBase + returnMargin
```

Robot rơi vào vùng critical khi:

```cpp
energy <= costToBase || energy <= MIN_EMERGENCY_ENERGY
```

Cách này làm policy phụ thuộc vào vị trí thật của robot: càng xa base thì robot càng thận trọng.

---

## 7. Return-to-base và graceful failure

Khi cần quay về base, robot không được lao vào vật cản động.

Nếu đường về bị chặn, robot xử lý theo chuỗi:

```text
RETURN_TO_BASE
→ RETURN_WAIT
→ tactical yield
→ detour
→ WAIT_FOR_COMMAND / POWER_SAVE
```

Tactical yield cho phép robot tạm lùi hoặc dịch sang một ô covered an toàn để giải phóng choke point.

Nếu coverage đã hoàn tất nhưng đường về không còn an toàn và năng lượng critical, robot chuyển sang `POWER_SAVE` thay vì chết vô ích.

---

## 8. Dynamic obstacles

Các vật cản động đại diện cho các thực thể trong môi trường nguy hiểm:

| Ký hiệu | Ý nghĩa |
|---|---|
| `G` | Guard / patrol |
| `V` | Vehicle / convoy |
| `W` | Unknown/random hazard |

Ghi chú: `W` ban đầu được mô hình hóa như random walker. Trong narrative mới, nó nên được hiểu rộng hơn là **unknown hazard**: một vật thể hoặc tác nhân có hành vi không đủ tin cậy để robot tối ưu hóa quanh nó. Robot phản ứng bằng cách tăng thận trọng, tránh xa, chờ hoặc replan.

Dynamic obstacle không được chủ động đâm xuyên qua robot. Robot cập nhật ô cần tránh cho môi trường bằng:

```cpp
setRobotAvoidanceCell(rb.pos);
```

---

## 9. Visualization

Project dùng OpenCV để hiển thị:

- grid map;
- vật cản tĩnh;
- vật cản động;
- ô đã coverage;
- vị trí và hướng robot;
- active path;
- trail robot đã đi;
- overlap trên cạnh;
- HUD trạng thái;
- năng lượng còn lại;
- số lần return/recharge.

Sau refactor, phần visualization được tách vào:

```text
srcs/visualization/
```

| Module | Trách nhiệm |
|---|---|
| `visual_layout` | screen size, cell size, offsets, coordinate transform |
| `visual_assets` | load icon guard/vehicle/random |
| `map_renderer` | vẽ map cells và grid lines |
| `entity_renderer` | vẽ robot, path, trail, dynamic obstacles |
| `hud_renderer` | HUD state và HUD drawing |
| `opencv.cpp` | façade cho `initWindow`, `drawFrame`, `waitFrame` |

---

## 10. Cấu trúc code hiện tại

```text
srcs/
  main.cpp

  environment/
    dynamic_obstacle.cpp
    guard.cpp
    vehicle.cpp
    random_walk.cpp

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

`coverage.cpp` hiện đóng vai trò entry point của mission controller, không còn chứa toàn bộ state machine chi tiết.

---

## 11. Format input

Chương trình đọc map từ thư mục:

```text
tests/
```

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
| `G` | Guard / patrol |
| `V` | Vehicle / convoy |
| `W` | Unknown/random hazard |

---

## 12. Demo scenarios

Một số scenario đề xuất:

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

| Scenario | Mục tiêu |
|---|---|
| `demo_01_open_room` | Coverage bình thường trong phòng mở |
| `demo_02_long_corridor` | Hành lang hẹp |
| `demo_03_rooms_with_door` | Coverage qua các phòng nối nhau |
| `demo_04_static_maze` | Điều hướng với vật cản tĩnh |
| `demo_05_guard_alert` | Robot cảnh giác/replan quanh guard |
| `demo_06_vehicle_corridor` | Tương tác với vehicle trong hành lang |
| `demo_07_vehicle_detour` | Đường bị chặn và có đường vòng |
| `demo_08_random_obstacle_room` | Unknown/random hazard |
| `demo_09_low_energy_large_room` | Test pin thấp, return và recharge |
| `demo_10_blocked_return_preserve` | Đường về bị chặn, critical và preserve |

Regression tối thiểu nên chạy sau mỗi refactor lớn:

```text
demo_01_open_room
demo_09_low_energy_large_room
demo_10_blocked_return_preserve
```

---

## 13. Build

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

Trên Windows/Code::Blocks, cần cấu hình:

Compiler search directories:

```text
srcs/environment
srcs/robot
srcs/world
srcs/visualization
<OpenCV include directory>
```

Linker settings:

```text
<OpenCV library directory>
opencv_worldxxx hoặc các OpenCV libs tương ứng
```

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

## 14. Run

Sau khi build:

```bash
./coverage_sim
```

Sau đó nhập tên test, ví dụ:

```text
demo_01_open_room
```

---

## 15. Design philosophy

Project này xem coverage không chỉ là bài toán đi qua mọi ô.

Trong môi trường có năng lượng hữu hạn và vật cản động, robot phải ra quyết định:

```text
Có nên tiếp tục coverage không?
Có nên quay về base không?
Có nên chờ không?
Có nên đi vòng không?
Có nên preserve không?
Có nên final push không?
```

Triết lý thiết kế:

```text
Constraint tạo ra tradeoff.
Tradeoff tạo ra quyết định.
Quyết định tạo ra trí tuệ.
```

Các keyword liên quan:

```text
bounded rationality
graceful degradation
survivability engineering
mission-aware autonomy
fail-safe systems
recoverability
```

Một hệ tự trị trưởng thành không phải là hệ không bao giờ thất bại. Đó là hệ biết thất bại an toàn, giữ lại giá trị, và bảo toàn khả năng phục hồi khi thành công tuyệt đối không còn được đảm bảo.

---

## 16. Giới hạn hiện tại

- Một robot duy nhất.
- Full map được biết trước.
- Chuyển động trên grid.
- Planner dùng Dijkstra unit-cost.
- Dynamic obstacle behavior còn đơn giản.
- Energy model hiện chủ yếu gắn với movement.
- Waiting/replanning chưa trừ pin trực tiếp.
- Chưa có physics thật.
- Chưa có multi-robot coordination.
- Chưa có test runner tự động.
- Chưa có headless mode chính thức.

---

## 17. Hướng phát triển tiếp theo

- Headless mode để chạy regression test không cần UI.
- Test runner tự động cho nhiều map.
- Mission summary rõ hơn.
- Baseline comparison:
  - Greedy CPP;
  - Energy-aware return;
  - Proposed mission-aware system.
- Energy model nâng cao:
  - turn cost;
  - acceleration cost;
  - speed penalty;
  - waiting/replanning cost.
- Unknown hazard semantics thay cho random walker.
- Rescue/recon maps:
  - choke points;
  - corridors;
  - rooms;
  - convoy intersections.
- Formal hóa policy:
  - objective;
  - trigger;
  - tradeoff;
  - action.
