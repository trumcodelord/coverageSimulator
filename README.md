# Coverage Simulator

Mô phỏng **coverage path planning** cho robot trên bản đồ dạng lưới, có vật cản tĩnh, vật cản động, ràng buộc năng lượng, logic quay về base, recharge và mission outcome theo hướng **mission-aware autonomy**.

Đây là đồ án tốt nghiệp cử nhân tại Đại học Bách Khoa Hà Nội.

---

## 1. Mục tiêu của project

Project này không chỉ mô phỏng robot đi qua mọi ô trống trên bản đồ.

Mục tiêu chính là xây dựng một hệ thống mô phỏng robot trinh sát tự trị trong môi trường có ràng buộc, nơi robot phải cân bằng giữa:

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
Dynamic obstacles = allied patrols, vehicles, moving operational hazards
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
- Vật cản động dạng guard/patrol và vehicle/convoy.
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

Transition contract mong muốn:

```text
NORMAL
  -> ALERT
  -> RETURN_TO_BASE

ALERT
  -> NORMAL
  -> HOLD_SAFE
  -> RETURN_TO_BASE

HOLD_SAFE
  -> ALERT
  -> RETURN_TO_BASE

RETURN_TO_BASE
  -> RECHARGING
  -> WAIT_FOR_COMMAND
  -> POWER_SAVE

RECHARGING
  -> NORMAL

WAIT_FOR_COMMAND
  -> POWER_SAVE
  -> FINAL_PUSH

POWER_SAVE
  -> terminal

FINAL_PUSH
  -> terminal strategic
```

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

Heroic behavior chỉ nên dùng khi coverage chưa hoàn tất, robot không còn khả năng return an toàn và hệ thống được cấp directive tương ứng.

Nếu robot còn khả năng trở về, robot không nên chọn tự sát anh dũng.

---

## 6. Chính sách năng lượng

Robot không dùng một ngưỡng phần trăm pin cố định. Thay vào đó, robot ước lượng chi phí quay về base tại vị trí hiện tại:

```cpp
costToBase = Dijkstra(robot.position, base);
returnMargin = max(MIN_RETURN_MARGIN, costToBase / RETURN_MARGIN_DIVISOR);
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

Ý tưởng thiết kế:

```text
Nếu còn an toàn để làm nhiệm vụ:
    tiếp tục coverage

Nếu pin/rủi ro vượt ngưỡng:
    return-to-base

Nếu về được base:
    recharge rồi tiếp tục nếu mission chưa xong

Nếu không thể return và trạng thái critical:
    WAIT_FOR_COMMAND hoặc POWER_SAVE

Nếu preserve được:
    POWER_SAVE

Nếu không thể return, không preserve, và có directive:
    FINAL_PUSH
```

---

## 7. Return-to-base và graceful failure

Khi cần quay về base, robot không được lao vào vật cản động.

Nếu đường về bị chặn, robot xử lý theo chuỗi:

```text
RETURN_TO_BASE
→ wait / replan
→ tactical yield
→ detour
→ WAIT_FOR_COMMAND / POWER_SAVE
```

Tactical yield cho phép robot tạm lùi hoặc dịch sang một ô covered an toàn để giải phóng choke point.

Nếu coverage đã hoàn tất nhưng đường về không còn an toàn và năng lượng critical, robot chuyển sang `POWER_SAVE` thay vì chết vô ích.

---

## 8. Dynamic obstacles

Các vật cản động đại diện cho các thực thể di chuyển trong môi trường nhiệm vụ:

| Ký hiệu | Ý nghĩa |
|---|---|
| `G` | Guard / patrol |
| `V` | Vehicle / convoy |

Dynamic obstacle không được chủ động đâm xuyên qua robot. Robot cập nhật ô cần tránh cho môi trường bằng:

```cpp
setRobotAvoidanceCell(rb.pos);
```

Các invariant quan trọng:

```text
Dynamic obstacle không chiếm base.
Dynamic obstacle không đi xuyên vật cản tĩnh.
Dynamic obstacle không chồng lên obstacle khác.
Dynamic obstacle không chủ động lao vào robot.
Robot không được bước vào dynamicBlocked cell.
Visualization không được mutate world.
Policy không được mutate world trực tiếp.
```

Dynamic obstacle chạy trên thread riêng. `simMutex` bảo vệ obstacle update, robot tick và render read.

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
| `visual_assets` | load icon guard/vehicle |
| `map_renderer` | vẽ map cells và grid lines |
| `entity_renderer` | vẽ robot, path, trail, dynamic obstacles |
| `hud_renderer` | HUD state và HUD drawing |
| `opencv.cpp` | façade cho `initWindow`, `drawFrame`, `waitFrame` |

Visualization chỉ phản ánh trạng thái simulation. Không dùng visualization để thay đổi policy hoặc mission semantics.

---

## 10. Cấu trúc code hiện tại

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
| `R` | Vị trí bắt đầu của robot / base |
| `0` | Ô trống |
| `1` | Vật cản tĩnh |
| `G` | Guard / patrol |
| `V` | Vehicle / convoy |

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
| `demo_09_low_energy_large_room` | Test pin thấp, return và recharge |
| `demo_10_blocked_return_preserve` | Đường về bị chặn, critical và preserve |

Regression tối thiểu nên chạy sau mỗi thay đổi đáng kể:

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

## 15. Thuật toán ở mức pseudo-code

Pseudo-code tổng quát của mission controller:

```text
initialize robot at base
initialize environment
mark current cell as covered

while mission is not terminal:
    update robot avoidance cell for dynamic obstacles
    estimate costToBase using Dijkstra
    evaluate energy risk and mission state

    if coverage is complete and robot is not at base:
        switch to RETURN_TO_BASE

    if energy is low and robot can still return:
        switch to RETURN_TO_BASE

    if state == NORMAL:
        choose nearest uncovered reachable cell
        build path using Dijkstra
        move one safe step

    if state == ALERT:
        re-evaluate path safety
        if safe:
            continue mission
        else if recoverable:
            replan
        else:
            HOLD_SAFE or RETURN_TO_BASE

    if state == HOLD_SAFE:
        wait briefly
        retry replan / recovery
        if risk remains high:
            RETURN_TO_BASE

    if state == RETURN_TO_BASE:
        build path to base
        if path is safe:
            move one safe step toward base
        else:
            try tactical yield or detour
        if robot reaches base:
            if coverage complete:
                MISSION_SUCCESS
            else:
                RECHARGING

    if state == RECHARGING:
        restore energy
        switch to NORMAL

    if state == WAIT_FOR_COMMAND:
        choose POWER_SAVE or FINAL_PUSH depending on directive

    if state == POWER_SAVE:
        stop safely and preserve mission value

    if state == FINAL_PUSH:
        continue only as terminal strategic behavior
```

Energy-aware return rule:

```text
costToBase = Dijkstra(robot.position, base)
returnMargin = max(MIN_RETURN_MARGIN, costToBase / RETURN_MARGIN_DIVISOR)

if energy <= costToBase + returnMargin:
    RETURN_TO_BASE

if energy <= costToBase or energy <= MIN_EMERGENCY_ENERGY:
    critical
```

---

## 16. Metrics / evaluation

Các metrics phù hợp để so sánh thuật toán:

```text
coverage percentage
total steps
energy used
remaining energy
return count
recharge count
time spent in ALERT / HOLD_SAFE
mission outcome
```

Các baseline có thể dùng trong report:

| Baseline | Ý nghĩa |
|---|---|
| Greedy CPP | Luôn đi tới ô chưa phủ gần nhất, ít hoặc không xét mission risk |
| Energy-aware return | Có xét chi phí quay về base |
| Mission-aware system | Có energy, return, recharge, dynamic obstacle safety, preserve/failure semantics |

Mục tiêu của project không phải chỉ tối đa hóa coverage percentage. Mục tiêu là tối đa hóa giá trị nhiệm vụ trong khi vẫn giữ recoverability khi điều kiện cho phép.

---

## 17. Design philosophy

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

## 18. Giới hạn hiện tại

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

## 19. Hướng phát triển tiếp theo

Các hướng nên ưu tiên theo blast radius thấp trước:

- Rescue/recon static maps:
  - choke points;
  - corridors;
  - rooms;
  - convoy intersections;
  - low-energy return risk;
  - blocked return preserve case.
- Mission summary rõ hơn.
- Baseline comparison trong report:
  - Greedy CPP;
  - Energy-aware return;
  - Proposed mission-aware system.
- Formal hóa policy:
  - objective;
  - trigger;
  - tradeoff;
  - action.
- Test runner nhỏ cho regression demos.
- Headless mode để chạy regression test không cần UI.
- Icon/base/recharge visualization rõ hơn nếu chỉ thay asset hoặc fallback draw.
- Energy model nâng cao:
  - turn cost;
  - acceleration cost;
  - speed penalty;
  - waiting/replanning cost.

Các hướng nên để sau vì blast radius cao:

- Dynamic map runtime.
- Refactor lớn folder/module.
- Render API lớn chỉ để thêm visual hint.
- Concurrency/mutex refactor khi chưa có symptom rõ.
