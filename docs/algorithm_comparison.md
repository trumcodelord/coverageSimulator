# So sánh thuật toán và định vị GRAD-67 Demo

**Project:** GRAD-67 Demo  
**Tên đầy đủ:** Grid-oriented Reconnaissance and Autonomy Demo  
**Subtitle:** Mission-Aware Coverage Simulator  
**Mục tiêu:** đặt project vào bối cảnh Coverage Path Planning (CPP), làm rõ trade-off thiết kế và trả lời câu hỏi: *GRAD-67 hơn gì, kém gì so với các hướng bên ngoài?*

---

## 1. Mục tiêu của phần so sánh

Phần so sánh này không nhằm chứng minh GRAD-67 Demo “mạnh hơn” các thuật toán kinh điển hoặc platform robot thật.

Mục tiêu đúng là:

```text
định vị project
làm rõ trade-off
chỉ ra điểm mạnh / điểm yếu
và chứng minh project có cơ sở nghiên cứu bên ngoài
```

GRAD-67 Demo tập trung vào bài toán:

```text
coverage trên grid map
+ vật cản tĩnh
+ vật cản động
+ ràng buộc năng lượng
+ return-to-base
+ recharge
+ mission outcome
```

Điểm quan trọng:

```text
coverage complete != mission complete
```

Trong project này, một mission chỉ được xem là thành công hoàn toàn khi robot vừa phủ xong vùng cần khảo sát vừa quay về base an toàn.

---

## 2. Cơ sở từ literature CPP

Survey của Jayalakshmi et al. [R1] cho thấy Coverage Path Planning là một bài toán quan trọng trong mobile robotics, đặc biệt trong môi trường động có vật cản, thay đổi môi trường và yêu cầu tính toán real-time.

Các metric thường dùng trong CPP gồm:

```text
coverage percentage
energy efficiency
path length
computation time
time-to-complete-coverage
overlap / redundancy
robustness
```

GRAD-67 Demo dùng các metric này làm nền, nhưng bổ sung thêm một nhóm metric/semantics ở tầng mission:

```text
return-to-base success
recharge count
mission outcome
partial preserved state
graceful failure
```

Vì vậy, project không chỉ trả lời câu hỏi:

```text
Robot đã phủ được bao nhiêu?
```

mà còn trả lời:

```text
Robot có về được base không?
Robot có giữ được khả năng recover không?
Nếu không thể thắng hoàn toàn, hệ thống preserve được gì?
```

---

## 3. Các đối tượng được chọn để so sánh

Phần này chọn 4 đối tượng bên ngoài:

| Đối tượng | Vai trò trong so sánh |
|---|---|
| Boustrophedon Cellular Decomposition | Baseline CPP kinh điển, mạnh ở coverage theory |
| Spanning Tree Coverage (STC) | Baseline grid/graph coverage, phù hợp với map dạng cell |
| Reinforcement Learning / Deep RL CPP | Hướng hiện đại, có tiềm năng học policy thích nghi |
| DJI Matrice 300 RTK | Platform UAV thật, dùng để neo project vào mission thực tế |

Cách chọn này bao phủ đủ các tầng:

```text
classical CPP theory
grid/graph systematic coverage
modern learning-based planning
real-world UAV platform
```

GRAD-67 Demo được định vị như một **mission-aware grid simulator**, không phải commercial UAV, không phải formal optimal CPP solver.

---

## 4. Đối tượng 1: Boustrophedon Cellular Decomposition

### 4.1. Tóm tắt

Boustrophedon Cellular Decomposition là một hướng coverage path planning kinh điển của Howie Choset và Philippe Pignon [R2].

Ý tưởng chính:

```text
chia môi trường thành các cell
quét từng cell bằng back-and-forth motion
xây graph thể hiện quan hệ kề giữa các cell
tìm đường đi exhaustive qua graph đó
```

CMU Robotics Institute mô tả Boustrophedon là một exact cellular decomposition approach cho coverage. Mỗi cell được phủ bằng chuyển động back-and-forth; khi tất cả cell được phủ thì toàn bộ môi trường được phủ. Cách này làm coverage problem trở thành bài toán tìm exhaustive path trên graph adjacency của các cell [R2].

### 4.2. Boustrophedon mạnh ở đâu?

Boustrophedon mạnh ở **coverage theory**:

- có decomposition rõ ràng;
- có cấu trúc coverage hình học;
- có reasoning về complete coverage;
- phù hợp với known static environment;
- trong assumption phù hợp, có thể có coverage completeness logic chặt hơn simulator thực dụng.

Nói ngắn:

```text
Boustrophedon mạnh ở câu hỏi:
"Làm sao phủ toàn bộ không gian một cách có cấu trúc?"
```

### 4.3. GRAD-67 khác ở đâu?

GRAD-67 không tập trung vào coverage optimality thuần túy.

GRAD-67 hỏi thêm:

```text
Phủ tiếp có còn đủ pin quay về không?
Nếu coverage xong rồi thì robot đã về base chưa?
Nếu đường về bị dynamic obstacle chặn thì xử lý thế nào?
Nếu không thể return thì preserve hay final push?
Mission outcome cuối cùng là gì?
```

Nói cách khác:

```text
Boustrophedon optimize coverage structure.
GRAD-67 optimize mission recoverability under constraints.
```

### 4.4. So sánh nhanh

| Tiêu chí | Boustrophedon | GRAD-67 Demo |
|---|---|---|
| Bài toán chính | Complete coverage | Mission-aware coverage |
| Không gian | Cell decomposition / geometry | 2D grid map |
| Coverage theory | Mạnh | Thực dụng hơn |
| Energy-aware return | Không phải trọng tâm | Trọng tâm |
| Recharge | Không phải trọng tâm | Có state `RECHARGING` |
| Mission outcome | Không explicit | Explicit |
| Điểm mạnh chính | Formal coverage structure | Mission semantics |

---

## 5. Đối tượng 2: Spanning Tree Coverage (STC)

### 5.1. Tóm tắt

Spanning Tree Coverage (STC) là một nhóm thuật toán coverage dựa trên graph/spanning tree. Gabriely và Rimon [R3] xét bài toán phủ một vùng planar liên tục bằng một tool hình vuông gắn trên mobile robot.

Theo mô tả của paper [R3], STC:

```text
xấp xỉ work-area bằng các disjoint cells
mỗi cell tương ứng với footprint của tool/robot
xây graph từ các cell
theo spanning tree để phủ vùng
```

STC có nhiều biến thể:

- offline STC: robot biết trước môi trường;
- online STC: robot dùng sensor để phát hiện obstacle và xây spanning tree khi đang cover;
- ant-like STC: không cần biết trước môi trường và dùng marker giống pheromone.

### 5.2. STC mạnh ở đâu?

STC mạnh vì rất gần với bài toán grid/cell coverage:

- có cấu trúc graph rõ ràng;
- phù hợp với cell/grid representation;
- systematic traversal;
- có cơ sở lý thuyết tốt hơn heuristic nearest-uncovered;
- là baseline đẹp để so với simulator grid như GRAD-67.

Nói ngắn:

```text
STC mạnh ở câu hỏi:
"Làm sao tổ chức coverage trên cell/grid một cách có hệ thống?"
```

Đây là lý do STC rất đáng đưa vào báo cáo, đặc biệt khi project cũng dùng grid map.

### 5.3. GRAD-67 khác ở đâu?

STC tập trung vào coverage path. GRAD-67 tập trung vào mission behavior bọc quanh coverage path.

GRAD-67 bổ sung các logic không phải trọng tâm chính của STC:

```text
energy-aware return
return-to-base after coverage
recharge
dynamic obstacle safety
blocked return handling
POWER_SAVE / WAIT_FOR_COMMAND / FINAL_PUSH
mission outcome
```

Vì vậy, có thể nói:

```text
STC là coverage planner mạnh.
GRAD-67 là mission controller trên nền grid coverage.
```

### 5.4. So sánh nhanh

| Tiêu chí | STC | GRAD-67 Demo |
|---|---|---|
| Bài toán chính | Systematic complete coverage | Mission-aware autonomous coverage |
| Representation | Cell/grid/graph | Grid |
| Coverage structure | Mạnh | Thực dụng |
| Energy constraint | Không phải điểm chính | Có |
| Dynamic obstacle | Có biến thể online, nhưng không phải narrative chính | Có guard/vehicle abstraction |
| Return-to-base | Không explicit | Explicit |
| Recharge | Không explicit | Explicit |
| Mission outcome | Không explicit | Built-in |
| Điểm mạnh chính | Structured traversal | Mission decision logic |

---

## 6. Đối tượng 3: Reinforcement Learning / Deep RL CPP

### 6.1. Tóm tắt

Reinforcement Learning (RL) và Deep RL là hướng hiện đại để giải CPP trong môi trường phức tạp hoặc động. Thay vì viết rule cố định, robot học policy qua tương tác với môi trường và reward.

Survey [R1] nhắc đến nhiều hướng RL/Deep RL trong CPP như:

```text
PPO
Q-learning
DQN
Double DQN
actor-critic
A3C / DDPG
```

RL có tiềm năng vì bài toán CPP trong dynamic environment có thể được nhìn như một Markov Decision Process: robot quan sát state, chọn action, nhận reward và cập nhật policy.

### 6.2. RL mạnh ở đâu?

RL có potential mạnh hơn rule-based policy ở các điểm:

- học được policy phức tạp;
- có thể optimize nhiều objective qua reward;
- có thể adapt tốt hơn trong dynamic environment nếu train đủ tốt;
- phù hợp với bài toán có state-action space lớn;
- có thể học behavior mà rule thủ công khó viết hết.

Nói ngắn:

```text
RL mạnh ở adaptive policy learning.
```

### 6.3. Vì sao GRAD-67 chưa dùng RL?

GRAD-67 chọn rule-based mission controller vì scope hiện tại cần:

```text
deterministic behavior
explainability
debuggability
regression stability
khả năng bảo vệ quyết định trước giảng viên
```

RL có các chi phí thực tế:

```text
training pipeline
reward shaping
non-deterministic behavior
slow / unstable convergence
khó giải thích vì sao chọn action cụ thể
khó đảm bảo không exploit reward
```

Nói ngắn:

```text
RL có thể mạnh hơn về policy learning.
GRAD-67 mạnh hơn ở explainable mission behavior trong scope hiện tại.
```

### 6.4. So sánh nhanh

| Tiêu chí | RL / Deep RL CPP | GRAD-67 Demo |
|---|---|---|
| Policy | Learned policy | Rule-based mission policy |
| Adaptability | Cao nếu train tốt | Hạn chế hơn |
| Explainability | Thấp hơn | Cao |
| Determinism | Thấp/trung bình | Cao |
| Training overhead | Cao | Không cần |
| Debug trước báo cáo | Khó | Dễ hơn |
| Phù hợp long-term research | Cao | Có thể mở rộng sau |
| Phù hợp scope hiện tại | Rủi ro cao | Phù hợp |

---

## 7. Đối tượng 4: DJI Matrice 300 RTK

### 7.1. Vì sao chọn DJI Matrice 300 RTK?

DJI Matrice 300 RTK là một enterprise UAV platform nổi tiếng. Nó không phải một thuật toán CPP, mà là một platform robot thật dùng trong các mission như mapping, inspection, public safety hoặc search-and-rescue style operations.

M300 RTK được chọn vì nó có các constraint gần với bài toán mission của project:

```text
pin hữu hạn
obstacle sensing
payload cho inspection/mapping
positioning
mission operation ngoài thực địa
```

Official DJI specs ghi Matrice 300 RTK có:

- max flight time 55 phút;
- hỗ trợ payload như Zenmuse H20/H20T/P1/L1;
- vision obstacle sensing range trước/sau/trái/phải 0.7–40 m;
- upward/downward 0.6–30 m;
- TB60 battery 5935 mAh, 274 Wh [R4].

### 7.2. DJI M300 mạnh ở đâu?

DJI M300 RTK mạnh hơn GRAD-67 gần như tuyệt đối ở tầng real-world deployment:

- hardware thật;
- flight controller thật;
- RTK positioning;
- sensor và obstacle sensing thật;
- payload ecosystem;
- battery management thật;
- vận hành ngoài môi trường thực.

Nói ngắn:

```text
DJI M300 mạnh ở deployment realism.
```

### 7.3. GRAD-67 khác ở đâu?

GRAD-67 không cố cạnh tranh hardware với DJI M300.

GRAD-67 tập trung vào mission abstraction:

```text
explicit mission states
return-to-base semantics
recharge state
mission outcome
inspectable decision logic
coverage complete != mission complete
```

Nếu DJI M300 là một platform mission thật, thì GRAD-67 là một simulator nhỏ để quan sát rõ logic ra quyết định của mission.

Nói ngắn:

```text
DJI M300 là robot thật để bay nhiệm vụ thật.
GRAD-67 là simulator để nghiên cứu decision layer.
```

### 7.4. So sánh nhanh

| Tiêu chí | DJI Matrice 300 RTK | GRAD-67 Demo |
|---|---|---|
| Loại hệ thống | Enterprise UAV thật | Grid-based simulator |
| Môi trường | Real world / 3D | 2D grid map |
| Cảm biến | Vision/ToF/RTK/payload thật | Dynamic obstacle abstraction |
| Pin | Battery thật | Energy variable |
| Obstacle avoidance | Sensor-based | `dynamicBlocked`, path safety |
| Coverage | Mapping / inspection workflow | Cell-level coverage |
| Return semantics | Operational/platform workflow | Explicit `RETURN_TO_BASE` |
| Recharge | Battery operation | Explicit `RECHARGING` |
| Mission outcome | External/operator-defined | Built-in |
| Realism | Rất cao | Thấp hơn |
| Explainability | Khó inspect toàn stack | Cao hơn |

---

## 8. Bảng so sánh tổng hợp

| Tiêu chí | Boustrophedon | STC | RL / Deep RL CPP | DJI M300 RTK | GRAD-67 Demo |
|---|---|---|---|---|---|
| Loại | Classical CPP | Grid/graph CPP | Learning-based CPP | Real UAV platform | Mission-aware simulator |
| Mục tiêu chính | Complete coverage | Systematic coverage | Learned adaptive policy | Real mission deployment | Explainable mission behavior |
| Môi trường | Known space | Cell/grid/graph | Dynamic/complex possible | Real world | 2D grid |
| Dynamic obstacle | Không phải trọng tâm | Có thể mở rộng online | Có thể học/adapt | Sensor-based | Có abstraction |
| Energy constraint | Không phải trọng tâm | Không phải trọng tâm | Có thể đưa vào reward | Battery thật | Explicit energy model |
| Return-to-base | Không explicit | Không explicit | Có thể học nếu reward đúng | Operational workflow | Explicit state |
| Recharge | Không explicit | Không explicit | Có thể mô hình hóa | Battery operation | Explicit state |
| Mission outcome | Không explicit | Không explicit | Có thể thiết kế reward/outcome | Operator-defined | Built-in |
| Explainability | Cao | Cao | Thấp/trung bình | Thấp hơn do stack phức tạp | Cao |
| Real-world realism | Thấp/trung bình | Thấp/trung bình | Tùy setup | Rất cao | Thấp |
| Điểm mạnh | Coverage theory | Structured grid traversal | Adaptivity | Deployment realism | Mission abstraction |

---

## 9. Định vị GRAD-67 Demo

GRAD-67 Demo không phải:

```text
formal optimal coverage solver
commercial UAV platform
physics-accurate robotics simulator
RL-based AI agent
```

GRAD-67 Demo là:

```text
mission-aware coverage simulator
trên grid map
có energy-aware return
có dynamic obstacle abstraction
có return/recharge semantics
có mission outcome rõ ràng
```

Project đánh đổi:

```text
giảm physical realism
để tăng explainability

giảm complexity của sensor/hardware
để tập trung vào mission decision logic

không claim optimal coverage theory
mà tập trung vào recoverability và graceful failure
```

Triết lý chính:

```text
Nếu còn an toàn:
    tiếp tục coverage

Nếu pin/rủi ro vượt ngưỡng:
    return-to-base

Nếu về được:
    recharge

Nếu không thể return:
    preserve hoặc chờ directive

Nếu không còn lựa chọn và có directive:
    final push
```

Thông điệp ngắn:

```text
Boustrophedon mạnh ở coverage theory.
STC mạnh ở structured grid coverage.
RL mạnh ở adaptive policy learning.
DJI M300 mạnh ở real-world deployment.

GRAD-67 mạnh ở explainable mission-aware autonomy.
```

---

## 10. Kết luận

Các thuật toán kinh điển như Boustrophedon và STC mạnh hơn GRAD-67 Demo ở nền tảng coverage path planning. RL/Deep RL mạnh hơn ở tiềm năng học policy phức tạp trong môi trường động. DJI Matrice 300 RTK mạnh hơn tuyệt đối ở phần cứng, cảm biến và triển khai thực tế.

Tuy nhiên, GRAD-67 Demo có vị trí riêng:

```text
một simulator nhỏ, deterministic, dễ kiểm thử,
tập trung vào mission-aware autonomy
dưới ràng buộc năng lượng, obstacle và return-to-base.
```

Đóng góp chính của project không nằm ở việc “đánh bại” các hệ thống lớn, mà nằm ở việc xây dựng một abstraction rõ ràng để nghiên cứu:

```text
Coverage Path Planning under Mission Constraints
```

---

## 11. Tài liệu tham khảo

**[R1]** K. P. Jayalakshmi, V. G. Nair, and D. Sathish,  
“A Comprehensive Survey on Coverage Path Planning for Mobile Robots in Dynamic Environments,”  
*IEEE Access*, 2025. DOI: `10.1109/ACCESS.2025.3556446`.

**[R2]** H. Choset and P. Pignon,  
“Coverage Path Planning: The Boustrophedon Decomposition,”  
Proceedings of the 1st International Conference on Field and Service Robotics, 1997.  
Link: https://www.ri.cmu.edu/publications/coverage-path-planning-the-boustrophedon-decomposition/

**[R3]** Y. Gabriely and E. Rimon,  
“Spanning-tree based coverage of continuous areas by a mobile robot,”  
*Annals of Mathematics and Artificial Intelligence*, vol. 31, no. 1/4, pp. 77–98, 2001.  
DOI: `10.1023/A:1016610507833`  
Link: https://doi.org/10.1023/A:1016610507833  
Metadata mirror: https://colab.ws/articles/10.1023%2Fa%3A1016610507833

**[R4]** DJI,  
“Support for Matrice 300 RTK,” official product/specification page.  
Link: https://www.dji.com/global/support/product/matrice-300

**[R5]** R. Bähnemann, N. Lawrance, J. J. Chung, M. Pantic, R. Siegwart, and J. Nieto,  
“Revisiting Boustrophedon Coverage Path Planning as a Generalized Traveling Salesman Problem,”  
arXiv:1907.09224, 2019.  
Link: https://arxiv.org/abs/1907.09224
