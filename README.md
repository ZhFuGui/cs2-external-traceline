# cs2-external-traceline

面向 CS2 静态碰撞场景的 DLL，提供遮挡、表面命中、穿越区间、子弹剩余伤害和目标生命／护甲结算。

## 构建

需要 Windows x64、Visual Studio 2022、MSVC v143、C++20 和 Windows SDK。打开 `cs2-external-traceline.sln`，选择 `Debug|x64` 或 `Release|x64` 生成。

产物位于 `out/x64/<配置>/`：运行时使用 `cs2-external-traceline.dll`，直接链接时配套使用同目录的 `.lib`。公开接口见 [api.h](include/cs2-external-traceline/api.h)、[types.h](include/cs2-external-traceline/types.h)；C++ 包装见 [traceline.hpp](include/cs2-external-traceline/traceline.hpp)。

## 输入数据

- **碰撞场景 `.cs2tl`**：包含几何、材质标识、加速索引及可用的原始物理元数据。使用 `cs2tl_load_scene` 加载。
- **伤害参数 `.cs2tlp`**：包含武器、开火模式和物理材质参数。使用 `cs2tl_load_profiles` 加载，通过 `cs2tl_get_profiles_info` 查询数据版本。
- **PHYS 的 KV3 文本**：使用 `cs2tl_import_scene` 导入，或通过 `cs2tl_build_cache` 转为场景缓存。导入器不直接解析 VPK 或编译后的物理资源。

所有路径由调用方显式传入，字符串使用 UTF-8。DLL 不扫描目录，不自动寻找地图、参数或伴随文件。也可通过 `cs2tl_create_scene` 直接提供三角形，或用 `cs2tl_import_sources`／`cs2tl_build_cache_sources` 提供多个物理来源及其仿射变换。

默认将可破坏物体作为完整掩体。需要忽略时，将有可靠元数据证明可破坏的来源标为 `CS2TL_SOURCE_BREAKABLE`，并在构建选项中设置 `CS2TL_BUILD_IGNORE_BREAKABLES`；未标记的来源保留。

## 主要接口

| 用途 | C 接口 |
| --- | --- |
| 几何遮挡 | `cs2tl_is_occluded` |
| 最近／全部表面命中 | `cs2tl_trace_closest`、`cs2tl_trace_all` |
| 穿越区间及厚度 | `cs2tl_query_intervals` |
| 绑定场景和伤害参数 | `cs2tl_create_ballistics` |
| 单条／批量剩余伤害 | `cs2tl_calculate_remaining_damage`、`cs2tl_calculate_remaining_damage_batch` |
| 按部位、规则和护甲结算目标伤害 | `cs2tl_calculate_target_damage` |
| 一次完成穿透和目标伤害结算 | `cs2tl_calculate_shot_damage` |

C 接口通过状态码和 `cs2tl_error` 报告错误；C++ 包装在失败时抛出 `library_error`。参数、输出及错误结构在调用前填写各自的 `struct_size`，保留字段置零。

### 几何查询与剩余伤害

场景和参数加载、绑定后可重复查询。以下 A、B 为调用方提供的世界坐标，武器 ID 使用游戏的 item definition index；示例中的 `7` 为 AK-47。

```cpp
#include <array>
#include <cs2-external-traceline/traceline.hpp>

namespace tl = cs2::traceline;
auto geometry = tl::scene::load("scene.cs2tl");
auto parameters = tl::profiles::load("ballistics.cs2tlp");
auto context = tl::ballistics_context::bind(geometry, parameters);

const cs2tl_vec3 a{0, 0, 64}, b{100, 0, 64};
const auto query = tl::segment(a, b, 0);
const bool visible = geometry.is_visible(query);
const auto flight = context.remaining_damage(7, query, CS2TL_FIRE_PRIMARY);
```

几何可见性只判断遮挡。剩余伤害按每颗子弹／弹丸计算距离衰减、穿透和停止：

- `CS2TL_SHOT_REACHED`：读取到达 B 时的 `remaining_damage`。
- `CS2TL_SHOT_STOPPED`：已确定子弹停止，剩余伤害为 0；原因见 `stop_reason`。
- 计算失败：检查状态或捕获异常，不能将 NaN／未知结果当成 0 伤害。

剩余伤害不包含目标的部位倍率和护甲结算。A、B 必须能够确定有效的穿越边界；起点在未解决的实体内部、B 位于实体内部或边界不确定时，会返回 `CS2TL_INCOMPLETE_GEOMETRY`。

### 目标生命与护甲

调用方提供首次命中目标的世界位置、实际 hitgroup、生命、护甲、头盔、阵营和规则快照。hitgroup 不使用骨骼或 hitbox 编号替代。组合接口的 `query.end` 应为首次目标命中位置。

```cpp
auto rules = tl::damage_rule_defaults();
cs2tl_target_state target{
    sizeof(target), CS2TL_HITGROUP_CHEST, 100, 100,
    CS2TL_TEAM_CT, CS2TL_TARGET_HAS_HELMET, 0, 0
};
cs2tl_shot_damage_request shot{
    sizeof(shot), 7, CS2TL_FIRE_PRIMARY, CS2TL_TEAM_T,
    query, &target, &rules
};
const auto result = context.shot_damage(shot);
```

`result.flight` 是穿透阶段结果，`result.target` 是目标结算结果：

- `health_damage` 是部位与护甲结算后的浮点伤害，未按目标当前 HP 封顶。
- `health_loss_min`／`health_loss_max` 是实际整数扣血范围，受当前 HP 限制；两者相等时扣血值确定。
- 目标小数伤害累计值已知时，填写 `[0,1)` 的 `damage_accumulator` 并设置 `CS2TL_TARGET_ACCUMULATOR_KNOWN`；未知时返回扣血范围，并通过标志说明。

默认规则标记为 `CS2TL_PREDICTION_ASSUMED_DAMAGE_RULES`。使用实际对局规则时填写对应倍率与开关，仅在核实实时快照后设置 `CS2TL_DAMAGE_RULES_LIVE_SNAPSHOT`。单独结算目标伤害可使用 `profiles::target_damage()`，入射伤害由调用方提供。

### 批量剩余伤害

批量接口同步按输入顺序执行，每项可指定不同武器、模式和线段。以下片段复用上面的 `context` 与 `query`：

```cpp
std::array<cs2tl_remaining_damage_request, 2> requests{{
    {sizeof(cs2tl_remaining_damage_request), 7, CS2TL_FIRE_PRIMARY, 0, query},
    {sizeof(cs2tl_remaining_damage_request), 9, CS2TL_FIRE_PRIMARY, 0, query}
}};
std::array<cs2tl_shot_prediction, 2> predictions{};
std::array<cs2tl_status, 2> item_status{};
for (auto &prediction : predictions) {
    prediction.struct_size = sizeof(prediction);
}
auto error = tl::error_output();
tl::check(cs2tl_calculate_remaining_damage_batch(
    context.native_handle(), requests.data(), 2,
    predictions.data(), item_status.data(), nullptr, &error), error);
```

整批返回 `CS2TL_OK` 只表示批次已处理，必须逐项检查 `item_status[i]`；单项失败不影响后续项。需要逐项诊断时传入已初始化 `struct_size` 的 `cs2tl_error` 数组。数组由调用方分配且互不重叠，调用结束后 DLL 不保留其指针。可分小批调用，在批次之间取消。

## 查询约定与适用范围

- 坐标、距离和厚度使用场景的世界单位。几何线段端点为开区间，`endpoint_epsilon` 从两端各排除指定距离；非有限坐标、零长度线段和无效容差返回参数错误。
- 表面接触算遮挡，严格共面的三角形运动忽略该三角形。起终点位于封闭实体内部时也会参与遮挡判断，包围盒不作为实体体积。
- 精确厚度需要可确认的封闭体积。开放、非流形、空腔归属不明等情况无法确定厚度时返回不完整几何；独立实体的重叠区间分别返回。
- 几何接口支持网格、凸包和解析球体／胶囊体。带完整物理元数据的场景使用原生碰撞事件计算伤害；其形状、属性及变换必须在支持范围内。目前原生伤害路径覆盖单位变换的网格和具备原始平面的凸包。
- 预测标志说明原生事件、交互信息可用性、规则假设和版本匹配情况。版本不匹配会告警并标记；所需材质未知返回 `CS2TL_DATA_NOT_FOUND`。`CS2TL_PREDICTION_NOT_LIVE_VALIDATED` 表明结果不构成完整实机开枪流程的等价保证。
- 模型使用静态碰撞数据。其他动态人物、实时破坏、散布和额外服务器伤害过滤器由调用方另行处理；DLL 不识别当前武器、不生成目标候选或选择瞄准位置。
- 同一场景和伤害上下文支持并发只读查询。上下文持有场景及参数；销毁对象或调用 `cs2tl_release_query_memory`／`cs2tl_release_ballistics_memory` 前，须等待该场景上的全部相关查询结束。
- 变长几何结果支持先查询数量再填写数组，也提供调用方分配器入口。工作区由场景复用，不累计查询历史。
- 大数据产生告警并继续处理；告警可由当前线程的 `cs2tl_set_warning_callback` 接收。损坏文件、非法几何、表示范围溢出和实际资源分配失败返回错误。
