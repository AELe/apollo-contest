### 一、场景回放与赛题分析

**核心需求**：

1. 车辆在交汇路口区域必须实现精确限速。
2. 限速区域需包含缓冲区（前向 3 m + 后向 2 m）。
3. 限速值应可动态配置（默认 5 m/s）。

**技术难点**：

- 动态识别路口区域边界
- 在参考线上无缝添加速度限制
- 插件与现有规划框架的集成
- 多缓冲区参数协同控制

**解决思路**：

1. 创建 `region_speed_limit` 交通规则插件。
2. 利用 HDMap 获取 `pnc_junction_overlaps` 路口数据。
3. 通过 `AddSpeedLimit` 方法实现区域限速。
4. 采用配置文件化参数管理。

### 二、整体技术流程图 & 相关插件解析

#### 插件架构图

traffic rule 插件概述：
planning 插件分为 scenario、task、traffic rule 三类。三类插件的作用：

- Traffic rule 用于处理各种交通规则，并将规则转化为停车（生成虚拟障碍物）与限速两类输出。
- Scenario 用于判断车辆所在场景，并按场景调用预定义 Task 组合。
- Task 用于执行具体任务，例如生成借道路径、处理障碍物边界、轨迹优化等。

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/62d50c9cda138a4a5b3cb7c8c1acbc1e84e43006)

traffic rule 插件的生成与调用逻辑如下图所示。插件继承自 traffic rule 基类，由 planning_base 中的 traffic_decider 统一生成并调用。

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/b25b044ca118920f3cb7fc282026ceea823b7b6b)

新增插件配置流程：

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/26b06ac6e353c00990611dcedf882b6ae32fdb46)

#### 核心代码解析

##### 1. 配置文件

- **路径**：`modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt`

- **参数说明**：

  ```text
  forward_buffer: 3.0   # 前向缓冲距离（米）
  backward_buffer: 2.0  # 后向缓冲距离（米）
  limit_speed: 15.0     # 限速值（米/秒）
  ```

##### 2. 插件初始化

```cpp
bool RegionSpeedLimit::Init(...) {
  // 加载配置文件
  return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
}
```

##### 3. 限速规则应用

```cpp
Status ApplyRule(...) {
  const auto& overlaps = reference_line.map_path().pnc_junction_overlaps();

  for (const auto& overlap : overlaps) {
    reference_line->AddSpeedLimit(
        overlap.start_s - config_.forward_buffer(),
        overlap.end_s + config_.backward_buffer(),
        config_.limit_speed());
  }
}
```

##### 4. 插件注册机制

```xml
<!-- plugins.xml -->
<library path="modules/planning/traffic_rules/region_speed_limit/libregion_speed_limit.so">
  <class type="apollo::planning::RegionSpeedLimit" base_class="apollo::planning::TrafficRule"></class>
</library>
```

**在 `traffic_rule_config.pb.txt` 中加入新增插件**：

```text
rule {
  name: "REGION_SPEED_SETTING"
  type: "RegionSpeedLimit"
}
```

### 三、解题方法

#### 1. 插件创建与配置

**步骤**：

```bash
# 创建插件模板
buildtool create --template plugin \
  --namespaces planning \
  --name region-speed-limit \
  --base_class_name TrafficRule \
  modules/planning/traffic_rules/region_speed_limit \
  --config_message_name RegionSpeedLimitConfig

# 初始化配置文件
buildtool profile config init --package planning --profile=default
aem profile use default
```

#### 2. 插件核心代码

路径：`region_speed_limit.cc`

```cpp
#include "modules/planning/traffic_rules/region_speed_limit/region_speed_limit.h"

#include <memory>

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool RegionSpeedLimit::Init(
    const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
}

Status RegionSpeedLimit::ApplyRule(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
  const std::vector<PathOverlap>& pnc_junction_overlaps =
      reference_line_info->reference_line().map_path().pnc_junction_overlaps();
  for (const auto& pnc_junction_overlap : pnc_junction_overlaps) {
    reference_line->AddSpeedLimit(
        pnc_junction_overlap.start_s - config_.forward_buffer(),
        pnc_junction_overlap.end_s + config_.backward_buffer(),
        config_.limit_speed());
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
```

路径：`region_speed_limit.h`

```cpp
#pragma once

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"
#include "modules/planning/traffic_rules/region_speed_limit/proto/region_speed_limit.pb.h"

namespace apollo {
namespace planning {

class RegionSpeedLimit : public TrafficRule {
 public:
  bool Init(
      const std::string& name,
      const std::shared_ptr<DependencyInjector>& injector) override;
  ~RegionSpeedLimit() override = default;

  common::Status ApplyRule(
      Frame* const frame,
      ReferenceLineInfo* const reference_line_info) override;

  void Reset() override {}

 private:
  RegionSpeedLimitConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit, TrafficRule);

}  // namespace planning
}  // namespace apollo
```

BUILD 文件描述了源码构建规则及依赖：

```bazel
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "region_speed_limit_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libregion_speed_limit.so",
    srcs = [
        "region_speed_limit.cc",
    ],
    hdrs = [
        "region_speed_limit.h",
    ],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_proto",
    ],
)

apollo_package()

cpplint()
```

`plugin_region_speed_limit_description.xml` 可调整为 `plugins.xml`。

```xml
<library path="modules/planning/traffic_rules/region_speed_limit/libregion_speed_limit.so">
  <class type="apollo::planning::RegionSpeedLimit" base_class="apollo::planning::TrafficRule"></class>
</library>
```

**配置 `cyberfile.xml`**：

```xml
<package format="2">
  <name>region-speed-limit</name>
  <version>local</version>
  <description>
    This is a demo package
  </description>

  <maintainer email="sample@sample.com">Apollo Developer</maintainer>
  <license>Apache License 2.0</license>
  <url type="website">https://www.apollo.auto/</url>
  <url type="repository">https://github.com/ApolloAuto/apollo</url>
  <url type="bugtracker">https://github.com/ApolloAuto/apollo/issues</url>

  <type>module</type>
  <src_path>//modules/planning/traffic_rules/region_speed_limit</src_path>
  <builder>bazel</builder>

  <depend type="binary" repo_name="cyber">cyber</depend>
  <depend type="binary" repo_name="planning-interface-base">planning-interface-base</depend>

  <depend>bazel-extend-tools</depend>
</package>
```

**配置参数定义**：

修改 `region_speed_limit.proto`：

```proto
syntax = "proto2";

package apollo.planning;

message RegionSpeedLimitConfig {
  optional double forward_buffer = 1 [default = 3];
  optional double backward_buffer = 2 [default = 2];
  optional double limit_speed = 3 [default = 5];
}
```

#### 3. 配置参数文件

##### ① 配置参数修改

```text
forward_buffer: 3.0
backward_buffer: 2.0
limit_speed: 6.0   # 根据场景需求调整
```

##### ② 插件注册（`modules/planning/planning_component/conf/traffic_rule_config.pb.txt`）

```text
rule {
  name: "BACKSIDE_VEHICLE"
  type: "BacksideVehicle"
}
rule {
  name: "CROSSWALK"
  type: "Crosswalk"
}
rule {
  name: "REGION_SPEED_SETTING"
  type: "RegionSpeedLimit"
}
rule {
  name: "DESTINATION"
  type: "Destination"
}
rule {
  name: "KEEP_CLEAR"
  type: "KeepClear"
}
rule {
  name: "REFERENCE_LINE_END"
  type: "ReferenceLineEnd"
}
rule {
  name: "REROUTING"
  type: "Rerouting"
}
rule {
  name: "STOP_SIGN"
  type: "StopSign"
}
rule {
  name: "TRAFFIC_LIGHT"
  type: "TrafficLight"
}
rule {
  name: "YIELD_SIGN"
  type: "YieldSign"
}
rule {
  name: "SPEED_SETTING"
  type: "SpeedSetting"
}
```

#### 4. 编译与验证

```bash
# 编译插件
buildtool build -p modules/planning/traffic_rules/region_speed_limit/

# 启动 DreamView 验证
aem bootstrap start --plus
```
