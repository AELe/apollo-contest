### 一、场景回放与赛题分析

**核心需求**：

1. 车辆在交汇路口区域必须实现精确限速
2. 限速区域需包含缓冲区（前向3m+后向2m）
3. 限速值应可动态配置（默认5m/s）

**技术难点**：

- 动态识别路口区域边界
- 在参考线上无缝添加速度限制
- 插件与现有规划框架的集成
- 多缓冲区参数协同控制

**解决思路**：

1. 创建`region_speed_limit`交通规则插件
2. 利用HDMap获取`pnc_junction_overlaps`路口数据
3. 通过`AddSpeedLimit`方法实现区域限速
4. 配置文件化参数管理

### 二、整体技术流程图&相关插件解析

#### 插件架构图

traffic rule 插件概述
planning插件分为scenario、task、traffic rule三类，各类插件的使用场景如下图所示。本节课我们将针对如何新增一个traffic rule插件展开讲解。
三类插件的作用：

* Traffic rule用于处理各种交通规则，并将各类规则转化为停车（生成虚拟障碍物实现）、限速两种输出类型。
* Scenario用于判断车辆所在场景，而后依据不同的场景调用事先定义的Task组合。
* Task用于执行执行具体任务，如生成借道路径、处理障碍物边界、轨迹优化等。
  三者的调用逻辑如下图所示。

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/62d50c9cda138a4a5b3cb7c8c1acbc1e84e43006)

traffic rule插件的生成与调用逻辑如下图所示。traffic rule插件继承自traffic rule基类，而后由planning_base中的traffic_decider对各个插件进行生成并调用。planning每进行一次规划任务，会通过traffic_decider调用各个traffic rule,从而使traffic rule插件生效。

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/b25b044ca118920f3cb7fc282026ceea823b7b6b)

新增插件配置流程

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/26b06ac6e353c00990611dcedf882b6ae32fdb46)

#### 核心代码解析

#### 核心代码解析

##### 1. 配置文件

**路径**：`modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt`  
**参数说明**：

```
forward_buffer: 3.0   # 前向缓冲距离(米)
backward_buffer: 2.0  # 后向缓冲距离(米)
limit_speed: 15.0     # 限速值(米/秒)
```

##### 2. 插件初始化

```
bool RegionSpeedLimit::Init(...) {
  // 加载配置文件
  return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
}
```

##### 3. 限速规则应用

```
Status ApplyRule(...) {
  // 获取所有交汇路口区域
  const auto& overlaps = reference_line.map_path().pnc_junction_overlaps();

  for (const auto& overlap : overlaps) {
    // 添加限速区域（含缓冲区）
    reference_line->AddSpeedLimit(
      overlap.start_s - config_.forward_buffer(),
      overlap.end_s + config_.backward_buffer(),
      config_.limit_speed()
    );
  }
}
```

##### 4. 插件注册机制

```
<!-- plugins.xml -->
<library path="modules/planning/traffic_rules/region_speed_limit/libregion_speed_limit.so">
    <class type="apollo::planning::RegionSpeedLimit" base_class="apollo::planning::TrafficRule"></class>
</library>
```

**traffic_rule_config.pb.txt中加入新增插件**

将新建插件加入traffic rule配置文件中，从而使planning调用该traffic rule

```PROTO
rule {
  name: "REGION_SPEED_SETTING"
  type: "RegionSpeedLimit"
}
```

### 三、解题方法

#### 1. 插件创建与配置

**步骤**：

```
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

写RegionSpeedLimit类代码文件以及配置相应BUILD文件

路径：`region_speed_limit.cc`

```
#include <memory>
#include "modules/planning/traffic_rules/region_speed_limit/region_speed_limit.h"

namespace apollo {
namespace planning {

/* 定义成员函数*/

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool RegionSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
    if (!TrafficRule::Init(name, injector)) {
        return false;
    }
    // Load the config this task.
    return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
}

Status RegionSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
    const std::vector<PathOverlap>& pnc_junction_overlaps
            = reference_line_info->reference_line().map_path().pnc_junction_overlaps();
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

- Init()函数：初始化RegionSpeedLimit类，读取配置文件信息到config_；

- ApplyRule()函数：traffic rule类调用接口，在运行中实际调用的函数；

- reference_line_info->reference_line().map_path()：获取道路信息，本插件获取了交汇路口信息pnc_junction_overlaps()；

- 针对交汇路口的限速功能，调用了ReferenceLine::AddSpeedLimit(double start_s, double end_s, double speed_limit),实现了在start_s处到end_s处最高速度为speed_limit的约束。

路径：`region_speed_limit.h`

```
#pragma once

#include <memory>
#include "cyber/plugin_manager/plugin_manager.h"

/* 添加了相应的头文件*/
#include "modules/common/status/status.h"
#include "modules/planning/traffic_rules/region_speed_limit/proto/region_speed_limit.pb.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class RegionSpeedLimit : public TrafficRule {
    /* 声明成员函数*/
public:
    bool Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) override;
    virtual ~RegionSpeedLimit() = default;

    common::Status ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info);

    void Reset() override {}

private:
    RegionSpeedLimitConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit, TrafficRule)

}  // namespace planning
}  // namespace apollo
```

Reset()函数：插件变量重置入口，清空上一次决策对插件内变量的更改。

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit,TrafficRule):声明该类为插件。

BUILD文件描述了源码的构建规则以及其依赖。

```
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
        # 添加该插件所需依赖
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_proto",

    ],
)

apollo_package()

cpplint()
```

**plugin_region_speed_limit_description.xml**

****plugin_region_speed_limit_description.xml文件修改成plugins.xml文件

![](https://apollo-studio-public.bj.bcebos.com/community/article/image/96055aeb07d59117f4c856fca8d5f74457fc9848)

```plugins.xml
<library path="modules/planning/traffic_rules/region_speed_limit/libregion_speed_limit.so">
    <class type="apollo::planning::RegionSpeedLimit" base_class="apollo::planning::TrafficRule"></class>
</library>
```

**配置cyberfile.xml**

```
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
  <!-- add new dependency-->
  <depend type="binary" repo_name="planning-interface-base">planning-interface-base</depend>
  
  <depend>bazel-extend-tools</depend>
</package>
```



**配置参数定义：**

修改proto文件的region_speed_limit.proto

```
syntax = "proto2";

package apollo.planning;

message RegionSpeedLimitConfig {
  // 声明RegionSpeedLimitConfig中的数据结构
  optional double forward_buffer = 1 [default = 3];
  optional double backward_buffer = 2 [default = 2];
  optional double limit_speed = 3 [default = 5];
}
```

#### 3.配置参数文件

##### ①配置参数修改

```
forward_buffer: 3.0
backward_buffer: 2.0
limit_speed: 6.0   # 根据场景需求调整
```

##### ② 插件注册 (`modules/planning/planning_component/conf/traffic_rule_config.pb.txt`)

`traffic_rule_config.pb.txt`

```
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

```
# 编译插件
buildtool build -p modules/planning/traffic_rules/region_speed_limit/

# 启动DreamView验证
aem bootstrap start --plus
```