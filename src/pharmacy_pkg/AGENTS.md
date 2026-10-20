# AGENTS.md

## 1. 重构目标

以 `Smart-Pharmacy/src/pharmacy_pkg/` 当前代码为唯一功能基线，将规范化后的代码放置到 `pharmacy_ISCC/` 中。

本次任务仅进行代码结构重构，不重新设计比赛方案，不擅自优化或修改现有业务逻辑。原有 `pharmacy_pkg` 必须保持不变。

## 2. 重构原则

1. **先分析，后修改：** 阅读当前代码、Launch、配置文件及资源引用，梳理节点依赖、ROS 话题、TCP 协议和任务执行流程。
2. **以当前版本为准：** 优先使用 `scripts/` 下的当前代码。
3. **保持功能一致：** 保留单车与双车交替执行、二维码任务选择、识别结果共享、裁判通信、导航状态机、语音播报及 MD5/Excel 记录功能。
4. **保持接口不变：** 不修改现有 ROS 话题名称、消息类型及字段格式、状态编号、TCP 消息协议、任务优先级和结果记录语义。
5. **保持当前默认行为：** 包括板二默认使用 TCP 结果来源、双车启动配置、导航参数及识别稳定判断规则，不得擅自切换模式。
6. **模块化拆分：** 将节点入口与业务逻辑分离，公共逻辑独立封装，避免重复代码和循环依赖。
7. **谨慎清理：** 只有确认文件未被调用、未被 Launch 引用且不承担必要功能后，才可移除。保留必要的测试数据、模板和语音资源。

## 3. 目标目录结构

将 `pharmacy_ISCC/` 作为独立 catkin 工作空间，实际 ROS 包位于 `src/pharmacy_iscc/`。

```text
pharmacy_ISCC/
├── AGENTS.md
├── README.md
├── .gitignore
└── src/
    ├── CMakeLists.txt
    └── pharmacy_iscc/
        ├── CMakeLists.txt
        ├── package.xml
        ├── setup.py
        ├── launch/
        ├── config/
        ├── scripts/
        ├── src/
        │   └── pharmacy_iscc/
        │       ├── __init__.py
        │       ├── config/
        │       ├── perception/
        │       ├── navigation/
        │       ├── communication/
        │       └── recording/
        ├── resources/
        │   ├── audio/
        │   └── board2_templates/
        ├── tests/
        │   ├── unit/
        │   ├── integration/
        │   └── fixtures/
        └── docs/
```

目录职责：

- `launch/`：统一单车、双车、裁判通信和测试启动入口。
- `config/`：集中管理比赛参数、车辆参数和视觉参数。
- `scripts/`：只保留 ROS 节点入口及独立工具。
- `perception/`：图像输入、板一二维码识别与任务选择、板二识别。
- `navigation/`：状态机、航点处理、导航辅助和语音播报。
- `communication/`：双车 TCP 通信与裁判系统通信。
- `recording/`：MD5 生成、SQLite 记录和 Excel 导出。
- `resources/`：语音、模板等运行资源。
- `tests/`：单元测试、集成测试及必要的测试数据。
- `docs/`：架构说明、接口约定、启动指南和维护文档。

## 4. 执行步骤

1. 梳理当前文件、调用关系、启动入口和接口，形成简要清单。
2. 建立新工作空间和功能包，完善 `package.xml`、`CMakeLists.txt`、`setup.py`。
3. 先拆分公共逻辑与结果记录模块，再拆分视觉、通信和导航模块。
4. 整理配置和资源路径，统一 Launch 引用及节点启动方式。
5. 补充必要的单元测试和接口测试，执行 catkin 构建及静态检查。
6. 更新 README，说明目录结构、依赖安装和启动命令。

## 5. 验收要求

- 原有 `pharmacy_pkg` 不被修改。
- 新功能包可独立构建，模块导入和资源路径正常。
- 所有 Launch 引用有效，启动参数和节点依赖完整。
- 单车、双车交替执行、裁判通信及记录导出逻辑保持一致。
- 对无法在当前环境实际验证的硬件功能，明确标记为待实车验证，不得声称测试通过。
- 不删除尚未确认用途的文件，不引入无关功能，不进行与重构无关的算法优化。

完成后输出重构结果、主要文件迁移对应关系、构建测试结果及待验证事项。
