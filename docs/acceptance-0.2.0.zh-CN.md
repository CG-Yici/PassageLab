# PassageLab 0.2.0 完成与验收报告

2026-10-28。按照项目设计的首版范围，**P0–P6 已完成实现及本机验收**；P7 是设计明确列出的可选进阶，不属于首版欠项。本报告记录发布前的本机验收；验收时修改尚未提交。发布入口为 [v0.2.0](https://github.com/CG-Yici/PassageLab/releases/tag/v0.2.0)，源码与发布包通过生产源码指纹对应。

交付包：[PassageLab 0.2.0 Windows x64](https://github.com/CG-Yici/PassageLab/releases/download/v0.2.0/PassageLab-0.2.0-windows-x64.zip)。解压后运行 `Start PassageLab.cmd`。桌面程序需要支持 OpenGL 3.3 的驱动，包内包含应用依赖和本地 Visual C++ 运行库。

演示：[三场景真实录屏](media/0.2.0/passagelab-demo.mp4)、[场景 A](media/0.2.0/scene-a.png)、[场景 B](media/0.2.0/scene-b.png)、[场景 C](media/0.2.0/scene-c.png)。素材由最终 0.2.0 二进制实际渲染，二进制指纹见媒体 `provenance.json`，未以参考运动替代求解结果。

## 本轮补齐的功能

- 两套规划器支持 `planar-xy`：固定初始高度和倾斜，允许 XY 平移及绕世界 Z 轴旋转；验证器同时限制状态空间，拒绝不兼容终点。
- 补全点/线段、点/箱体和射线/三角形基础查询及退化、边界、变换测试。
- 路径记录请求编号、端点容差；验证报告记录实际最大步长、细分数量和失败边/对象诊断。
- 三场景各 25 项真实控件输入回放，覆盖规划、播放、shortcut、导入导出、编辑失效、场景重载、搜索边/最小间隙、相机和拾取。
- 修复播放时“聚焦选中物体”使用起点位姿的问题，聚焦现在跟随显示的播放或搜索检查位姿。
- 补齐几何难度、错位程度及公平平面对照实验；增加纯源码重建、性能/内存验收和压缩包验证脚本。

## 阶段验收

| 阶段 | 完成交付 | 主要证据 |
|---|---|---|
| P0 | C++20 核心库、CLI、桌面端、固定依赖、严格 JSON | 全新源码目录配置编译和 40 项 CTest |
| P1 | 共享位姿与组合几何、渲染、相机、拾取 | 数学/查询测试、三场景截图、输入回放 |
| P2 | SAT、解析/特征距离、状态和离散运动检查 | FCL 对照、薄墙与旋转扫碰、BVH 一致性 |
| P3 | 自研 RRT-Connect、预算、取消、保存与播放 | 三场景 seed 42 求解、取消和失效自检 |
| P4 | shortcut、指标、三场景参数和参考运动 | 原始/简化路径六次 0.02 复核、编辑失效检查 |
| P5 | BVH、OMPL、固定种子实验与统计图表 | 690 份报告、失败样本、Wilson 区间和时间分布 |
| P6 | 文档、演示、许可证、可运行包和重建流程 | 同机纯源码干净目录构建、真实媒体、压缩包验证脚本 |

设计第 17 节首版清单的十项均已满足本机验收：

- [x] 单刚体与静态环境的完整 SE(3) 规划闭环。
- [x] 自研位姿、基础几何查询、SAT 和运动验证。
- [x] 自研 RRT-Connect、预算、取消和准确结果状态。
- [x] 三组参数化演示与独立验证的参考运动。
- [x] 播放、关键姿态、碰撞和距离可视化。
- [x] 重新验证的路径简化和指标比较。
- [x] 场景、路径、运行报告导出及适用的重载流程。
- [x] BVH/全遍历一致性、薄障碍和旋转扫碰回归。
- [x] 固定种子基准及可复现实验配置。
- [x] 真实构建、测试、演示和离散验证边界说明。

## 验证结果

- **40/40 CTest**，**75/75 桌面输入检查**通过；额外桌面上下文与并发自检通过。
- A/B/C 固定种子全部求解；原始/简化路径共 **6/6** 次按平移、旋转、表面步长 **0.02** 独立复核通过。
- **690 次实验**完整保留：标准基准 360/360 成功；几何/运动空间实验 280/330 成功，50 次预算耗尽。
- 同端点 C 对照：SE(3) 30/30 成功，平面 0/30；窄开口配置 12/30 成功。预算耗尽不等于无解。
- 所有 640 个成功运行均有完整指标、通过验证的路径及不增加既定目标值的 shortcut 结果。
- 参考机取消延迟 **31.436 ms**，目标 ≤1000 ms；1440×900 渲染中位耗时 **0.419 ms**，目标 ≤33.333 ms；观测峰值工作集 **98.40 MiB**，目标 ≤512 MiB。渲染测试禁用垂直同步、等待 GPU 完成，排除前十帧热身。

证据：[完整验收记录](../benchmarks/data/worktree-0.2.0/verification/verification.json)、[干净构建](../benchmarks/data/worktree-0.2.0/clean-build/verification.json)、[实验审计](../benchmarks/data/worktree-0.2.0/audit.json)、[详细实验分析](experiments.md)、[功能映射](implementation-status.md)。压缩包检查记录生成于 `results/package-0.2.0/verification.json`，用于对应最终 ZIP 指纹。

源码 SHA-256：`55ddafaf7c410f5e2916476d46028f25f24c5d64a3368fabd67875a5f1486789`。主构建、纯源码重建、实验和媒体使用相同生产源码；报告中的基础提交 `c9881a2` 不代表尚未提交的全部修改。

## 复现与边界

在当前 `passagelab-codex` 内执行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -UseInstalledDependencies -BuildDirectory build/worktree
powershell -ExecutionPolicy Bypass -File tools/verify.ps1 -BuildDirectory build/worktree -OutputDirectory results/new-acceptance
powershell -ExecutionPolicy Bypass -File tools/clean_build.ps1 -Destination results/new-clean-source
python tools/experiments.py --exe build/worktree/Release/passagelab_cli.exe --suite baseline --runs 30 --seconds 5 --out results/new-baseline
python tools/experiments.py --exe build/worktree/Release/passagelab_cli.exe --suite comparisons --runs 30 --seconds 5 --out results/new-comparisons
python tools/design_experiments.py --exe build/worktree/Release/passagelab_cli.exe --runs 30 --seconds 1 --out results/new-design
powershell -ExecutionPolicy Bypass -File tools/verify_package.ps1 -OutputDirectory results/new-package-check
```

输出目录必须是新目录。纯源码重建复用了同机固定版本依赖，未声称已在新物理机器或 VM 上重新安装依赖。压缩包检查使用重新解压的副本和仅含系统目录的 PATH；其他 GPU、DPI 和实体输入设备属于尚未实测的兼容性范围。CI 工作流已提供；远程执行状态以 GitHub Actions 为准，不混入本报告的本机验收结论。

几何后端采用经过对照验证的箱体特征距离，并保留 FCL 审计；这是针对已复现 FCL 差异做出的明确设计调整，见算法文档。所有路径保证仍为指定分辨率下的离散检查，绝不包装为连续安全证明。网格、动力学、连续碰撞和学习采样属于可选 P7。

本次保留所有旧代码与 0.1.0/0.1.1 产物；项目文件改动均在当前 Codex 仓库内，未访问 Claude 工作区。
