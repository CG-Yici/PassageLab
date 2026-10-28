# PassageLab 当前工作区状态

日期：2026-10-28。

## 接手时检查

- 外层目录为 `D:\personal project\求职\Rapid die cut\PassageLab project`，Git 分支 `master`，检查时尚无提交。
- 实际代码位于 `passagelab-codex/`，保留独立 `.git`，分支 `main`，接手提交为 `c9881a2`。这是嵌套仓库布局，不是 Git linked worktree 元数据布局；此次未调整。
- 已阅读外层 `PassageLab_Project_Design.md`。架构是 C++20 计算库、CLI、OpenGL/ImGui 桌面端；已有几何、规划、IO、三个场景、测试、基准和演示素材。
- 迁移来的 `build/windows/CMakeCache.txt` 指向旧源码目录，不能当作当前构建。初始状态文档仍为 Pending，与已有文件不一致。
- 接手时未跟踪的 `assets/examples/`、`benchmarks/data/`、`docs/media/` 和 `tools/capture_demo.py` 均保留。

## 当前完成状态

当前版本为 **0.2.0**，设计中的 P0–P6 已完成实现及本机验收。新增受限平面规划和公平对照、点/线段/箱体及射线/三角形查询、请求和验证诊断元数据、真实控件输入回放，并修复播放时聚焦物体使用错误位姿的问题。

- 40/40 CTest 通过；三场景共 75 项桌面输入检查通过。
- 三场景固定种子求解，原始与简化路径共六次 0.02 精度复核通过。
- 纯源码副本在全新目录构建、测试通过；使用同机已有固定版本依赖。
- 标准基准、几何难度及平面限制实验均保留原始报告，包含失败。
- 0.1.0/0.1.1 代码成果、证据、媒体与包保留；新产物使用 0.2.0 专属目录。
- 所有项目改动限定在本仓库；未访问 Claude 工作区；验收时未提交，随后按用户要求进行 Git 推送及公开发布。

完成清单、数据与交付入口见 [0.2.0 中文验收报告](acceptance-0.2.0.zh-CN.md)。P7 为设计明确列出的可选进阶；其他机器/GPU 的兼容性尚未实测。公开发布入口：[v0.2.0](https://github.com/CG-Yici/PassageLab/releases/tag/v0.2.0)。

## 在此代码仓库继续操作

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -UseInstalledDependencies -BuildDirectory build/worktree
./build/worktree/Release/passagelab_desktop.exe
```

命令及 Git 操作都应在 `passagelab-codex` 内执行。重复验收必须选择新的输出目录，避免覆盖已有证据。
