# 小车 scripts 版本管理

## 仓库和基线

- 代码目录：`/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts`
- Git 根目录：`/home/EPRobot/robot_ws/src`，沿用现有仓库，不建立嵌套仓库。
- 基线标签：`pharmacy-scripts-baseline-20261016_112739`。
- 本次归档：`/home/EPRobot/code_archives/pharmacy_pkg_scripts/20261016_112739/scripts.tar.gz`。
- `manifest.json` 保存 13 个原始代码文件的大小、权限和 SHA-256；压缩包已逐文件校验。
- 原有 `6.1verion/` 历史代码原样保留，当前根目录代码也原样保留。
- 提交身份仅对当前仓库生效：`EPRobot <EPRobot@EPRobot2.local>`，表示设备维护提交。
- 保留现有 GitHub origin；本次未推送。

## 查看与提交

在代码目录中执行：

```bash
cd /home/EPRobot/robot_ws/src/pharmacy_pkg/scripts
git scripts-status
git scripts-log
git diff -- .
# 先检查改动，再仅提交指定文件；--only 不包含其他目录已暂存的内容。
git commit --only -m "fix: describe the change" -- dual_car_config.py
# 新文件先添加，再使用相同的 --only 方式提交。
git add -- new_script.py
git commit --only -m "feat: add new script" -- new_script.py
```

仓库还包含其他功能包和地图。避免在根目录使用 `git add .` 或 `git commit -a`，以免混入其他改动。

## 版本与发布

```bash
git tag --list 'pharmacy-scripts-*'
# 验证当前版本后创建新的注释标签（每次使用新版本号）。
git tag -a pharmacy-scripts-v1.0.1 -m "Validated scripts release 1.0.1"
git remote -v
# 确认要同步后，显式推送分支和指定标签。
# git push origin main
# git push origin pharmacy-scripts-v1.0.1
```

`pull.ff=only` 避免拉取时自动生成合并提交；`core.autocrlf=input` 保持仓库使用 LF。
`.gitignore` 忽略缓存、日志、虚拟环境、临时文件和本地凭据。

## 归档与查看旧版

每次重大修改前先在仓库外备份：

```bash
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p /home/EPRobot/code_archives/pharmacy_pkg_scripts
cd /home/EPRobot/robot_ws/src/pharmacy_pkg
tar -czf "/home/EPRobot/code_archives/pharmacy_pkg_scripts/scripts_${stamp}.tar.gz" scripts
sha256sum "/home/EPRobot/code_archives/pharmacy_pkg_scripts/scripts_${stamp}.tar.gz"
```

不覆盖当前工作区地查看基线：

```bash
git show pharmacy-scripts-baseline-20261016_112739:pharmacy_pkg/scripts/dual_car_config.py
# 将基线代码导出到一个新目录，不覆盖正在运行的小车代码。
mkdir -p /tmp/pharmacy_scripts_baseline
git archive pharmacy-scripts-baseline-20261016_112739 pharmacy_pkg/scripts | tar -x -C /tmp/pharmacy_scripts_baseline
```

最初归档还保存了仓库状态、工作区补丁、暂存区补丁和原 Git 配置/索引，供人工恢复时参考。归档目录只允许当前用户访问。原有其他目录的改动和暂存项未纳入本次提交。
