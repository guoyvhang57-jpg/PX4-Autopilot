# Git 日常使用速查

> 面向：一个人维护 PX4 fork 的日常开发
> 环境：git 2.43 · GitHub fork 工作流 · HTTPS + gh 凭据
> 本机路径：`/home/g/PX4-Autopilot`

---

## 目录

1. [心智模型（先看这个）](#1-心智模型先看这个)
2. [每天的循环（TL;DR）](#2-每天的循环tldr)
3. [同步：拿最新的代码](#3-同步拿最新的代码)
4. [开分支](#4-开分支)
5. [提交：核心操作](#5-提交核心操作)
6. [推送](#6-推送)
7. [查看](#7-查看)
8. [救命命令（出错了怎么办）](#8-救命命令出错了怎么办)
9. [fork 工作流的远程操作](#9-fork-工作流的远程操作)
10. [commit message 规范](#10-commit-message-规范px4-强制)
11. [速查表](#11-速查表)

---

## 1. 心智模型（先看这个）

### 四个区

```
① 工作区              你编辑器里改的文件
   │  git add                       ← 挑菜：选哪些改动进这次提交
   ▼
② 暂存区              "购物车"
   │  git commit                    ← 存档：打包成快照，写进本地历史
   ▼
③ 本地仓库            一条条 commit（可随时回退）
   │  git push
   │     ├─ 去哪？        remote.origin.url
   │     ├─ 以谁身份？    credential.helper → gh → keyring → token
   │     └─ 哪个分支？    branch.<分支>.remote / .merge
   ▼
④ GitHub
```

**一句话**：`add` 挑菜 → `commit` 存档到本地 → `push` 上传。

- `add` **不是保存**，只是"把哪些改动放进这次提交"
- `commit` 才真正存下来
- `push` 只是把本地已有的 commit 传到 GitHub

### 三个核心概念

| 概念 | 是什么 | 存在哪 |
|---|---|---|
| **commit** | 一份完整快照 + 元数据（作者/时间/message）| `.git/objects/` |
| **branch** | 指向某个 commit 的**可移动指针** | `.git/refs/heads/<名字>` |
| **HEAD** | "我现在在哪个分支上" | `.git/HEAD` |

> 分支不是"复制代码"。一个分支 = 一个 41 字节的文件，里面一行 hash。
> 建分支 0.003 秒，972 个远程分支只占 12 KB —— 所以**可以随便开分支**。

### 配置分四层（下面的覆盖上面的）

```
① /etc/gitconfig        系统级
② ~/.gitconfig          用户级   ← 你的署名、凭据、代理在这
③ .git/config           仓库级   ← 你的 remote 在这
④ git -c key=val ...    命令级
```

排查配置问题的神器：

```bash
git config --list --show-origin      # 每个配置来自哪个文件
git config --list --show-scope       # 标明 system/global/local
```

### 本机的现状（2026-10）

```
origin    https://github.com/guoyvhang57-jpg/PX4-Autopilot.git    ← 你的 fork（读写）
upstream  https://github.com/PX4/PX4-Autopilot.git                ← 官方（只读）
main      跟踪 upstream/main
代理      http.proxy = http://127.0.0.1:7890
```

**权限对照**：`origin` = 可 push ✅ ／ `upstream` = 只读 ❌

---

## 2. 每天的循环（TL;DR）

```bash
# ① 同步
git switch main
git fetch upstream
git merge --ff-only upstream/main       # 快进到官方最新

# ② 开分支
git switch -c feat/xxx

# ③ 改代码 …… 然后小步提交（可重复多次）
git status                              # 看改了什么
git add <文件>                          # 挑菜
git commit -m "feat(scope): 为什么这么改"

# ④ 推送
git push -u origin feat/xxx             # 第一次带 -u
git push                                # 之后直接 push
```

**三条纪律**（比命令重要）：

| 纪律 | 说明 |
|---|---|
| **小步提交** | 完成一个小逻辑就 commit，不要攒一天 |
| **一条 commit 一件事** | commit 是"可回退的最小单位"，混在一起就没法单独撤销 |
| **main 永远干净** | 所有开发在分支上，main 只用来同步官方 |

---

## 3. 同步：拿最新的代码

```bash
git switch main                         # 回到 main
git fetch upstream                      # 拉官方最新（只更新本地快照，不碰你的代码）
git merge --ff-only upstream/main       # 快进合并
```

**`--ff-only` 的意义**：只允许"快进"。如果 main 上有你自己的提交（不该有），它会拒绝，而不是偷偷造一个合并提交。

**同步到你的 fork**：

```bash
git push origin main
```

或者一条命令搞定：

```bash
gh repo sync
```

**查看落后多少**：

```bash
git rev-list --count main..upstream/main     # main 落后官方几个提交
git log --oneline main..upstream/main | head # 看具体是哪些
```

> ⚠️ `git fetch` **永远不会改你的代码**，只更新 `upstream/*` 这些本地快照。所以随时可以跑。

---

## 4. 开分支

```bash
git switch -c feat/my-mission
```

| 部分 | 含义 |
|---|---|
| `switch` | 切换分支 |
| `-c` | create，先建再切 |
| `feat/my-mission` | 分支名 |

旧写法 `git checkout -b feat/my-mission` 等价。

### 分支命名

| ✅ 推荐 | ❌ 不推荐 |
|---|---|
| `feat/my-mission` | `test` |
| `feat/offboard-mission` | `temp` |
| `fix/land-detector` | `a`、`123` |
| `docs/commander-notes` | `我的分支` |

前缀取 conventional commit 类型：`feat/` `fix/` `docs/` `refactor/` `test/` `perf/` `ci/`

**别用的名字**（会和上游 972 个分支撞车）：`main`、`master`、`develop`、`release/*`、`v1.18`

### 相关

```bash
git switch -                # 切回上一个分支
git branch -vv              # 看所有本地分支 + 跟踪关系
git switch main             # 切到 main
git branch -m 新名字         # 重命名当前分支
```

> **未提交的改动会跟着你切分支**。切分支只换"标签指向哪个 commit"，不碰工作区文件。

---

## 5. 提交：核心操作

### 5.1 看改了什么

```bash
git status                  # 哪些文件改了/新增了/没被跟踪
git status --short          # 紧凑版
git diff                    # 未暂存的改动（具体哪几行）
git diff --staged           # 已暂存的改动
git diff <文件>              # 只看某个文件
```

`git status --short` 的符号：

```
 M  已跟踪的文件被修改了
??  未跟踪的新文件
 A  新增并已暂存
 D  删除
 ?  子模块里有未跟踪内容
```

### 5.2 挑菜（add）

```bash
git add <文件>              # 挑一个文件
git add <目录>/             # 挑一个目录
git add -p <文件>           # 交互式，逐个 hunk 挑（进阶但很有用）
git add .                   # ⚠️ 全部（本项目慎用，见下）
```

> ⚠️ **本项目不要用 `git add .`**
> `Tools/simulation/gz` 是子模块，里面有未跟踪内容。`add .` 可能做出意外的事。
> **永远明确指定文件路径。**

### 5.3 存档（commit）

```bash
git commit -m "feat(my_mission): add forward-fly phase"
```

多行 message（推荐，第二行起写"为什么"）：

```bash
git commit -m "feat(my_mission): add forward-fly phase" \
           -m "The mission went straight from takeoff to hover, so horizontal
position control was never exercised. Add a FORWARD state that flies
MYM_FWD_DIST metres along the heading locked at takeoff." \
           -m "Assisted-by: <工具:模型>"
```

不写 `-m` 会打开编辑器（`vi`）让你写。

### 5.4 撤销 / 修改刚才的操作

```bash
# 反悔：把文件从暂存区拿出来（改动还在）
git restore --staged <文件>

# 反悔：丢掉文件的所有未提交改动（⚠️ 不可恢复）
git restore <文件>

# 反悔：丢掉全部未提交改动（⚠️ 危险）
git restore .

# 改上一条 commit 的 message
git commit --amend

# 漏 add 了文件，补进上一条 commit（message 不变）
git add <漏掉的文件>
git commit --amend --no-edit

# 撤销上一条 commit，但保留改动在工作区
git reset --soft HEAD~1

# 撤销上一条 commit，改动也一起丢（⚠️ 危险）
git reset --hard HEAD~1
```

### 5.5 临时放一放

```bash
git stash                   # 把当前未提交的改动收起来，工作区变干净
git stash list              # 看存了哪些
git stash pop               # 拿出来（并从栈里删掉）
git stash apply             # 拿出来（保留在栈里）
git stash drop              # 丢掉最近一条
```

**什么时候用**：改到一半突然要切分支处理别的事。

---

## 6. 推送

```bash
git push -u origin feat/my-mission      # 第一次：推 + 建立跟踪关系
git push                                # 之后：直接推当前分支
```

`-u` = `--set-upstream`，把本地分支和远程分支绑定。绑定之后直接 `git push` 就行。

**验证**：

```bash
git branch -vv              # [origin/feat/my-mission] 就是绑定成功了
```

**远程有你的新提交而本地没有**：

```bash
git pull                    # fetch + merge
git pull --rebase           # fetch + rebase（历史更干净）
```

---

## 7. 查看

```bash
# 历史
git log                                  # 完整历史
git log --oneline                        # 一行一条
git log --oneline -10                    # 最近 10 条
git log --oneline --graph --all          # 带分支图
git log --oneline main..feat/xxx          # 这条分支比 main 多了什么
git log -p <文件>                         # 某个文件的修改历史（含 diff）

# 某个文件每一行是谁改的
git blame <文件>

# 分支
git branch -vv                           # 本地分支 + 跟踪关系
git branch -r                            # 远程分支
git branch -a                            # 全部

# 远程
git remote -v                            # 所有远程
git remote show origin                   # 详情（会联网）

# 某次提交改了什么
git show <commit>
git show --stat <commit>                 # 只看文件列表

# 搜索
git log --grep="land detector"           # 按 message 搜
git log -S "ACCEL_VIBE_METRIC_MAX"       # 按代码内容搜（哪次提交引入了这个字符串）
```

**`git log --oneline --graph --all --decorate`** 是理解分支结构最直观的命令，建议设个别名：

```bash
git config --global alias.lg "log --oneline --graph --all --decorate"
git lg
```

---

## 8. 救命命令（出错了怎么办）

### 8.1 万能的后悔药：reflog

**`git reflog` 记录了 HEAD 的每一次移动**，包括你没推上去的、已经"删掉"的操作。

```bash
git reflog
# abc1234 HEAD@{0}: commit: feat(my_mission): ...
# def5678 HEAD@{1}: checkout: moving from main to feat/xxx
# 9ab0123 HEAD@{2}: reset: moving to HEAD~1

git reset --hard HEAD@{2}       # 回到两步之前
```

**几乎所有"搞砸了"都能用 reflog 救回来。**

### 8.2 常见情况对照

| 情况 | 命令 |
|---|---|
| **commit message 写错了** | `git commit --amend` |
| **漏 add 了文件** | `git add <文件>` → `git commit --amend --no-edit` |
| **多 add 了文件** | `git restore --staged <文件>` |
| **改乱了，想丢掉某个文件** | `git restore <文件>` |
| **改乱了，想全丢** | `git restore .` |
| **提交到了错的分支** | `git reset --soft HEAD~1` → `git switch 正确分支` → `git commit` |
| **已经 push 了想撤销** | `git revert <commit>` → `git push`（**不要**用 reset，会改写历史）|
| **误删了分支** | `git reflog` 找到那个 commit → `git switch -c 分支名 <commit>` |
| **rebase 搞砸了** | `git reflog` → `git reset --hard HEAD@{n}` |
| **想看看某个文件以前长什么样** | `git show <commit>:<路径>` |
| **merge 冲突了想放弃** | `git merge --abort` |
| **rebase 冲突了想放弃** | `git rebase --abort` |

### 8.3 冲突怎么解决

```bash
git merge upstream/main
# 提示：CONFLICT (content): Merge conflict in <文件>
```

打开冲突文件，会看到：

```
<<<<<<< HEAD
你的版本
=======
对方的版本
>>>>>>> upstream/main
```

**处理**：改成你想要的样子，删掉 `<<<<<<<` `=======` `>>>>>>>` 这三行标记。

```bash
git add <解决好的文件>
git commit                  # merge 完成
# 或者
git merge --abort           # 不干了，恢复原状
```

**推荐配置**（冲突时显示更多上下文）：

```bash
git config --global merge.conflictStyle zdiff3
```

### 8.4 ⚠️ 永远不要在这些情况下用 `--hard` / `-f`

```bash
git reset --hard        # 丢掉所有未提交改动，不可恢复
git push --force        # 覆盖远程历史，别人会遭殃
git clean -fd           # 删掉所有未跟踪文件，不可恢复
```

**`git push --force` 在你自己的功能分支上用一下是可以的（`--force-with-lease` 更安全），但绝不能对 `main` 用。**

---

## 9. fork 工作流的远程操作

### 9.1 两个远程的分工

| 远程 | 指向 | 用途 |
|---|---|---|
| `upstream` | PX4 官方 | **拉更新**（只读）|
| `origin` | 你的 fork | **推代码**（读写）|

### 9.2 常用命令

```bash
# 拉官方最新
git fetch upstream
git merge --ff-only upstream/main

# 看自己相对官方改了什么
git log --oneline upstream/main..main
git diff upstream/main --stat

# 推到自己 fork
git push origin feat/xxx

# 同步 fork 的 main
git push origin main         # 或者 gh repo sync
```

### 9.3 远程管理

```bash
git remote -v                                  # 列出所有远程
git remote add <名字> <URL>                     # 加一个
git remote rename <旧> <新>                     # 改名
git remote remove <名字>                        # 删掉
git remote set-url <名字> <新URL>               # 改地址
```

> `origin` / `upstream` **只是名字**，随便起，git 都认。它们只存在于你本地的 `.git/config`，GitHub 不知道。
> 名字本身没有魔力，起作用的是 URL。

### 9.4 提 PR

```bash
gh pr create --repo PX4/PX4-Autopilot --base main --head guoyvhang57-jpg:feat/xxx
```

---

## 10. commit message 规范（PX4 强制）

格式：

```
type(scope): 描述
```

`type` 只能从这 11 个里选：

| type | 含义 |
|---|---|
| `feat` | 新功能 |
| `fix` | 修 bug |
| `docs` | 文档 |
| `style` | 格式/空白，不改代码逻辑 |
| `refactor` | 重构（既不修 bug 也不加功能）|
| `perf` | 性能优化 |
| `test` | 测试 |
| `build` | 构建系统/依赖 |
| `ci` | CI 配置 |
| `chore` | 其他 |
| `revert` | 回滚 |

`scope` = 受影响的模块：`ekf2` / `commander` / `mavlink` / `navigator` / `sensors` / `drivers` / 板子名 …

### 好的例子

```
feat(my_mission): add forward-fly phase between takeoff and hover
fix(mavlink): correct BATTERY_STATUS_V2 parsing
docs(ekf2): update tuning guide
feat(boards/cuav_nora): enable my_mission
```

### 会被 CI 打回的

```
fix                        ← 太模糊，没 scope
update                     ← 同上
ekf2: fix something        ← 缺 type
WIP: trying something      ← 没写完
oops                       ← 没描述
apply suggestions from code review   ← 应该 squash 进上一条
```

### AI 辅助必须声明

PX4 的 `CONTRIBUTING.md` 规定：

> 任何含 AI 生成或辅助内容的提交，必须在 body 里加 `Assisted-by: NAME:MODEL` trailer。

而且：**你才是作者**，每一行都要能解释、能辩护。AI 工具不能出现在 `Signed-off-by` 里。

---

## 11. 速查表

### 最常用（记住这 10 条就够日常用）

```bash
git status                      # 我改了什么
git diff                        # 具体改了哪几行
git add <文件>                  # 挑菜
git commit -m "..."             # 存档
git push                        # 上传
git switch main                 # 切分支
git switch -c feat/xxx          # 开新分支
git log --oneline -10           # 看最近 10 条存档
git restore <文件>              # 丢掉某个文件的改动
git reflog                      # 后悔药
```

### 按场景

| 我想…… | 命令 |
|---|---|
| 看改了什么 | `git status` / `git diff` |
| 提交 | `git add <文件>` → `git commit -m "..."` |
| 推上去 | `git push` |
| 开分支 | `git switch -c feat/xxx` |
| 切分支 | `git switch <名字>` |
| 合并官方最新 | `git fetch upstream` → `git merge --ff-only upstream/main` |
| 撤销未提交改动 | `git restore <文件>` |
| 撤销已提交 | `git revert <commit>`（已推送）/ `git reset --soft HEAD~1`（未推送）|
| 改上一条 message | `git commit --amend` |
| 临时收起改动 | `git stash` / `git stash pop` |
| 救回误删的分支 | `git reflog` → `git switch -c <名字> <commit>` |
| 看某行谁写的 | `git blame <文件>` |
| 看某文件的历史 | `git log -p <文件>` |
| 搜索结果 | `git log --grep="关键词"` |

---

## 12. 本项目（my_mission）的实际例子

### 提交这个模块

```bash
cd /home/g/PX4-Autopilot

# ① 确认在干净的分支上
git status
git branch -vv

# ② 开分支
git switch -c feat/my-mission

# ③ 分条提交
git add src/modules/my_mission
git commit -m "feat(my_mission): add offboard takeoff-forward-hover-land mission module"

git add ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt \
        ROMFS/px4fmu_common/init.d/airframes/4021_towed_quad \
        ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
        ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_towed
git commit -m "feat(airframes): add 4021 towed quad and 4022 gz towed"

git add boards/cuav/nora/default.px4board boards/px4/sitl/default.px4board
git commit -m "feat(boards): enable my_mission on cuav_nora and px4_sitl"

# ④ 推送
git push -u origin feat/my-mission
```

### 查看自己相对 PX4 官方改了多少

```bash
git log --oneline upstream/main..feat/my-mission
git diff upstream/main --stat
```

---

## 附：几个值得设的全局配置

```bash
git config --global init.defaultBranch main       # 新仓库默认分支叫 main
git config --global core.editor vim               # commit message 用哪个编辑器
git config --global pull.rebase false             # 不设的话 git 每次会警告
git config --global push.autoSetupRemote true     # 新分支第一次 push 不用加 -u
git config --global fetch.prune true              # fetch 时清掉远程已删的分支
git config --global merge.conflictStyle zdiff3    # 冲突时显示更多上下文
git config --global alias.lg "log --oneline --graph --all --decorate"
git config --global alias.st status
```

撤销任意一条：

```bash
git config --global --unset <键名>
```
