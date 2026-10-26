# 我的 Git 工作流

> 项目：PX4-Autopilot fork + 自研 `my_mission` 模块
> 最后更新：2026-10-14

---

## 〇、先记住一句话

```
写代码 → commit（存到本地）→ push（上传到 GitHub）

commit 和 push 是两件事：
  commit 只影响你本地
  push   才影响 GitHub
```

**常见的口误**：说"pull 到 GitHub" —— 那是错的。**上传是 push，下载是 pull。**

---

## 一、分支结构

```
main (297f361dd0)  ●━━━●━━━●━━━●━━━●━━━●━━━●   官方 PX4 release/1.18 的镜像
                                             │      （只用来同步，永不提交）
                                             │
                                             └─●━●━●━●   feat/my-mission-1.18
                                                          你的开发分支 ← 永远在这写代码
```

| 分支 | 是什么 | 你会在上面做什么 |
|---|---|---|
| **`main`** | 官方 `release/1.18` 的干净镜像 | ✅ 同步官方更新 ❌ **绝不提交** |
| **`feat/my-mission-1.18`** | `main` + 你的提交 | ✅ **所有开发都在这** |

**日常铁律**：`git branch --show-current` 必须输出 `feat/my-mission-1.18`。

---

## 二、日常开发（99% 的时间走这条）

```bash
cd ~/PX4-Autopilot

# ── ① 确认在开发分支上
git branch --show-current
#   必须输出 feat/my-mission-1.18
#   如果不是：
git switch feat/my-mission-1.18

# ── ② 写代码
vim src/modules/my_mission/MyMission.cpp

# ── ③ 编译验证（重要！不要提交编不过的代码）
make cuav_nora_default

# ── ④ 看改了什么
git status                 # 哪些文件动了
git diff                   # 具体改了哪几行

# ── ⑤ 提交到本地
git add src/modules/my_mission/MyMission.cpp
git commit -m "feat(my_mission): 一句话说清为什么改"

# ── ⑥ 推送到 GitHub
git push
```

**完成后**：本地有存档，GitHub 上也有备份。

---

## 三、同步官方更新（每隔一两周，或官方发修复时）

```bash
# ── ① 先看官方有没有新东西（只读，绝对安全）
git fetch upstream
git log --oneline main..upstream/release/1.18
#   有输出  = 官方有新的 bug fix
#   无输出  = 已经是最新，下面都不用做

# ── ② 让 main 跟上官方
git switch main
git merge --ff-only upstream/release/1.18
git push origin main
#   ⚠️ 这里必须写 origin —— main 跟踪的是 upstream（只读），
#      不加 origin 会尝试推官方仓库，被拒

# ── ③ 对照：官方新增了什么
git log --oneline feat/my-mission-1.18..main

# ── ④ 把 main 合进自己的分支
git switch feat/my-mission-1.18
git merge main
git push
```

**为什么不"挑着合"**：官方 `release/1.18` 上的新提交**全是 bug fix**，你全都要。挑着合会造成历史割裂，而且容易漏。

**为什么大概率不冲突**：你的改动是「**新文件 + 6 行注册**」，官方改的是别的地方。`git merge main` 通常会直接自动完成。

---

## 四、提交信息怎么写（PX4 强制格式）

```
type(scope): 一句话描述
```

| type | 什么时候用 |
|---|---|
| `feat` | 加功能 |
| `fix` | 修 bug |
| `docs` | 只改文档/注释 |
| `refactor` | 重构，不改行为 |
| `perf` | 性能 |
| `test` | 测试 |
| `build` `ci` `chore` `style` `revert` | 其他 |

`scope` 用模块名：`my_mission` / `ekf2` / `commander` / `mavlink` / `boards` …

**好例子**：
```
feat(my_mission): add forward-fly phase between takeoff and hover
fix(my_mission): clamp MYM_DESC_RATE to the land detector threshold
docs(my_mission): document the offboard topic chain
```

**会被打回的**：
```
fix                    ← 太模糊
update                 ← 同上
改了一下                ← 没 type
WIP                    ← 没写完
```

**message 写"为什么"，不写"做了什么"**：
```
❌ fix bug
❌ 修改了 offboard 模块

✅ feat(my_mission): add forward-fly phase between takeoff and hover

   The mission previously went straight from takeoff to hover, so
   horizontal position control was never exercised. Add a FORWARD
   state that flies MYM_FWD_DIST metres along the heading locked
   at takeoff.
```

代码 diff 已经说明了"做了什么"，**只有你知道"为什么"**。

---

## 五、几个高频场景

### 编译通过了，但不想提交

```bash
git restore <文件>          # 丢掉某个文件的所有改动（⚠️ 不可恢复）
git restore .               # 丢掉全部未提交改动（⚠️ 危险）
```

### 提交完发现漏了文件

```bash
git add <漏掉的文件>
git commit --amend --no-edit      # 补进上一条提交，message 不变
git push --force-with-lease       # 已经推过的话要 force
```

### 提交完发现 message 写错了

```bash
git commit --amend                # 会打开编辑器让你改
git push --force-with-lease
```

### 改到一半要切分支处理别的事

```bash
git stash                    # 把当前改动收起来，工作区变干净
git switch main              # 去干别的事
git switch feat/my-mission-1.18
git stash pop                # 拿回来
```

### 撤销一条已经推送的提交

```bash
git revert <commit>          # 生成一条"反向"提交（安全，不改历史）
git push
```

### 搞砸了想找回

```bash
git reflog                   # 看所有 HEAD 移动记录
git reset --hard HEAD@{2}    # 回到两步之前
```

**`git reflog` 是万能后悔药** —— 误删分支、rebase 出错、reset 过头都能救。

---

## 六、每日检查清单

开工前：

- [ ] `git branch --show-current` → `feat/my-mission-1.18`
- [ ] `git status` → 没有意料之外的改动

提交前：

- [ ] `make cuav_nora_default` 编译通过
- [ ] `git diff` 看过内容，没有多余的调试代码
- [ ] commit message 符合 `type(scope): 描述` 格式
- [ ] 一条 commit 只做一件事

推送后：

- [ ] `git status` → `您的分支与上游分支一致`
- [ ] 需要的话去 GitHub 看一眼

---

## 七、命令速查

| 我想…… | 命令 |
|---|---|
| 我在哪个分支 | `git branch --show-current` |
| 切到开发分支 | `git switch feat/my-mission-1.18` |
| 看改了什么 | `git status` / `git diff` |
| 提交 | `git add <文件>` → `git commit -m "..."` |
| 推送 | `git push` |
| 拉官方更新到 main | `git fetch upstream` → `git merge --ff-only upstream/release/1.18` |
| 把 main 合进我的分支 | `git switch feat/my-mission-1.18` → `git merge main` |
| 看官方新增了什么 | `git log --oneline feat/my-mission-1.18..main` |
| 看我的贡献 | `git log --oneline main..feat/my-mission-1.18` |
| 撤销未提交改动 | `git restore <文件>` |
| 撤销已推送的提交 | `git revert <commit>` |
| 改上一条 message | `git commit --amend` |
| 临时收起改动 | `git stash` / `git stash pop` |
| 万能的后悔药 | `git reflog` |

---

## 八、三条铁律

### 1. `main` 永远不提交

它只是"官方代码长什么样"的参考点。一旦在 `main` 上提交，`git merge --ff-only upstream/release/1.18` 就永远失败了。

### 2. 一条 commit 只做一件事

commit 是**可回退的最小单位**。把三件事塞进一条，以后想撤销其中一件就做不到。

### 3. 小步提交

每完成一个小逻辑就 commit，不要攒一天。

| 小步提交 | 攒一天 |
|---|---|
| 出问题能精确定位 | 得在 500 行里找 |
| 回退粒度小 | 要么全退 |
| 别人 review 得动 | 看不动 |
| message 写得清楚 | 只能写 "update" |

---

## 九、当前状态（供对照）

```
本地                                     GitHub
main                  297f361dd0   ←→   main                  297f361dd0
feat/my-mission-1.18  e03e92aedd   ←→   feat/my-mission-1.18  e03e92aedd
```

| 远程 | 地址 | 权限 |
|---|---|---|
| `origin` | `github.com/guoyvhang57-jpg/PX4-Autopilot` | 读写 ✅ |
| `upstream` | `github.com/PX4/PX4-Autopilot` | 只读 ❌ |
