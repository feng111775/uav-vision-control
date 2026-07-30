# 外部依赖

本目录只保存可复现的依赖清单，不保存第三方源码。

`px4_msgs.repos` 将 `px4_msgs` 固定到 `release/1.16` 上的提交
`392e831c1f659429ca83902e66820d7094591410`。导入后的源码位于
`src/px4_msgs`，该目录已被 `.gitignore` 排除，不属于本仓库提交内容。

从工作区根目录导入：

```bash
vcs import . < dependencies/px4_msgs.repos
```
