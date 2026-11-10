# 共享内存协议

物理端与网页端通过一块**匿名共享内存**通信，用 seqlock 防止读到写了一半的数据。

## 布局

偏移量按 `nq / nv / nu` 动态计算（两端必须得到同一结果）：

| 段 | 内容 |
|---|---|
| 0 | 头部：`u32 magic`(0x524D3237) `u32 version` `u64 seq` `f64 t` `i32 nq` `i32 nv` `i32 nu` `u32 flags` `u32 size` |
| 40 | `qpos float64[nq]` |
| +8·nq | `qvel float64[nv]` |
| +8·nv | 指令头：`u64 seq` `f64 t` |
| +16 | `ctrl float64[nu]` |
| +8·nu | 附加指令 `float64[4]`：核心三关节 + 磁吸 |
| +32 | 任务块 `TASK`：`phase playing cycle lock depth p_deg q_deg grip axis c1` |

## seqlock

```text
写端：seq += 1（变奇数）→ 写数据 → seq += 1（变偶数）
读端：读 seq（奇数则重试）→ 拷数据 → 再读 seq，两次一致才采信
```

## 版本校验

v1.7 起头部带 `size`，两端算出的布局字节数不一致会**直接报错退出**，避免旧版上位机读到错位数据。

## 标志位

| 位 | 含义 |
|---|---|
| 1 | 请求物理端退出 |
| 2 | 手动接管（暂停回放） |
| 4 | 请求重播 |
| 8 | 请求复位 |
