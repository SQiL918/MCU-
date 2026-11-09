# 主机仿真：Q2 灰度循迹（不是固件的一部分）

这个目录把工程里**真正的** `Keil工程\App\path.c` 和 `App\app_config.h`
原样编到 PC 上（`stm32f10x.h` 换成这里的空壳头文件），外面套一个差速小车 +
8 路灰度传感器的数学模型，用来在不上车的情况下验证 Q2 的循迹逻辑和改参数。

## 怎么用

1. 双击 `build_and_run.bat`（需要 Visual Studio 2022 的 MSVC；
   脚本会自动找 `vcvars64.bat`，找不到就在 VS 的 "x64 Native Tools"
   命令行里执行 `build_and_run.bat`）。
2. 输出是一张表：每个场景是否跑完、用时、全程最大横向偏差、结束航向和位置。

```
case                             result     time   maxdev      yaw
Q2 gray 600, centred             DONE      9.14s     6.9mm   -364.6  end=(25,0)
Q2 gray 600, 20 mm               DONE      9.16s     9.7mm   -366.5  end=(27,-2)
Q2 gray 600, link drop 2..4s     DONE      9.15s     3.9mm   -367.0  end=(30,-2)
Q2 NO gray 600 (fallback)        DONE      9.14s     7.5mm   -365.6  end=(23,-0)
...
```

## 模型是什么

- 赛道：Q2 的"跑道形"——1000 mm 直线 → 右 180°（R=400 mm）→ 1000 mm 直线 →
  右 180°，和 `g_path_q2[]` 假设的几何一致。
- 车：差速模型，左右轮指令 → 一阶滞后（τ=80 ms）→ 航向/位置；
  轮距 150 mm，速度取 UI 的 `base_speed_mmps`。
- 传感器：8 路探头按 7 mm 间距装在车轴上（OUT1 在左），
  压到线（线宽 20 mm）记黑，**100 Hz 采样**，控制 200 Hz。
- `e`、`lt`、位图都来自真实的 `path.c`，不是另写一套算法。
- 场景表在 `sim_q2.c` 的 `main()` 里，可以直接加行（初始偏移、速度、链路断开区间）。

## 注意

- 仿真只能证明控制律/几何没写反、能收敛，**不能代替实车**：真车的轮胎打滑、
  传感器高度/光照、速度环差异都在模型之外。
- 它读的是 `..\Keil工程\App\app_config.h`，所以在 Keil 里改的增益会直接生效，
  改完重新双击 `build_and_run.bat` 即可。
- `stm32f10x.h` 只是给 PC 编译用的空壳（只提供 `stdint.h`），
  不要拷回 Keil 工程。
