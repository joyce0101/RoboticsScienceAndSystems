# 左右轮 CSV 数据（2026-11-17）

- left_wheel_samples.csv：977 条左轮原始样本，含死区 RUN 和稳态 SETTLE/MEASURE。
- right_wheel_samples.csv：975 条右轮样本，含新增 180 三次复测；已按要求删除旧的单次 180。原始六列不变，增加来源文件、来源行号及汇总选择标记。
- settled_trial_means.csv：48 条正式稳态试次均值，每轮 40～180、步长 20、每档 3 次；每次使用 14 条 MEASURE，单位 mm/s。
- motor_gain_colab.csv：同一组 48 次均值，列名和轮标签匹配课程练习 5/6 Colab，速度转换为 counts/s。

## 180 补测与来源

当前 device-monitor-261117-145718.log 已包含右轮 180 的 trial 1～3，共 55 条（13 条 SETTLE、42 条 MEASURE），来源行号为 399～453。全部 CSV 的右轮 180 均取自该文件；测量值与此前整理的数据一致，仅更新来源文件和对应行号。此前中断的旧单次数据不在本次导出中。试次编号可能在不同日志重启，数据用 source_file 与 trial 联合识别。

样本 CSV 的 use_for_steady_fit=1 仅表示选入本次稳态试次汇总；RUN、SETTLE 为 0，不代表这些行无效。左轮死区全部 232 条零速样本保留，没有删去失败启动。零值不是缺失值。

## 单位及 Colab 使用

原始及 native 试次表均为 mm/s。Colab 表按 cps = speed_mm_s × 358.3 / (2π × 16) 换算，使用固件名义轮半径 16 mm、每圈 358.3 counts；这是由已舍入轮缘速度反算的近似 counts/s，不是另行采集的原始计数。direction=forward 来自本批正 PWM 单轮测试配置。

练习 5/6 中设置 USE_EXAMPLE_DATA=False、CSV_FILENAME="motor_gain_colab.csv"，SELECTED_WHEEL 选择 left 或 right，拟合范围按实际 40～180 设置。速度边界参数应使用同一 counts/s 单位。保留验证 PWM 须另行明确选择；本次仅整理数据，未自动划分训练/验证集。

CSV 使用 UTF-8、英文表头、逗号分隔、小数点为点。原始日志未修改；manifest.json 保存文件 SHA-256、记录数、列名和转换参数。本次未重写之前的拟合报告或固件系数。
