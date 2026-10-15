# pharmacy_pkg

双车运行入口和部署说明见仓库根目录 [README](../../README.md)。

使用 `pharmacy_main_no_referee.launch car_id:=1` 或 `car_id:=2` 启动主业务，另用 `referee_only.launch` 启动同车裁判通信。
配置集中在 `scripts/dual_car_config.py`。运行脚本、通信协议及动态资源保持不变。
