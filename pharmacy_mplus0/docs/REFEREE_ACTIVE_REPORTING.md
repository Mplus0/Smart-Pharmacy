# 裁判 TCP：仅当前任务车上报

## 规则

本车裁判上报权话题为：

```text
/referee_active
类型：std_msgs/Bool
```

它跟随双车任务令牌：

```text
car1 初始拥有任务令牌
→ car1 连接并向裁判软件发送
→ car2 不连接、不发送

car1 完成化验窗口并准备返航
→ car1 先关闭裁判发送并断开 TCP
→ 再向 car2 发送 done

car2 收到 done 并获得任务令牌
→ car2 才连接并向裁判软件发送
```

上一辆车即使仍在状态 15 返航，也不会和下一辆车同时向裁判软件发送。

## 配置

配置位于 `config/communication.yaml` 的 `referee`：

- `send_only_when_active: true`：只允许当前任务车连接和发送；
- `send_only_when_active: false`：恢复两车都持续连接的行为；
- `reset_task_cv_on_activate: true`：车辆重新获得上报权时，将 `task/CV1/CV2` 重置为 `R/None/None`，避免上一轮残留值先被发送。

两辆车的裁判节点仍照常启动；没有上报权的节点保持运行，但不会连接裁判服务器。
