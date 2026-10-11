# competition_loop — competition state machine and orchestration
# 中文说明：本包包含任务级纯逻辑模块（状态机、多楼层 FSM、任务计划、危险源路由、
# 结果写出与聚合、计时、楼层完成门、接口合规校验、门/电梯服务客户端）。
# 这些模块刻意不直接发布速度，只做决策与落盘，运动下发统一由 control 侧网关负责。
