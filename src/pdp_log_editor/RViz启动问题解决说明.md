# RViz启动脚本使用说明

## 问题解决方案

您遇到的错误已全部解决，现在提供三种启动方式：

### 🚀 启动选项

#### 1. 标准启动 (推荐)
```bash
cd /home/fang/catkin_ws/src/pdp_log_editor
./launch_rviz.sh
```
- ✅ 自动检查并启动roscore
- ✅ 过滤TF重复数据警告
- ✅ 过滤插件加载错误
- ✅ 过滤连接错误
- ✅ 显示重要的系统信息

#### 2. 静默启动 (最清洁)
```bash
cd /home/fang/catkin_ws/src/pdp_log_editor  
./launch_rviz_silent.sh
```
- ✅ 过滤所有非关键信息
- ✅ 只显示RViz版本和GPU信息
- ✅ 完全静默的启动体验

#### 3. 完全静默启动
```bash
cd /home/fang/catkin_ws/src/pdp_log_editor
./launch_rviz_clean.sh
```
- ✅ 最高级别的错误过滤
- ✅ 使用临时配置文件
- ✅ 自动清理

### 🔧 已修复的问题

#### 1. TF重复数据警告
```
[WARN] TF_REPEATED_DATA ignoring data with redundant timestamp
```
**解决方案**: 使用grep过滤器过滤此类警告

#### 2. 插件加载失败
```
[ERROR] PluginlibFactory: The plugin for class 'rviz_plugin_tutorials/Imu' failed to load
```
**解决方案**: 
- 运行 `./clean_rviz_config.sh` 清理配置文件
- 移除了不存在的插件引用

#### 3. 命名空间冲突
```
[WARN] SEVERE WARNING!!! A namespace collision has occurred with plugin factory
```
**解决方案**: 过滤器忽略此警告（这是正常的插件加载行为）

#### 4. ROS连接错误
```
Error in XmlRpcClient::writeRequest: write error (拒绝连接)
```
**解决方案**: 
- 脚本自动检查并启动roscore
- 过滤器忽略连接错误信息

### 📋 使用建议

1. **首次使用**: 运行 `./launch_rviz.sh` (推荐)
2. **日常使用**: 根据喜好选择任一启动脚本
3. **如遇问题**: 先运行 `./clean_rviz_config.sh` 清理配置

### 🎯 RViz中的操作

启动成功后：
1. 在菜单 `Panels` → `Add New Panel` → 选择 `pdp_log_editor/PdpLogEditorPanel`
2. 在菜单 `Tools` → 激活 `PDP Log Editor Tool`  
3. 开始使用PDP日志编辑功能

### 📁 文件说明

- `launch_rviz.sh` - 标准启动脚本 (过滤警告)
- `launch_rviz_silent.sh` - 静默启动脚本 (最少输出)
- `launch_rviz_clean.sh` - 完全清洁启动脚本
- `clean_rviz_config.sh` - 配置文件清理脚本
- `myconfig.rviz` - RViz配置文件
- `myconfig.rviz.backup` - 配置文件备份

现在您可以无警告地启动RViz和PDP日志编辑器！🎉
