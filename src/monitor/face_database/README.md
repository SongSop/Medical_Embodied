# 人脸数据库说明

此目录用于存储已知人员的面部照片，用于人脸识别。

## 文件命名规则

文件名格式：`person_id.扩展名`

其中 `person_id` 为纯数字，用于标识不同人员。

例如：
- `1.jpg` - person_id 为 1 的照片
- `2.jpg` - person_id 为 2 的照片
- `3.png` - person_id 为 3 的照片

## 支持的图像格式

- JPG (.jpg, .jpeg)
- PNG (.png)

## 使用说明

1. 将人员照片按上述命名规则放入此目录
2. 照片建议：
   - 清晰的正脸照片
   - 光线充足
   - 避免遮挡（眼镜、口罩等）
   - 每人可以有多张照片，系统会全部加载
3. 重新启动人脸识别服务即可加载新的人脸数据

## 测试流程

1. 启动人脸识别服务：
   ```bash
   roslaunch behavior_tree face_identify.launch
   ```

2. 测试服务（另开一个终端）：
   ```bash
   rosservice call /face_identify
   ```

3. 查看识别结果和调试图像：
   ```bash
   rostopic echo /face_identify/debug_image
   ```
   或者使用 rqt_image_view：
   ```bash
   rqt_image_view /face_identify/debug_image
   ```

## 注意事项

- 如果摄像头设备不是 /dev/video0，需要在 launch 文件中修改 `camera_id` 参数
- 识别阈值默认为 0.6，可以在代码中调整
- 识别超时时间默认为 10 秒
