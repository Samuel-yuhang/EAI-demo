# Hardware

This section gives the hardware checklist.

## 物料清单 (Bill of Materials - BOM)

|Component       |Quantity| Specification/Model      | Notes                              |
| ---------------------- | --------------- | --------------------------------------- | ------------------------------------------ |
| Main Control  | 1               | Raspberry Pi 4B (4GB or 8GB)            | 树莓派4B，建议4GB或更高内存版本            |
| Chassis      | 1               | 4WD Mecanum Wheel Chassis Kit           | 包含4个电机和4个麦克纳姆轮的底盘套件       |
| Motor Driver | 1               | L298N or other suitable driver          | 需能驱动4个直流电机                      |
| Camera        | 1               | Raspberry Pi CSI Camera Module V2       | CSI接口摄像头                            |
| Battery         | 1               | 12V LiPo Battery Pack / Power Bank      | 需能同时为树莓派和电机驱动板供电           |
| 其他 (Others)          | -               | 杜邦线、螺丝、扎带等                    | Jumper wires, screws, zip ties, etc.     |


**关键连接说明:**
- **电机:** 4个电机分别连接到L298N驱动板的输出端。
- **驱动板:**
    - `IN1`-`IN4` 连接到树莓派的GPIO引脚，用于控制电机方向。
    - `ENA`, `ENB` 连接到树莓派的PWM功能的GPIO引脚，用于调速。
    - `12V` 输入连接到电池正极。
    - `GND` 连接到电池负极。
- **树莓派供电:** 电池通过12V转5V降压模块后，连接到树莓派的5V和GND引脚。
- **摄像头:** 通过CSI排线连接到树莓派的CSI接口。