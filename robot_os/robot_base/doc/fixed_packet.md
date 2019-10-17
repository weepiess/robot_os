# Fixed Packet(固定长度数据包)文档

### 1.数据包协议

* 数据包为16 Byte定长数据包（长度可变）

### 2.数据包封装API

__数据包创建流程：__

* 新建对象
* 加载数据
* 打包

```c++
FixedPacket packet;//新建对象
float angle=10;
//显式装载数据（建议显式）
packet.loadData<float>(angle,3);//一个参数为数据，第二个数据为数据位置
//隐式装载
packet.loadData(angle,3);
packet.pack();//打包数据
```

__数据包解析流程：__

* 新建对象

* 解包（含包校验）
* 取出数据

```c++
/*******自定义解析数据************/
FixedPacket packet;//新建对象
packet.unPack(buffer,unpack_len)//这一步通常封装起来，第一个参数为要处理的buffer，第二个参数代表buffer的长度（要解析的数据长度）
float angle=0;
packet.unloadData(angle,3);//取出数据（隐式，建议隐式）
```

### 3.数据包使用示例

* 基于fixed_common_base模块

发送数据：

```c++
FixedPacket packet;
packet.loadData<unsigned char(protocol_example::Gimbal_Angle_Control,1);
packet.loadData<unsigned char>(0x00,2);
packet.loadData<float>(info.pitch_angle,3);
packet.loadData<float>(info.yaw_angle,7);
packet.pack();
sendPacket(packet);
```

接收数据：

```c++
FixedPacket packet;
//该函数为堵塞函数，已经包含(unpack操作，无需再次unpack
if(recvPacket(packet)==0){
	unsigned char cmd;
	packet.unloadData(cmd,1);
｝
```







