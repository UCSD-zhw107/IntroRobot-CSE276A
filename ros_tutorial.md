# **ROS2 Tutorial**

## **CLI**

### **Domain ID**

- 给一个组的node赋予一个公用的Domin ID:1-101
    - 组内的node可以互相沟通
  - ``export ROS_DOMAIN_ID=<your_domain_id>``
    - 每次开启都得先用 or bashrc


### **Local Host**

- 规定node之前只能在local host交流不会和同一个wifi下的其他node冲突
- ``export ROS_LOCALHOST_ONLY=1``
  - 每次开启都得先用 or bashrc

### **Package Executable**

- 查看一个package下面的所有可以运行的node
- ``ros2 pkg executables <package-name>``

### **Run Node**

- 运行一个package下的一个node
- ``ros2 run <package-name> <node-name>``

### **List**

- list出activae的东西
- ``ros2 node list``
- ``ros2 topic list``
- ``ros2 service list``
- ``ros2 action list``

### **Remapping**

- 把一个node的东西赋予另一个node上
- ``ros2 run <package-name> <node-name> --ros-args --remap <from>:=<to>``
- applies to each and every node that ``<node-name>`` spawns

### **Parameter Assignment**

- 给一个node之前通过YAML规定过的parameter通过cmd赋值
- ``ros2 run <package-name> <node-name> --ros-args --param <param_name>:=<param_val>``
- applies to each and every node that ``<node-name>`` spawns

### **Log Level**

- 改变一个node的log level
- ``ros2 run <package-name> <node-name> --ros-args --log-level DEBUG``

### **Node Information**

- 查看一个node的info，比如subscriber，service等
- ``ros2 node info /<node-name>``

### **Topic Log**
- 查看一个topic的log
- ``ros2 topic echo <topic_name>``

### **Topic Information**
- 查看一个topic的**Message**类型，有多少publisher和subscriber
- ``ros2 topic info <topic_name>``

### **Topic Frequency**
- 查看一个topic的data发布的frequency
- ``ros2 topic hz <topic_name>``

### **Message Type**
- 查看一个Message的构造
- ``ros2 interface show <msg_type>``
- ``<msg_type>`` 可通过**Topic Info**查看

### **Message Publication**
- 通过cmd直接向一个topic发送message
- ``ros2 topic pub --<rate> <topic_name> <msg_type> '<args>'``
  - ``<rate>`` 为发布的频率比如once， rate 1：1 HZ
  - ``<args>`` 必须为YAML格式且format与``<msg_type>``一致，比如``"{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"``



## **Packages**

### **rqt**

- ``rqt``
- 用来visualize active的东西
  - **Plugins=>Service** 可以查看activate service

- ``rqt_graph``
- visualize the changing nodes and topics, as well as the connections between them
- **Plugins > Introspection > Node Graph**也可以

  
  
