1.  python grpc教程
   1. 链接：https://grpc.io/docs/languages/python/quickstart/
   2. 常用命令（更新proto生成）在src目录下
   ```
   python3 -m grpc_tools.protoc -I../proto --python_out=./libproto --grpc_python_out=./libproto ./proto/data_transfer.proto
   ```
2. 多次启动节点后，可能会出现接收不到数据，需要强制杀死节点
   ```
   ps aux | grep gps
   kill -9 xxxx
   <!-- xxx为进程号 -->
   ```

3. reset命令
```bash
   rostopic pub -r 1 /ResetRequest std_msgs/Bool "data: true" 
```