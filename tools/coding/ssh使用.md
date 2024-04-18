# ssh 的基本使用

##　安装SSH

```bash
sudo apt install ssh
```

SSH分为客户端 openssh-client 和服务器 openssh-server，可以利用以下命令确认电脑上是否安装了客户端和服务器。

```bash
dpkg -l | grep ssh
```



## SSH相关操作

```bash
# 首先确认ssh-server是否已经启动了
ps -e | grep ssh
# 启动ssh服务器
service ssh start
#或
sudo /etc/init.d/ssh start 

# 停止和重启服务
sudo /etc/init.d/ssh stop  #server停止ssh服务 
sudo /etc/init.d/ssh restart  #server重启ssh服务
```

## SSH两种级别的远程登录

### 口令登录

口令登录非常简单，只需要一条命令，命令格式为： **ssh 客户端用户名@服务器ip地址** 

```bash
ssh ldz@192.168.0.1
# 例
ssh nvidia@172.20.10.13

# 如果客户机的用户名和服务器的用户名相同，登录时可以省略用户名。
ssh 192.168.0.1
```

还要说明的是，SSH服务的**默认端口是22**，也就是说，如果你不设置端口的话登录请求会自动送到远程主机的22端口。我们可以使用 **-p 选项来修改端口号**，比如连接到服务器的1234端口：

```bash
ssh -p 1234 ldz@192.168.0.1
```

如果需要**调用图形界面程序**可以使用 **-X 选项**

```bash
ssh -X ldz@192.168.0.1
```

退出远程登录

```bash
Ctrl+D 或 exit
```

### 公钥登录

每次登录远程主机都需要输入密码是很不方便的，如果想要省去这一步骤，可以利用**密钥对进行连接**，还可以提高安全性。

1、**在本机生成密钥对**

```bash
ssh-keygen -t rsa   #-t表示类型选项，这里采用rsa加密算法
```

然后根据提示一步步的按enter键即可（其中有一个提示是要求设置私钥口令passphrase，不设置则为空，这里看心情吧，如果不放心私钥的安全可以设置一下），执行结束以后会在 /home/当前用户 目录下生成一个 .ssh 文件夹,其中包含私钥文件 id_rsa 和公钥文件 id_rsa.pub。
2、**将公钥复制到远程主机中**

使用ssh-copy-id命令将公钥复制到远程主机。ssh-copy-id会将公钥写到远程主机的 ~/ .ssh/authorized_key 文件中

```bash
ssh-copy-id ldz@192.168.0.1
```

经过以上两个步骤，以后再登录这个远程主机就不用再输入密码了。

## 使用远程主机不中断的跑程序

当我们利用ssh在远程主机上跑程序的时候，只要关闭了终端就会中断ssh连接，然后远程主机上正在跑的程序或者服务就会自动停止运行。我们可以利用 nohup + 需要运行的程序 使运行的程序在切断ssh连接的时候仍然能够继续在远程主机中运行。nohup即no hang up(不挂起)。

除此之外还有很多远程操作应用，包括 **数据传输、端口操作(将不加密的网络连接绑定到ssh端口实现间接加密)** 等等，可以参考柚子皮大神的博客：https://blog.csdn.net/pipisorry/article/details/52269785
