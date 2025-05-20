# 安装记录

每个镜像在udoke up -c XX -u 后，会删除掉webots，需要安装，但是apt中找不到webots功能包。

尝试使用snapd安装，但是会出现socket问题。

```
error: cannot communicate with server: Post "http://localhost/v2/snaps/webots": dial unix /run/snapd.socket: connect: no such file or directory
```

尝试在apt中增加webots的第三方库，流程如下：

```
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
sudo apt-get update
sudo apt-get install webots

export WEBOTS_HOME=/usr/local/webots
export webots_home=/usr/local/webots
```

如果出现`sudo: apt-add-repository: command not found`，则需要安装apt-add-repository

```
sudo apt install software-properties-common
```

