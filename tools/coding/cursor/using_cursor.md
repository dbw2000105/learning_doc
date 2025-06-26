# Cursor 使用说明

cursor是集成了AI代码工具的智能IDE，其基于VSCode开发，使用体验与VSCode无异。

##  Cursor 安装步骤

* 从官网下载AppImage（目前在Ubuntu上只提供了这种方式，没有deb）：https://www.cursor.com/cn 

* 注册一个cursor账号，可以免费使用14天，后面需要使用cursor-vip-free重新刷新机器码重置试用。
* 创建桌面快捷方式：
  * 从网上下载一张cursor图片，保存在本机的一个位置，比如/home/owen_du/app/cursor/icon/cursor.jpeg
  * 在指定位置创建.desktop快捷方式：/home/owen_du/.local/share/applications/cursor.desktop

```
[Desktop Entry]
Version=1.0
Name=Cursor
Exec=/home/owen_du/app/cursor/Cursor-1.1.6-x86_64.AppImage
Icon=/home/owen_du/app/cursor/icon/cursor.jpeg
Terminal=false
Type=Application
```

* 增加cursor软连接，这样可以在任意位置使用cursor .打开当前文件或工程(与VSCode相同)：

```
sudo ln -s /home/owen_du/app/cursor/Cursor-1.1.6-x86_64.AppImage /usr/local/bin/cursor
```

**注意AppImage的位置必须使用绝对路径**

## Cursor-Vip-free 安装步骤

遇到的问题：

1. ubuntu20.04不适配:GLIBC_2.35

```
[PYI-1324989:ERROR] Failed to load Python shared library '/tmp/_MEIcPyxQB/libpython3.13.so.1.0': dlopen: /lib/x86_64-linux-gnu/libm.so.6: version `GLIBC_2.35' not found (required by /tmp/_MEIcPyxQB/libpython3.13.so.1.0)
```

2. 最低使用python3.10，可以conda新建一个环境
3. 安装路径出现问题，默认会找/root/Download这个路径，但是应该是/home/owen_du/Download。

上面的问题需要从工程重新编译出新的AppImage使用。如果重新编译包，就不需要使用官方的install.sh，直接chmod赋权限然后启动即可

第三个问题需要在main.py中将路径修改为绝对路径