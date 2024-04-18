# 添加ssh密钥

**1.产生ssh key**

在D盘根目录下(或者其他目录)下右键，选择Git Bash Here打开Git Bash

输入ssh-keygen -t rsa -C "Your_email@qq.com"

其中，Your_email@qq.com换成你自己的邮箱，然后一路回车，其中它会叫你输入生产密钥的文件名和密码，直接回车就好，会生成默认的文件名和空密码。

**2.拷贝保存密钥**

执行命令 clip < ~/.ssh/id_rsa.pub，此时Key已经生成并复制到剪贴板里了，此时拷贝的ssh key可以先复制到txt的文本文件中，最为后面使用。

**3.GitHub添加生成的ssh key**

登录Github点击用户头像选择Settings,然后选择SSH and GPG keys:
![img](https://img-blog.csdnimg.cn/20190129213714540.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlcm9ubg==,size_16,color_FFFFFF,t_70)

点击New SSH key:

Title任意填写，把之前生成的SSH key黏贴到key一栏，保存。

大公告成！

**检测ssh是否配置成功**

```shell
ssh -T git@github.com
```

# 对本地的git进行配置

1.查看git配置信息

```shell
 git config --list
```

2.设置git用户名、密码、邮箱的配置（全局配置）

```shell
 git config --global user.name 用户名
 git config --global user.password 密码
 git config --global user.email “xxx@qq.com”
```

如果你想要修改某一项配置的话，直接用上面的命令进行修改即可。

# 建立本地仓库并上传文件

```shell
git init
```

找到github仓库的地址并copy

```shell
git remote add 项目别名  仓库地址
git status //查看修改代码状态(红色 未保存)
git add .
git status //再次查看状态（绿色 已保存）
git commit -m "提交说明"
git push 项目别名 master
git pull origin 分支名 push分支之前先pull一下，看有没有其他人的修改冲突，若有，需解决冲突再走流程
git push origin 分支名

```

删除本地仓库

```c++
 git remote rm origin
```

添加本地仓库

```c++
git remote add origin XXXX  //origin是远程主机的名字 xxx是github仓库对应的网址
```

```bash
git remote -v // 查看本地已经关联的远程仓库
git remote rename old_name new_name  // # 修改仓库名
```

# 删除远程仓库上的某一个文件

```bash
1.当电脑存在对应的本地文件夹时，直接拉取远程信息，并更新至最新数据。 
git pull origin main # 拉取远程仓库
1.当电脑不存在对应的本地文件夹时，直接克隆远程仓库到本地。
git clone -b main https://xxx.git # 替换为自己的仓库地址
2.查看当前仓库有哪些文件及文件夹
dir
3.删除指定的文件和文件夹
git rm -r --cached file_name # 将file_name改为自己要删除的文件名
4.将更改后的项目推送到Github仓库
git push -u origin main # 将main修改为自己的分支名
```



# 将测试完成的分支添加到master主分支

1. 创建切换到`develop`分支

```shell
 git checkout -b develop 
```

2. 切换到develop分支

```shell
 git checkout  develop 
```

step 1: 切换到`develop`分支

```shell
 git checkout develop 
```

step 2: 分出一个功能性分支

```shell
 git checkout -b feature-discuss 
```

step 3: 在功能性分支上进行开发工作，多次commit，测试以后...

step 4: 把做好的功能合并到`develop`中

```shell
 git checkout develop      # 回到develop分支      
 git merge --no-ff feature-discuss     # 把做好的功能合并到develop中      
 git branch -d feature-discuss     # 删除功能性分支      
 git push origin develop     # 把develop提交到自己的远程仓库中  
```

   这样，就完成一次功能的开发和提交。

查看当前分支

```bash
git branch
```



