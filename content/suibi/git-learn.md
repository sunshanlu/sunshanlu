---
authors: 孙善路-github, 孙善路-bilibili
title: git工具使用总结
tags: git
date: 2025-06-30
slug: git-learn
Category: 随笔
description: 对git使用中遇到的问题进行详细总结,还涉及submodule attributes等git的高级功能
---


## 1. git仓库创建

### 1.1 本地创建并推送

```bash
git init
git remote add origin <your-ssh-or-https-url>

git branch -M <branch-name>         # 推荐将分支名称设置为与origin一致
echo "hello world" > README.md      # 替换成你实际对工作区的操作
git add .                           # 添加到暂存区
git commit -m "your commit message" # 将展存区的内容添加到本地存储库 
git push -u origin <branch-name>    # 将当前分支推送到远程仓库，推荐本地和远程之间的分支名称一致
```
注意：

- 本地创建仓库的**默认名称**为`master`，`git branch -M`操作是强制对一个分支进行重命名，等同于`git branch --move --force`操作。
- 在执行`git remote` 和`git push`操作之前，需要保证远程仓库已经建立完成。

### 1.2 远程创建并`clone`

更加推荐的一种方法，可以直接在`github`或者`gitee`等平台上创建仓库，然后通过`git clone`命令完成`clone`操作。

```bash
git clone <your-ssh-or-https-url>
```

使用这种方式，可以保证你仓库的**分支名称**、和**远程仓库的关联关系**等保持一致。

## 2. 本地代码存储逻辑

### 2.1 工作区遇到的问题

试想一下，你对**工作区**中的内容作了某些修改，但是你发现修改的内容并不能修复你目前存在的问题，你想将修改的部分删除掉，应该怎么做呢？

```bash
git restore <file>  # 对单独的某个文件做修改后的恢复操作
git restore .       # 对整个工作区中产生修改的所有文件进行恢复操作
```

假设在工作区中，你新建了某个文件，但是发现这个文件貌似没有起到什么作用，但恰好你不知道如何那个文件需要被删除，这里提供两种解决方案：

1. 使用`git status`查看当前工作区状态，在**未跟踪**部分找到新建的文件，然后使用系统文件管理功能针对性的删除文件。
2. 使用命令行的方式实现删除操作，当然是在清除需要删除的文件之后执行。

```bash
git clean -f <file> # 删除某个文件
git clean -df <dir> # 删除某个目录，递归删除
```

下面是`git clean`命令常见的参数：

- `-f`：强制删除
- `-i`：在可交互模式下删除
- `-d`：对于某个文件夹，递归的删除

### 2.2 暂存区存储问题

我认为对于暂存区出现的任何问题都可以通过直接修改当前工作区，然后通过`git add`命令添加到暂存区解决。不过这里我还是贴出某些问题的**命令行**解决方法：

- 工作区添加到暂存区的方法

```bash
git add <file>  # 添加新文件或文件修改到暂存区
git add .       # 添加所有更改和新创建文件到暂存区
```

- 添加到暂存区后,出现问题,如何修改?

```bash
git reset <file>    # 撤销某个指定文件的暂存区的更改
git reset .         # 撤销暂存区的所有更改
```

### 2.3 本地版本库存储问题

当团队组合开发某个应用功能时，需要**创建**新分支，然后当某个分支合并完成后，需要将一个分支进行**删除**时应该如何操作呢？

```bash
git checkout -b <new-branch-name>           # 以当前HEAD指向的commit为起始点，创建并切换到分支下
git branch <new-branch-name> <commit>       # 以某次提交为起始点，创建分支，不切换
git branch <new-branch-name> <branch-name>  # 以branch指向的commit为起始点，创建分支，不切换
git branch <new-branch-name> <tag-name>     # 以某个tag指向的commit为起始点，创建分支，不切换
git branch -d <branch-name>                 # 删除某个分支
```

有一种比较清晰的`branch`管理方法，即使用`name/task`的命令方式，`name`对应实际的人名，而`task`代表这个人在这个分支下需要完成的人物，这样比较清晰明了，并且当其完成某个任务后，可以为这个分支打上一个`name/task`的`tag`标签这么作有以下两个好处：

- 当`task`没有完成时，分支跟着`commit`向前滚动，可以明确当前时间下，哪个人员在作哪些事情。
- 当`task`完成后，可以将最终的`commit`标记为一个`tag`，做到工作留痕，并可以在未完成时（经过市场验证，发现解决的不是特别好），可以根据这个`tag`回滚，继续完善这个任务。

当团队协作过程中，某个人在`name/task`分支下，完成了功能开发以后，经测试没问题，进行向主分支合并时，或许会遇到一些问题：

- 当`main`分支指向的提交为`name/task`指向提交的直接祖先时，`git`默认会采用`fast forward`策略，不创建提交，而是直接将`main`指向`name/task`指向的提交。
- 当`name/task`分支在解决问题，不断的创建提交的过程中，`main`分支也在不断的被更新创建提交时，会出现以下两种情况：
  - 当分支`name/task`和`main`同时拥有的文件**没有发生改变时**，将无冲突发生，`git merge`会默认合并到`main`分支中去，中间会创建新的commit。
  - 当分支`name/task`和`main`同时拥有的文件同时发生了修改时，将有冲突发生，`git merge`会将冲突产生的文件里面的内容进行标记，并放在暂存区，等待用户解决。
- 当`git merge`命令可以自己合并分支时，如果产生了提交，`git merge`会自动生成提交语句，如果用户想自定义这个提交应该怎么解决？使用下面的命令即可解决这个问题。

```bash
git merge --no-commit <name/task> # 前提保证目前HEAD指向的是main分支
```
- 代码里面，`--no-commit`参数代表的意思是不需要默认提交，而是将修改放到暂存区，等到用户边界提交内容后，再统一提交。
- 当存在冲突时，`--no-commit`没有任何作用，因为冲突需要用户手动处理，不会默认提交。


当你刚提交完某个`commit`到本地后，发现`comment content`并不满意，无法体现你所做工作的内容，因此你想修改这个提交内容，应该怎么做呢？下面是解决思路：

```bash
git reset --soft HEAD^ # 将当前分支回退到commit之前
git add .
git commit -m "<your-new-commit-conten>"
```

注意：

- `--soft`表示不删除工作区文件，退回到指定commit之后，工作区的内容并没有发生修改
- 保证最近提交没有同步到远程仓库，否则reset后，提交到远程仓库会失败

### 2.4 提交`commit`的别名`tag`

经常逛`github`的你，肯定会发现，某个存储库，除了使用`branch`以外，还会有一个名为`tag`的东西，这是什么？

- `tag`是一个`commit`的类型别名，当使用某个`commit`创建了`tag`后，这个`tag`就不会随着`commit`的增加而更新指向了
- `branch`会随着`commit`的不断更新维护所有的提交，并维护的最后一个`commit`，这是`branch`与`tag`之间最大的区别

```bash
git tag <your-tag-name> <commit-hash> -m "<your-tag-desc-message>"  # 以某个commit添加tag
git tag -d <your-tag-name>                                          # 删除tag
git push origin <your-tag-name>                                     # 推送tag到远程仓库
```

## 3. 远程仓库存储逻辑

### 3.1 本地与远程的同步操作

远程到本地的同步操作：

```bash
# 存储库同步操作
git clone <your-remote-repo-url>    # 本地向远程同步所有存储库内容

# 分支同步操作
git pull                            # 从远程仓库中拉取merge所有更新(tag、branch、commit)
git pull origin <your-branch-name>  # 同步指定分支，拉取并merge，同名分支
git pull origin <remote-branch>:<local-branch> # 拉取并merge指定分支，不同名分支(不推荐)

git fetch                           # 从远程仓库中拉取所有更新，但不merge
git fetch origin <your-branch-name> # 拉取指定分支，不merge，同名分支
git fetch origin <remote-branch>:<local-branch> # 拉取指定分支，不merge，不同名分支(不推荐)

# tag同步操作
git pull origin --tags                              # 同步指定分支
git pull origin <tag-name>                          # 同步指定tag，并且tag重名
git pull origin <remote-tag-name>:<local-tag-name>  # 同步指定tag，并且tag不同名
```

本地到远程的同步操作：

```bash
# 分支同步操作
git push                        # 当前HEAD指向的分支
git push origin <brach-name>    # 推送指定分支到origin，同名
git push origin <local-branch-name>:<remote-branch-name> # 推送local-branch，不同名

git push origin <tag-name>      # 推送指定tag，同名
git push origin <local-tag-name>:<remote-tag-name> # 推送local-tag，不同名
git push origin --tags          # 推送所有tag

git push --delete origin <branch-name>  # 删除远程分支操作
git push --delete origin <tag-name>     # 删除远程tag
```

### 3.2 远程冲突的解决

远程冲突的发生，往往是因为直接使用`git pull`或者`git merge`操作时，导致的冲突，原因都可以归结为`git merge`操作，解决远程冲突与本地冲突一样，详情间**2.3**本地合并的冲突问题。

## 4. 其他内容

### 4.1 gitignore和gitattributes和gitsubmodule

`.gitignore`文件定义了工作区间内，什么的文件或者文件夹不因该被`git`所管理。对于`C++`的项目来讲，`build`文件夹下的编译内容不应该被管理，`.idea`下的`IDE`配置不应该被管理等等。此外，`.gitignore`文件还支持正则表达式匹配。下面就是一个`C++`项目的`.gitignore`文件。

```bash
# 不同平台下，源文件编译生成的二进制文件
*.slo
*.lo
*.o
*.obj

# 预编译的头文件
*.gch
*.pch

# 编译的动态库文件
*.so
*.dylib
*.dll

# 编译的静态库文件
*.lai
*.la
*.a
*.lib

# 编译的可执行文件
*.exe
*.out
*.app

# 编译中间文件的文件夹
build/

# IDE生成的文件
.vscode/

# 可自行文件的文件夹
bin/

# 库文件文件夹
lib/
```

注意，对于不同分支下，可能会存在不同的`ignore`操作，比如分支`a`里面使用了`ros2`完成了某项功能，而分支`b`使用`ros`完成了某项功能，两个分支`.gitignore`应该都需要忽略`ros`项目的`build devel log`编译文件夹和`ros2`项目的`build install logs`文件夹，这种情况下，两分支下的`.gitignore`应该保持一致，避免在切换分支时的**误添加**操作。

`.gitattributes`文件主要有以下说明：

- **行尾符处理**，不同的操作系统，对换行符的理解存在差异，可以指定某些系统强相关的文件指定换行符为特定符号，例如指定`*.bat`和`*.cmd`结尾的windows脚本文件以`crlf`结尾，而其余文件指定`lf`为结尾，而对于可跨平台的文本文件，则在`IDE`指定换行符即可。
- **`merge`规则指定**，对于不同的文件，需要指定不同的合并规则，例如`*.md`，`*.cpp`等文本文件，应该使用`merge=text`策略，将内容差异拼接到工作空间的对应文件夹中，进行人为的修改合并即可。而对于`binary`文件，其并不会像`text`文本文件那些将内容拼接，而是等待用户自行修改，可以选择传入新的文件，也可以使用`git checkout --ours`或者`git checkout --theirs`来选择当前`HEAD`或者其他`commit`中的文件。
- **文件归档控制**，`git archive`命令可以输出归档后的源文件信息，而某些文件不需要出现在源文件中，例如`.gitattributes`文件，`.gitignore`文件，文档文件夹，测试文件夹等都不需要归档打包，因此在`.gitattributes`文件中，可以使用`export-ignore`关键字进行声明；

一个比较成熟的项目`.gitattributes`文件如下：

```bash
# 行尾符处理
* text=auto
*.txt text
*.md text
*.cpp text
*.h text
*.py text eol=lf
*.sh text eol=lf

# windows脚本文件行尾说明
*.bat text eol=crlf
*.cmd text eol=crlf

# 二进制文件，即当merge时，采用binary合并策略
*.png binary
*.jpg binary
*.pdf binary
*.exe binary

# 合并策略声明
*.conf merge=union

# 归档控制 - 排除开发相关文件
.gitignore export-ignore
.gitattributes export-ignore

# 归档控制 - 排除测试和文档目录
tests/ export-ignore
docs/ export-ignore
examples/ export-ignore

# 归档控制 - 排除构建相关目录
*.log export-ignore
```

- `export-ignore`在归档时，控制着`commit`中忽视的内容
- `git archive -o xxx/xxx.tar.gz [commit]`，会查看`.gitattributes`文家里面有关的归档操作。
- `.gitattributes`文件里面的等号相连之间**不能存在空格**，否则为非法，就像`json`文件里面不能使用单引号表示字符串一样。

`.gitsubmodules`文件用于管理项目下的子模块,其主要在以下几种情况下比较有用:

- 当子模块和当前项目都是你们自己团队的任务时,你们可以同时对子模块和当前项目进行开发;
- 当子模块为项目依赖的某个开源三方库时,使用子模块操作,可以减少当前项目占用的仓库空间;

为项目添加一个现有的子模块,可以使用下面的命令实现:

```bash
git submodule add -b <branch-name> -- <url> <path>
```

- `branch-name`为模块依赖的分支名称
- `url`为模块对应的远程仓库地址
- `path`为模块在项目工作空间存放的路径位置

当添加了某个子模块后,应该如何删除呢?

```bash
git submodule deinit -- <path> # 取消子模块与当前工作区间的关联
rm -rf .git/modules/<path>     # 删除.git下对于子模块的存储逻辑
nano .gitmodules               # 针对性的处理.gitmodules文件,对相关模型进行删除
```

当模块中的内容发生改变时,项目和子模块如何使用`git`进行管理呢?
```bash
cd <submodule-path>
git add .
git commit =m "<module-commit-message>"
cd <project-path>
git add .
git commit -m "<project-sync-module-message>"
git push   # 同步更新子模块远程仓库和项目远程仓库
```

当`clone`某个带有子模块的项目时,应该同步`clone`它的子模块呢?
```bash
git clone <project-with-submodule>
git submodule update --init --recursive
```

- `--init` 初始化子模块和项目关联
- `--recursive` 递归的处理所有子模块下的子模块



### 4.2 diff操作查看文件差异

```bash
git diff <commit>          # 查看工作区和某次commit之间的差异
git diff --staged <commit> # 查看暂存区和某次commit之间的差异，默认为HEAD
git diff <commit-a> <commit-b> -- file # 查看两个commit之间的file差异
git diff <commit-a> <commit-b>         # 查看两个commot之间所有文件差异
```

- 针对两个`commit`之间的差异，`<commit-a>`为原始`commit`而`<commit-b>`为修改`commit`，相当于查看`<commit-b>`相比与`<commit-a>`之间修改了什么内容。
- `--stat`可选参数可以输出统计信息，即显示某个文件修改的行数，可以快速查看那些文件作了多少的修改，比较直观。


### 4.3 log操作查看某次提交的提交记录

- 当直接使用`git log`时，会将`HEAD`指向的`commit`对应的所有父级（包含其本身）对应的`commit`对应的`hash`值和`commit content`内容打印出来（包括时间、作者和邮箱信息）。
- 注意，这里的父级不仅包含同一`branch`下的`commit`关系，还包含了`merge`后，`merge`节点对应的`commit`的父子关系。
- 当`git log <commit/branch/tag>`时，对应的都是其指向的`commit`对应的所有父级（包含其本身）的`commit`节点信息。
