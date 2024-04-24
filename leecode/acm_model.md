# ACM模式输入输出攻略

## 输入 

### cin

cin是C++中， 标准的输入流对象，下面列出cin的两个用法，**单独读入**，和**批量读入**

cin的原理，简单来讲，是有一个缓冲区，我们键盘输入的数据，会先存到缓冲区中，用cin可以从缓冲区中读取数据。

> 注意1：cin可以连续从键盘读入数据
>
> 注意2：cin以空格、tab、换行符作为分隔符
>
> 注意3：cin从第一个非空格字符开始读取，直到遇到分隔符结束读取

```cpp
// 用法1，读入单数据
int num;
cin >> num;
cout << num << endl;  // 输出读入的整数num

// 用法2，批量读入多个数据
vector<int> nums(5);
for(int i = 0; i < nums.size(); i++) {
	cin >> nums[i];
}
// 输出读入的数组
for(int i = 0; i < nums.size(); i++) {
	cout << nums[i] << " ";
}
```

### getline()

从cin的注意中，也可以看出，当我们要求读取的字符串中间存在空格的时候，cin会读取不全整个字符串，这个时候，可以采用getline()函数来解决。

> 注意1：使用getline()函数的时候，需要包含头文件`<string>`
>
> 注意2：getline()函数会读取一行，读取的字符串包括空格，遇到换行符结束

```cpp
string s;
getline(cin, s);
// 输出读入的字符串
cout << s << endl;
```

###getchar()

该函数会从缓存区中读出一个字符，经常被用于判断是否换行

```cpp
char ch;
ch = getchar();
// 输出读入的字符
cout << ch << endl;
```

## 输出

同样的，在C++语言中，要使用标准的输出，也需要包含头文件`<iostream>`

输出这边，主要介绍一个函数，就是用的最多的`cout`，需要注意的是，如果输出`endl`对象的时候，会输出一个换行符，类似`\n`。

```cpp
string s = "hello, Irray~";
// 看看二者有何不同
cout << "hello, Irray~";
cout << s << endl;
```

当然，C++中的输入输出函数不止这几个，其他的输入函数包括scanf()，cin.get()等等方式，输出函数也有printf()，clog，cerr等方式，要根据具体的使用场景，选择具体的输入输出函数。

但，接下来的案例中，掌握上述三个方法是足够的。不想介绍太多，也是因为，记忆太多方法，容易记混，不如用**最简洁的方式**实现全部问题。

## 案例

### 一维数组

此类输入，每个元素为一个int或者char，有两类常见的案例：

#### 固定数目

输入格式：

```
3
1 2 3
```

or

```
3 1 2 3
```

解析：

对于第一组，第一行的3为整数的个数，第二行为三个用空格隔开的整数，因此可以采用cin来进行读取

对于第二组，第一行的3为整数的个数，空格后面的数据为三个用空格隔开的整数，因此可以采用cin来进行读取

此类问题，可以先创建一个vector<int>，大小设置为给定值，然后通过for循环来循环输入
答案：

```cpp
int n;
cin >> n; // 读入3，说明数组的大小是3
vector<int> nums(n); // 创建大小为3的vector<int>
for(int i = 0; i < n; i++) {
	cin >> nums[i];
}

// 验证是否读入成功
for(int i = 0; i < nums.size(); i++) {
	cout << nums[i] << " ";
}
cout << endl;
```

#### 不固定数目

输入格式

```
1 2 3 4
```

**解析：**

输入的数据为四个用空格间隔的整数，没有指定整数个数，因此可以用`while`循环结合`cin`来处理该问题。

```cpp
vector<int> nums;
int num;
while(cin >> num) {
	nums.push_back(num);
	// 读到换行符，终止循环
	if(getchar() == '\n') {
		break;
	}
}

// 验证是否读入成功
for(int i = 0; i < nums.size(); i++) {
	cout << nums[i] << " ";
}
cout << endl;
```

### 二维数组

#### 常规模式

输入格式：

```cpp
2 3
1 2 3
1 2 3
```

**解析：**

第一行的2，代表数据为2行，3代表数据为3列，因此根据第一行，可以得出，所输入数据为2行3列的二维数组。接下来的6个数字，就是按照空格和换行符分隔开的2x3二维数组，因此用`for`循环和`cin`即可处理

```c++
int m; // 接收行数
int n; // 接收列数

cin >> m >> n;

vector<vector<int>> matrix(m, vector<int>(n));

for(int i = 0; i < m; i++) {
	for(int j = 0; j < n; j++) {
		cin >> matrix[i][j];
	}
}

// 验证是否读入成功
for(int i = 0; i < m; i++) {
	for(int j = 0; j < n; j++) {
		cout << matrix[i][j] << " ";
	}
	cout << endl;
}
```

#### 每一行数据是逗号隔开的整数

输入格式：

```cpp
2 3
1,2,3
1,2,3
```

解析：

第一行的2，代表数据为2行，3代表数据为3列，因此根据第一行，可以得出，所输入数据为2行3列的二维数组。接下来的2行，分别是一个字符串，字符串中用逗号隔开每个整数。这里采用读入字符串的方式，并将读入的字符串进行按逗号分开。

```cpp
int m; // 接收行数
int n; // 接收列数

cin >> m >> n;

vector<vector<int>> matrix(m);

for(int i = 0; i < m; i++) {
    // 读入字符串
	string s;
	getline(cin, s);
	
	// 将读入的字符串按照逗号分隔为vector<int>
	vector<int> vec;
	int p = 0;
	for(int q = 0; q < s.size(); q++) {
		p = q;
		while(s[p] != ',' && p < s.size()) {
			p++;
		}
		string tmp = s.substr(q, p - q);
		vec.push_back(stoi(tmp));
		q = p;
	}
	
	//写入matrix
	matrix[i] = vec;
	vec.clear();
}

// 验证是否读入成功
for(int i = 0; i < matrix.size(); i++) {
	for(int j = 0; j < matrix[i].size(); j++) {
		cout << matrix[i][j] << " ";
	}
	cout << endl;
}
```

这里要使用将字符串转换的函数工具：stoi()，和读取字符串函数substr()，下面讲解两个函数如何使用

+ stoi()函数的作用

将**字符串**转为相应进制(字符不可以)，可以是8进制，10进制，16进制等，默认的情况下是10进制

例子：把字符串11转为 一个int ，8进制，10进制，16进制

```cpp
#include <iostream>
#include <string>
using namespace std;
int main()
{
    string str ="11";
    int a = stoi(str);
    int b = stoi(str,nullptr,8);
    int c = stoi(str,nullptr,16);
    cout<<"a="<<a<<endl; // 打印结果a=11
    cout<<"b="<<b<<endl; // 打印结果a=9
    cout<<"c="<<c<<endl; // 打印结果a=17
    return 0;
 
}
```

+ substr()函数

s.substr（i，len）：从字符串s中的索引i开始打印长度为len的子字符串。

### 字符串

#### 单字符串

输入格式：

```
abc
```

**解析**：

用`cin`读入即可

```cpp
string s;
cin >> s;

// 验证是否读入成功
cout << s << endl;
```

#### 给定数目多字符串

**输入格式**：

```
3 abc ab a
```

**解析**：

第一行的3，代表有3个字符串，后续为用空格隔开的3个字符串，采用`for`循环和`cin`读入即可

####不给定数目多字符串

**输入格式**：

```
abc ab a d
```

**解析**：

输入为用空格隔开的若干个字符串。

```cpp
vector<string> strings;
string str;
while(cin >> str) {
	strings.push_back(str);
	// 读到换行符，终止循环
	if(getchar() == '\n') {
		break;
	}
}

// 验证是否读入成功
for(int i = 0; i < strings.size(); i++) {
	cout << strings[i] << " ";
}
cout << endl;
```

#### 字符串转整数数组

**输入格式**：

```
11,22,3,4
```

**解析**

输入为一个完整字符串，字符串内容是按照逗号隔开的一个数组，可以先读入完成字符串，然后根据逗号进行分隔

```cpp
vector<int> vec;

// 读入字符串
string s;
getline(cin, s);

// 将读入的字符串按照逗号分隔为vector<int>
	int p = 0;
	for(int q = 0; q < s.size(); q++) {
		p = q;
		while(s[p] != ',' && p < s.size()) {
			p++;
		}
		string tmp = s.substr(q, p - q);
		vec.push_back(stoi(tmp));
		q = p;
	}

// 验证是否读入成功
for(int i = 0; i < vec.size(); i++) {
	cout << vec[i] << " ";
}
cout << endl;
```

## 常见数据结构定义

### 链表

```cpp
#include <iostream>
using namespace std;
// 链表定义，并给出两个有参构造函数
struct ListNode
{
    int val;
    ListNode* next;
    ListNode(int _val):val(_val),next(nullptr){}
    ListNode(int _val,ListNode* _next):val(_val),next(_next){}
};
int main()
{
	// 根据控制台的输入，创建一条单链表
    ListNode* LHead = new ListNode(-1);
    ListNode* pre = LHead;
    ListNode* cur = nullptr;    
    int num;
    while(cin >> num)
    {
    	// 为了简单起见，设置为-1退出，后续可优化，这里只是给出一个例子
        if(num == -1) break;
        cur = new ListNode(num);
        pre->next = cur;
        pre = cur;
    }   
    cur = LHead->next;   
    // 输出单链表的value
    while(cur)
    {
        cout << cur->val << " ";
        cur = cur->next;
    }  
    cout << endl;    
    return 0;
}
```

###  二叉树

```cpp
#include <iostream>
#include <vector>
#include <queue>

using namespace std;

//定义树节点
struct TreeNode
{
    int val;
    TreeNode* left;
    TreeNode* right;
    TreeNode():val(0),left(nullptr),right(nullptr){}
    TreeNode(int _val):val(_val),left(nullptr),right(nullptr){}
    TreeNode(int _val,TreeNode* _left,TreeNode* _right):val(0),left(_left),right(_right){}
};

//根据数组生成树
TreeNode* buildTree(const vector<int>& v)
{
    vector<TreeNode*> vTree(v.size(),nullptr);
    TreeNode* root = nullptr;
    for(int i = 0; i < v.size(); i++)
    {
        TreeNode* node = nullptr;
        if(v[i] != -1)
        {
            node = new TreeNode(v[i]);
        }
        vTree[i] = node;
    }
    root = vTree[0];
    for(int i = 0; 2 * i + 2 < v.size(); i++)
    {
        if(vTree[i] != nullptr)
        {
            vTree[i]->left = vTree[2 * i + 1];
            vTree[i]->right = vTree[2 * i + 2];
        }
    }
    return root;
}

//根据二叉树根节点层序遍历并打印
void printBinaryTree(TreeNode* root)
{
    if(root == nullptr) return;
    vector<vector<int>> ans;
    queue<TreeNode*> q;
    q.push(root);
    while(!q.empty())
    {
        int size = q.size();
        vector<int> path;
        for(int i = 0;i<size;i++)
        {
            TreeNode* node = q.front();
            q.pop();
            if(node == nullptr)
            {
                path.push_back(-1);
            }
            else
            {
                path.push_back(node->val);
                q.push(node->left);
                q.push(node->right);
            }
        }
        ans.push_back(path);
    }   
    for(int i = 0;i<ans.size();i++)
    {
        for(int j = 0;j<ans[i].size();j++)
        {
            cout << ans[i][j] << " ";
        }
        cout << endl;
    }
    return;
}
int main()
{
	// 验证
    vector<int> v = {4,1,6,0,2,5,7,-1,-1,-1,3,-1,-1,-1,8};
    TreeNode* root = buildTree(v);
    printBinaryTree(root);
    
    return 0;
}
```

