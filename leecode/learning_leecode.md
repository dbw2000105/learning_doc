难点题目

1.滑窗

2.三数之和

3.KMP

4.滑动窗口最大值(栈与队列)

5.前 K 个高频元素

# 算法积累

## 数组

### 二分查找

重要思路：题目需要满足两个条件：

1.**是有序数组**；2.**数组中无重复元素**。

二分法程序最重要的就是**区间的定义**。一般分为两种：**左闭右闭**和**左闭右开**。两种写法对应两种不同的程序范式。

+ 左闭右闭区间

注意**右**边界要定义为==数组长度-1==；循环中right=middle - 1 而 left=middle +  1



有些题目可能需要涉及两个二分查找，比如[第34题](https://leetcode.cn/problems/find-first-and-last-position-of-element-in-sorted-array/description/)，我们需要筛选出所查找元素的第一个和最后一个位置。在刚接触二分查找时建议使用两个查找算法去寻找所查找元素的左右边界。边界的分布主要分为三种情况：

- 情况一：target 在数组范围的右边或者左边，例如数组{3, 4, 5}，target为2或者数组{3, 4, 5},target为6，此时应该返回{-1, -1}
- 情况二：target 在数组范围中，且数组中不存在target，例如数组{3,6,7},target为5，此时应该返回{-1, -1}
- 情况三：target 在数组范围中，且数组中存在target，例如数组{3,6,7},target为6，此时应该返回{1, 1}

在写程序时需要分别对这三种情况进行讨论：

```c++
int leftBorder = getLeftBorder(nums, target);
int rightBorder = getRightBorder(nums, target);
// 情况一
if (leftBorder == -2 || rightBorder == -2) return {-1, -1};
// 情况三
if (rightBorder - leftBorder > 1) return {leftBorder + 1, rightBorder - 1};
// 情况二
return {-1, -1};
```

想清楚三种情况之后，先专注于寻找右区间，然后专注于寻找左区间，左右根据左右区间做最后判断。

+ 小技巧

遍历时防止溢出的操作：

```c++
int middle = left + ((right - left) / 2);// 防止溢出 等同于(left + right)/2
```

### 移除元素

移除数组中的**指定元素**的方法有多种，思路最简单是的暴力求解法，使用两层for循环即可实现。这里主要介绍==双指针法(快慢指针法)==，这在考数组、链表的题目中是非常常见的，因此思路一定要清晰。

![27.�yd�CQ }-�Sc���l](learning_leecode.assets/27.移除元素-双指针法.gif)

双指针法（快慢指针法）： **通过一个快指针和慢指针在一个for循环下完成两个for循环的工作。**

这种方法的特点是**没有改变元素的相对位置**。

定义快慢指针

- 快指针：寻找新数组的元素 ，新数组就是不含有目标元素的数组
- 慢指针：指向更新新数组下标的位置

```cpp
// 时间复杂度：O(n)
// 空间复杂度：O(1)
class Solution {
public:
    int removeElement(vector<int>& nums, int val) {
        int slowIndex = 0;
        for (int fastIndex = 0; fastIndex < nums.size(); fastIndex++) {
            //这里这个if相当于是更新慢指针的一个操作，他只有在快指针指向的值不是目标元素时才会执行
            if (val != nums[fastIndex]) {
                nums[slowIndex++] = nums[fastIndex];
            }
        }
        return slowIndex;
    }
};
```

需要注意移除元素不能增加新的内存空间，因此需要**原地操作**，也就是进行数组的覆盖。

简单来说，**快指针去寻找新数组的元素(这里新数组的元素是不含指定元素的)，找到的值赋值给慢指针，也就是说慢指针是新数组的下标值。**

### 有序数组的平方 

本题的关键是数组是有序的，**因此平方后数组最大的数只能是第一个或者最后一个**，通过双指针法可以求解此题。

![img](learning_leecode.assets/977.有序数组的平方.gif)

此时可以考虑双指针法了，i指向起始位置，j指向终止位置。

定义一个新数组result，和A数组一样的大小，让k指向result数组终止位置。

如果`A[i] * A[i] < A[j] * A[j]`  那么`result[k--] = A[j] * A[j];`  。

如果`A[i] * A[i] >= A[j] * A[j]` 那么`result[k--] = A[i] * A[i];` 。

有如下题解：

```c++
class Solution {
public:
    vector<int> sortedSquares(vector<int>& A) {
        int k = A.size() - 1;
        vector<int> result(A.size(), 0);
        for (int i = 0, j = A.size() - 1; i <= j;) { // 注意这里要i <= j，因为最后要处理两个元素
            if (A[i] * A[i] < A[j] * A[j])  {
                result[k--] = A[j] * A[j];
                j--;
            }
            else {
                result[k--] = A[i] * A[i];
                i++;
            }
        }
        return result;
    }
};
```

### 长度最小的子数组

介绍滑动窗口法：

所谓滑动窗口，**就是不断的调节子序列的起始位置和终止位置，从而得出我们要想的结果**。

所以 只用一个for循环，那么这个循环的索引，一定是表示 滑动窗口的终止位置。

那么问题来了， 滑动窗口的起始位置如何移动呢？

这里还是以题目中的示例来举例，s=7， 数组是 2，3，1，2，4，3，来看一下查找的过程：

![209.长度最小的子数组](learning_leecode.assets/209.长度最小的子数组.gif)

在本题中实现滑动窗口，主要确定如下三点：

- 窗口内是什么？
- 如何移动窗口的起始位置？
- 如何移动窗口的结束位置？

窗口就是满足其和 ≥ s 的长度最小的连续子数组。

窗口的起始位置如何移动：如果当前窗口的值大于s了，窗口就要向前移动了（也就是该缩小了）。

窗口的结束位置如何移动：窗口的结束位置就是遍历数组的指针，也就是for循环里的索引。

解题的关键在于 窗口的起始位置如何移动，如图所示：

![leetcode_209](learning_leecode.assets/20210312160441942.png)

可以发现**滑动窗口的精妙之处在于根据当前子序列和大小的情况，不断调节子序列的起始位置。从而将O(n^2)暴力解法降为O(n)。**

C++代码如下：

```cpp
class Solution {
public:
    int minSubArrayLen(int s, vector<int>& nums) {
        int result = INT32_MAX;
        int sum = 0; // 滑动窗口数值之和
        int i = 0; // 滑动窗口起始位置
        int subLength = 0; // 滑动窗口的长度
        for (int j = 0; j < nums.size(); j++) {
            sum += nums[j];
            // 注意这里使用while，每次更新 i（起始位置），并不断比较子序列是否符合条件
            while (sum >= s) {
                subLength = (j - i + 1); // 取子序列的长度
                result = result < subLength ? result : subLength;
                sum -= nums[i++]; // 这里体现出滑动窗口的精髓之处，不断变更i（子序列的起始位置）
            }
        }
        // 如果result没有被赋值的话，就返回0，说明没有符合条件的子序列
        return result == INT32_MAX ? 0 : result;
    }
};
```

**注：题目的精髓在于for循环中确定的j表示滑动窗口的结尾，首先动j，当满足滑窗中总和大于s时，动起始位置i，直到找到最小的长度。**

+ 注：除了76，最小滑窗外，还有最大滑窗[904](https://leetcode.cn/problems/fruit-into-baskets/solutions/1/shen-du-jie-xi-zhe-dao-ti-he-by-linzeyin-6crr/)这种类型。

求最大滑窗和最小滑窗在思路上有一些区别。

**最小滑窗模板**：给定数组 nums，定义滑窗的左右边界 i, j，求满足某个条件的滑窗的最小长度。

```c++
while j < len(nums):
    判断[i, j]是否满足条件
    while 满足条件：
        不断更新结果(注意在while内更新！)
        i += 1 （最大程度的压缩i，使得滑窗尽可能的小）
    j += 1
```

**最大滑窗模板**：给定数组 nums，定义滑窗的左右边界 i, j，求满足某个条件的滑窗的最大长度。

```c++
while j < len(nums):
    判断[i, j]是否满足条件
    while 不满足条件：
        i += 1 （最保守的压缩i，一旦满足条件了就退出压缩i的过程，使得滑窗尽可能的大）
    不断更新结果（注意在while外更新！）
    j += 1
```

是的，关键的区别在于，**最大滑窗是在迭代右移右边界的过程中更新结果，而最小滑窗是在迭代右移左边界的过程中更新结果**。

关于76和904这种题需要结合哈希表(unordered_map)和滑动窗口实现。

### 螺旋矩阵

螺旋矩阵更多的是考验对数组的掌控能力而不是什么特别的算法。可以说，螺旋矩阵的解法是充满套路的。总结来说，螺旋矩阵的要点是坚持**循环不变量原则**。一般需要定义4个for循环，每个for循环掌管遍历一个方向。

模拟顺时针画矩阵的过程:

- 填充上行从左到右
- 填充右列从上到下
- 填充下行从右到左
- 填充左列从下到上

![img](learning_leecode.assets/20220922102236.png)

对于不变量保持**左闭右开**的思路进行(也可以左闭右闭，但个人认为更加麻烦)。

```c++
class Solution {
public:
    vector<vector<int>> generateMatrix(int n) {
        vector<vector<int>> res(n, vector<int>(n, 0)); // 使用vector定义一个二维数组
        int startx = 0, starty = 0; // 定义每循环一个圈的起始位置
        int loop = n / 2; // 每个圈循环几次，例如n为奇数3，那么loop = 1 只是循环一圈，矩阵中间的值需要单独处理
        int mid = n / 2; // 矩阵中间的位置，例如：n为3， 中间的位置就是(1，1)，n为5，中间位置为(2, 2)
        int count = 1; // 用来给矩阵中每一个空格赋值
        int offset = 1; // 需要控制每一条边遍历的长度，每次循环右边界收缩一位
        int i,j;
        while (loop --) {
            i = startx;
            j = starty;

            // 下面开始的四个for就是模拟转了一圈
            // 模拟填充上行从左到右(左闭右开)
            for (j = starty; j < n - offset; j++) {
                res[startx][j] = count++;
            }
            // 模拟填充右列从上到下(左闭右开)
            for (i = startx; i < n - offset; i++) {
                res[i][j] = count++;
            }
            // 模拟填充下行从右到左(左闭右开)
            for (; j > starty; j--) {
                res[i][j] = count++;
            }
            // 模拟填充左列从下到上(左闭右开)
            for (; i > startx; i--) {
                res[i][j] = count++;
            }

            // 第二圈开始的时候，起始位置要各自加1， 例如：第一圈起始位置是(0, 0)，第二圈起始位置是(1, 1)
            startx++;
            starty++;

            // offset 控制每一圈里每一条边遍历的长度
            offset += 1;
        }

        // 如果n为奇数的话，需要单独给矩阵最中间的位置赋值
        if (n % 2) {
            res[mid][mid] = count;
        }
        return res;
    }
};
```

代码的核心思路是设置一个**总的循环圈数**，对于奇数次要单独遍历一次最中间的元素，对于偶数次则不需要。并且设置4个for循环依次操作矩阵四个方向的赋值。还需要设置一个**offset**用于控制每一条边遍历的长度。

关于螺旋矩阵还有一种巧妙的解题思路，如[这里](https://leetcode.cn/problems/spiral-matrix/solutions/7155/cxiang-xi-ti-jie-by-youlookdeliciousc-3/)，方法是不需要记录已经走过的路径，通过不断地更新上下边界进行确定。

+ 首先设定上下左右边界
+ 其次向右移动到最右，此时第一行因为已经使用过了，可以将其从图中删去，体现在代码中就是重新定义上边界
+ 判断若重新定义后，上下边界交错，表明螺旋矩阵遍历结束，跳出循环，返回答案
+ 若上下边界不交错，则遍历还未结束，接着向下向左向上移动，操作过程与第一，二步同理
+ 不断循环以上步骤，直到某两条边界交错，跳出循环，返回答案

```c++
class Solution {
public:
    vector<int> spiralOrder(vector<vector<int>>& matrix) {
        vector <int> ans;
        if(matrix.empty()) return ans; //若数组为空，直接返回答案
        int u = 0; //赋值上下左右边界
        int d = matrix.size() - 1;
        int l = 0;
        int r = matrix[0].size() - 1;
        while(true)
        {
            for(int i = l; i <= r; ++i) ans.push_back(matrix[u][i]); //向右移动直到最右
            if(++ u > d) break; //重新设定上边界，若上边界大于下边界，则遍历遍历完成，下同
            for(int i = u; i <= d; ++i) ans.push_back(matrix[i][r]); //向下
            if(-- r < l) break; //重新设定有边界
            for(int i = r; i >= l; --i) ans.push_back(matrix[d][i]); //向左
            if(-- d < u) break; //重新设定下边界
            for(int i = d; i >= u; --i) ans.push_back(matrix[i][l]); //向上
            if(++ l > r) break; //重新设定左边界
        }
        return ans;
    }
};
```

## 链表

什么是链表，链表是一种通过指针串联在一起的线性结构，每一个节点由两部分组成，一个是数据域一个是指针域（存放指向下一个节点的指针），最后一个节点的指针域指向null（空指针的意思）。

链表的入口节点称为链表的头结点也就是head。

如图所示：

![链表1](learning_leecode.assets/20200806194529815.png)

链表有：单链表、双链表和循环链表三种类型。

单链表的基本定义：

```cpp
// 单链表
struct ListNode {
    int val;  // 节点上存储的元素
    ListNode *next;  // 指向下一个节点的指针
    ListNode(int x) : val(x), next(NULL) {}  // 节点的构造函数
};
```

**链表的基本操作：**

添加链表的节点：

![链表-添加节点](learning_leecode.assets/20200806195134331-20230310121503147.png)

删除链表的节点：

![链表-删除节点](learning_leecode.assets/20200806195114541-20230310121459257.png)

链表与数组的性能分析:

![链表-链表与数据性能对比](learning_leecode.assets/20200806195200276.png)

链表操作主要有两种方式：

+ **直接使用原来的链表来进行删除操作。**
+ **设置一个虚拟头结点在进行删除操作。**

使用第一种方法时，需要单独考虑删除头结点这种情况，而第二种方法不需要单独考虑。我这里更偏向使用第二种方法。

以下是一个链表删除元素的例题：

```c++
/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    ListNode* removeElements(ListNode* head, int val) {
        ListNode *dummyHead = new ListNode(0); //创建一个虚拟头结点
        dummyHead->next = head;

        ListNode *cur = dummyHead; //需要遍历的指针先指向虚拟的头结点
        while(cur->next != nullptr){
            if(cur->next->val == val){
                ListNode * tmp = cur->next; //指针tmp指向的是要被删除的元素
                cur->next = cur->next->next;
                delete tmp; //这里需要对被删掉的元素进行内存的回收，所以上面需要有一个指针指向它
            }else{
                cur = cur->next;
            }
        }
        head = dummyHead->next; //删除虚拟头节点
        delete dummyHead;
        return head;
    }
};
```

### 设计链表

设计链表需要我们实现链表的四种功能：

- get(index)：获取链表中第 index 个节点的值。如果索引无效，则返回-1。
- addAtHead(val)：在链表的第一个元素之前添加一个值为 val 的节点。插入后，新节点将成为链表的第一个节点。
- addAtTail(val)：将值为 val 的节点追加到链表的最后一个元素。
- addAtIndex(index,val)：在链表中的第 index 个节点之前添加值为 val  的节点。如果 index 等于链表的长度，则该节点将附加到链表的末尾。如果 index  大于链表长度，则不会插入节点。如果index小于0，则在头部插入节点。
- deleteAtIndex(index)：如果索引 index 有效，则删除链表中的第 index 个节点。

```c++
class MyLinkedList {
public:
    // 定义链表节点结构体
    struct LinkedNode {
        int val;
        LinkedNode* next;
        LinkedNode(int val):val(val), next(nullptr){}
    };
    
    //构造函数创建一个虚拟头结点以及初始化链表长度
    MyLinkedList(){
        _dummyHead = new LinkedNode(0);
        _size = 0;
    }
    
    int get(int index) {
        if(index < 0 || index >= _size){
            return -1;
        }
        LinkedNode* cur = _dummyHead->next; //从虚拟头节点的下一个(真正的头结点)开始遍历

        //不断遍历，直到index为0时，跳出while循环，此时cur指针也走了index次
        while(index--){
            cur = cur->next;
        }
        return cur->val;
    }
    
    void addAtHead(int val) {
        //这个函数的实现是有坑的，赋值的顺序有讲究，要先用cur赋值后面的量，再让dummy指向cur
        LinkedNode* cur = new LinkedNode(val);
        cur->next = _dummyHead->next;
        _dummyHead->next = cur;
        _size ++;
    }
    
    void addAtTail(int val) {
        LinkedNode* cur = _dummyHead;
        //找到链表的最后一个元素
        while(cur->next != NULL){
            cur = cur->next;
        }
        LinkedNode* tmp = new LinkedNode(val);
        cur->next = tmp;
        tmp->next = NULL;
        _size++;
    }
    
    void addAtIndex(int index, int val) {
        if(index < 0){
            index = 0;
        }else if (index > _size){
            return;
        }
        LinkedNode* tmp = new LinkedNode(val);
        LinkedNode* cur = _dummyHead;
        while(index--){
            cur = cur->next;
        }
        tmp->next = cur->next;
        cur->next = tmp;
        _size++;
    }
    
    void deleteAtIndex(int index) {
        if(index < 0 || index >= _size){
            return;
        }
        LinkedNode* cur = _dummyHead;
        int val = get(index);
        while(index--){
            cur = cur->next;
        }
        LinkedNode* tmp = cur->next;
        cur->next = cur->next->next;
        delete tmp;
        tmp = nullptr;
        _size--;
    }
        // 打印链表
    void printLinkedList() {
        LinkedNode* cur = _dummyHead;
        while (cur->next != nullptr) {
            cout << cur->next->val << " ";
            cur = cur->next;
        }
        cout << endl;
    }
private:
    int _size; //链表的长度
    LinkedNode* _dummyHead; //虚拟头节点
};

/**
 * Your MyLinkedList object will be instantiated and called as such:
 * MyLinkedList* obj = new MyLinkedList();
 * int param_1 = obj->get(index);
 * obj->addAtHead(val);
 * obj->addAtTail(val);
 * obj->addAtIndex(index,val);
 * obj->deleteAtIndex(index);
 */
```

第一次做这道题目的时候超时了，原因是遍历链表的操作写的太冗余，当给定的是链表的索引的时候，可以通过以下操作遍历链表：（十分的巧妙）

```c++
while(index--){
    cur = cur->next;
}
```

### 翻转链表(常考)

翻转链表是笔试中常考的题目，思路也较为简单，如动画所示：

![img](learning_leecode.assets/206.翻转链表.gif)

```c++
class Solution {
public:
    ListNode* reverseList(ListNode* head) {

        ListNode* cur = head;
        ListNode* pre = nullptr;
        ListNode* tmp = nullptr;
        
        while(cur){
            tmp = cur->next; //暂存
            cur->next = pre;
            pre = cur;            
            cur = tmp;      
        }
        return pre;
    }
};
```

翻转链表的关键在于设定终止条件和设定一个暂存的指针tmp。终止条件就是当cur指向一个空指针的时候，这时pre指向链表最后一个元素，此时循环终止。

**注：翻转链表不需要设定虚拟头结点**



### 两两交换链表中的节点

在操作链表节点的时候，每次需要操作三个节点，因此判断的条件应该是：当当前节点的下个节点和下下个节点都不为空时，执行while循环。

+ note1：链表的操作顺序分为三个步骤：

![24.两两交换链表中的节点1](learning_leecode.assets/24.两两交换链表中的节点1.png)

执行完三个步骤后，链表变成这样：

![24.两两交换链表中的节点3](learning_leecode.assets/24.两两交换链表中的节点3.png)

循环这三个步骤，完成链表节点的交换。

```c++
class Solution {
public:
    ListNode* swapPairs(ListNode* head) {
        ListNode* dummyHead = new ListNode(0);
        dummyHead->next = head;

        ListNode* cur = dummyHead;
        while(cur->next != nullptr && cur->next->next !=nullptr){
            ListNode* tmp1 = cur->next; //暂存1节点
            ListNode* tmp2 = cur->next->next->next; //暂存3节点


            cur->next = cur->next->next;    // 步骤一
            cur->next->next = tmp1;  //步骤二
            cur->next->next->next = tmp2;  //步骤三

            cur = cur->next->next;
        }
        return dummyHead->next;
    }
```

### 删除链表的倒数第N个节点

这道题不难，但是思路很巧妙，值得学习。

双指针的经典应用，如果要删除倒数第n个节点，让fast移动n步，然后让fast和slow同时移动，直到fast指向链表末尾。删掉slow所指向的节点就可以了。

思路是这样的，但要注意一些细节。

分为如下几步：

- 首先这里我推荐大家使用虚拟头结点，这样方便处理删除实际头结点的逻辑。
- 定义fast指针和slow指针，初始值为虚拟头结点，如图：

![img](https://code-thinking.cdn.bcebos.com/pics/19.%E5%88%A0%E9%99%A4%E9%93%BE%E8%A1%A8%E7%9A%84%E5%80%92%E6%95%B0%E7%AC%ACN%E4%B8%AA%E8%8A%82%E7%82%B9.png)

- **fast首先走n + 1步 ，为什么是n+1呢，因为只有这样同时移动的时候slow才能指向删除节点的上一个节点（方便做删除操作），如图：** ![img](https://code-thinking.cdn.bcebos.com/pics/19.%E5%88%A0%E9%99%A4%E9%93%BE%E8%A1%A8%E7%9A%84%E5%80%92%E6%95%B0%E7%AC%ACN%E4%B8%AA%E8%8A%82%E7%82%B91.png)
- fast和slow同时移动，直到fast指向末尾，如题： ![img](https://code-thinking.cdn.bcebos.com/pics/19.%E5%88%A0%E9%99%A4%E9%93%BE%E8%A1%A8%E7%9A%84%E5%80%92%E6%95%B0%E7%AC%ACN%E4%B8%AA%E8%8A%82%E7%82%B92.png)
- 删除slow指向的下一个节点，如图： ![img](https://code-thinking.cdn.bcebos.com/pics/19.%E5%88%A0%E9%99%A4%E9%93%BE%E8%A1%A8%E7%9A%84%E5%80%92%E6%95%B0%E7%AC%ACN%E4%B8%AA%E8%8A%82%E7%82%B93.png)

```c++
class Solution {
public:
    ListNode* removeNthFromEnd(ListNode* head, int n) {
        ListNode* dummyHead = new ListNode(0);
        dummyHead->next = head;
        ListNode* fast = dummyHead;
        while(n-- && fast != nullptr){
            fast = fast->next; //fast先走n步
        }
        fast = fast->next; // ※关键一步：fast再提前走一步，因为需要让slow指向删除节点的上一个节点
        ListNode* slow = dummyHead;
        while(fast != nullptr){
            fast = fast->next;
            slow = slow->next;
        }
        slow->next = slow->next->next;
        return dummyHead->next;
        
    }
};
```

### 链表相交

求两个链表交点节点的指针。需要注意，交点不是数值相等，而是指针相等。常见的做法是让短的链表与长链表的末尾对其。然后挨个遍历比较两个指针所指向的值是否相同，返回相同的指针。如果都不相同，返回空指针。

![面试题02.07.链表相交_2](learning_leecode.assets/面试题02.07.链表相交_2.png)

这里需要注意：一个编程技巧，不管链表A和链表B哪个链表长哪个短，都变换为链表A为长链表，B为短链表。

```c++
class Solution {
public:
    ListNode *getIntersectionNode(ListNode *headA, ListNode *headB) {
        ListNode* curA = headA;
        ListNode* curB = headB;

        int lenA = 0;
        int lenB = 0;
        while(curA != nullptr){
            lenA++;
            curA = curA->next;
        }
        while(curB != nullptr){
            lenB++;
            curB = curB->next;
        }
        curA = headA;
        curB = headB;
        //交换A和B，让A链表始终是最长的链表
        if(lenA < lenB){
            swap(curA, curB);
            swap(lenA, lenB);
        }
        int diff = lenA - lenB;

        while(diff--){
            curA = curA->next; //让A移动到和B平齐的位置
        }

        while(curA != nullptr){
            if(curA->val == curB->val){
                return curA;
            }else{
                curA = curA->next;
                curB = curB->next;
            }
        }
        return NULL;
    }
};
```

### 环形链表

这道题的代码比较好写，但是思路相对比较难想一些。主要的思想还是快慢指针。题目要求返回入环的第一个节点。

首先定义一个快指针fast和一个慢指针slow。那么当链表有环，**则快慢指针一定会相遇**。因为我们假设慢指针每次走一步，快指针每次走两步。快指针相对于慢指针每次多走一步，所以当快慢指针都入环时，二者一定会相遇。

![img](learning_leecode.assets/20220925103433.png)

假设头结点到入口的距离为$x$，快指针和慢指针相遇的节点距离入口$y$，相遇节点再到入口的距离为$z$，如图，那么快慢指针走过的距离分别为：

slow：$x+y$

fast: $x+y+n(y+z)$  这里$n(y+z)$表示快指针比慢指针多走了n圈

slow是$x+y$而不是$x+y+n(y+z)$的原因是**慢指针在第一圈走完之前就会被快指针追上。**

根据快节点走过的距离是慢节点的二倍列出等式：
$$
x+y=n(y+z)
$$
我们要求的是入环节点，也就是$x$，则
$$
x=n(y+z)-y
$$
整理后如下：
$$
x=(n-1))(y+z)+z
$$
当$n=1$时，$x=z$，这就意味着，**从头结点出发一个指针，从相遇节点 也出发一个指针，这两个指针每次只走一个节点， 那么当这两个指针相遇的时候就是 环形入口的节点**。

也就是在相遇节点处，定义一个指针index1，在头结点处定一个指针index2。

让index1和index2同时移动，每次移动一个节点， 那么他们相遇的地方就是 环形入口的节点。

动画如下：

![142.环形链表II（求入口）](learning_leecode.assets/142.环形链表II（求入口）.gif)

```c++
**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    ListNode *detectCycle(ListNode *head) {
        ListNode* fast = head;
        ListNode* slow = head;
        while(fast != NULL && fast->next != NULL) {
            slow = slow->next;
            fast = fast->next->next;
            // 快慢指针相遇，此时从head 和 相遇点，同时查找直至相遇
            if (slow == fast) {
                ListNode* index1 = fast;
                ListNode* index2 = head;
                while (index1 != index2) {
                    index1 = index1->next;
                    index2 = index2->next;
                }
                return index2; // 返回环的入口
            }
        }
        return NULL;
    }
};
```

## 哈希表

哈希表的基础理论见代码随想录：[here](https://www.programmercarl.com/%E5%93%88%E5%B8%8C%E8%A1%A8%E7%90%86%E8%AE%BA%E5%9F%BA%E7%A1%80.html#%E5%93%88%E5%B8%8C%E5%87%BD%E6%95%B0)。这里简单说一下解决哈希碰撞的方法：**拉链法**和**线性探测法**。

**拉链法：**

刚刚小李和小王在索引1的位置发生了冲突，发生冲突的元素都被存储在链表中。 这样我们就可以通过索引找到小李和小王了。

（数据规模是dataSize， 哈希表的大小为tableSize）

其实拉链法就是要选择适当的哈希表的大小，这样既不会因为数组空值而浪费大量内存，也不会因为链表太长而在查找上浪费太多时间。

![哈希表4](learning_leecode.assets/20210104235015226.png)

**线性探测法：**

使用线性探测法，一定要保证tableSize大于dataSize。 我们需要依靠哈希表中的空位来解决碰撞问题。

例如冲突的位置，放了小李，那么就向下找一个空位放置小王的信息。所以要求tableSize一定要大于dataSize ，要不然哈希表上就没有空置的位置来存放 冲突的数据了。如图所示：

![哈希表5](learning_leecode.assets/20210104235109950.png)

### 有效的字母异位词

这道题的思路较为简单，所谓有效字母的异位词，就是两个字符串s和t，如果两个字符串**出现字母的个数都相同**，那么两个字符串互为异位词，返回true，否则返回false。 

这道题的关键是定义一个数组用于记录字母出现的次数，==一个小技巧是记录字母相对于a的ASCII的值，而不是记录字母的原始ASCII，这样从字母a到z就是序号0-26。==

首先记录s中所有字母，record中相应位置加1，再遍历t中所有的字母，每出现一个字母，相应位置减1，这样当两个字符串中有不同字母时，整个数组不全为0，这时返回false，否则返回true。

```c++
class Solution {
public:
    bool isAnagram(string s, string t) {
        int record[26] = {0};
        for(int i = 0; i < s.size(); i++){
            record[s[i] - 'a'] ++; //s[i] - 'a'表示s字符串中的每个字母相对于a的ASCII的相对值，不需要记录完整的码。
        }
        for(int i = 0; i < t.size(); i++){
            record[t[i] - 'a'] --; 
        }
        for(auto i : record){
            if(i){
                return false;
            }
        }
        return true;
    }
};
```

### 快乐数 

```c++
class Solution {
public:
    int getNum(int n){
        int sum = 0;
        while(n){
            sum += (n % 10) * (n % 10);
            n /=10;
        }
        return sum;
    }
    bool isHappy(int n) {
        unordered_set<int> set;
        while(1){
            int sum = getNum(n);
            if(sum == 1){
                return true;
            }

            if(set.find(sum) != set.end()){
                return false;
            }else{
                set.insert(sum);
            }
            n = sum;
        }
    }
};
```

这道题的关键有两个：

1. 如何获取一个正整数的每位的平方和：

```c++
while(n){
    sum += (n % 10) * (n % 10);
    n /=10;
}
```

2. 如何判断终止条件：

```c++
// 如果这个sum曾经出现过，说明已经陷入了无限循环了，立刻return false
if(set.find(sum) != set.end()){
    return false;
}
```

## 哈希表

### 两数之和

首先我再强调一下 **什么时候使用哈希法**，当我们需要查询一个元素是否出现过，或者一个元素是否在集合里的时候，**就要第一时间想到哈希法**。

哈希法常用的数据结构有三种：数组，set和map。对于本题来说，我们不仅要知道元素有没有遍历过，还要知道这个元素对应的下标，**需要使用 key value结构来存放，key来存元素，value来存下标，那么使用map正合适**。

关于数组和set做哈希法的局限：

- 数组的大小是受限制的，而且如果元素很少，而哈希值太大会造成**内存空间的浪费**。
- set是一个集合，里面放的元素只能是一个key，而两数之和这道题目，不仅要判断y是否存在而且还要记录y的下标位置，因为要返回x 和 y的下标。所以set 也不能用。

那么判断**元素是否出现**，这个元素就要作为key，所以数组中的元素作为key，有key对应的就是value，value用来存下标。

在遍历数组的时候，**只需要向map去查询是否有和目前遍历元素匹配的数值**，如果有，就找到的匹配对，如果没有，就把目前遍历的元素放进map中，因为map存放的就是我们访问过的元素。

### 四数相加II

这道题目看起来有些绕，说白了就是在四个数组中取数，每个数组取一个数，统计四个取出的数字相加等于0的个数。这道题不需要考虑有重复的四个元素相加等于0的情况，相对于**18. 四数之和，题目15.三数之和**要简单一些。

**本题解题步骤：**

1. 首先定义 一个unordered_map，key放a和b两数之和，value 放a和b两数之和出现的次数。
2. 遍历大A和大B数组，统计两个数组元素之和，和出现的次数，放到map中。
3. 定义int变量count，用来统计 a+b+c+d = 0 出现的次数。
4. 在遍历大C和大D数组，找到如果 0-(c+d) 在map中出现过的话，就用count把map中key对应的value也就是出现次数统计出来。
5. 最后返回统计值 count 就可以了

==note:== 这道题的关键思路在于**将四个数组分成两组**，首先统计A和B数组中所有的两数和，然后在遍历C和D数组的时候统计如果有0-(c+d)出现，那么count就增加map中0-(c+d)出现的次数。

```c++
class Solution {
public:
    int fourSumCount(vector<int>& A, vector<int>& B, vector<int>& C, vector<int>& D) {
        unordered_map<int, int> umap; //key:a+b的数值，value:a+b数值出现的次数
        // 遍历大A和大B数组，统计两个数组元素之和，和出现的次数，放到map中
        for (int a : A) {
            for (int b : B) {
                umap[a + b]++;
            }
        }
        int count = 0; // 统计a+b+c+d = 0 出现的次数
        // 在遍历大C和大D数组，找到如果 0-(c+d) 在map中出现过的话，就把map中key对应的value也就是出现次数统计出来。
        for (int c : C) {
            for (int d : D) {
                if (umap.find(0 - (c + d)) != umap.end()) {
                    count += umap[0 - (c + d)];
                }
            }
        }
        return count;
    }
}
```

###赎金信

==note:== 涉及到字母的题都可以优先考虑数组，因为字母只有26个，而且使用数组十分节省内存。

本题只有两个需要注意的点：

- 第一点“为了不暴露赎金信字迹，要从杂志上搜索各个需要的字母，组成单词来表达意思” 这里*说明杂志里面的字母不可重复使用。*
- 第二点 “你可以假设两个字符串均只含有小写字母。” *说明只有小写字母*，这一点很重要

```c++
class Solution {
public:
    bool canConstruct(string ransomNote, string magazine) {
        int num_list[26] = {0};
        for(auto &i : magazine){
            num_list[i - 'a'] ++;
        }
        
        int count = 0;
        for(auto &i : ransomNote){
            if(num_list[i - 'a'] != 0){
                num_list[i - 'a'] --;
                count ++;
            }
        }
        if(count == ransomNote.size()){
            return true;
        }
        return false;
    }
};
```

### [三数之和(较难)](https://leetcode.cn/problems/3sum/)

==三数之和的难点在于如何去重。==

### 四数之和

四数之和和三数之和的思路一致，需要额外添加一层for循环。

## 字符串

###  反转字符串

这道题是典型的使用双指针进行解题。我们定义一个头指针和一个尾指针，让两个指针同时向中间移动。

![344.反转字符串](learning_leecode.assets/344.反转字符串.gif)

```c++
void reverseString(vector<char>& s) {
    for (int i = 0, j = s.size() - 1; i < s.size()/2; i++, j--) {
        swap(s[i],s[j]);
    }
}
```

### 反转字符串II

这道题相当于对反转字符串添加了一系列规则，从字符串开头算起，每计数至 `2k` 个字符，就反转这 `2k` 字符中的前 `k` 个字符。在做翻转操作的时候，可以直接使用reverse库函数，要注意，reverse的区间是**左闭右开**的。

```c++
class Solution {
public:
    string reverseStr(string s, int k) {
        for(int i = 0; i < s.size(); i += (2 * k)){
            if(i + k <= s.size()){ //这里需要判断，i+k不能大于字符串的长度
                reverse(s.begin() + i, s.begin() + i + k); //对i到i+k的数组进行翻转
                continue;
            }
            reverse(s.begin() + i, s.end());
        }
        return s;
    }
};
```

### 替换数字

这道题目的思路主要是将字符串先扩增到指定大小，然后从后向前进行操作，过程如下图所示。

![img](learning_leecode.assets/20231030173058.png)

有同学问了，为什么要从后向前填充，从前向后填充不行么？

从前向后填充就是O(n^2)的算法了，因为每次添加元素都要将添加元素之后的所有元素整体向后移动。

**其实很多数组填充类的问题，其做法都是先预先给数组扩容带填充后的大小，然后在从后向前进行操作。**

这么做有两个好处：

1. 不用申请新数组。
2. 从后向前填充元素，避免了从前向后填充元素时，每次添加元素都要将添加元素之后的所有元素向后移动的问题。

```c++
#include<iostream>
using namespace std;
int main() {
    string s;
    while (cin >> s) {
        int count = 0; // 统计数字的个数
        int sOldSize = s.size();
        for (int i = 0; i < s.size(); i++) {
            if (s[i] >= '0' && s[i] <= '9') {
                count++;
            }
        }
        // 扩充字符串s的大小，也就是每个空格替换成"number"之后的大小
        s.resize(s.size() + count * 5);
        int sNewSize = s.size();
        // 从后先前将空格替换为"number"
        for (int i = sNewSize - 1, j = sOldSize - 1; j < i; i--, j--) {
            if (s[j] > '9' || s[j] < '0') {
                s[i] = s[j];
            } else {
                s[i] = 'r';
                s[i - 1] = 'e';
                s[i - 2] = 'b';
                s[i - 3] = 'm';
                s[i - 4] = 'u';
                s[i - 5] = 'n';
                i -= 5;
            }
        }
        cout << s << endl;
    }
}
```

### 反转字符串中的单词

这题是经典的双指针解法，我们定义一个快指针和一个慢指针，快指针用于搜索元素，而慢指针用于更新元素的位置。

这道题可以分成两个步骤进行，首先第一步用双指针删除多余的空格，然后再反转字符串中的顺序。因此这里我们写了两个函数。removeExtraSpaces和reverseWords。

```c++
class Solution {
public:
    void reverse(string& s, int start, int end){ //翻转，区间写法：左闭右闭 []
        for (int i = start, j = end; i < j; i++, j--) {
            swap(s[i], s[j]);
        }
    }

    void removeExtraSpaces(string& s) {//去除所有空格并在相邻单词之间添加空格, 快慢指针。
        int slow = 0;   //整体思想参考https://programmercarl.com/0027.移除元素.html
        for (int i = 0; i < s.size(); ++i) { //
            if (s[i] != ' ') { //遇到非空格就处理，即删除所有空格。
                if (slow != 0) s[slow++] = ' '; //手动控制空格，给单词之间添加空格。slow != 0说明不是第一个单词，需要在单词前添加空格。
                while (i < s.size() && s[i] != ' ') { //补上该单词，遇到空格说明单词结束。
                    s[slow++] = s[i++];
                }
            }
        }
        s.resize(slow); //slow的大小即为去除多余空格后的大小。
    }

    string reverseWords(string s) {
        removeExtraSpaces(s); //去除多余空格，保证单词之间之只有一个空格，且字符串首尾没空格。
        reverse(s, 0, s.size() - 1); //将整个字符串反转
        int start = 0; //removeExtraSpaces后保证第一个单词的开始下标一定是0。
        for (int i = 0; i <= s.size(); ++i) {
            if (i == s.size() || s[i] == ' ') { //到达空格或者串尾，说明一个单词结束。进行翻转。
                reverse(s, start, i - 1); //翻转，注意是左闭右闭 []的翻转。
                start = i + 1; //更新下一个单词的开始下标start
            }
        }
        return s;
    }
};
```

对于removeExtraSpaces函数，有一些注意的点：

+ 首先需要单独一个逻辑判断字符串的第一个是不是空格：

```c++
if (slow != 0) s[slow++] = ' '; //手动控制空格，给单词之间添加空格。slow != 0说明不是第一个单词，需要在单词前添加空格。
```

+ 之后对每个单词进行操作：

```c++
while (i < s.size() && s[i] != ' ') { //补上该单词，遇到空格说明单词结束。
    s[slow++] = s[i++];
}
```

对于reverseWords函数，有一些注意的点：

+ 反转单词采用的是整体反转+局部反转的方式

```c++
removeExtraSpaces(s); //去除多余空格，保证单词之间之只有一个空格，且字符串首尾没空格。
reverse(s, 0, s.size() - 1); //将整个字符串反转
int start = 0; //removeExtraSpaces后保证第一个单词的开始下标一定是0。
for (int i = 0; i <= s.size(); ++i) {
    if (i == s.size() || s[i] == ' ') { //到达空格或者串尾，说明一个单词结束。进行翻转。
        reverse(s, start, i - 1); //翻转，注意是左闭右闭 []的翻转。
        start = i + 1; //更新下一个单词的开始下标start
    }
}
```

### 右旋字符串

右旋字符串的思路比较巧妙，延续了前面反转单词**整体反转+局部反转**的思想，先将整个字符串反转过来，然后将分过段的两个子段再分别翻转，最终实现目的。

![img](learning_leecode.assets/20231106171557.png)

![img](learning_leecode.assets/20231106172058.png)

```c++
// 版本一
#include<iostream>
#include<algorithm>
using namespace std;
int main() {
    int n;
    string s;
    cin >> n;
    cin >> s;
    int len = s.size(); //获取长度

    reverse(s.begin(), s.end()); // 整体反转
    reverse(s.begin(), s.begin() + n); // 先反转前一段，长度n
    reverse(s.begin() + n, s.end()); // 再反转后一段

    cout << s << endl;

} 
```

### 实现 strStr() (KMP算法)

**KMP算法主要用于解决字符串的匹配问题。**

kmp算法关键在于：在当前对文本串和模式串检索的过程中，若出现了不匹配，**如何充分利用已经匹配的部分**。 

匹配到不正确的地方以前的子串跟上面的子串相等所以==下面的子串的最长前缀肯定跟上面最长子串的后缀有匹配==。

前后缀的定义：

前缀：包含首字母，不包含尾字母的**所有子串**。

后缀：包含尾字母，不包含首字母的**所有子串**。

例如一个字符串：aabaaf

前缀：a aa aab aabaa

后缀：f af aaf baaf abaaf

**我们需要求最长相等的前后缀的长度，构成next前缀表**

KMP算法的核心是计算出字符串的next数组，然后在匹配的过程中根据next数组指示我们应该跳过几个元素。

前缀表next的作用是什么：**前缀表是用来回退的，它记录了模式串与主串(文本串)不匹配的时候，模式串应该从哪里开始重新匹配。**

next数组的求法：

+ 初始化：

初始化需要定义两个指针i和j，==其中j指向前缀末尾的位置，i指向后缀末尾的位置。==

+ 前后缀不相同

因为j初始化为0，那么i就从1开始，进行s[i] 与 s[j+1]的比较。所以遍历模式串s的循环下标i 要从 1开始，代码如下：

```c++
for (int i = 1; i < s.size(); i++) {
```

如果 s[i] 与 s[j+1]不相同，也就是遇到 前后缀末尾不相同的情况，就要向前回退。

怎么回退呢？

next[j]就是记录着j（包括j）之前的子串的相同前后缀的长度。

那么 s[i] 与 s[j+1] 不相同，就要找 j+1前一个元素在next数组里的值（就是next[j]）。

所以，处理前后缀不相同的情况代码如下：

```c++
while (j >= 0 && s[i] != s[j + 1]) { // 前后缀不相同了
    j = next[j]; // 向前回退
}
```

+ 前后缀相同

如果 s[i] 与 s[j + 1] 相同，那么就同时向后移动i 和j 说明找到了相同的前后缀，同时还要将j（前缀的长度）赋给next[i], 因为next[i]要记录相同前后缀的长度。

```c++
if (s[i] == s[j + 1]) { // 找到相同的前后缀
    j++;
}
next[i] = j;
```

最后整体构建next数组的函数代码如下：

```c++
void getNext(int* next, const string& s){
    int j = 0;
    next[0] = j;
    for(int i = 1; i < s.size(); i++) { // 注意i从1开始
        while (j >= 0 && s[i] != s[j + 1]) { // 前后缀不相同了
            j = next[j]; // 向前回退
        }
        if (s[i] == s[j + 1]) { // 找到相同的前后缀
            j++;
        }
        next[i] = j; // 将j（前缀的长度）赋给next[i]
    }
}
```

最后用next数组来做匹配

```c++
int j = 0; // 因为next数组里记录的起始位置为0
for (int i = 0; i < s.size(); i++) { // 注意i就从0开始
    while(j > 0 && s[i] != t[j]) { // 不匹配
        j = next[j]; // j 寻找之前匹配的位置
    }
    if (s[i] == t[j]) { // 匹配，j和i同时向后移动
        j++; // i的增加在for循环里
    }
    if (j == t.size()  ) { // 文本串s里出现了模式串t
        return (i - t.size() + 1);
    }
```

计算next数组的过程其实和动态规划的过程类似，从第一个字符开始遍历，求出当前子串的最长相等前后缀，再逐个填写到next数组中。

问：为什么使用前缀表可以告诉我们匹配失败之后跳到哪里重新匹配？

**下标5之前这部分的字符串（也就是字符串aabaa）的最长相等的前缀 和 后缀字符串是 子字符串aa ，因为找到了最长相等的前缀和后缀，匹配失败的位置是后缀子串的后面，那么我们找到与其相同的前缀的后面重新匹配就可以了。**

提供一篇讲的比较好的知乎文章：https://www.zhihu.com/question/21923021/answer/281346746

完整代码：

```c++
`    void getNext(int* next, const string& s) {
        int j = 0; //初始化
        next[0] = 0;
        for(int i = 1; i < s.size(); i++) {
            //如果s[i] 不等于 s[j]的情况
            while (j > 0 && s[i] != s[j]) { // j要保证大于0，因为下面有取j-1作为数组下标的操作
                j = next[j - 1]; // 注意这里，是要找前一位的对应的回退位置了
            }
            //如果s[i]等于s[j]的情况
            if (s[i] == s[j]) {
                j++;
            }
            next[i] = j;
        }
    }
    int strStr(string haystack, string needle) {
        if (needle.size() == 0) {
            return 0;
        }
        int next[needle.size()];
        getNext(next, needle);
        int j = 0; //next数组里记录的起始位置为0
        for(int i = 0; i< haystack.size(); i++){
            //不匹配的情况
            while(j > 0 && haystack[i] != needle[j]){
                j = next[j - 1]; //当不匹配的时候，要选择next数组上一个位置的值
            }
            //匹配，则i和j同时加一
            if(haystack[i] == needle[j]){
                j++;
            }

            if(j == needle.size()){
                return (i - needle.size() + 1);
            }
        }
        return -1;
    }
```

## 栈与队列

### 基础知识扫盲

问题：

1.C++中stack 是容器么？

答：不是容器，而是容器适配器。

2.我们使用的stack是属于哪个版本的STL？

答： SGI STL。

3.我们使用的STL中stack是如何实现的？

答：栈是以底层容器完成其所有的工作，对外提供统一的接口，底层容器是可插拔的（也就是说我们可以控制使用哪种容器来实现栈的功能）。默认是以deque为缺省情况下栈的底层结构。

4.stack 提供迭代器来遍历stack空间么？

答：栈不提供走访功能，也不提供迭代器。

+ 栈

**栈先进后出**。栈提供push 和 pop 等等接口，所有元素**必须符合先进后出规则**，所以==栈不提供走访功能，也不提供迭代器(iterator)==。 不像是set 或者map 提供迭代器iterator来遍历所有元素。不像是set 或者map 提供迭代器iterator来遍历所有元素。

栈是以底层容器完成其所有的工作，对外提供统一的接口，底层容器是可插拔的（也就是说我们可以控制使用哪种容器来实现栈的功能）。所以STL中**栈往往不被归类为容器**，而被归类为container adapter（容器适配器）。

栈的内部结构：

![栈与队列理论3](learning_leecode.assets/20210104235459376.png)

我们常用的SGI STL，如果没有指定底层实现的话，默认是以**deque**为缺省情况下栈的底层结构。

+ 队列

deque是一个双向队列，只要封住一段，只开通另一端就可以实现栈的逻辑了。队列中先进先出的数据结构，同样不允许有遍历行为，不提供迭代器, **SGI STL中队列一样是以deque为缺省情况下的底部结构。**所以STL 队列也不被归类为容器，而被归类为container adapter（ 容器适配器）。

### 用栈实现队列



### 用队列实现栈

### 有效的括号

由于栈结构的特殊性，非常适合做**对称匹配类**的题目。

不匹配的情况只有三种情况：

1.字符串里左方向的括号多余了

2.括号没有多余，但是 括号的类型没有匹配上。

3.字符串里右方向的括号多余了，所以不匹配。

在代码中我们只需要解决上述的三种匹配错误的情况就可以了。

+ 第一种情况：已经遍历完了字符串，但是栈不为空，说明有相应的左括号没有右括号来匹配，所以return false

+ 第二种情况：遍历字符串匹配的过程中，发现栈里没有要匹配的字符。所以return false

+ 第三种情况：遍历字符串匹配的过程中，栈已经为空了，没有匹配的字符了，说明右括号没有找到对应的左括号return false

那么什么时候说明左括号和右括号全都匹配了呢，就是字符串遍历完之后，栈是空的，就说明全都匹配了。

```c++
class Solution {
public:
    bool isValid(string s) {
        stack<int> sl;
        if(s.size() % 2 != 0) return false;
        for(int i = 0; i < s.size(); i++){
            if(s[i] == '[') sl.push(']');
            else if(s[i] == '{') sl.push('}');
            else if(s[i] == '(') sl.push(')');
            else if(sl.empty() || sl.top() != s[i]) return false;  //这一行是整个代码中最难理解的
            //这里需要先判断是否为空，再判断是否匹配。
            else sl.pop();
        }
        if(!sl.empty()){
            return false;
        }
        return true;
    }
};
```

代码的第十行是相对最难理解的，判断字符是否匹配的关键是：sl.top() != s[i]；当前面三个if都不满足时，说明我们遇到了右边的括号，这时候我们查询栈中最外面的括号是否是匹配的，如果不匹配则直接返回false

### 删除字符串中的所有相邻重复项

这题也是典型的用栈去解决的问题，但是为了减少最后一步将栈转换成字符串，==这题我们可以用字符串去模拟栈！==

我们定义一个字符串，当我们遍历的字符串的元素和我们定义的字符串最后一个元素不相等时，我们就将这个遍历的字符串元素加入到我们定义的字符串的最后面

```c++
class Solution {
public:
    string removeDuplicates(string s) {
        string result;
        for(auto &i : s){
            if(result.empty() || result.back() != i) result.push_back(i);
            else result.pop_back();
        }
        return result;
    }
};
```

我们也可以用经典的stack实现，但是在将stack转换为string时有坑！

```c++
class Solution {
public:
    string removeDuplicates(string s) {
        stack<char> result;
        for(int i = 0; i < s.size(); i++){
            if(result.empty() || result.top() != s[i]) result.push(s[i]);
            else result.pop();
        }
        string aa;
        int num  = result.size(); 
        for(int i = 0; i < num; i++){ // 坑在这里for循环遍历的终止条件
            aa.push_back(result.top());
            result.pop();
        }
        reverse(aa.begin(), aa.end());
        return aa;
    }
};
```

因为栈没有迭代器，不能通过[]访问其元素，因此，只能执行top()，push()，pop()等操作，每当我们pop出一个新元素时，**栈内的长度就发生了变化**，此时如果for循环里面我们写入的是i < result.size() 就会出现意想不到的错误！

### 逆波兰表达式求值

逆波兰表达式实际上是后缀表达式，他是计算机处理多项式运算的一种十分方便的表达式类型。逆波兰表达式实际上是二叉树的后续遍历。比如(1 + 2) × (3 + 4)我们转换成逆波兰表达式就成：12+34×，也就是从二叉树的叶子节点向上不断地遍历。

这道题的思路是分为两种情况处理：

1.遇到运算符：

从栈中取出两个元素，进行相应的运算。

2.遇到数字：

加入到栈中。

```c++
class Solution {
public:
    int evalRPN(vector<string>& tokens) {
        stack<long long> st;
        for(int i = 0; i < tokens.size(); i++){
            if(tokens[i] == "+" || tokens[i] == "-" || tokens[i] == "*" || tokens[i] == "/"){
                long long num1 = st.top(); st.pop();
                long long num2 = st.top(); st.pop();
            if(tokens[i] == "+") st.push(num2 + num1); //这道题的坑是对于num1和num2的运算顺序
            if(tokens[i] == "-") st.push(num2 - num1);
            if(tokens[i] == "*") st.push(num2 * num1);
            if(tokens[i] == "/") st.push(num2 / num1);
            }else st.push(stoll(tokens[i]));
        }
        int result = st.top();
        st.pop();
        return result;
    }
};
```

这道题的坑是对于num1和num2的运算顺序，因为stack的存储顺序是后进先出，因此，num1存储的是后面的那个数，而num2存储的才是前面那个数。比如题目给出波兰表达式："12+" 实际上对应于"1+2"，那么num1是2，num2是1

### 滑动窗口最大值



### 前 K 个高频元素（优先级队列，大顶堆，小顶堆）

==note：对元素出现的频率进行排序，用map这种数据结构更合适！==

## 二叉树

### 理论基础

+ 二叉树的种类：满二叉树，完全二叉树，二叉搜索树，  平衡二叉搜索树。

**满二叉树**：满二叉树：如果一棵二叉树只有度为0的结点和度为2的结点，并且度为0的结点在同一层上，则这棵二叉树为满二叉树。

<img src="learning_leecode.assets/20200806185805576.png" alt="img" style="zoom:50%;" />

**完全二叉树**：除了最底层节点可能没填满外，其余每层节点数都达到最大值，并且最下面一层的节点都集中在该层最左边的若干位置。(**堆就是一棵完全二叉树**)

<img src="learning_leecode.assets/20200920221638903.png" alt="img" style="zoom:50%;" />

**二叉搜索树**：

<img src="learning_leecode.assets/20200806190304693.png" alt="img" style="zoom:50%;" />

若它的左子树不空，则左子树上所有结点的值均小于它的根结点的值；

若它的右子树不空，则右子树上所有结点的值均大于它的根结点的值；

它的左、右子树也分别为二叉排序树

**平衡二叉搜索树**：

<img src="learning_leecode.assets/20200806190511967.png" alt="img" style="zoom:50%;" />

它是一棵空树或它的左右两个子树的高度差的绝对值不超过1，并且左右两个子树都是一棵平衡二叉树。

最后一棵 不是平衡二叉树，因为它的左右两个子树的高度差的绝对值超过了1。

**C++中map、set、multimap，multiset的底层实现都是平衡二叉搜索树**，所以map、set的增删操作时间时间复杂度是logn

+ 二叉树的存储方式

**二叉树可以链式存储，也可以顺序存储。**

那么链式存储方式就用**指针**， 顺序存储的方式就是用**数组**。

链式存储方式：

<img src="learning_leecode.assets/2020092019554618.png" alt="img" style="zoom:50%;" />

顺序存储方式：

<img src="learning_leecode.assets/20200920200429452.png" alt="img" style="zoom:50%;" />

+ 二叉树的遍历方式

主要两种遍历方式：

1. 深度优先遍历：先往深走，遇到叶子节点再往回走。

   前中后序遍历 (递归法，迭代法)

   - 前序遍历：中左右
   - 中序遍历：左中右
   - 后序遍历：左右中 

<img src="learning_leecode.assets/20200806191109896.png" alt="img" style="zoom:50%;" />



1. 广度优先遍历：一层一层的去遍历。

​	层级遍历(迭代法)

最后再说一说二叉树中深度优先和广度优先遍历实现方式，我们做二叉树相关题目，经常会使用递归的方式来实现**深度优先遍历**，也就是实现前中后序遍历，使用递归是比较方便的。

**之前我们讲栈与队列的时候，就说过栈其实就是递归的一种实现结构**，也就说前中后序遍历的逻辑其实都是可以==借助栈使用递归的方式==来实现的。

而**广度优先遍历**的实现一般使用==队列==来实现，这也是队列先进先出的特点所决定的，因为需要先进先出的结构，才能一层一层的来遍历二叉树。



note:二叉树遍历：第i个节点的左子节点：2i+1 ；第i个节点的右子节点：2i+2

### ※二叉树的递归遍历

三步走：

1.确定递归函数的参数和返回值

2.确定终止条件

3.确定单层递归的逻辑

大多数二叉树的题目需要传入递归函数的参数不多，**一般就是传入一个根节点以及一个数组，用来放遍历的结果**。

**二叉树的前序遍历**

```c++
class Solution {
public:
    vector<int> preorderTraversal(TreeNode* root) {
        vector<int> result;
        traversal(root, result);
        return result;
    }

    void traversal(TreeNode* cur, vector<int>& vec){
        if(cur == nullptr){
            return;
        }
        vec.push_back(cur->val);    // 中
        traversal(cur->left, vec);  //左
        traversal(cur->right, vec);  //右
    }
};
```

前序，中序，后续遍历的思想都一样，只是13-15行的顺序不同，只需要遵循前面说的遍历顺序即可。

### 二叉树的迭代遍历

所有的递归操作在理论上都可以通过**栈**这种数据结构实现。

二叉树的前序遍历和后续遍历方法可以互相改，只需要更改三行即可。

+ 前序遍历和后序遍历

前序遍历：

```c++
vector<int> preorderTraversal(TreeNode* root) {
    stack<TreeNode*> st;
    vector<int> result;
    if (root == NULL) return result;
    st.push(root);
    while (!st.empty()) {
        TreeNode* node = st.top();                       // 中
        st.pop();
        result.push_back(node->val);
        if (node->right) st.push(node->right);           // 右（空节点不入栈）
        if (node->left) st.push(node->left);             // 左（空节点不入栈）
    }
    return result;
}
```

后序遍历：

```c++
vector<int> postorderTraversal(TreeNode* root) {
    stack<TreeNode*> st;
    vector<int> result;
    //判断result是否为空节点
    if (root == NULL) return result;
    st.push(root);
    while (!st.empty()) {
        TreeNode* node = st.top();                       
        st.pop();
        result.push_back(node->val);
        if (node->left) st.push(node->left);            

        if (node->right) st.push(node->right);           
    }
    reverse(result.begin(), result.end());
    return result;
}
```

由于前序遍历的顺序是中左右，在用迭代法实现的过程中，我们使用栈来模拟递归，那么根据栈后进先出的规则，就应该先将根节点放入栈中，再放**右节点，再放左节点**。因为查询栈中节点的时候是先查询后进入的节点。

后序遍历的顺序只需要先将左子节点放入，再将右子节点放入即可，最后注意要反转整个结果，结果才是左右中

由于中序遍历的访问顺序和处理顺序不一致，所以上面两种方法在这不适用。那么**在使用迭代法写中序遍历，就需要借用指针的遍历来帮助访问节点，栈则用来处理节点上的元素。**

中序遍历：

**思路：栈中存放之前遍历过的节点，在程序循环运行的时候，最先遍历所有的左子节点(直到叶子节点)，然后根据栈中遍历的历史节点再自下而上遍历。**

```c++
class Solution {
public:
    vector<int> inorderTraversal(TreeNode* root) {
        vector<int> result;
        stack<TreeNode*> st;
        TreeNode* cur = root;
        while (cur != NULL || !st.empty()) {
            if (cur != NULL) { // 指针来访问节点，访问到最底层
                st.push(cur); // 将访问的节点放进栈
                cur = cur->left;                // 左
            } else {
                cur = st.top(); // 从栈里弹出的数据，就是要处理的数据（放进result数组里的数据）
                st.pop();
                result.push_back(cur->val);     // 中
                cur = cur->right;               // 右
            }
        }
        return result;
    }
};
```

其实迭代法也是可以实现统一风格的代码的，我们需要对待处理的节点加上标记，使用一个栈完成所有的操作。

```c++
class Solution {
public:
    vector<int> inorderTraversal(TreeNode* root) {
        vector<int> result;
        stack<TreeNode*> st;
        if (root != NULL) st.push(root);
        while (!st.empty()) {
            TreeNode* node = st.top();
            if (node != NULL) {
                st.pop(); // 将该节点弹出，避免重复操作，下面再将右中左节点添加到栈中
                if (node->right) st.push(node->right);  // 添加右节点（空节点不入栈）

                st.push(node);                          // 添加中节点
                st.push(NULL); // 中节点访问过，但是还没有处理，加入空节点做为标记。

                if (node->left) st.push(node->left);    // 添加左节点（空节点不入栈）
            } else { // 只有遇到空节点的时候，才将下一个节点放进结果集
                st.pop();           // 将空节点弹出
                node = st.top();    // 重新取出栈中元素
                st.pop();
                result.push_back(node->val); // 加入到结果集
            }
        }
        return result;
    }
};
```

### 二叉树的层序遍历(广度优先搜索)

二叉树的层序遍历实际上思想就是图论中的广度优先搜索，而实现这种算法一般需要使用==队列==这种数据结构。记住层序遍历的代码模版即可完成一类这样的题。

```c++
class Solution {
public:
    vector<vector<int>> levelOrder(TreeNode* root) {
        queue<TreeNode*> que;
        if(root != NULL) que.push(root);
        vector<vector<int>> result;
        while(!que.empty()){
            int size = que.size(); //记录当前层节点的个数
            vector<int> line_;
            while(size --) { //这个循环中加入每一层的node，size-- 用于控制层数
                TreeNode* node = que.front();
                que.pop();
                if(node->left != NULL) que.push(node->left);
                if(node->right != NULL) que.push(node->right);
                line_.push_back(node->val);
            }
            result.push_back(line_);
        }
        return result;
        
    }
};
```
