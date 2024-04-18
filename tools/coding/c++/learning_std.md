# std::minmax_element()函数

`std::minmax_element` 是 C++ STL 中的一个算法函数，用于查找**一个范围内的最小值和最大值**。它返回一个 `**std::pair**` 对象，其中第一个元素是指向最小元素的迭代器，第二个元素是指向最大元素的迭代器。

`std::minmax_element` 函数的第三个参数是一个可选的比较函数，用于指定如何比较元素的大小。如果不提供该参数，则**默认使用 `<` 运算符进行比较**。

比较函数需要接受两个参数，分别表示要比较的两个元素，返回值为 `bool` 类型。如果第一个元素小于第二个元素，则返回 `true`，否则返回 `false`。

在使用 `std::minmax_element` 函数时，==如果元素类型是内置类型或支持 `<` 运算符==，可以不提供比较函数。但如果元素类型是自定义类型，或者需要按照不同的比较方式进行排序，就需要提供比较函数。

以下是一个查找pcl点云的实例：

```c++
auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.y < p2.y; });
```

实例中，第三个参数是 \[](const PointType& p1, const PointType& p2) { return p1.x < p2.x; }，在本例中由于操作的对象是点云中的三维点，本身是不支持<运算符的，所以需要提供比较函数。



#std::transform()函数

`std::transform`是C++标准库中的一个算法函数，用于将一个序列的元素转换为另一个序列。它的基本语法如下：

```c++
template<class InputIt, class OutputIt, class UnaryOperation>
OutputIt transform(InputIt first1, InputIt last1, OutputIt d_first, UnaryOperation unary_op);
```

其中，`first1`和`last1`是输入序列的起始和结束迭代器，`d_first`是输出序列的起始迭代器，`unary_op`是一个一元函数对象，用于对输入序列中的每个元素进行转换。

`std::transform`函数会遍历输入序列中的每个元素，对每个元素调用`unary_op`函数，将其转换为一个新的值，并将该值存储到输出序列中。输出序列的大小应该与输入序列相同，或者至少应该足够大以容纳所有转换后的元素。

以下是一个简单的示例，演示如何使用`std::transform`将一个整数序列中的每个元素加倍，并将结果存储到另一个序列中：

```c++
#include <iostream>
#include <vector>
#include <algorithm>
int main()
{
    std::vector<int> input = {1, 2, 3, 4, 5};
    std::vector<int> output(input.size());

    std::transform(input.begin(), input.end(), output.begin(), [](int x) { return 2 * x; });

    for (auto x : output)
    {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    return 0;
}
```

# std::copy()函数

和std::transform()函数功能类似，但功能更加简单，只是简单的将容器内的元素进行复制。`std::copy`的基本语法如下：

```c++
template<class InputIt, class OutputIt>
OutputIt copy(InputIt first, InputIt last, OutputIt d_first);
```

其中，`first`和`last`是输入序列的起始和结束迭代器，`d_first`是输出序列的起始迭代器。`std::copy`函数会遍历输入序列中的每个元素，并将其复制到输出序列中。输出序列的大小应该与输入序列相同，或者至少应该足够大以容纳所有复制后的元素。

# std::function()函数

`std::function`是C++11标准库中的一个类模板，用于封装任意可调用对象（函数、函数指针、成员函数指针、函数对象等）。

`std::function`的基本语法如下：

```c++
template<class R, class... Args>
class function<R(Args...)>;
```

其中，`R`是返回值类型，`Args...`是参数类型列表。`std::function`可以用来声明一个函数对象，该函数对象可以接受参数类型为`Args...`的参数，并返回类型为`R`的值。

以下是一个简单的示例，演示如何使用`std::function`来封装一个函数对象：

```c++
#include <iostream>
#include <functional>

int add(int a, int b)
{
    return a + b;
}

int main()
{
    std::function<int(int, int)> f = add;

    std::cout << f(2, 3) << std::endl;

    return 0;
}
```

在这个示例中，`add`是一个普通的函数，它接受两个整数参数并返回它们的和。`std::function<int(int, int)>`声明了一个函数对象，该函数对象接受两个整数参数并返回一个整数值。`f`是一个函数对象，它被初始化为`add`函数的指针。最后，`f(2, 3)`调用了`add`函数，并输出了它的返回值。

除了封装普通函数之外，`std::function`还可以用来**封装函数对象、函数指针、成员函数指针**等。以下是一个示例，演示如何使用`std::function`来封装一个函数对象和一个成员函数指针：

```c++
#include <iostream>
#include <functional>

class Foo
{
public:
    int add(int a, int b)
    {
        return a + b;
    }
};

int main()
{
    Foo foo;
    std::function<int(int, int)> f1 = foo.add;
    std::function<int(int, int)> f2 = [](int a, int b) { return a * b; };

    std::cout << f1(2, 3) << std::endl;
    std::cout << f2(2, 3) << std::endl;

    return 0;
}
```

# std::back_inserter()函数

`std::back_inserter`是C++标准库中的一个迭代器适配器，用于将元素插入到容器的末尾。`std::back_inserter`的基本语法如下：

```c++
template<class Container>
std::back_insert_iterator<Container> back_inserter(Container& c);
```

其中，`Container`是容器类型，`c`是容器对象。`std::back_inserter`返回一个`std::back_insert_iterator`对象，该对象可以用于将元素插入到`c`容器的末尾。

以下是一个简单的示例，演示如何使用`std::back_inserter`将元素插入到`std::vector`容器的末尾：

```c++
#include <iostream>
#include <vector>
#include <iterator>

int main()
{
    std::vector<int> v1 = {1, 2, 3};
    std::vector<int> v2;

    std::copy(v1.begin(), v1.end(), std::back_inserter(v2));

    for (auto x : v2)
    {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    return 0;
}
```

在这个示例中，`v1`是一个`std::vector`容器，包含三个整数元素。`v2`是另一个`std::vector`容器，初始为空。`std::copy`算法使用`std::back_inserter(v2)`作为输出迭代器，将`v1`中的所有元素插入到`v2`容器的末尾。最后，程序输出了`v2`容器中的所有元素。

#std::for_each()函数

`std::for_each`是C++标准库中的一个算法函数，用于对容器中的每个元素执行指定的操作。`std::for_each`的基本语法如下：

```c++
template<class InputIt, class UnaryFunction>
UnaryFunction for_each(InputIt first, InputIt last, UnaryFunction f);
```

其中，`first`和`last`是输入序列的起始和结束迭代器，`f`是一个一元函数对象，用于对输入序列中的每个元素执行指定的操作。

`std::for_each`函数会==遍历输入序列中的每个元素，对每个元素调用`f`函数==，执行指定的操作。`f`函数可以是一个函数指针、函数对象或lambda表达式。

以下是一个简单的示例，演示如何使用`std::for_each`对一个`std::vector`容器中的每个元素执行指定的操作：

```c++
#include <iostream>
#include <vector>
#include <algorithm>

void print(int x)
{
    std::cout << x << " ";
}

int main()
{
    std::vector<int> v = {1, 2, 3};

    std::for_each(v.begin(), v.end(), print);

    return 0;
}
```

# std::execution()函数

`std::execution`是C++17中引入的一个命名空间，用于支持并行算法的执行策略。它提供了三种执行策略：`std::execution::seq`、`std::execution::par`和`std::execution::par_unseq`。

- `std::execution::seq`：顺序执行策略，表示算法将按照顺序执行，不会并行化。
- `std::execution::par`：并行执行策略，表示算法将并行执行，但不保证元素的顺序。
- `std::execution::par_unseq`：并行执行策略，表示算法将并行执行，并且可以改变元素的顺序。

# std::pair()函数

`std::pair`是C++标准库中的一个模板类，用于表示两个值的有序对。`std::pair`的定义如下：

```c++
template<class T1, class T2>
struct pair;
```

其中，`T1`和`T2`是两个类型参数，分别表示有序对中的第一个值和第二个值。

以下是一个简单的示例，演示了如何使用`std::pair`来表示两个值的有序对：

```c++
#include <iostream>
#include <utility>

int main()
{
    std::pair<int, double> p1(1, 3.14);
    std::pair<std::string, bool> p2("hello", true);

    std::cout << p1.first << " " << p1.second << std::endl;
    std::cout << p2.first << " " << p2.second << std::endl;

    return 0;
}
```

在这个示例中，定义了两个`std::pair`对象：`p1`和`p2`。`p1`表示一个整型和一个双精度浮点型的有序对，`p2`表示一个字符串和一个布尔型的有序对。然后，程序输出了`p1`和`p2`中的两个值。

`std::pair`还可以用于其他一些场景，例如：

- 作为函数的返回值，用于返回多个值。
- ==作为容器的元素类型，用于存储多个值。==
- ==作为关联容器的键值类型，用于存储键值对。==

需要注意的是，`std::pair`只能表示两个值的有序对，如果需要表示更多的值，可以使用`std::tuple`。

# std::distance()

用于计算两个迭代器之间的距离。其函数原型如下：

```c++
template <class InputIt>
typename std::iterator_traits<InputIt>::difference_type
    std::distance(InputIt first, InputIt last);
```

其中，`first` 和 `last` 分别表示要计算距离的迭代器的起始位置和终止位置。该函数返回两个迭代器之间的距离，即 `last - first` 的值，类型为 `std::iterator_traits<InputIt>::difference_type`。

需要注意的是，`std::distance()` 函数只适用于随机访问迭代器，因为只有随机访问迭代器才支持 `operator-` 运算符。对于其他类型的迭代器，可以使用 `std::advance()` 函数来实现类似的功能。

以下是一个示例代码，演示了 `std::distance()` 函数的用法：

```c++
#include <iostream>
#include <vector>
#include <iterator>

int main()
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    auto first = v.begin();
    auto last = v.end();

    std::cout << "Distance between first and last: "
              << std::distance(first, last) << std::endl;

    return 0;
}
```

输出结果如下：Distance between first and last: 5

# std::accumulate()

用于计算一个序列中所有元素的累加和。以下是`std::accumulate`的基本用法：

```c++
#include <iostream>
#include <numeric>
#include <vector>

int main()
{
    std::vector<int> v = {1, 2, 3, 4, 5};

    int sum = std::accumulate(v.begin(), v.end(), 0);

    std::cout << "sum = " << sum << std::endl;

    return 0;
}
```

在这个示例中，定义了一个整型向量`v`，包含了五个整数。然后，使用`std::accumulate`函数计算了`v`中所有元素的累加和，并将结果存储在`sum`变量中。最后，程序输出了`sum`的值。

`std::accumulate`函数的第一个参数是序列的起始迭代器，第二个参数是序列的结束迭代器，第三个参数是累加器的初始值。`std::accumulate`函数会从起始迭代器开始遍历序列，将每个元素加到累加器中，并返回最终的累加结果。

`std::accumulate`函数还可以接受一个可调用对象作为第四个参数，用于指定累加操作。例如，可以使用`std::multiplies`函数对象计算序列中所有元素的乘积：

````c++
#include <iostream>
#include <numeric>
#include <vector>

int main()
{
    std::vector<int> v = {1, 2, 3, 4, 5};

    int product = std::accumulate(v.begin(), v.end(), 1, std::multiplies<int>());

    std::cout << "product = " << product << std::endl;

    return 0;
}
````

在这个示例中，使用`std::multiplies<int>()`函数对象作为第四个参数，用于指定累乘操作。`std::multiplies<int>()`函数对象表示整数乘法操作，可以将两个整数相乘。最终，程序输出了序列中所有元素的乘积。

需要注意的是，`std::accumulate`函数只能用于支持加法和乘法等基本操作的类型，例如整型、浮点型、指针等。如果需要对自定义类型进行累加操作，需要提供一个自定义的累加函数。
