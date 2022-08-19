# Log++ Logging framework

Header only library for a unified logging interface. 

## Usage

```c++
#include<log++.h>
int main() {
  int foo = 5;
  LOG(I, "Foo: " << foo);
  return 0;
}
```

## Serverity levels

- **Verbose:** Verbose in-depth debug information.
- **Debug:** Everything too verbose during normal execution and can be ignored.
- **Info:** Everything that could be of interest during a normal execution, but could also be ignored.
- **Warning:** Things that may need attention, but are probably not dangerous.
- **Error:** Things that need attention and probably are dangerous if not handled.
- **Fatal:** Everything that should stop execution immediately. Calls abort().

## Logging methods
| Method                      | Log++           | Glog                | Roslog                           |
|-----------------------------|-----------------|---------------------|----------------------------------|
| Default logging             | LOG(I)          | LOG(INFO)           | ROS_INFO()                       |
| Conditional logging         | LOG(I, COND)    | LOG_IF()            | ROS_INFO_COND()                  |
| Occasional logging          | LOG_EVERY(I, N) | LOG_EVERY_N()       | ROS_INFO_THROTTLE()              |
| Logging first N occurrences | LOG_FIRST(I, N) | LOG_FIRST_N(INFO,1) | ROS_INFO_ONCE(), ROS_INFO_COND() |

***

### Default logging

```c++
int foo = 1;
int bar = 3;
LOG(I, "Values: " << foo << bar);
```

###  Conditional logging
```c++
int foo = 1;
int bar = 3;
LOG(I, foo != bar, "Values: " << foo << bar)
```

### Occasional logging
```c++
int foo = 1;
int bar = 3;
LOG_EVERY(I, 20, "Values: " << foo << bar) // Log every 20 calls.
```

### Logging first N occurrences
```c++
int foo = 1;
int bar = 3;
LOG_FIRST(I, 5, "Values: " << foo << bar)
```