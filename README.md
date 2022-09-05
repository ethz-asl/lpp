# Log++ Logging framework

Header only library to standardize roslog and glog output at compile time.

## Usage

```c++
#include <ros/ros.h>
#include <log++.h> // Must be defined AFTER <ros/ros.h> or <glog/logging.h>

int main(int argc, char **argv) {
  LOG_INIT(argv[0]);
  int foo = 5;
  LOG(I, "Foo: " << foo);
  return 0;
}
```

Add the following line to cmake to select desired mode:
```cmake
# Valid modes are: MODE_GLOG MODE_ROSLOG MODE_LPP MODE_DEFAULT
target_compile_definitions(my_executable PRIVATE MODE_LPP) 
```

## Modes
- **MODE_GLOG:** Google Logging syntax. Calls abort() if it logs a fatal error.
- **MODE_ROSLOG:** ROS Logging syntax.
- **MODE_LPP** Log++ Logging syntax.
- **MODE_DEFAULT:** Disables Logging standardization. Messages are logged according to their framework

## Severity levels

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
***

## Limitations
- ROS_INFO() only accepts one argument at the moment
- Header file is not self-contained (include order matters)
***

## Tests

- All modes (default, glog, lpp, roslog)
- Test Logging syntax for each mode
- All severity levels (Info, Warning, Error, Fatal)
- Maybe test Stream operator with different types
