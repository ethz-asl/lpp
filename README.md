# Log++ Logging framework

Header only library to standardize roslog and glog output at compile time.

## Usage

The `log++.h` header must be defined in every file containing log functions:
```c++
#include <ros/ros.h>
#include <log++.h>

int main(int argc, char **argv) {
  LOG_INIT(argv[0]);
  int foo = 5;
  LOG(I, "Foo: " << foo);
  return 0;
}
```

Add the following line to `CMakeLists.txt` to select the desired mode for...

- a project:
```cmake
# Valid modes are: MODE_LPP MODE_GLOG MODE_ROSLOG MODE_DEFAULT
add_compile_definitions(MODE_LPP)
```

- a specific executable or library:
```cmake
target_compile_definitions(my_executable PRIVATE MODE_LPP) 
```

## Modes
- **MODE_GLOG:** Google Logging syntax. Calls abort() if it logs a fatal error.
- **MODE_ROSLOG:** ROS Logging syntax.
- **MODE_LPP** Log++ Logging syntax.
- **MODE_DEFAULT:** Disables Logging standardization. Messages are logged according to their framework

## Severity levels

- **Debug:** Everything too verbose during normal execution and can be ignored.
- **Info:** Everything that could be of interest during a normal execution, but could also be ignored.
- **Warning:** Things that may need attention, but are probably not dangerous.
- **Error:** Things that need attention and probably are dangerous if not handled.
- **Fatal:** Everything that should stop execution immediately. Calls abort().

## Logging methods
| Method                       | Log++           | Glog                 | Roslog                              | 
|------------------------------|-----------------|----------------------|-------------------------------------|
| Default logging              | LOG(I)          | LOG(INFO)            | ROS_INFO()                          |
| Conditional logging          | LOG(I, COND)    | LOG_IF(INFO, COND)   | ROS_INFO_COND()                     |
| Occasional logging (every n) | LOG_EVERY(I, N) | LOG_EVERY_N(INFO, N) | -                                   |
| Occasional logging (by time) | -               | -                    | ROS_INFO_THROTTLE()                 |
| Logging first N occurrences  | LOG_FIRST(I, N) | LOG_FIRST_N(INFO, N) | ROS_INFO_ONCE() (only 1 occurrence) |

***

## Examples

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

for (int i = 0; i < 50; i++) {
  LOG_EVERY(I, 20, "Values: " << foo << bar) // Log every 20 calls.
}

```

### Logging first N occurrences
```c++
int foo = 1;
int bar = 3;

for (int i = 0; i < 10; i++) {
  LOG_FIRST(I, 5, "Values: " << foo << bar) //Log first 5 calls.
}

```

# Roadmap
- Implement ROS_INFO_THROTTLE()
- Implement ROS_INFO_NAMED()
- Make log policies thread-safe

# Tests
- All modes (glog, lpp, roslog) have a separate test suite
- Test Logging syntax for each mode
- All severity levels (Info, Warning, Error, Fatal)
- Maybe test Stream operator with different types
