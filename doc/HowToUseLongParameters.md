# How to use `long` parameters
**Description**: This tutorial will tell you how to use `long` parameters (which are not supported by the parameter server).

**Tutorial Level**: EASY


## Problem
`xmlrpclib.py` as used by `roslaunch` does not support type `long` (integers outside `(-2**31,2**31-1)`):

```xml
<arg name="long_param" value="12345678910111213" />
```
results in an error.

## Background

see
* https://github.com/python/cpython/blob/3.7/Lib/xmlrpc/client.py#L158
* https://docs.python.org/3.6/library/xmlrpc.client.html#module-xmlrpc.client
* http://xmlrpc.com/spec.md (search for "32-bit")

## Workaround
We use the following workaround:
- Add the substring `L` to the `long` parameter, making it a string (in case it is outside the valid integer range), e.g.
```python
gen.add("long_param", paramtype="long", description="A long parameter", default="12345678910111213L")
# or
gen.add("long_param", paramtype="long", description="A long parameter", default="1L")
# or
gen.add("long_param", paramtype="long", description="A long parameter", default=1)
```

- when setting it via an launchfile argument:
```xml
<arg name="long_param" value="12345678910111213L" />
<!-- or -->
<arg name="long_param" value="1L" />
<!-- or -->
<arg name="long_param" value="1" />
```

- when setting it via a yaml:
```yaml
long_param: "12345678910111213L"
# or
long_param: "1L"
# or
long_param: 1
```

- For `long` parameters, a string is automatically cast to `long` when reading from the parameter server
- Any `long` parameter is stored as `string` when writing to the parameter server
