# MTRN2500 Tutorial Week 8 2018

##Templates

Templates are a construct available in C++ which allows type-strict C++ code to handle generic types. This means that the type which the class can handle is not known until compile time. At compile time, the classes / types which declared as an input type (using the <...> notation) to the templated class then receive a generated copy of the templated code which substitutes the generic types (in the case below, all mentions of T) with the input type.
```
// ExampleTuple.h
#include <iostream>

namespace Week8 {
  template <typename T>
  class Tuple {
    public:
      Tuple(T first, T second) : first(first), second(second) {}

      T get_first() { return this->first; }
      T get_second() { return this->second; }
    private:
      T first, second;
  };
}
Week8::Tuple<int> simple_tuple(2, 3);
```
The above code snippet creates an object of the Tuple type using the int type. All occurrences of T are then replaced with int.

```
Week8::Tuple<char> char_tuple('a', 'z');
```

Likewise, the above code snippet creates an object of the Tuple type using the char type. All occurrences of T are then replaced with char. These two constructors can be called in the same file as int and char versions are created separately from each other.

```
namespace Week8 {
  template <typename T, typename U>
  class BetterTuple {
    public:
      BetterTuple(T first, U second) : first(first), second(second) {}
      T get_first() { return this->first; }
      U get_second() { return this->second; }
    private:
      T first;
      U second;
  };
}
```
Since two typenames are used in the template declaration, all uses of T are substituted with the first input type and all uses of U are substituted with the second input type.

```
Week8::BetterTuple<int, char> nice_tuple(8, 'a');
```

The above code snippet creates an object of the BetterTuple type using the int and char types. All occurrences of T are then replaced with int, and all occurrences of U are then replaced with char.

```
Week8::BetterTuple<int, DNS::DNSEntry> custom_tuple('a', entries[0]);
```
The above code snippet creates an object of the BetterTuple type using the int and DNS::DNSEntry (which is a custom type) types. All occurrences of T are then replaced with int, and all occurrences of U are then replaced with DNS::DNSEntry.
