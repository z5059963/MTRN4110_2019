# MTRN4110 Tutorial 3 C++
This document covers advanced C++ features that may not nessesarily be covered in MTRN2500/MTRN3500

A high quality C++ reference can be found [here](https://en.cppreference.com/w/)

# [Auto keyword](https://en.cppreference.com/w/cpp/language/auto)
The keyword [Auto](https://en.cppreference.com/w/cpp/language/auto) specify the type it is replacing will be automatically deduced by the compiler. For example
```C++
auto i = 0; // declare a variable i with type int.
auto d = 0.0; // declare a variable d with type double.
```

The type is deduced using template argument deduction rules.

# Return type deduction
The keyword auto can also be used to tell the compiler to deduce the return type of a function.
```C++
auto foo (int) {
  auto i = 22;
  return i; // Automatically deduce the return type to be int.
}
```

# [Trailing return type](https://en.cppreference.com/w/cpp/language/function)
In the code 
```C++
auto example (int input) -> double;
```
The `->` indicates trailing return type where the return type is moved to the end of the function signature. This feature is useful for when the return type depends on
the argument names, and make the code align nicer.

# [Enumeration](https://en.cppreference.com/w/cpp/language/enum)
An enumeration is often used to represents a set of names constants. Enum allows names (enumerators) to be assigned to integer value for example
```C++
enum Day {Sat, Sun, Mon, Tue, Wed, Thu, Fri = 10};
```
creates an enum `Day` that is used to represent the day of the week. `auto today = Sat` creates a variable `today` with value representing Saturday. By default enum start number from 0. Different value can be assigned to enum, for example `Fri` was assigned the value 10.

The C type enum as seen above are not scoped, meaning the enumerators can be implicitly converted to an integer. C++ introduces scoped enumerator which can't be implicitly convereted to the underlying value. Eg:
```C++
enum class io_mode : uint8_t
{
    input = 0x00U,
    output = 0x01U,
};

io_mode a = io_mode::input; // OK
int b = io_mode::output; // Error
int c = static_cast<uint8_t>( io_mode::output); // explicit casting ok. c = 1;

```
Here `: uint8_t` indicate the underlying storage type is uint8_t ( unsigned int with 8 bits). `0x` denotes the value is represented in hexidecimal. `U` indicate the value is unsigned.

# [Using-declaration](https://en.cppreference.com/w/cpp/language/using_declaration)
`using pin_t = uint8_t;` makes pin_t an aliase of type uint8_t. Used to give type more meaning full names or make complex type shorter. Replaces C typedef.

# [Explicit instantiation](https://en.cppreference.com/w/cpp/language/class_template)
All the class template in the provided headers need to be explicitly instantiated in the implimentation file, otherwise the linker will complain with undefined reference. Example of the syntax is
```C++
template class digital_pin<1U>; //Instantiate the digital_pin with template argument 1.
```


