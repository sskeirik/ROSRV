#include <iostream>

void test_monitor()
{
    throw std::runtime_error("Oops.");
}

int main()
{
    test_monitor();
    return 0;
}

