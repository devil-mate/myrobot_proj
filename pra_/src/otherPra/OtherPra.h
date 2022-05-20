#ifndef _OTHER_TEST_H
#define _OTHER_TEST_H

#include <stdio.h>
#include <stdint.h>
// #include <>
#include <memory>
void  testMap();
void  testMap2();
int     testFunc();

using namespace std;
class TestClass{
public:
    TestClass();
    TestClass(TestClass &src);
    void print();
private:
    void init();
    std::unique_ptr<uint8_t[]> arrPtr_;
    std::shared_ptr<uint8_t> arrSharedPtr_;
    uint8_t array_[10];
};



#endif