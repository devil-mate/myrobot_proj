/**
 * @file OtherPra.cpp 一些基础语法/临时功能验证测试等
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <memory.h>
#include <map>

#include "OtherPra.h"
uint8_t MAC_ADDR_ARR1[]={0x2F,0xA1,0xB5,0x0D,0x00,0x4B,0x12,0x00}; 
uint8_t MAC_ADDR_ARR2[]={0xAF,0x61,0x97,0x0E,0x00,0x4B,0x12,0x00};
uint8_t MAC_ADDR_ARR3[]={0xE8,0x12,0xAF,0x12,0x00,0x4B,0x12,0x00};
uint8_t MAC_ADDR_ARR4[]={0x31,0xD1,0x49,0x09,0x00,0x4B,0x12,0x00};
uint8_t MAC_ADDR_ARR5[]={0xDD,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xC1};
#define MACSIZE             (8)

std::map<int,int(*)[3]> idmap;
std::map<uint8_t*,uint16_t >                idMap_;
void printArr(unsigned char *buff,unsigned int length){
    if(length<=0){
        return;
    }
    for(unsigned int i = 0; i < length; i++){
        printf("%02x ",buff[i]);
    }
    // printf("\n");

}
void printMap(std::map<uint8_t*,uint16_t > &map){
    for(auto &id:map){
        printf("(");
        printArr(id.first,MACSIZE);
        printf(",%x),\n",id.second);
    }
    printf("\n");
}

void  testMap2(){

}
void  testMap(){
    uint16_t shortAddr=0x1223;
    idMap_.insert(std::make_pair(MAC_ADDR_ARR1,shortAddr));
    idMap_.insert(std::make_pair(MAC_ADDR_ARR2,shortAddr));
    idMap_.insert(std::make_pair(MAC_ADDR_ARR3,shortAddr));
    printf("idMap_[%d]: \n",idMap_.size());
    printMap(idMap_);
    idMap_.erase(MAC_ADDR_ARR2);
    for(auto &id:idMap_){
        int8_t cnt=idMap_.count(id.first);
        printf("cnt: %d\n",cnt);
    }
    printf("idMap_[%d]: \n",idMap_.size());
    printMap(idMap_);
    // (idMap_.at(MAC_ADDR_ARR1).first);
    

}
// TestClass::TestClass(){
//     int();
// }
TestClass::TestClass():array_({0,1,2}){
    int();
}
TestClass::TestClass(TestClass &src){
    int();
}
void TestClass::init(){
    // 这里初始化没有作用,需要初始化列表进行初始化?
    // memset(&array_[0],0,sizeof(array_));
    // array_[1]=20;
    // 怎么让一个智能指针指向已经存在的指针?
    // array_=arrPtr_.get();
    arrPtr_ = std::unique_ptr<uint8_t[]>((uint8_t*)array_);
    arrSharedPtr_ = std::shared_ptr<uint8_t>((uint8_t*)array_);

}
void TestClass::print(){
    printf("array [");
    for(int i=0; i<10;i++){
        // printf("%d,",(arrPtr_[1]));
    }
    printf("]\n");
    printf("array_[%p], arrptr_[%p], arrSharedptr_[%p]\n",
                array_,arrPtr_.get(),arrSharedPtr_.get());
}

int testFunc(){

    TestClass test;
    test.print();
    return 0;
}