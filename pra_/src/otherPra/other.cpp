/**
 * @file other.cpp 一些基础语法/临时功能验证测试等
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <map>
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