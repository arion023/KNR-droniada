#include<iostream>
#include <cstring> // for memcpy
using namespace std;

int main() {
    unsigned char x[4] = { 0x41, 0x42, 0x43, 0x44};


    cout<<x[0]<<endl;
    cout<<x[1]<<endl;
    cout<<x[2]<<endl;
    cout<<x[3]<<endl;

    float value;
    memcpy(&value, x, sizeof(value));

    cout<<value<<endl;


    return 0;
}