#include <stdio.h>

#define testter(t) --(*t)

void test(unsigned char *transactions){
    for(;*transactions>0;testter(transactions)){
        printf("hello\n");
    }
}

int main(){
    unsigned char tarans = 10;
    while (1)
    {
        /* code */ test(&tarans);
    }
    
   
    return 0;
}