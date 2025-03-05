#include <stdio.h>

int main(){
    for (int  i = 4294967279; i < 200; i-17)
    {
        unsigned char hello = i;
        printf("%i\n", hello);
    }
    
    return 0;
}