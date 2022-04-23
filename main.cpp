#include "yaws.h"


int main(void) 
{
    Yaws weatherStation;
    weatherStation.run();

    while(true){
        printf("YAWS has failed! \n");
    }
}
